/*
 * Copyright (c) 2013, Marcus Linderoth, http://forfunandprof.it
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/**
 * \file
 *    hpdl1414-clock.c
 * \author
 *         Marcus Linderoth <linderoth.marcus@gmail.com>
 * \desc
 *    A simple clock based on the HPDL-1414 display
 */

/*
  Hardware: a msp430g2452 or g2553 with 32.768 kHz xtal to XIN/XOUT and a press-
      button to P1.3, just like on the regular Launchpad. Then a HPDL-1414
      connected like this


 *            1.6   2.3   2.2   2.1   2.0    GND        LP/MSP430G2553 pin
 *            12    11    10    9     8      7          pin# acc to datasheet
 *            |     |     |     |     |      |
 *         +--|-----|-----|-----|-----|------|----+
 *         |  D6    D3    D2    D1    D0    GND   |     pin function
 *         |                                      |
 *         |                                      |
 *         |    (1)     (2)       (3)     (4)     |     char# acc to this driver
 *         |                                      |
 *         |                                      |
 *         |  D5    D4    WR    A1    A0    Vcc   |     pin function
 *         +--|-----|-----|-----|-----|------|----+
 *            |     |     |     |     |      |
 *            1     2     3     4     5      6          pin# acc to datasheet
 *            2.5   2.4   1.7   1.5   1.4    +5V        LP/MSP430G2553 pin



  The clock works like this:
    * it starts in clock mode
    * once every second, a second-counter is increased and the display is updated
    * if a button is pressed, the clock changes into 'set-time-mode', indicated by
      flashing of the display a few times.
    * in 'set-time-mode', pressing the button increases hours by one. Holding
      the button will increase setting-speed
    * when the button is not pressed for a while, setting will change to setting
      of minutes
    * after not pressing the button for a while again, the clock returns to clock
      mode.

  Issues/todo:
      losing power == losing time == messy to start over with setting time
        --Should on every minute update store time in flash or similar and load on bootup
        --no, perhaps on every hour with some wear-leveling mechanism. 10^4-10^5 cycles
          means less than 1.5 years before meltdown if on the hour and no WL mechanism.
          come up with something better
        ++skip storing time (unless we later get an RTC/EEPROM or similar), made setting
          time simpler and faster instead.
      increase power efficiency
        --don't update the display every second
        --reduce clock speed (1 MHz, or slower)
        --power off the display after a while, turn on by button press
        --more efficient voltage regulator
        --display dims after a while
      alarm functionality
        could use the pin for LED to instead power a vibrator (perhaps /and/) an LED
        how set alarm?
      debug/show-off mode
        for showing the beautiful display
        --if button held at startup, according to some pattern, then show all chars
        --or, start this from clock mode; then we won't have to reset the clock to demo it
      fix smoother dims: sinus particularly
      temperature sensor (msp430 built-in)
        when show? how show?
          --pressing once to set time, then when timeouts, shows temp.
        accuracy?
          --doens't matter that much, just for fun anyway.
        better separation of modules, eg UI/display updater vs UI/button handler
          --well, yes, but not a priority at this stage, the project is not anticipated to swell much more
        for what use the LED
          blinks on every second can be annoying
        configuration mode?
          to set eg
              LED blink
              display dimming (global override)
              display off or dimming during the night + set hours for that
        check long-term accuracy of the clock
          how much due to cheap 32.768 xtal?
          how much due to using etimer wait for 1 second?
            --instead perhaps use clock_seconds() if better



  done
      enable dimming the display
        highest might be to bright
          --no, skip that, always brightest
        for fading strings
          --use simplepwm
      setting time goes either too fast or too slow.
        --Should start slow and then go faster
        ++ done
      dim "Set Time"
      broadcast dim_done_event
 */

#include "contiki.h"
#include "dev/button.h"
#include "simple-pwm.h"
#include "dev/adc.h"

/* setting DEBUG uses LED and prints over serial port instead of HPDL1414 */
#define DEBUG                             0
#define DEBUG_FASTTIME                    0   /* minutes are sped up to 5 seconds */
#define ENABLE_DEMO_MODE_ON_STARTUP       1
#define ENABLE_SPLASH_MESSAGE             1

/* ensure we never  */
#if !DEBUG
#warning "Warning! We are not compiling in DEBUG mode, yet DEBUG_FASTTIME is set! Is this what you want? (hpdl1414-clock.c)"
//#undef DEBUG_FASTTIME
//#define DEBUG_FASTTIME 0
#endif

/* set fading style, only one can be non-zero at once */
#define FADE_STYLE_LINEAR                 0   /* doesn't work very well now.. */
#define FADE_STYLE_SINUS                  0   /* doesn't work very well now.. */
#define FADE_STYLE_JUST_CUT               0
#define FADE_STYLE_SIMPLE                 1
/*---------------------------------------------------------------------------*/
#if DEBUG
  #include <stdio.h>
  #include "dev/leds.h"
  #define hpdl_init()                 printf("HPDL:init\n");
  #define hpdl_clear()                printf("HPDL Clr\n");
  #define hpdl_write_string(b)        printf("HPDL W:%s\n", b);
  #define hpdl_write_char(p, c)       printf("HPDL Wchar: %u:%c\n", p, c);
#else   /* DEBUG */
  #include "dev/hpdl1414.h"
  #define printf(...)
  #define leds_on(...)
  #define leds_off(...)
#endif  /* DEBUG */


/* configurations-------------------------------------------------------------*/
/* splash message */
static const char splash_message[] =    "      HPDL-1414 clock - Contiki 2.6      ";
#define SPLASH_LENGTH                   (sizeof(splash_message) + 1)
#define SPLASH_UPDATE_INTERVAL          (CLOCK_SECOND / 8 + CLOCK_SECOND / 16)
#define SPLASH_POST_SPLASH_WAIT         (CLOCK_SECOND / 2)

#define DEMO_START_CONDITION_DURATION       (CLOCK_SECOND * 2)
#define DEMO_SPINNER_DURATION               (CLOCK_SECOND * 4)
/*---------------------------------------------------------------------------*/
/* button interface / setting time */
/* the time for time-set-mode to timeout if button isn't pressed again */
#define UI_TIMEOUT                              (CLOCK_SECOND * 2)

/*
 * increment speed when setting time;
 * range 1..(BUTTON_HOLD_CHECK_RATE-1)
 * higher is faster
 */
#define BUTTON_HOLD_MINUTES_UPDATERATE_SLOW     16
#define BUTTON_HOLD_MINUTES_UPDATERATE_FAST     55

/* this many time-increments before switching to the fast update rate */
#define BUTTON_HOLD_UPDATE_FAST_THRESHOLD       5

/* when switching between clock-mode and set-time-mode, the display is blinked
    this many times with this interval */
#define MODE_SWITCH_BLINK_INTERVAL              (CLOCK_SECOND / 16)
#define MODE_SWITCH_BLINK_COUNT                 5

/* derived and other definitions-------------------------------------------- */
/* how often to check the button to see if it is held down */
#define BUTTON_HOLD_CHECK_RATE            64
#define BUTTON_HOLD_CHECK_INTERVAL        (CLOCK_SECOND / BUTTON_HOLD_CHECK_RATE)

/* this uses the defintions from dev/button.h */
#define BTN_IS_HELD_DOWN()                (!(BUTTON_PORT(IN) & BUTTON_2))

/* sanity checks */
#if BUTTON_HOLD_MINUTES_UPDATERATE_SLOW == 0 || BUTTON_HOLD_MINUTES_UPDATERATE_SLOW > BUTTON_HOLD_CHECK_RATE
#error BUTTON_HOLD_MINUTES_UPDATERATE_SLOW is misconfigured, check range (hpdl1414-clock.c)
#endif

#if BUTTON_HOLD_MINUTES_UPDATERATE_FAST == 0 || BUTTON_HOLD_MINUTES_UPDATERATE_FAST > BUTTON_HOLD_CHECK_RATE
#error BUTTON_HOLD_MINUTES_UPDATERATE_FAST is misconfigured, check range (hpdl1414-clock.c)
#endif

#if BUTTON_HOLD_MINUTES_UPDATERATE_SLOW > BUTTON_HOLD_MINUTES_UPDATERATE_FAST
#warning ********   Button update speeds might be misconfigured; now SLOW is faster than FAST. Do you want that? (hpdl1414-clock.c)
#endif
/*---------------------------------------------------------------------------*/
#define MSGCOPY(s, d)   do {d[0]=s[0];d[1]=s[1];d[2]=s[2];d[3]=s[3];} while(0);
/*---------------------------------------------------------------------------*/
/* string buffers for the two messages */
/*static volatile char *msg_first;*/
/*static volatile char *msg_second;*/
static volatile char msg_first[5];
static volatile char msg_second[5];

static process_event_t dim_event;
static process_event_t dim_done_event;

/* temperature readings */
static volatile uint16_t temperature = 0;
/* -------------------------------------------------------------------------- */
PROCESS(clockdisplay_process, "HPDL-1414 Process");
PROCESS(ui_process, "UI Process");
PROCESS(display_dim_process, "Display fade process");
AUTOSTART_PROCESSES(&clockdisplay_process, &ui_process, &display_dim_process);
/*---------------------------------------------------------------------------*/
static void update_ascii_buffer(char *buf);
static void byte_to_ascii(char *buf, uint8_t val, uint8_t zero_tens_char);
/*---------------------------------------------------------------------------*/
/* the main clock timer that is used in the clock loop */
static struct etimer clock_timer;

/* keeps track of time */
static volatile uint8_t seconds = 0;
static volatile uint8_t minutes = 0;
static volatile uint8_t hours = 9;

/* ASCII buffer for time */
static char hpdlbuf[5];

/* flag that halts regular time-keeping while clock is in set-mode */
static volatile uint8_t clock_is_in_confmode = 0;

/* this flag is set when all booting (demo, splash etc) is done; holds off the
  UI process so we don't configure the clock before it is running. */
static volatile uint8_t clock_bootdone = 0;
/*--------------------------------------------------------------------------*/
/* for a blinking LED, this timer and callback turns the LED off */
static struct ctimer led_ctimer;
void
led_off_cb(void *d)
{
  P1OUT &= ~(1<<0);
}
/*---------------------------------------------------------------------------*/
/* this is the normal clock-mode process */
PROCESS_THREAD(clockdisplay_process, ev, data)
{
  PROCESS_BEGIN();
  static uint8_t i;
  static uint8_t onoff = 0;

  static struct etimer etio;

  /* init LED */
  P1DIR |= 1<<0;
  P1SEL &= ~(1<<0);
  P1SEL2 &= ~(1<<0);

  /* allocate the dim event */
  dim_event = process_alloc_event();
  dim_done_event = process_alloc_event();

  /* init display and ASCII buffer */
  hpdl_init();
  msg_first[4] = 0;
  msg_second[4] = 0;

#if ENABLE_DEMO_MODE_ON_STARTUP
  {
    static char buf[4];
    static const char anim[] = {'|', '/', '-', '\\', '|', '/', '-', '\\'};
    static clock_time_t t0;

    /* back off a little while so the user have time to press the button */
    etimer_set(&clock_timer, CLOCK_SECOND/2);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&clock_timer));

    /* check for demo conditions */
    t0 = clock_time();
    while((clock_time() < (t0 + DEMO_START_CONDITION_DURATION)) && BTN_IS_HELD_DOWN()) {
      etimer_set(&clock_timer, CLOCK_SECOND/8);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&clock_timer));
    }

    if(BTN_IS_HELD_DOWN()) {
      /* do demo-mode */
      static uint8_t c = 0;

      /* spinner */
      hpdl_clear();
      t0 = clock_time();
      while(clock_time() < (t0 + DEMO_SPINNER_DURATION)) {
        /* positions are 1..4 counting from left to right */
        hpdl_write_char(2, anim[c]);
        hpdl_write_char(3, anim[c]);
        c++;
        if(c > 7) {
          c = 0;
        }
        etimer_set(&etio, CLOCK_SECOND/16);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etio));
      }
      hpdl_clear();

      /* all characters */
      //valid range: i > 0x20, i < 0x5f
      for(i = 0x21; i < 0x5f-3; i += 1) {
        buf[0] = i;
        buf[1] = i + 1;
        buf[2] = i + 2;
        buf[3] = i + 3;
        hpdl_write_string(buf);
        etimer_set(&etio, CLOCK_SECOND/4);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etio));
      }
      etimer_set(&etio, CLOCK_SECOND/2);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etio));
      hpdl_clear();
    }
  }

#endif /* if 0; commented out code */


  /* show splash message */
#if ENABLE_SPLASH_MESSAGE
  for(i = 0; i < SPLASH_LENGTH - 1; i += 1) {
    hpdl_write_string(&(splash_message[i]));
    etimer_set(&clock_timer, SPLASH_UPDATE_INTERVAL);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&clock_timer));
  }
  etimer_set(&clock_timer, SPLASH_POST_SPLASH_WAIT);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&clock_timer));

  MSGCOPY("    ", msg_first);
  MSGCOPY("Hi! ", msg_second);
  process_post(&display_dim_process, dim_event, NULL);
  etimer_set(&clock_timer, CLOCK_SECOND);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&clock_timer));

  MSGCOPY("Hi! ", msg_first);
  MSGCOPY("Set ", msg_second);
  process_post(&display_dim_process, dim_event, NULL);
  etimer_set(&clock_timer, CLOCK_SECOND);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&clock_timer));

  MSGCOPY("Set ", msg_first);
  MSGCOPY("time", msg_second);
  process_post(&display_dim_process, dim_event, NULL);
  etimer_set(&clock_timer, CLOCK_SECOND*2);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&clock_timer));

  MSGCOPY("time", msg_first);
  MSGCOPY("    ", msg_second);
  process_post(&display_dim_process, dim_event, NULL);
  etimer_set(&clock_timer, CLOCK_SECOND);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&clock_timer));
#endif /* if 0; commented out code */

  /* prepare and set initial time */
  hpdlbuf[4] = 0;
  update_ascii_buffer(hpdlbuf);
  hpdl_write_string(hpdlbuf);
  clock_bootdone = 1;

  /* the big 'ole clock loop; counts seconds and sets time accordingly. */
  etimer_set(&clock_timer, CLOCK_SECOND);
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&clock_timer));
    etimer_reset(&clock_timer);

    /* get temperature reading (10 bits) from the internal temperature sensor */
    adc_get_noblock(TEMP, &temperature);

    /* blink the LED */
    P1OUT |= 1<<0;
    ctimer_set(&led_ctimer, CLOCK_SECOND/64, led_off_cb, NULL);

    /* if we are currently setting time, we freeze updating time here */
    if(!clock_is_in_confmode) {
      uint8_t update = 0;
      /* find new time, and if we need to, update the display */
      seconds++;
#if DEBUG_FASTTIME
      if(seconds >= 5) {
#else
      if(seconds >= 60) {
#endif
        seconds = 0;
        minutes++;
        update = 1;
      }
      if(minutes >= 60) {
        minutes = 0;
        hours++;
        update = 1;
      }
      if(hours >= 24) {
        hours = 0;
      }

      if(update) {
        /* copy the old buffer */
        msg_first[0] = hpdlbuf[0];
        msg_first[1] = hpdlbuf[1];
        msg_first[2] = hpdlbuf[2];
        msg_first[3] = hpdlbuf[3];

        /* update and copy the new buffer */
        update_ascii_buffer(hpdlbuf);
        msg_second[0] = hpdlbuf[0];
        msg_second[1] = hpdlbuf[1];
        msg_second[2] = hpdlbuf[2];
        msg_second[3] = hpdlbuf[3];
        process_post(&display_dim_process, dim_event, NULL);
        //printf("update: %s -> %s\n", msg_first, msg_second);
        //printf("temperature: %u\n", temperature);
      }
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/* UI-keep-alive-timer */
static struct timer ui_timeout_timer;
/* button update timer */
static struct etimer button_ui_update_timer;

/* this is the UI/set-time-mode process. It waits for a button press to start
    the setting of time, returning after a time-out-timer has expired. */
PROCESS_THREAD(ui_process, ev, data)
{
  PROCESS_BEGIN();

  while(1) {
    static uint8_t button_hold_count = 0;

    /* count the number of blinks when switching modes */
    static uint8_t blink_counter;

    /* how many minutes increased during this hold of the button; for knowning when to switch to fast update rate */
    static uint8_t time_increase = 0;

    /* this is the target count before increasing minutes */
    static uint8_t update_counter = BUTTON_HOLD_CHECK_RATE - BUTTON_HOLD_MINUTES_UPDATERATE_SLOW;

    PROCESS_WAIT_EVENT_UNTIL(ev == button_event && clock_bootdone);

    /* user has started to set time */
    clock_is_in_confmode = 1;
    printf("*** in confmode\n");

    /* showing temperature */
#if 0
    MSGCOPY("    ", msg_first);
    MSGCOPY("Temp", msg_second);
    process_post(&display_dim_process, dim_event, NULL);
    etimer_set(&clock_timer, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&clock_timer));

    if(temperature < 500) {
      msg_first[0] = '-';
    } else {
      msg_first[0] = '+';
    }
    msg_first[3] = 'C';
#endif /* if 0; commented out code */





    /* setting hours -------------------------------------------------------- */
    /* blink to show that we are in set-time-mode, setting hours */
    for(blink_counter = 0; blink_counter < MODE_SWITCH_BLINK_COUNT; blink_counter += 1) {
      etimer_set(&button_ui_update_timer, MODE_SWITCH_BLINK_INTERVAL);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&button_ui_update_timer));
      hpdl_write_char(1, ' ');
      hpdl_write_char(2, ' ');

      etimer_set(&button_ui_update_timer, MODE_SWITCH_BLINK_INTERVAL);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&button_ui_update_timer));
      hpdl_write_string(hpdlbuf);
    }

    /* handle button presses until button haven't been pressed for a while */
    timer_set(&ui_timeout_timer, UI_TIMEOUT);
    do {
      etimer_set(&button_ui_update_timer, BUTTON_HOLD_CHECK_INTERVAL);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&button_ui_update_timer));

      if(BTN_IS_HELD_DOWN()) {
        /* reset UI-keep-alive-time */
        timer_set(&ui_timeout_timer, UI_TIMEOUT);

        if(button_hold_count == 0) {
          /* on first press, increase time */
          button_hold_count++;
          hours++;
          time_increase++;
          if(hours == 24) {
            hours = 0;
          }
          update_ascii_buffer(hpdlbuf);
          hpdl_write_string(hpdlbuf);

        } else if(button_hold_count == update_counter) {
          /* held long enough, increase time */
          button_hold_count = 1;
          hours++;
          time_increase++;
          if(time_increase == BUTTON_HOLD_UPDATE_FAST_THRESHOLD) {
            /* we've held the button for enough time, increase spinning speed */
            update_counter = BUTTON_HOLD_CHECK_RATE - BUTTON_HOLD_MINUTES_UPDATERATE_FAST;
          }

          if(hours == 24) {
            hours = 0;
          }
          update_ascii_buffer(hpdlbuf);
          hpdl_write_string(hpdlbuf);
        } else {
          button_hold_count++;
        }

      } else {
        /* the button is released, do nothing; the timer will time-out if we
            don't press the button, returning clock to clock-mode */
        button_hold_count = 0;
        time_increase = 0;
        update_counter = BUTTON_HOLD_CHECK_RATE - BUTTON_HOLD_MINUTES_UPDATERATE_SLOW;
      }
    } while(!timer_expired(&ui_timeout_timer));

    /* setting minutes ------------------------------------------------------ */
    /* blink to show that we are in set-minutes-mode */
    for(blink_counter = 0; blink_counter < MODE_SWITCH_BLINK_COUNT; blink_counter += 1) {
      etimer_set(&button_ui_update_timer, MODE_SWITCH_BLINK_INTERVAL);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&button_ui_update_timer));
      hpdl_write_char(3, ' ');
      hpdl_write_char(4, ' ');

      etimer_set(&button_ui_update_timer, MODE_SWITCH_BLINK_INTERVAL);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&button_ui_update_timer));
      hpdl_write_string(hpdlbuf);
    }

    /* handle button presses until button haven't been pressed for a while */
    timer_set(&ui_timeout_timer, UI_TIMEOUT);
    do {
      etimer_set(&button_ui_update_timer, BUTTON_HOLD_CHECK_INTERVAL);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&button_ui_update_timer));

      if(BTN_IS_HELD_DOWN()) {
        /* reset UI-keep-alive-time */
        timer_set(&ui_timeout_timer, UI_TIMEOUT);

        if(button_hold_count == 0) {
          /* on first press, increase time */
          button_hold_count++;
          minutes++;
          time_increase++;
          if(minutes == 60) {
            minutes = 0;
          }
          update_ascii_buffer(hpdlbuf);
          hpdl_write_string(hpdlbuf);

        } else if(button_hold_count == update_counter) {
          /* held long enough, increase time */
          button_hold_count = 1;
          minutes++;
          time_increase++;
          if(time_increase == BUTTON_HOLD_UPDATE_FAST_THRESHOLD) {
            /* we've held the button for enough time, increase spinning speed */
            update_counter = BUTTON_HOLD_CHECK_RATE - BUTTON_HOLD_MINUTES_UPDATERATE_FAST;
          }

          if(minutes == 60) {
            minutes = 0;
          }
          update_ascii_buffer(hpdlbuf);
          hpdl_write_string(hpdlbuf);
        } else {
          button_hold_count++;
        }

      } else {
        /* the button is released, do nothing; the timer will time-out if we
            don't press the button, returning clock to clock-mode */
        button_hold_count = 0;
        time_increase = 0;
        update_counter = BUTTON_HOLD_CHECK_RATE - BUTTON_HOLD_MINUTES_UPDATERATE_SLOW;
      }
    } while(!timer_expired(&ui_timeout_timer));

    /* blink to show that we are going back to clock mode */
    for(blink_counter = 0; blink_counter < MODE_SWITCH_BLINK_COUNT; blink_counter += 1) {
      etimer_set(&button_ui_update_timer, MODE_SWITCH_BLINK_INTERVAL);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&button_ui_update_timer));
      hpdl_clear();

      etimer_set(&button_ui_update_timer, MODE_SWITCH_BLINK_INTERVAL);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&button_ui_update_timer));
      hpdl_write_string(hpdlbuf);
    }

    /* return to normal clock-mode */
    seconds = 0;
    clock_is_in_confmode = 0;
    printf("*** clock mode\n");

  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/* converts an uint8_t < 100 to ASCII; if <10, the char for tens is replaced
  with zero_tens_char. */
void
byte_to_ascii(char *buf, uint8_t val, uint8_t zero_tens_char)
{
  if(val < 10) {
    buf[0] = zero_tens_char;
    goto fix_ones;
  } else if(val < 20) {
    buf[0] = '1';
    val -= 10;
  } else if(val < 30) {
    buf[0] = '2';
    val -= 20;
  } else if(val < 40) {
    buf[0] = '3';
    val -= 30;
  } else if(val < 50) {
    buf[0] = '4';
    val -= 40;
  } else if(val < 60) {
    buf[0] = '5';
    val -= 50;
  } else if(val < 70) {
    buf[0] = '6';
    val -= 60;
  } else if(val < 80) {
    buf[0] = '7';
    val -= 70;
  } else if(val < 90) {
    buf[0] = '8';
    val -= 80;
  } else {
    buf[0] = '9';
    val -= 90;
  }

fix_ones:
  buf[1] = '0' + val;
}
/*---------------------------------------------------------------------------*/

static void
update_ascii_buffer(char *buf)
{
  byte_to_ascii(&(buf[0]), hours, '0');
  byte_to_ascii(&(buf[2]), minutes, '0');
}
/*---------------------------------------------------------------------------*/
/* callbacks invoked from simplepwm for dimming the clock display */
volatile char *msg_string;

/* invoked when PWM is in 'on'-mode; sets the display message according to string pointer */
void
pwm_on_cb(void)
{
#if DEBUG
  P1OUT &= ~(1<<0);
#else
  hpdl_clear();
#endif
}

/* invoked when PWM is in 'off'-mode; clears the display */
void
pwm_off_cb(void)
{
#if DEBUG
  P1OUT |= 1<<0;
#else
  hpdl_write_string(msg_string);
#endif
}
/*---------------------------------------------------------------------------*/
/* for linear: to dim faster, increase step or reduce interval */
#define PWM_MIN                     0
#define PWM_MAX                     100
#define PWM_STEP                    10

/* for sinus: lookup table */
#if FADE_STYLE_SINUS
  /* The top half look are not that different (all bright), so we use only a subset, the bottom elements. */
  #define SIN_ELEMENTS_MAXUSED      37
  #define SIN_ELEMENTS_STEP         3
  /* Pre-generated vector of delays from sin(x) where x==[0..1,5..90], 63 elements */
#if 0
  const uint8_t sin_lut[] = {0, 2, 4, 6, 13, 20, 26, 33, 40, 46, 53, 59, 66, 72,
                              79, 85, 91, 97, 104, 110, 116, 122, 127, 133, 139,
                          144, 150, 155, 161, 166, 171, 176, 181, 185, 190, 194,
                          198, 203, 207, 210, 214, 218, 221, 224, 228, 231, 233,
                          236, 238, 241, 243, 245, 247, 248, 250, 251, 252, 253,
                          254, 255, 255, 255, 255, 255};  // 64th elements
#endif /* if 0; commented out code */

  const uint8_t sin_lut[] =  {0, 3,  6, 10, 13, 17, 20, 24, 27, 30, 34, 37, 40, 43,   // 14
                                46, 49, 52, 55, 58, 61, 64, 66, 69, 71, 74, 76, 78,   // 13
                                80, 82, 84, 86, 88, 89, 91, 92, 93, 95, 96, 97, 97,   // 13
                                98, 99, 99, 99, 99};                                  // +5 = 45 elements
#endif

/* common for all */
#define PWM_STEP_INTERVAL           CLOCK_SECOND/16

/* this process changes the text on the display by dimming down the current text,
  changes to the new, and dims up to full strength. */
PROCESS_THREAD(display_dim_process, ev, data)
{
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER();
  PROCESS_BEGIN();

  static uint8_t i = 1;     /* counter */
  static struct etimer etr;
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == dim_event);
    printf("dim event\n");
    if(1) {
      printf("%s -> %s\n", msg_first, msg_second);
  #if FADE_STYLE_LINEAR
      /* dim down the first message (old time) */
      msg_string = msg_first;
      for(i = PWM_MAX; i >= PWM_MIN + PWM_STEP; i -= PWM_STEP) {
        simple_pwm_on(i);
        etimer_set(&etr, PWM_STEP_INTERVAL);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
      }

      /* dim up the new message (new time) */
      msg_string = msg_second;
      for(i = PWM_MIN; i <= PWM_MAX - PWM_STEP; i += PWM_STEP) {
        simple_pwm_on(i);
        etimer_set(&etr, PWM_STEP_INTERVAL);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
      }

      /* full dim/intensity */
      simple_pwm_on(100);
      hpdl_write_string(msg_second);
  #endif

  #if FADE_STYLE_SINUS
      /* dim down the first message (old time) */
      msg_string = msg_first;
      for(i = SIN_ELEMENTS_MAXUSED; i >= SIN_ELEMENTS_STEP; i -= SIN_ELEMENTS_STEP) {
        simple_pwm_on(sin_lut[i]);
        etimer_set(&etr, PWM_STEP_INTERVAL);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
      }

      /* dim up the new message (new time) */
      msg_string = msg_second;
      for(i = 0; i <= SIN_ELEMENTS_MAXUSED - SIN_ELEMENTS_STEP; i += SIN_ELEMENTS_STEP) {
        simple_pwm_on(sin_lut[i]);
        etimer_set(&etr, PWM_STEP_INTERVAL);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
      }
      /* full dim/intensity */
      simple_pwm_on(100);
      hpdl_write_string(msg_second);
  #endif

  #if FADE_STYLE_JUST_CUT
    /* no fancy dimming, just change the text to the new time */
    hpdl_write_string(msg_second);
  #endif

  #if FADE_STYLE_SIMPLE
      {
        /* dim down the first message (old time) */
        msg_string = msg_first;
        hpdl_write_string(msg_string);
        simple_pwm_on(90);
        etimer_set(&etr, PWM_STEP_INTERVAL);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
        simple_pwm_on(60);
        etimer_set(&etr, PWM_STEP_INTERVAL);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
        simple_pwm_on(30);
        etimer_set(&etr, PWM_STEP_INTERVAL);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
        simple_pwm_on(10);
        etimer_set(&etr, PWM_STEP_INTERVAL);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
        simple_pwm_on(5);
        etimer_set(&etr, PWM_STEP_INTERVAL);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));

        msg_string = msg_second;
        simple_pwm_on(10);
        etimer_set(&etr, PWM_STEP_INTERVAL);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
        simple_pwm_on(30);
        etimer_set(&etr, PWM_STEP_INTERVAL);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
        simple_pwm_on(60);
        etimer_set(&etr, PWM_STEP_INTERVAL);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
        simple_pwm_on(90);
        etimer_set(&etr, PWM_STEP_INTERVAL);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
        simple_pwm_on(100);
        hpdl_write_string(msg_string);
      }
  #endif
      process_post(PROCESS_BROADCAST, dim_done_event, NULL);
    }
  }
  PROCESS_END();
}
/*--------------------------------------------------------------------------.*/



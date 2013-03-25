/*
 * Copyright (c) 2012
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
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */
/**
 * \file
 *    hpdl1414-clock.c
 * \author
 *    Marcus Lunden <marcus.lunden@gmail.com>
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
      setting time goes either too fast or too slow.
        --Should start slow and then go faster
        ++ done
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
 */

#include "contiki.h"
#include "dev/button.h"
#include "simple-pwm.h"

/* setting DEBUG makes the clock use leds and serial output instead of the HPDL-1414 */
#define DEBUG 0

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
static const char splash_message[] =    "HPDL-1414 clock - Contiki 2.6";
#define SPLASH_LENGTH                   (sizeof(splash_message) + 1)
#define SPLASH_UPDATE_INTERVAL          (CLOCK_SECOND / 8)
#define SPLASH_POST_SPLASH_WAIT         (CLOCK_SECOND)

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
/*---------------------------------------------------------------------------*/
/* sanity check */
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
/* string buffers for the two messages */
#if 1
static char msg_first[4];
static char msg_second[4];

struct dim_msg_data_s {
  const char* first;
  const char* second;
} dim_msg;
#if 0
 = {
  .first = msg_first;
  .second = msg_second;
};
#endif /* if 0; commented out code */
#endif /* if 0; commented out code */

process_event_t dim_event;

/* -------------------------------------------------------------------------- */
PROCESS(clockdisplay_process, "HPDL-1414 Process");
PROCESS(ui_process, "Serial echo Process");
PROCESS(display_dim_process, "Display dimmer process");
AUTOSTART_PROCESSES(&clockdisplay_process, &ui_process);
/*---------------------------------------------------------------------------*/
static void update_ascii_buffer(void);
static void byte_to_ascii(char *buf, uint8_t val, uint8_t zero_tens_char);
/*---------------------------------------------------------------------------*/
/* flag that halts regular time-keeping while clock is in set-mode */
static volatile uint8_t clock_is_in_confmode = 0;

/* keeps track of time */
static volatile uint8_t seconds = 0;
static volatile uint8_t minutes = 0;
static volatile uint8_t hours = 0;

/* ASCII buffer of time */
static char hpdlbuf[5];
/*---------------------------------------------------------------------------*/
static struct etimer clock_timer;

/* this is the normal clock-mode process */
PROCESS_THREAD(clockdisplay_process, ev, data)
{
  PROCESS_BEGIN();
  static uint8_t i;

  /* allocate the dim event */
  dim_event = process_alloc_event();
  dim_msg.first = msg_first;
  dim_msg.second = msg_second;

  /* init display and clock ASCII buffer, reading last time from flash */
  hpdl_init();

  /* show splash message */
  for(i = 0; i < SPLASH_LENGTH; i += 1) {
    hpdl_write_string(&(splash_message[i]));
    etimer_set(&clock_timer, SPLASH_UPDATE_INTERVAL);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&clock_timer));
  }
  etimer_set(&clock_timer, SPLASH_POST_SPLASH_WAIT);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&clock_timer));

  hpdl_write_string("Set");
  etimer_set(&clock_timer, CLOCK_SECOND / 2);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&clock_timer));
  hpdl_write_string("time");
  etimer_set(&clock_timer, CLOCK_SECOND / 2);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&clock_timer));

  /* prepare and set initial time */
  hpdlbuf[0] = 0;
  hpdlbuf[1] = 0;
  hpdlbuf[2] = 0;
  hpdlbuf[3] = 0;
  hpdlbuf[4] = 0;
  update_ascii_buffer();
  hpdl_write_string(hpdlbuf);
  
  /* the big 'ole clock loop; counts seconds and sets time accordingly. */
  while(1) {
    etimer_set(&clock_timer, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&clock_timer));

    /* if we are currently setting time, we freeze updating time here */
    if(!clock_is_in_confmode) {
      uint8_t update = 0;
      /* find new time, and if we need to, update the display */
      seconds++;
      if(seconds >= 60) {
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
        update_ascii_buffer();
        
        
        // XXX here, should update the ascii buffers for before and after dim!
        
        dim_msg.first = "";
        dim_msg.second = "";
        
        process_post(&display_dim_process, dim_event, (void*) &dim_msg);
        //hpdl_write_string(hpdlbuf);
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
    /* count the number of blinks when switching modes */
    static uint8_t blink_counter;
    
    static uint8_t button_hold_count = 0;

    /* how many minutes increased during this hold of the button; for knowning when to switch to fast update rate */
    static uint8_t time_increase = 0;

    /* this is the target count before increasing minutes */
    static uint8_t update_counter = BUTTON_HOLD_CHECK_RATE - BUTTON_HOLD_MINUTES_UPDATERATE_SLOW;

    PROCESS_WAIT_EVENT_UNTIL(ev == button_event);
    
    /* user has started to set time */
    clock_is_in_confmode = 1;
    printf("*** in confmode\n");
    leds_on(LEDS_RED);

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
          update_ascii_buffer();
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
          update_ascii_buffer();
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
          update_ascii_buffer();
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
          update_ascii_buffer();
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
    clock_is_in_confmode = 0;
    printf("*** clock mode\n");
    leds_off(LEDS_RED);

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
update_ascii_buffer(void)
{
  byte_to_ascii(&(hpdlbuf[0]), hours, '0');
  byte_to_ascii(&(hpdlbuf[2]), minutes, '0');
}
/*---------------------------------------------------------------------------*/
/* callbacks invoked from simplepwm for dimming the clock display */
char *msg_string;

void
pwm_on_cb(void)
{
  hpdl_write_string(msg_string);
}

void
pwm_off_cb(void)
{
  hpdl_clear();
}
/*---------------------------------------------------------------------------*/
/* to dim faster, increase step or reduce interval */
#define PWM_MIN                     0
#define PWM_MAX                     100
#define PWM_STEP                    1
#define PWM_STEP_INTERVAL           CLOCK_SECOND/64

/* set style, only one can be non-zero at once */
#define PWM_STYLE_LINEAR            1
#define PWM_STYLE_SINUS             0
#define PWM_STYLE_INVERT_SINUS      0
#define PWM_STYLE_JUST_CUT          0

/* lookup table */
#if PWM_STYLE_SINUS || PWM_STYLE_INVERTED_SINUS
  /* The top half look are not that different (all bright), so we use only a subset, the bottom elements. */
  #define SIN_ELEMENTS_MAXUSED      37
  /* Pre-generated vector of delays from sin(x) where x==[0..1,5..90], 63 elements */
#if 0
  const uint8_t sin_lut[] = {0, 2, 4, 6, 13, 20, 26, 33, 40, 46, 53, 59, 66, 72,
                              79, 85, 91, 97, 104, 110, 116, 122, 127, 133, 139,
                          144, 150, 155, 161, 166, 171, 176, 181, 185, 190, 194,
                          198, 203, 207, 210, 214, 218, 221, 224, 228, 231, 233,
                          236, 238, 241, 243, 245, 247, 248, 250, 251, 252, 253,
                          254, 255, 255, 255, 255, 255};  // 64th elements
#endif /* if 0; commented out code */

  const uint8_t sin_lut[] = {0, 3, 6, 10, 13, 17, 20, 24, 27, 30, 34, 37, 40, 43,
                              46, 49, 52, 55, 58, 61, 64, 66, 69, 71, 74, 76, 78,
                              80, 82, 84, 86, 88, 89, 91, 92, 93, 95, 96, 97, 97,
                              98, 99, 99, 99, 99}         // 45 elements
#endif


/* this process changes the text on the display by dimming down the current text,
  changes to the new, and dims up to full strength. */
PROCESS_THREAD(display_dim_process, ev, data)
{
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER();
  PROCESS_BEGIN();

  static uint8_t i = 1;     /* counter */
  static struct etimer etr;
  PROCESS_WAIT_EVENT_UNTIL(ev == dim_event);
  if(data != NULL) {

#if PWM_STYLE_LINEAR
    /* dim down */
    msg_string = ((struct dim_msg_data_s*) data)->first;
    for(i = PWM_MAX; i >= PWM_STEP; i -= PWM_STEP) {
      simple_pwm_on(i);
      etimer_set(&etr, PWM_STEP_INTERVAL);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
    }

    /* dim up */
    msg_string = ((struct dim_msg_data_s*) data)->second;
    for(i = PWM_MIN; i <= PWM_MAX; i += PWM_STEP) {
      simple_pwm_on(i);
      etimer_set(&etr, PWM_STEP_INTERVAL);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
    }
    /* full dim/intensity */
    simple_pwm_on(100);
#endif

#if PWM_STYLE_SINUS
    /* dim down */
    msg_string = ((struct dim_msg_data_s*) data)->first;
    for(i = SIN_ELEMENTS_MAXUSED; i > 0; i -= 1) {
      simple_pwm_on(sin_lut[i]);
      etimer_set(&etr, PWM_STEP_INTERVAL);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
    }

    /* dim up */
    msg_string = ((struct dim_msg_data_s*) data)->second;
    for(i = 0; i <= SIN_ELEMENTS_MAXUSED; i += 1) {
      simple_pwm_on(sin_lut[i]);
      etimer_set(&etr, PWM_STEP_INTERVAL);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
    }
    /* full dim/intensity */
    simple_pwm_on(100);
#endif

#if PWM_STYLE_INVERT_SINUS
    /* full dim/intensity */
    simple_pwm_on(100);
#endif

#if PWM_STYLE_JUST_CUT
  /* no fance dimming, just change the text */
  hpdl_write_string(((struct dim_msg_data_s*) data)->second);
#endif

  }
  PROCESS_END();
}
/*--------------------------------------------------------------------------.*/ 



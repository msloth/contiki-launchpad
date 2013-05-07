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
 *         Break timer application.
 *          Simple applicaion that will signal with an LED when it's time to
 *          take a break from all the hard debugging and coding.
 * \author
 *         Marcus Linderoth <linderoth.marcus@gmail.com>
 */

#include <stdio.h>
#include "contiki.h"
#include "dev/leds.h"
#include "dev/button.h"
#include "simple-pwm.h"

/*---------------------------------------------------------------------------*/
/* in this application, a signal is a fade in/fade out of eg an LED or vibration
  motor. They are defined according to the struct below. */

/* define how a fade-signal looks like. */
struct signal_s {
  uint8_t pin;            /* what pin to PWM on */
  uint8_t pwm_min;        /* min fade value, in %, >0 */
  uint8_t pwm_max;        /* max fade value, in %, <=100 */
  uint8_t pwm_step;       /* fade step */
  uint8_t signal_cnt;     /* this many times fade in-fade out */
  clock_time_t interval;  /* wait this long between setting a new PWM setting */
};

/* some defines to be used for defining signals */
#define SIGNAL_LED_PIN       (7)   // P1.7
#define PWM_MIN           0
#define PWM_MAX           100
#define PWM_STEP          2
#define SIGNAL_CNT        4
#define INTERVAL          CLOCK_SECOND/128

/* declare a number of signals that can be sent */
static const struct signal_s slow2_signal = {SIGNAL_LED_PIN, PWM_MIN, PWM_MAX, 1, 2, CLOCK_SECOND/64};
static const struct signal_s fast4_signal = {SIGNAL_LED_PIN, PWM_MIN, PWM_MAX, 3, 4, CLOCK_SECOND/128};
/*---------------------------------------------------------------------------*/
PROCESS(signal_process, "Singal process");
PROCESS(timer_process, "Break Timer process");
AUTOSTART_PROCESSES(&timer_process);
/*---------------------------------------------------------------------------*/
/* PWM process; finds and sets the PWM. */
PROCESS_THREAD(signal_process, ev, data)
{
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER(simple_pwm_off(););
  static struct etimer etr;
  static uint8_t up = 1;    /* counting up or down? */
  static uint8_t i = 1;     /* counter */
  static uint8_t signal_count;
  PROCESS_BEGIN();
  struct signal_s *s;
  if(data != NULL) {
    s = (struct signal_s *)data;
  } else {
    s = &fast4_signal;
  }

  simple_pwm_confpin(s->pin);

  /* each 'signal' is one fade up&down of the LED */
  for(signal_count = 0; signal_count < s->signal_cnt; signal_count += 1) {
    up = 1;
    i = s->pwm_min;

    while(up != 100) {    /* magic value to break out */
      /* find next PWM setting */
      if(up) {
        if(i < s->pwm_max - s->pwm_step) {
          i += s->pwm_step;
        } else {
          /* start fading down */
          i = s->pwm_max;
          up = 0;
        }
      } else {
        if(i > s->pwm_min + s->pwm_step) {
          i -= s->pwm_step;
        } else {
          /* done fading down, now break out of this fade-loop */
          up = 100;
        }
      }
      simple_pwm_on(i);

      /* wait a little while */
      etimer_set(&etr, s->interval);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
    }
  }
  simple_pwm_off();
  PROCESS_END();
}
/*--------------------------------------------------------------------------.*/
#define BREAK_INTERVAL_MINUTES    45
#define BREAK_LENGTH_MINUTES      5
/* ie repeats after BREAK_LENGTH_MINUTES + BREAK_INTERVAL_MINUTES minutes */
#define SECPERMIN                 60    /* seconds per minute (use in debugging) */
PROCESS_THREAD(timer_process, ev, data)
{
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER(leds_off(LEDS_ALL););
  static struct etimer breaktimer;
  PROCESS_BEGIN();

  leds_off(LEDS_ALL);
  while(1) {
    static uint8_t i, j;
    /* wait one break interval */
    leds_on(LEDS_RED);

    for(i = 0; i < BREAK_INTERVAL_MINUTES; i += 1) {
      /* wait one minute */
      for(j = 0; j < SECPERMIN; j += 1) {
        etimer_set(&breaktimer, CLOCK_SECOND);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&breaktimer));
      }
    }
    leds_off(LEDS_RED);
    leds_on(LEDS_GREEN);

    /* signal it's time to take a break */
    process_start(&signal_process, (void *)&slow2_signal);

    /* wait one break interval */
    for(i = 0; i < BREAK_LENGTH_MINUTES; i += 1) {
      /* wait one minute */
      for(j = 0; j < SECPERMIN; j += 1) {
        etimer_set(&breaktimer, CLOCK_SECOND);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&breaktimer));
      }
    }

    /* signal it's time to get back to work */
    process_start(&signal_process, (void *)&fast4_signal);
    leds_off(LEDS_GREEN);
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/


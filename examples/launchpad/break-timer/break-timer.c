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
 *         Break timer application. 
 *          Simple applicaion that will signal with an LED when it's time to 
 *          take a break from all the hard debugging and coding.
 * \author
 *         Marcus Lunden <marcus.lunden@gmail.com>
 */

#include <stdio.h>
#include "contiki.h"
#include "dev/leds.h"
#include "dev/button.h"
#include "simple-pwm.h"

/* pins I want to use for PWM (only one is used) at a time with this */
#define SIGNAL_LED_PIN       (7)   // P1.7

/* the PWM duty cycle will step back and forth between these limits, with this step */
#define PWM_MIN           2
#define PWM_MAX           99
#define PWM_STEP          2

#define SIGNAL_CNT        4

/* wait this long between setting a new PWM setting */
#define INTERVAL          CLOCK_SECOND/128
/*---------------------------------------------------------------------------*/
PROCESS(signal_process, "Singal process");
PROCESS(timer_process, "Break Timer process");
AUTOSTART_PROCESSES(&timer_process);
/*---------------------------------------------------------------------------*/ 
/* PWM process; finds and sets the PWM. */
static struct etimer etr;
static uint8_t i = 1;     /* counter */
static uint8_t up = 1;    /* counting up or down? */
PROCESS_THREAD(signal_process, ev, data)
{
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER();
  PROCESS_BEGIN();
  static uint8_t signal_count;
/*  printf("[%u] break begins\n", (uint16_t)clock_seconds());*/
  simple_pwm_confpin(SIGNAL_LED_PIN);

  /* each 'signal' is one fade up&down of the LED */
  for(signal_count = 0; signal_count < SIGNAL_CNT; signal_count += 1) {
    up = 1;
    i = PWM_MIN;

    while(up != 100) {    /* magic value to break out */
      /* find next PWM setting */
      if(up) {
        if(i < PWM_MAX - PWM_STEP) {
          i += PWM_STEP;
        } else {
          /* start fading down */
          i = PWM_MAX;
          up = 0;
        }
      } else {
        if(i > PWM_MIN + PWM_STEP) {
          i -= PWM_STEP;
        } else {
          /* done fading down, now break out of this fade-loop */
          up = 100;
        }
      }
      simple_pwm_on(i);
/*      printf("pwm %u\n", i);    */
      /* wait a little while */
      etimer_set(&etr, INTERVAL);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
    }
  }
/*  printf("[%u] break over\n", (uint16_t)clock_seconds());*/
  simple_pwm_off();
  PROCESS_END();
}
/*--------------------------------------------------------------------------.*/
static struct etimer breaktimer;
#define BREAK_INTERVAL_MINUTES    10
#define BREAK_LENGTH_MINUTES      5
#define SECPERMIN                 1    /* seconds per minute (use in debugging) */
PROCESS_THREAD(timer_process, ev, data)
{
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER();
  PROCESS_BEGIN();
/*  leds_off(LEDS_ALL);*/
  while(1) {
    static uint8_t i, j;
    /* wait one break interval */
    leds_on(LEDS_RED);

/*    printf("[%u] work!\n", (uint16_t)clock_seconds());*/
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
/*    printf("[%u] break!\n", (uint16_t)clock_seconds());*/
    process_start(&signal_process, NULL);

    /* wait one break interval */
    for(i = 0; i < BREAK_LENGTH_MINUTES; i += 1) {
      /* wait one minute */
      for(j = 0; j < SECPERMIN; j += 1) {
        etimer_set(&breaktimer, CLOCK_SECOND);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&breaktimer));
      }
    }
    leds_off(LEDS_GREEN);
/*    printf("[%u] break over!\n", (uint16_t)clock_seconds());*/
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/


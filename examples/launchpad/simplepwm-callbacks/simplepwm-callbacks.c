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
 *         Simple hack to make simplepwm invoke a callback instead of directly
 *         manipulate the LED. This way we can do more, such as instead change
 *         a display.
 * \author
 *         Marcus Lunden <marcus.lunden@gmail.com>
 */

/* TODO:
  verify that the on_cb is actually the cb being invoked when /on/ and not vice versa
      ie if DC=80% then on_cb should be invoked just before the 80% period starts  
  
   */


#include <stdio.h>
#include "contiki.h"
#include "dev/leds.h"
#include "dev/button.h"
#include "simple-pwm.h"

/* the PWM duty cycle will step back and forth between these limits, with this step */
#define PWM_MIN           0
#define PWM_MAX           100
#define PWM_STEP          1

/* wait this long between setting a new PWM setting */
#define INTERVAL          CLOCK_SECOND/64
/*---------------------------------------------------------------------------*/
PROCESS(pwmled_process, "PWM LED process");
AUTOSTART_PROCESSES(&pwmled_process);
/*---------------------------------------------------------------------------*/ 
#define TEST_PORT(type)          P1##type
#define pin                      6
void
pwm_on_cb(void)
{
  TEST_PORT(OUT) |= (1 << pin);
}
/*---------------------------------------------------------------------------*/
void
pwm_off_cb(void)
{
  TEST_PORT(OUT) &= ~(1 << pin);
}
/*--------------------------------------------------------------------------*/
/* PWM process; finds and sets the PWM. */
static struct etimer etr;
static uint8_t i = 1;     /* counter */
static uint8_t up = 1;    /* counting up or down? */

PROCESS_THREAD(pwmled_process, ev, data)
{
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER();
  PROCESS_BEGIN();

  leds_off(LEDS_ALL);

  /* no need to do simplepwm_confpin() as it no longer changes those. */
  TEST_PORT(IE) &= ~(1 << pin);
  TEST_PORT(DIR) |= (1 << pin);
  TEST_PORT(OUT) |= (1 << pin);
  TEST_PORT(SEL) &= ~(1 << pin);
  TEST_PORT(SEL2) &= ~(1 << pin);

#if 0
  while(1) {
    simple_pwm_on(20);
    etimer_set(&etr, CLOCK_SECOND/2);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
    simple_pwm_on(40);
    etimer_set(&etr, CLOCK_SECOND/2);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
    simple_pwm_on(60);
    etimer_set(&etr, CLOCK_SECOND/2);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
    simple_pwm_on(80);
    etimer_set(&etr, CLOCK_SECOND/2);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
    simple_pwm_on(100);
    etimer_set(&etr, CLOCK_SECOND/2);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
  }
#endif /* if 0; commented out code */

  while(1) {
    simple_pwm_on(i);

    /* find next PWM setting */
    if(up) {
      if(i < PWM_MAX - PWM_STEP) {
        i += PWM_STEP;
      } else {
        i = PWM_MAX;
        up = 0;
      }
    } else {
      if(i > PWM_MIN + PWM_STEP) {
        i -= PWM_STEP;
      } else {
        i = PWM_MIN ;
        up = 1;
      }
    }
    
    /* wait a little while */
    etimer_set(&etr, INTERVAL);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
  }
  PROCESS_END();
}
/*--------------------------------------------------------------------------.*/ 

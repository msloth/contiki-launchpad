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
 *         PWM example application,
 *          Demonstrates using the PWM API to dim an LED.
 *          expected result from running it:
 *          red and green LEDs fading in and out, out of phase from eachother.
 * \author
 *         Marcus Lunden <marcus.lunden@gmail.com>
 */

#include <stdio.h>
#include "contiki.h"
#include "dev/leds.h"
#include "dev/pwm.h"
#include "dev/button.h"

/* pins to PWM */
#define LEDRED_PIN       (0)   // P1.0
#define LEDGRN_PIN       (6)   // P1.6
/*---------------------------------------------------------------------------*/
PROCESS(button_process, "Button reader");
PROCESS(pwmled_process, "PWM LED process");
AUTOSTART_PROCESSES(&pwmled_process, &button_process);
/*---------------------------------------------------------------------------*/ 
/* set the PWM duty cycle on the pins (ie the LEDs) out of phase from eachother */
static struct etimer etr;

PROCESS_THREAD(pwmled_process, ev, data)
{
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER();
  static uint8_t i = 1;     /* counter */
  static uint8_t up = 1;    /* counting up or down? */
  PROCESS_BEGIN();

  while(1) {
    /* set PWM; there are two possible PWM-devices/-channels (the 0 and 1) and
      this is how they are set */
    pwm_on(0, LEDGRN_PIN, 100-i);
    pwm_on(1, LEDRED_PIN, i);

    /* find next PWM setting */
    if(up) {
      i++;
      if(i == 99) {
        up = 0;
      }
    } else {
      i--;
      if(i == 3) {
        up = 1;
      }
    }
    etimer_set(&etr, CLOCK_SECOND/64);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
  }

  PROCESS_END();
}
/*--------------------------------------------------------------------------.*/ 
/* Kill the PWM'ing with the push of the button. */
PROCESS_THREAD(button_process, ev, data)
{
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER();
  PROCESS_BEGIN();
  /* wait for a button press */
  PROCESS_WAIT_EVENT_UNTIL(ev == button_event);
  
  /* kill the PWM and the PWM process */
  pwm_off(0);
  pwm_off(1);
  process_exit(&pwmled_process);
  PROCESS_END();
}
/* -------------------------------------------------------------------------- */

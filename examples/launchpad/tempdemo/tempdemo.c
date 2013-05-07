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
 *         Re-implementation of the TI temp demo.
 *          Starts in "pre-application mode", then at button press goes into
 *          temperature sense and LED dim mode.
 * \author
 *         Marcus Linderoth <linderoth.marcus@gmail.com>
 */

#include <stdio.h>
#include "contiki.h"
#include "dev/leds.h"
#include "dev/adc.h"
#include "dev/button.h"
#include "dev/pwm.h"

/*---------------------------------------------------------------------------*/
#define MAVG_LEN  8
#define LED_PIN   0   // P1.0
#define TEMP_MIN  100 // minimum possible temp sensor reading
#define TEMP_MAX  500 // maximum possible temp sensor reading
/*---------------------------------------------------------------------------*/
static uint16_t temp_sense[MAVG_LEN];   /* ADC buffer for moving average */
static uint16_t temp_avg;               /* calculated average */
static uint8_t mavg_ix = 0;             /* index in buffer */
static uint8_t led_pwmdc;               /* LED PWM setting */
/*---------------------------------------------------------------------------*/
PROCESS(tempdemo_process, "My Process");
AUTOSTART_PROCESSES(&tempdemo_process);
/*---------------------------------------------------------------------------*/
static struct etimer et;
PROCESS_THREAD(tempdemo_process, ev, data)
{
  PROCESS_EXITHANDLER(pwm_off(0); leds_off(LEDS_ALL););
  PROCESS_BEGIN();
  /* pre-application mode: toggle LEDs */
  while(ev != button_event) {
    leds_toggle(LEDS_ALL);
    etimer_set(&et, CLOCK_SECOND/5);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  }

  /* application mode: get ADC of internal temp, showing avg (trend?) with LED */
  memset(temp_sense, 0, MAVG_LEN * sizeof(uint16_t));
  while(1) {
    uint8_t i;
    temp_sense[mavg_ix] = adc_get(TEMP);
    mavg_ix++;    /* mavg_ix = mavg_ix < (MAVG_LEN - 1) ? mavg_ix++ : 0; */
    if(mavg_ix == MAVG_LEN) {
      mavg_ix = 0;
    }

    temp_avg = 0;
    for(i = 0; i < MAVG_LEN; i += 1) {
      temp_avg += temp_sense[i];
    }
    temp_avg = temp_avg / MAVG_LEN;
    /* 100% duty cycle if full temp, 0% if lowest temp */
    led_pwmdc = (100 * (temp_avg - TEMP_MIN) / (TEMP_MAX - TEMP_MIN));
    pwm_on(0, LED_PIN, led_pwmdc);

    etimer_set(&et, CLOCK_SECOND/8);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

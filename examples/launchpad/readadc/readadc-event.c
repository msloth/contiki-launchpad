/**
 * \addtogroup launchpad-examples
 *
 * @{
 */

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
 *    readadc-event.c
 * \author
 *         Marcus Linderoth <linderoth.marcus@gmail.com>
 * \brief
 *    ADC reading example. One process periodically blinks an LED.
 *    Another reads the ADC with the event-based approach, setting LED on/off
 *    according to the result, which can be found in the data pointer.
 *
 */

#include <stdio.h>
#include "contiki.h"
#include "dev/leds.h"
#include "dev/adc.h"

/*--------------------------------------------------------------------------*/
PROCESS(adc_reading_process, "ADC reading process");
PROCESS(heartbeat_process, "My Process");
AUTOSTART_PROCESSES(&heartbeat_process, &adc_reading_process);
/*--------------------------------------------------------------------------*/
static struct etimer etb;
PROCESS_THREAD(heartbeat_process, ev, data) {
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER();
  PROCESS_BEGIN();

  leds_off(LEDS_ALL);

  while(1) {
    // XXX the leds implementation seems a bit shaky, why is this inverted??
    leds_off(LEDS_GREEN);
    etimer_set(&etb, CLOCK_SECOND/32);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etb));

    leds_on(LEDS_GREEN);
    etimer_set(&etb, CLOCK_SECOND - CLOCK_SECOND/32);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etb));
  }
  PROCESS_END();
}
/* -------------------------------------------------------------------------- */
PROCESS_THREAD(adc_reading_process, ev, data)
{
  PROCESS_BEGIN();
  while(1) {
    /*
     * read analog in A7, store the result in variable adc when done (no
     * notification will be given when so happens)
     */
    adc_get_event(A7, PROCESS_CURRENT());
    PROCESS_WAIT_EVENT_UNTIL(ev == adc_event);
    if(data != NULL) {
      printf("ADC: %u\n", *((uint16_t*)data));
      if(*((uint16_t*)data) > 500) {
        /* middle value, eg Vcc/2 */
        leds_on(LEDS_RED);
      } else {
        leds_off(LEDS_RED);
      }
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/** @} */

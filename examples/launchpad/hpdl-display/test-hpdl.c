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
 *         Testing button example
 *          When pressing the button, the red LED will toggle until enough presses,
 *          then the green blinks stops and the button becomes unresponsive.
 * \author
 *         Marcus Linderoth <linderoth.marcus@gmail.com>
 */

#include <stdio.h>
#include "contiki.h"
#include "dev/hpdl1414.h"
#include "dev/serial-line.h"
#include "dev/button.h"

/* -------------------------------------------------------------------------- */
PROCESS(testhpdl_process, "HPDL-1414 Process");
PROCESS(serial_echo_process, "Serial echo Process");
AUTOSTART_PROCESSES(&serial_echo_process, &testhpdl_process);
/*---------------------------------------------------------------------------*/
const char anim[] = {'|', '/', '-', '\\', '|', '/', '-', '\\'};
static char hpdlbuf[4];

static struct etimer et;
PROCESS_THREAD(testhpdl_process, ev, data)
{
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER();
  PROCESS_BEGIN();

  hpdl_init();

  while(1) {
    static uint8_t c = 0;
    static clock_time_t last_time = 0;

    /* "clock", shows seconds from start */
    while(clock_seconds() < last_time + 10) {
      snprintf(hpdlbuf, 4, "%u", (uint16_t)clock_seconds());
      hpdl_write_string(hpdlbuf);
      etimer_set(&et, CLOCK_SECOND/4);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    }
    hpdl_clear();

    /* Shows a simple animation, spinning bar */
    while(clock_seconds() < last_time + 20) {
      /* positions are 1..4 counting from left to right */
      hpdl_write_char(2, anim[c]);
      hpdl_write_char(3, anim[7-c]);
      c++;
      if(c > 7) {
        c = 0;
      }
      etimer_set(&et, CLOCK_SECOND/16);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    }
    hpdl_clear();

    /* prints some strings */
    while(clock_seconds() < last_time + 30) {
      hpdl_write_string("Hi!");
      etimer_set(&et, CLOCK_SECOND/2);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

      hpdl_write_string("/(*+");
      etimer_set(&et, CLOCK_SECOND/2);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

      hpdl_write_string(".__.");
      etimer_set(&et, CLOCK_SECOND/2);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

      hpdl_write_string("|<>|");
      etimer_set(&et, CLOCK_SECOND/2);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

      hpdl_write_string(" :) ");
      etimer_set(&et, CLOCK_SECOND/2);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    }
    hpdl_clear();

    last_time = clock_seconds();
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/* This process awaits a button press to stop the HPDL-1414 demo mode, then echoes
    back whatever it gets on the serial port. */
PROCESS_THREAD(serial_echo_process, ev, data)
{
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER(hpdl_clear(););
  PROCESS_BEGIN();

  PROCESS_WAIT_EVENT_UNTIL(ev == button_event);
  process_exit(&testhpdl_process);
/*  hpdl_init();*/

  hpdl_clear();
  while(1) {
    char* buf;
    PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message);
    buf = data;
    hpdl_write_string(buf);
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

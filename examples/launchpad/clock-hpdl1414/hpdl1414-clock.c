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

#include <stdio.h>
#include "contiki.h"
#include "dev/hpdl1414.h"
#include "dev/button.h"
/* -------------------------------------------------------------------------- */
PROCESS(clockdisplay_process, "HPDL-1414 Process");
PROCESS(buttonmonitor_process, "Serial echo Process");
AUTOSTART_PROCESSES(&clockdisplay_process, &buttonmonitor_process);
/*---------------------------------------------------------------------------*/
/* converts an uint8_t < 100 to ASCII; if <10, the char for tens is replaced
  with zero_tens_char. */
void
byte_to_ascii(char *buf, uint8_t val, uint8_t zero_tens_char)
{
  uint8_t tens = 0;
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
static char hpdlbuf[5];
static struct etimer clock_timer;

static volatile uint8_t seconds = 0;
static volatile uint8_t minutes = 0;
static volatile uint8_t hours = 0;

PROCESS_THREAD(clockdisplay_process, ev, data)
{
  PROCESS_BEGIN();

  /* init display and clock ASCII buffer */
  hpdl_init();
  hpdlbuf[4] = 0;
  byte_to_ascii(&(hpdlbuf[0]), hours, '0');
  byte_to_ascii(&(hpdlbuf[2]), minutes, '0');
  
  /* the big 'ole clock loop */
  while(1) {
    etimer_set(&clock_timer, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&clock_timer));

    /* find new time, and if we need to, update the display */
    seconds++;
    if(seconds >= 60) {
      seconds = 0;
      minutes++;
      if(minutes >= 60) {
        minutes = 0;
        hours++;
        if(hours >= 24) {
          hours = 0;
        }
        byte_to_ascii(&(hpdlbuf[0]), hours, '0');
      }
      byte_to_ascii(&(hpdlbuf[2]), minutes, '0');
      hpdl_clear();
      hpdl_write_string(hpdlbuf);
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/* handles the user interface (setting time/showing time) */
PROCESS_THREAD(buttonmonitor_process, ev, data)
{
  PROCESS_BEGIN();
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == button_event);
    process_exit(&clockdisplay_process);
  }  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/





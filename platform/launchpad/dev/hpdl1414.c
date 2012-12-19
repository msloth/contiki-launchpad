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
 *         Driver for HPDL-1414 "Four character Smart Alphanumeric Displays" 
 *         from Avago. Parallell interface, decoding ASCII and shows on the
 *         beautiful 2.85 mm display.
 * \author
 *        Marcus Lunden <marcus.lunden@gmail.com>
 */

#include <stdio.h>
#include "contiki.h"
#include "hpdl1414.h"
/*--------------------------------------------------------------------------*/
/* convert an ASCII character to a character valid for this display, see
 the datasheet for which are available. */
static uint8_t
ascii_to_hpdl(uint8_t k)
{
  if(k >= 'a' && k <= 'z') {
    return 'A' + k - 'a';
  }
  if(k > 0x5f || k < 0x20) {
    /* for invalid characters, just print a space (ie nothing) */
    return ' ';
  }
  return k;
}
/*--------------------------------------------------------------------------*/
/* write a character to the display at a given position 1..4 */
void
hpdl_write_char(uint8_t pos, uint8_t ch)
{
  uint8_t tkn;

  /* sanity check */
  if(pos > 4 || pos == 0) {
    return;
  }
  
  tkn = ascii_to_hpdl(ch);

  /* set address; acc to datasheet should be before clearing WR */
  switch(pos) {
  case 1:
    HPDL_ADDRESS_PORT(OUT) &= ~(HPDL_A0_PIN | HPDL_A1_PIN);
    break;
  case 2:
    HPDL_ADDRESS_PORT(OUT) &= ~(HPDL_A1_PIN);
    HPDL_ADDRESS_PORT(OUT) |= (HPDL_A0_PIN);
    break;
  case 3:
    HPDL_ADDRESS_PORT(OUT) &= ~(HPDL_A0_PIN);
    HPDL_ADDRESS_PORT(OUT) |= (HPDL_A1_PIN);
    break;
  case 4:
    HPDL_ADDRESS_PORT(OUT) |= (HPDL_A0_PIN | HPDL_A1_PIN);
    break;
  }
  
  /* set writing conditions */
  HPDL_WR_PORT(OUT) &= ~(HPDL_WR_PIN);

  /* set digits */
  HPDL_DIGIT_PORT(OUT) = tkn;
  if(tkn & (1<<7)) {
    HPDL_DIGIT_EP_PORT(OUT) |= HPDL_DIGIT_EP_PIN;
  } else {
    HPDL_DIGIT_EP_PORT(OUT) &= ~HPDL_DIGIT_EP_PIN;
  }

  /* clear writing conditions */
  HPDL_WR_PORT(OUT) |= (HPDL_WR_PIN);
}
/*--------------------------------------------------------------------------*/
/* write a string to the display; handles non-characters in string too */
void
hpdl_write_string(char *s)
{
  uint8_t z;
  if(s == NULL) {
    return;
  }

  /* just print up until an null termination, then print spaces after that */
  for(z = 0; z < 4; z += 1) {
    if(s[z] != 0) {
      hpdl_write_char(z+1, s[z]);
    } else {
      for(; z < 4; z += 1) {
        hpdl_write_char(z+1, ' ');
      }
      return;
    }
  }
}
/*---------------------------------------------------------------------------*/
#if HPDL_USE_SCROLL
// XXX XXX XXX not tested and not sure working so don't use yet.
PROCESS(hpdl_process, "HPDL-1414 Process");
PROCESS_THREAD(hpdl_process, ev, data)
{
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER(etimer_stop(&hpdl_et));

  static clock_time_t intvl = CLOCK_SECOND/4;
  static struct etimer hpdl_et;
  static char *str;
  static uint8_t strpos = 0;

  PROCESS_BEGIN();
  if(data == NULL) {
    PROCESS_EXIT();
  }

  str = (char *)data;
  strpos = 0;
  while(1) {
    hpdl_write_string(str[strpos]);
    strpos++;
    // XXX check for 0 in string
    etimer_set(&hpdl_et, intvl);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&hpdl_et));
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
hpdl_scroll_stop(void)
{
  process_exit(&hpdl_process);
}
/*---------------------------------------------------------------------------*/
void
hpdl_scroll_string(char *s, clock_time_t intvl)
{
  process_start(&hpdl_process, s);
}
#endif    /* HPDL_USE_SCROLL */
/*--------------------------------------------------------------------------*/
/* clear the display */
void
hpdl_clear(void)
{
  hpdl_write_char(1, ' ');
  hpdl_write_char(2, ' ');
  hpdl_write_char(3, ' ');
  hpdl_write_char(4, ' ');

#if HPDL_USE_SCROLL
  hpdl_scroll_stop();
#endif    /* HPDL_USE_SCROLL */
}
/*--------------------------------------------------------------------------*/
/* init the display */
void
hpdl_init(void)
{
  /* init address pins */
  HPDL_ADDRESS_PORT(IE) &= ~(HPDL_A0_PIN | HPDL_A1_PIN);
  HPDL_ADDRESS_PORT(SEL) &= ~(HPDL_A0_PIN | HPDL_A1_PIN);
  HPDL_ADDRESS_PORT(SEL2) &= ~(HPDL_A0_PIN | HPDL_A1_PIN);
  HPDL_ADDRESS_PORT(DIR) |= (HPDL_A0_PIN | HPDL_A1_PIN);
  HPDL_ADDRESS_PORT(OUT) &= ~(HPDL_A0_PIN | HPDL_A1_PIN);

  /* init WR pin */
  HPDL_WR_PORT(IE) &= ~(HPDL_WR_PIN);
  HPDL_WR_PORT(SEL) &= ~(HPDL_WR_PIN);
  HPDL_WR_PORT(SEL2) &= ~(HPDL_WR_PIN);
  HPDL_WR_PORT(DIR) |= (HPDL_WR_PIN);
  HPDL_WR_PORT(OUT) |= (HPDL_WR_PIN);

  /* init digit pins */
  HPDL_DIGIT_PORT(IE) &= ~HPDL_DIGIT_PINS;
  HPDL_DIGIT_PORT(SEL) &= ~HPDL_DIGIT_PINS;
  HPDL_DIGIT_PORT(SEL2) &= ~HPDL_DIGIT_PINS;
  HPDL_DIGIT_PORT(DIR) |= HPDL_DIGIT_PINS;
  HPDL_DIGIT_PORT(OUT) &= ~HPDL_DIGIT_PINS;

  HPDL_DIGIT_EP_PORT(IE) &= ~HPDL_DIGIT_EP_PIN;
  HPDL_DIGIT_EP_PORT(SEL) &= ~HPDL_DIGIT_EP_PIN;
  HPDL_DIGIT_EP_PORT(SEL2) &= ~HPDL_DIGIT_EP_PIN;
  HPDL_DIGIT_EP_PORT(DIR) |= HPDL_DIGIT_EP_PIN;
  HPDL_DIGIT_EP_PORT(OUT) &= ~HPDL_DIGIT_EP_PIN;

}
/*--------------------------------------------------------------------------*/

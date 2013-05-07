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
 *         LEDs implementation for Launchpad.
 *          The other implementations are either too large (ordinary) or not
 *          working very well for our only two LEDs. Also, we don't mind ROM
 *          but RAM is very, very scarce on msp430g2452 and '2553. So we sacrifice
 *          some ROM and some cycles for a simplistic approach.
 * \author
 *         Marcus Linderoth <linderoth.marcus@gmail.com>
 */

#include "contiki.h"
#include "dev/leds.h"

/*---------------------------------------------------------------------------*/
void
leds_init(void)
{
  LEDS_PORT(IE) &= ~LEDS_CONF_ALL;
  LEDS_PORT(DIR) |= LEDS_CONF_ALL;
  LEDS_PORT(OUT) &= ~LEDS_CONF_ALL;
  LEDS_PORT(SEL) &= ~LEDS_CONF_ALL;
  LEDS_PORT(SEL2) &= ~LEDS_CONF_ALL;
}
/*---------------------------------------------------------------------------*/
void
leds_on(unsigned char leds)
{
  if(leds & LEDS_GREEN) {
    LEDS_PORT(OUT) |= LEDS_GREEN;
  }
  if(leds & LEDS_RED) {
    LEDS_PORT(OUT) |= LEDS_RED;
  }
}
/*---------------------------------------------------------------------------*/
void
leds_off(unsigned char leds)
{
  if(leds & LEDS_GREEN) {
    LEDS_PORT(OUT) &= ~LEDS_GREEN;
  }
  if(leds & LEDS_RED) {
    LEDS_PORT(OUT) &= ~LEDS_RED;
  }
}
/*---------------------------------------------------------------------------*/
void
leds_toggle(unsigned char leds)
{
  if(leds & LEDS_GREEN) {
    if(LEDS_PORT(OUT) & LEDS_GREEN) {
      LEDS_PORT(OUT) &= ~LEDS_GREEN;
    } else {
      LEDS_PORT(OUT) |= LEDS_GREEN;
    }
  }

  if(leds & LEDS_RED) {
    if(LEDS_PORT(OUT) & LEDS_RED) {
      LEDS_PORT(OUT) &= ~LEDS_RED;
    } else {
      LEDS_PORT(OUT) |= LEDS_RED;
    }
  }
}
/*---------------------------------------------------------------------------*/

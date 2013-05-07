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
 *         Launchpad button driver
 * \author
 *         Marcus Linderoth <linderoth.marcus@gmail.com>
 */

#include "contiki.h"
#include "button.h"
#include "isr_compat.h"

#include "dev/leds.h"

/*---------------------------------------------------------------------------*/
#define DEBOUNCE_TIME     (CLOCK_SECOND/8)
static struct timer debouncetimer;
/*---------------------------------------------------------------------------*/
/*
 * init the button by initing the port and pins, set up the interrupt and
 * register what process should be notified upon button pressed
 */
void
button_init(void)
{
  /* this gives the button_event a unique integer identification */
  button_event = process_alloc_event();

  dint();
  BUTTON_PORT(SEL) &= ~BUTTON_PINS;
  BUTTON_PORT(SEL2) &= ~BUTTON_PINS;
  BUTTON_PORT(DIR) &= ~BUTTON_PINS;

  /* the clock hardware needs pull-up resistor enabled and high->low edge select */
  BUTTON_PORT(REN) |= BUTTON_PINS;    /* enable pull-up/down */
  BUTTON_PORT(OUT) |= BUTTON_PINS;    /* pull-up */
  BUTTON_PORT(IES) &= ~BUTTON_PINS;    /* l->h transition */

  BUTTON_PORT(IE) |= BUTTON_PINS;
  BUTTON_PORT(IFG) &= ~BUTTON_PINS;
  eint();

  timer_set(&debouncetimer, CLOCK_SECOND/CLOCK_SECOND);
}
/*---------------------------------------------------------------------------*/
/* check if the button is pressed */
uint8_t
button_pressed(void)
{
  return BUTTON_PORT(DIR) & BUTTON_PINS;
}
/*---------------------------------------------------------------------------*/
/* ISR for the port the button is on; does not care what button is pressed, it just
 * recognizes one is. Any interested process should wait for button_event and
 * read out from the passed data pointer (eg if(*data & ) */
volatile uint8_t btnp = 0;
ISR(PORT1, button_interrupt)
{
  if(P1IFG & BUTTON_PINS) {
    /* read input port so what pin was activated can be checked */
    btnp = BUTTON_PORT(IFG);

    /* clear interrupt flag */
    P1IFG &= ~BUTTON_PINS;

    /* handle debouncing so that only one button press is registrered */
    if(timer_expired(&debouncetimer)) {
      btnp = BUTTON_PORT(IFG);
      timer_set(&debouncetimer, DEBOUNCE_TIME);

      /* send a button event to all processes */
      process_post(PROCESS_BROADCAST, button_event, (void*) &btnp);

      /* wake up so processes can handle the button event */
      LPM4_EXIT;
    }
  }
}
/*---------------------------------------------------------------------------*/

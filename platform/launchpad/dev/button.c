/*
 * Copyright (c) 2006, Swedish Institute of Computer Science
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

#include "contiki.h"
#include "button.h"
#include "isr_compat.h"

#include "dev/leds.h"

#define DEBOUNCE_TIME     (CLOCK_SECOND/8)

/*--------------------------------------------------------------------------*/
static struct timer debouncetimer;
/*--------------------------------------------------------------------------*/
/* 
 * init the button by initing the port and pins, set up the interrupt and 
 * register what process should be notified upon button pressed
 */
void
button_init(void)
{
  button_event = process_alloc_event();

  BUTTON_PORT(SEL) &= ~BUTTON_PINS;
  BUTTON_PORT(SEL2) &= ~BUTTON_PINS;
  BUTTON_PORT(DIR) &= ~BUTTON_PINS;
  BUTTON_PORT(OUT) |= BUTTON_PINS;
  #if BOARD_OLD_REVISION
  /* 
   * the old revision of boards have a resistor R34 in place for the button; if
   * your board is such, then this becomes necessary (but untested now).
   */
  BUTTON_PORT(REN) |= BUTTON_PINS;
  #endif
  dint();
  BUTTON_PORT(IES) |= BUTTON_PINS;
  BUTTON_PORT(IFG) &= ~BUTTON_PINS;

  BUTTON_PORT(IE) |= BUTTON_PINS;
  eint();

  timer_set(&debouncetimer, CLOCK_SECOND/CLOCK_SECOND);
}


/*--------------------------------------------------------------------------*/
#if 0
/* 
 * Another process can take ownership of the button ie become the one being
 * notified when pressed
 */
void
button_own(struct process *proc)
{
  button_owner_process = proc;
  if (proc != NULL) {
    BUTTON_PORT(IE) |= BUTTON_PINS;
  } else {
    BUTTON_PORT(IE) &=~ BUTTON_PINS;
  }
}

/*--------------------------------------------------------------------------*/
/* 
 * If a process, currently owning it, wants to not be notified any longer about
 * button presses. Any process owning a button should always have this in the 
 * process exithandler. You need to pass PROCESS_CURRENT() as parameter since
 * this function will check if it actually owns the button at the time of 
 * disowning.
 */

void
button_disown(struct process *proc)
{
  if (proc == button_owner_process) {
    button_owner_process = NULL;
    BUTTON_PORT(IE) &= ~BUTTON_PINS;
  }
}

/*--------------------------------------------------------------------------*/
struct process*
button_owner(void)
{
  return button_owner_process;
}
#endif
/*--------------------------------------------------------------------------*/
/* check if the button is pressed */
uint8_t
button_pressed(void)
{
  return BUTTON_PORT(DIR) & BUTTON_PINS;
}

/*--------------------------------------------------------------------------*/
/* ISR for the port the button is on; does not care what button is pressed, it just
 * recognizes one is. Any interested process should wait for button_event and
 * read out from the passed data pointer (eg if(*data & ) */
volatile uint8_t btnp = 0;
ISR(PORT1, button_interrupt)
{
  if(P1IFG & BUTTON_PINS) {
    btnp = BUTTON_PORT(IFG);
    
    P1IFG &= ~BUTTON_PINS;
    if(timer_expired(&debouncetimer)) {
      btnp = BUTTON_PORT(IFG);
      timer_set(&debouncetimer, DEBOUNCE_TIME);
      process_post(PROCESS_BROADCAST, button_event, (void*) &btnp);
      LPM4_EXIT;
    }
  }
}
/*--------------------------------------------------------------------------*/


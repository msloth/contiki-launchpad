/**
 * \addtogroup launchpad-platform
 *
 * @{
 */

/*
 * Copyright (c) 2012.
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

/*
 * \file
 *         A simple ADC-implementation
 * \author
 *         Marcus Lunden <marcus.lunden@gmail.com>
 *         
 */


#include "contiki.h"
#include "adc.h"
#include "isr_compat.h"
#include "dev/leds.h"

#define DEBUG   1
#if DEBUG
  #define PH(x)   P2OUT|=(1<<x)
  #define PL(x)   P2OUT&=~(1<<x)
#else
  #define PH(x)
  #define PL(x)
#endif
/*--------------------------------------------------------------------------*/
enum ADC_STATE {
  OFF,
  LONG_RUNNING,
  POLL,
  EVENT,
  ASYNCH,
  SYNCH,
};

/* holds the ADC result */
static uint16_t adcbuf;
/* ADC current state/mode */
static uint8_t state = OFF;
/* ev process calling the adc module */
struct process *calling_process = NULL;
/*--------------------------------------------------------------------------*/
static void
init_pininput(uint8_t pin)
{
  /* all analog in are on port 1 on 2553 */
  P1DIR &= ~(pin);
  P1SEL |= (pin);
  P1SEL2 |= (pin);
  P1REN &=~ (pin);
  ADC10AE0 = (pin);
/*  ADC10CTL1 |= (pin);*/   // XXX ooops, shouldn't have been here! check that it works, then delete
}
/*--------------------------------------------------------------------------*/
/* init the button by initing the port and pins, set up the interrupt and 
 * register what process should be notified upon button pressed */
void
adc_init(void)
{
  /* 
   * use Vcc, GND as references, single channel
   */
  ADC10CTL0 = SREF_0 | ADC10ON | ADC10SHT_2;
  ADC10CTL1 = (SHS_0 | ADC10SSEL_2 | CONSEQ_0);

  adc_event = process_alloc_event();
}
/*--------------------------------------------------------------------------*/
/* stop an ADC */
void
adc_stop(void)
{
  // XXX not yet implemented
  return;
}
/*--------------------------------------------------------------------------*/
/* returns true if the ADC is currently doing a conversion */
uint8_t
adc_busy(void)
{
  volatile uint16_t r = ADC10CTL1 & ADC10BUSY;
  return (uint8_t)r;
}
/*--------------------------------------------------------------------------*/
/* 
 * use this when you plan on using the conversion a bit later (>50 us later) 
 * and it is not that important. Will not block while converting.
 * val will contain the result when done.
 */
void
adc_asynch_get(uint8_t adc_ch, uint16_t *val)
{
  PH(0);

  /* wait for already running ADC to finish */
  while (adc_busy()) {;}
  state = ASYNCH;
  
  /* set up ports and pins */
  if(adc_ch <= A7) {
    init_pininput(1 << adc_ch);
  }
  ADC10CTL1 |= adc_ch << 12;

  /* this needs irq */
  ADC10CTL0 |= ADC10IE;
  ADC10CTL0 |= ADC10SC | ENC;

  PL(0);
}
/*--------------------------------------------------------------------------*/
/*
 * use this when you need the result as soon as it is done; it will block until
 * it is finished with the conversion (ca x ms)
 */
uint16_t
adc_synch_get(uint8_t adc_ch)
{
  PH(0);
  /* wait for already running ADC to finish */
  while (adc_busy()) {;}
  state = SYNCH;
  
  /* set up ports and pins */
  if(adc_ch <= A7) {
    /* all analog in are on port 1 on 2553 */
    init_pininput(1 << adc_ch);
  }
  ADC10CTL1 |= adc_ch << 12;
/*  ADC10CTL0 |= ADC10IE;*/   // test wo, then remove XXX

  /* set up, start ADC */
  ADC10CTL0 |= ADC10SC | ENC;

  /* wait for ADC to finish, return result */
  while (adc_busy()) {;}
  PL(0);
  return adcbuf;
}
/*--------------------------------------------------------------------------*/
/*
 * use this when you want to be notified as soon as the conversion is done.
 * The process p will be sent an event when conversion is done.
 *    adc_irqevent_get(A7, PROCESS_CURRENT());
 *    PROCESS_WAIT_EVENT_UNTIL(ev == adc_event);
 * 
 */
void
adc_irqevent_get(uint8_t adc_ch, struct process *p)
{
  PH(0);
  /* wait for already running ADC to finish */
  while (adc_busy()) {;}
  state = EVENT;
  calling_process = p;
  
  /* set up ports and pins */
  if(adc_ch <= A7) {
    /* all analog in are on port 1 on 2553 */
    init_pininput(1 << adc_ch);
  }
  ADC10CTL1 |= adc_ch << 12;

  /* set up, start ADC */
  ADC10CTL0 |= ADC10IE;
  ADC10CTL0 |= ADC10SC | ENC;

  PL(0);
}
/*--------------------------------------------------------------------------*/
/*
 * use this when you want to be notified as soon as the conversion is done.
 * The process will be polled when conversion is done.
 */
static uint16_t *valdest;
void
adc_irqpoll_get(uint8_t adc_ch, uint16_t *buf, struct process *p)
{
  PH(0);
  /* wait for already running ADC to finish */
  while (adc_busy()) {;}
  state = POLL;
  calling_process = p;
  valdest = buf;

  /* set up ports and pins */
  if(adc_ch <= A7) {
    /* all analog in are on port 1 on 2553 */
    init_pininput(1 << adc_ch);
  }
  ADC10CTL1 |= adc_ch << 12;

  /* set up, start ADC */
  ADC10CTL0 |= ADC10IE;
  ADC10CTL0 |= ADC10SC | ENC;

  PL(0);
}
/*--------------------------------------------------------------------------*/
/* ADC10 ISR */
ISR(ADC10, adc10_interrupt)
{
  PH(1);
  /* reset ADC registers as needed; copy conversion result */
/*  ADC10CTL0 &=~ ADC10IE;*/
  adcbuf = ADC10MEM;

  /* check for how adc was called */
  switch(state) {
    case POLL:
      if(calling_process != NULL) {
        if(valdest != NULL) {
          valdest = adcbuf;
        }
        process_poll(calling_process);
      }
      break;

    case EVENT:
      if(calling_process != NULL) {
        /* should it post to just caller, or broadcast? */
        process_post(calling_process, adc_event, &adcbuf);
/*        process_post(PROCESS_BROADCAST, adc_event, &adcbuf);*/
      }
      break;

    case ASYNCH:
      /* does nothing (ADC10MEM already copied) */
    case SYNCH:
    default:
      break;
  }
  
  /* wake the mcu up so the event can be handled */
  state = OFF;
  LPM4_EXIT;
  PL(1);
}
/*--------------------------------------------------------------------------*/
/** @} */

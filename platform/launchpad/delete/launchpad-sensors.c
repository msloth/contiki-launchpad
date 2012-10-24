/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
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
 * This file is part of the Contiki operating system.
 */
#include "contiki.h"
#include "lib/sensors.h"

#define ADC12MCTL_NO(adcno) ((unsigned char *) ADC12MCTL0_)[adcno]
/* what pins? On 2553 the entire port 1 doubles as ADC in, eg P1.4 = A4 */
#define ADC_PINS    (1<<4)   // P1.4, P1.5
/*#define ADC_PINS    (1<<4 | 1<<5)   // P1.4, P1.5*/

#define MAX_ADC_CHAN  11

static uint16_t adc_on;
static uint16_t ready;
static uint16_t adcbuf[MAX_ADC_CHAN];
/*---------------------------------------------------------------------------*/
static void
start(void)
{
#if 0
  /* set up ports and pins */
  P1DIR &= ~ADC_PINS;
  P1SEL |= ADC_PINS;
  ADC10AE0 = ADC_PINS;

  /* set up ADC10: use Vcc, GND as references, data transfer block to buffer,
   * start at temp sensor and work down
   */
  ADC10CTL0 = SREF_0 + MSC;
  ADC10CTL1 = INCH_11 | CONSEQ_3;

  /* continously store ADC values in the adcbuf vector */
  ADC10DTC0 = ADC10CT;
  ADC10DTC1 = MAX_ADC_CHAN;
  ADC10SA = adcbuf;

  /* start conversion */
  ADC10CTL0 |= ADC10ON;
  ADC10CTL0 |= ADC10SC | ENC;
#endif
}
/*---------------------------------------------------------------------------*/
/* stop ADC10; not really needed as we will only sample one at a time and synchronuously */
static void
stop(void)
{
#if 0
  ADC10CTL0 &= ~ENC;
  /* wait for conversion to stop */
  while(ADC10CTL1 & ADC10BUSY);

  /* clear any pending interrupts */
  ADC10CTL0 &= ~ADC10IFG;
#endif
}
/*--------------------------------------------------------------------------*/
uint16_t
launchpad_sensors_value(uint8_t s)
{
#if 0
  if (s > 10) {
    /* error, wrong sensor value */
    return 0;
  }
  return adcbuf[s];
#endif
  return 0;
}
/*---------------------------------------------------------------------------*/
int
launchpad_sensors_status(uint16_t input, int type)
{
#if 0
  if(type == SENSORS_ACTIVE) {
    return ADC10CTL0 & ENC;
  }
  if(type == SENSORS_READY) {
    return ((ADC10CTL0 & ENC) & 1);    // later find some way of knowing >=1 DTC completed
  }
  return 0;
#endif
  return 0;
}
/*---------------------------------------------------------------------------*/
int
launchpad_sensors_configure(uint16_t input, uint8_t ref, int type, int value)
{
#if 0
  if(type == SENSORS_ACTIVE) {
    /* called at SENSORS_ACTIVATE and SENSORS_DEACTIVATE */
    if (value == 1) {
      start();
    } else {
      // no, we don't allow anyone to deactivate the ADC as it is messy
      //stop();
    }
  }
  return 1;
#endif
  return 0;
}
/*---------------------------------------------------------------------------*/

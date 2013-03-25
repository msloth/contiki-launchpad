/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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
 *
 * $Id: rtimer-arch.c,v 1.17 2010/11/27 15:27:20 nifi Exp $
 */

/**
 * \file
 *         rtimer implementation for msp430g2553 and 2452
 *         also handles the simplepwm
 * \author
 *         Marcus Lunden <marcus.lunden@gmail.com>
 */

#include "contiki.h"
#include "sys/energest.h"
#include "sys/rtimer.h"
#include "sys/process.h"
#include "dev/watchdog.h"
#include "isr_compat.h"
#include "simple-pwm.h"


/* for simple_pwm */
struct pwm_s {
  /* pin on pre-configured port */
  uint8_t pin;
  /* on_time is time in on-state; duty cycle == on_time/period */
  uint16_t on_time;
} spwm;

static const uint16_t period = (SIMPLE_PWM_SECOND / SIMPLE_PWM_FREQ) - 1;
/* we are using the same CCR for both end of period and end of pulse */
static uint8_t period_end = 0;

extern void pwm_on_cb(void);
extern void pwm_off_cb(void);


/*--------------------------------------------------------------------------*/
ISR(TIMER0_A1, rtimer_a01_isr)
{
  /* store the IV register as any read/write resets interrupt flags */
  uint16_t ivreg = TA0IV;

  if(ivreg & TA0IV_TACCR1) {
    /* rtimer interrupt */
    TA0CCTL1 &= ~CCIFG;
    watchdog_start();

    /* check for and run any pending rtimers */
    rtimer_run_next();

    /* also check for any pending events and wake up if needed */
    if(process_nevents() > 0) {
      LPM4_EXIT;
    }
    watchdog_stop();




  } else if(ivreg & TA0IV_TACCR2) {
    /* simple pwm interrupt */
    TA0CCTL2 &= ~CCIFG;

    if(spwm.on_time > 0) {
      if(spwm.on_time == period) {
        TA0CCTL2 &= ~CCIE;  /* no need for interrupt, is at 100% DC */
/*        SIMPLE_PWM_PORT(OUT) |= (1 << spwm.pin);*/
        pwm_on_cb();

      } else {
        /* normal on-case */
        if(period_end) {
          period_end = 0;
          TA0CCR2 = TAR + spwm.on_time;
/*          SIMPLE_PWM_PORT(OUT) |= (1 << spwm.pin);*/
          pwm_off_cb();

        } else {
          period_end = 1;
          TA0CCR2 = TAR + (period - spwm.on_time);
/*          SIMPLE_PWM_PORT(OUT) &= ~(1 << spwm.pin);*/
          pwm_on_cb();
        }
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
void
rtimer_arch_init(void)
{
  /* uses the same timer as the clock timer so use compare register 1 for rtimers */
  /* Enable interrupt on CCR1. */
  TA0CCTL1 = CCIE;

  /* init simple_pwm */
  spwm.pin = 0;   /* default is P1.0 ie LED 1 */
  spwm.on_time = 0;
}
/*---------------------------------------------------------------------------*/
rtimer_clock_t
rtimer_arch_now(void)
{
  volatile rtimer_clock_t t1, t2;
  do {
    t1 = TAR;
    t2 = TAR;
  } while(t1 != t2);
  return t1;
}
/*---------------------------------------------------------------------------*/
void
rtimer_arch_schedule(rtimer_clock_t t)
{
  /* set the compare register; interrupt will fire on this count */
  TA0CCR1 = t;
}
/*---------------------------------------------------------------------------*/

/* these are the simple_pwm functions; bad place here but shared ISR with rtimer
means they should reside closeby */

/*---------------------------------------------------------------------------*/
/* set up a pin as the pin to use for simple pwm. Must be done before any on() */
void
simple_pwm_confpin(uint8_t pin)
{
  if(pin > 7) {
    return;
  }
  spwm.pin = pin;
  SIMPLE_PWM_PORT(IE) &= ~(1 << pin);
  SIMPLE_PWM_PORT(DIR) |= (1 << pin);
  SIMPLE_PWM_PORT(OUT) |= (1 << pin);
  SIMPLE_PWM_PORT(SEL) &= ~(1 << pin);
  SIMPLE_PWM_PORT(SEL2) &= ~(1 << pin);
}
/*---------------------------------------------------------------------------*/
/*
 * set pwm on at a duty cycle expressed in percent from 0 to 100. If crucial,
 * calculate your own pulsetime and use that function due to rounding errors in
 * this function.
 */
void
simple_pwm_on(uint8_t dc)
{
  if(dc > 100) {
    return;
  }

  if(dc == 100) {
    TA0CCTL2 &= ~CCIE;  /* no need for interrupt */
    spwm.on_time = period;
/*    SIMPLE_PWM_PORT(OUT) |= (1 << spwm.pin);*/
    pwm_on_cb();

  } else if(dc == 0) {
    TA0CCTL2 &= ~CCIE;  /* no need for interrupt */
    spwm.on_time = 0;
/*    SIMPLE_PWM_PORT(OUT) &= ~(1 << spwm.pin);*/
    pwm_off_cb();

  } else {
    /* convert duty cycle to PWM clock ticks */
    uint8_t finetime = (dc * period) / 100;
    if(finetime == 0 && dc != 0) {
      /* rounding errors, go for minimum-like */
      finetime = 2;
    }

    /* set up a compare match for the next PWM period */
    TA0CCR2 = TAR + finetime;
    TA0CCTL2 = CCIE;
    spwm.on_time = finetime;
  }
}
/*---------------------------------------------------------------------------*/
/*
 * set pwm on at a specified pulsetime measured in SIMPLE_PWM_SECOND ticks/s.
 * 
 * The period is (SIMPLE_PWM_SECOND / SIMPLE_PWM_FREQ) - 1, measured in the
 * same number of ticks/s. Default is 128 Hz and 32768 ticks/s, leaving us w
 * 255 ticks/period, so the range becomes 0..255.
 * Use this for precision work, eg a servo motor.
 */
void
simple_pwm_pulsetime(uint16_t finetime)
{
  if(finetime >= period) {
    TA0CCTL2 &= ~CCIE;  /* no need for interrupt */
    spwm.on_time = period;
/*    SIMPLE_PWM_PORT(OUT) |= (1 << spwm.pin);*/
    pwm_on_cb();

  } else if(finetime == 0) {
    TA0CCTL2 &= ~CCIE;  /* no need for interrupt */
    spwm.on_time = 0;
/*    SIMPLE_PWM_PORT(OUT) &= ~(1 << spwm.pin);*/
    pwm_off_cb();

  } else {
    /* set up a compare match for the next PWM period */
    TA0CCR2 = TAR + finetime;
    TA0CCTL2 = CCIE;
    spwm.on_time = finetime;
  }
}
/*---------------------------------------------------------------------------*/
/* just turn off the PWM. */
void
simple_pwm_off(void)
{
  TA0CCTL2 &= ~CCIE;  /* no need for interrupt */
/*  SIMPLE_PWM_PORT(OUT) &= ~(1 << spwm.pin);*/
  pwm_off_cb();
}
/*---------------------------------------------------------------------------*/

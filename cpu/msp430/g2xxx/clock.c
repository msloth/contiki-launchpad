/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
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

/**
 * \file
 *         Clock drivers for MSP430G2xxx chips
 * \author
 *         Marcus Linderoth <linderoth.marcus@gmail.com> for this file,
 *         but based on previous files for other msp430's
 */

#include "contiki.h"
#include "sys/clock.h"
#include "sys/etimer.h"
#include "rtimer-arch.h"
#include "dev/watchdog.h"
#include "isr_compat.h"

/* Clock is running off LFXT1 with divider */
#define INTERVAL  ((32768/1)/CLOCK_CONF_SECOND)

#define MAX_TICKS (~((clock_time_t)0) / 2)

static volatile unsigned long seconds;
static volatile clock_time_t count = 0;
/* last_tar is used for calculating clock_fine */
static volatile uint16_t last_tar = 0;

/* Make sure the CLOCK_CONF_SECOND is a power of two, to ensure
  that the modulo operation below becomes a logical and and not
  an expensive divide. Algorithm from Wikipedia:
  http://en.wikipedia.org/wiki/Power_of_two */
#if (CLOCK_CONF_SECOND & (CLOCK_CONF_SECOND - 1)) != 0
#error CLOCK_CONF_SECOND must be a power of two (i.e., 1, 2, 4, 8, 16, 32, 64, ...).
#error Change CLOCK_CONF_SECOND in contiki-conf.h.
#endif
/*---------------------------------------------------------------------------*/
void
clock_set(clock_time_t clock, clock_time_t fclock)
{
  TAR = fclock;
  TACCR0 = fclock + INTERVAL;
  count = clock;
}
/*---------------------------------------------------------------------------*/
int
clock_fine_max(void)
{
  return INTERVAL;
}
/*---------------------------------------------------------------------------*/
/* returns the number of timer ticks since last clock tick. Ie, if the interval
 * of the timer is 3600 timer ticks, then this should return 0..3600 */
unsigned short
clock_fine(void)
{
  unsigned short t;
  /* Assign last_tar to local varible that can not be changed by interrupt */
  t = last_tar;
  /* perform calc based on t, TAR will not be changed during interrupt */
  return (unsigned short) (TAR - t);
}
/*---------------------------------------------------------------------------*/
clock_time_t
clock_time(void)
{
  clock_time_t t1, t2;
  do {
    t1 = count;
    t2 = count;
  } while(t1 != t2);
  return t1;
}
/*---------------------------------------------------------------------------*/

ISR(TIMER0_A0, timera0_interrupt)
{
  /* update the timer */
  TACCTL0 &= ~CCIFG;
  TACCR0 += INTERVAL;

  watchdog_start();

  /* update system time/clock */
  do {
    ++count;
    if(count % CLOCK_CONF_SECOND == 0) {
      ++seconds;
    }
  } while((TACCR0 - TAR) > INTERVAL);

  /* remember last counter value */
  last_tar = TAR;

  /* check if any etimer is expired - wake up if so */
  if(etimer_pending() && (etimer_next_expiration_time() - count - 1) > MAX_TICKS) {
    etimer_request_poll();
    LPM4_EXIT;
  }

  /* check for pending process events - wake up if so */
  if(process_nevents() >= 0) {
    LPM4_EXIT;
  }

  watchdog_stop();
}
/*--------------------------------------------------------------------------*/
void
clock_init(void)
{
  dint();

  /* Timer is in cont up mode, irq when counter TAR == CCR0 */
  /* using ACLK, which is 32768 ext osc; div 1 */
  TACTL = TASSEL_1 | ID_0;
  TACCR0 = INTERVAL;
  TACCTL0 = CCIE;
  TACTL |= MC_2;
  count = 0;

  eint();
}
/*---------------------------------------------------------------------------*/
/**
 * Delay the CPU for a multiple of x us.
 */
void
clock_delay(unsigned int i)
{
  while(i--) {
    _NOP();
  }
}
/*---------------------------------------------------------------------------*/
/**
 * Wait for a multiple of 10 ms.
 *
 */
void
clock_wait(clock_time_t i)
{
  clock_time_t start;

  start = clock_time();
  while(clock_time() - start < (clock_time_t)i);
}
/*---------------------------------------------------------------------------*/
void
clock_set_seconds(unsigned long sec)
{
  int s;
  s = splhigh();
  seconds = sec;
  splx(s);
}
/*---------------------------------------------------------------------------*/
unsigned long
clock_seconds(void)
{
  unsigned long t1, t2;
  do {
    t1 = seconds;
    t2 = seconds;
  } while(t1 != t2);
  return t1;
}
/*---------------------------------------------------------------------------*/
rtimer_clock_t
clock_counter(void)
{
  rtimer_clock_t t1, t2;
  do {
    t1 = TAR;
    t2 = TAR;
  } while(t1 != t2);
  return t1;
}
/*---------------------------------------------------------------------------*/

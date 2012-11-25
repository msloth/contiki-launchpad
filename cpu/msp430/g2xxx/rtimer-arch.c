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
 * \author
 *         Marcus Lunden <marcus.lunden@gmail.com>
 */

#include "contiki.h"
#include "sys/energest.h"
#include "sys/rtimer.h"
#include "sys/process.h"
#include "dev/watchdog.h"
#include "isr_compat.h"
/*--------------------------------------------------------------------------*/
ISR(TIMER0_A1, rtimer_a01_isr)
{
  if(TA0IV == TA0IV_TACCR1) {
    TA0CCTL1 &= ~CCIFG;
    watchdog_start();

    /* check for and run any pending rtimers */
    rtimer_run_next();

    /* also check for any pending events and wake up if needed */
    if(process_nevents() > 0) {
      LPM4_EXIT;
    }
    watchdog_stop();
  }
}
/*---------------------------------------------------------------------------*/
void
rtimer_arch_init(void)
{
  /* uses the same timer as the clock timer so use compare register 1 */
  /* Enable interrupt on CCR1. */
  TA0CCTL1 = CCIE;
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

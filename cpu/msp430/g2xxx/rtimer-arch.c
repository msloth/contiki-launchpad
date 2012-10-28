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
 *         MSP430-specific rtimer code
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"

#include "sys/energest.h"
#include "sys/rtimer.h"
#include "sys/process.h"
#include "dev/watchdog.h"
#include "isr_compat.h"

#define DEBUG 1
#if DEBUG
#include <stdio.h>
  #define PRINTF(...) printf(__VA_ARGS__)
  #define PH(x)   P2OUT|=(1<<x)
  #define PL(x)   P2OUT&=~(1<<x)
#else
  #define PRINTF(...)
  #define PH(x)
  #define PL(x)
#endif
/*--------------------------------------------------------------------------*/
ISR(TIMER0_A1, rtimer_a01_isr)
{
  PH(0);
  TACCTL1 &= ~CCIFG;
  if (TAIV == TA0IV_TACCR1) {
    watchdog_start();
    rtimer_run_next();
    if(process_nevents() > 0) {
      LPM0_EXIT;
    }
    watchdog_stop();
  }
  PL(0);
}
/*---------------------------------------------------------------------------*/
void
rtimer_arch_init(void)
{
  /* input clock source LFXT1 @ 32768 Hz, div by 1; with 16-bit timer = 2s*/

  /* disable interrupts */
  dint();

  /* Enable interrupts. */
  TA0CCTL1 = CCIE;
  eint();
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
  return *((volatile uint16_t*)(0x170));
  return t1;
}
/*---------------------------------------------------------------------------*/
void
rtimer_arch_schedule(rtimer_clock_t t)
{
/*  PH(1);*/
  TA0CCR1 = t;
/*  PL(1);*/
}
/*---------------------------------------------------------------------------*/


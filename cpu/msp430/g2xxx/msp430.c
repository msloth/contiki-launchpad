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
 *         MCU drivers for MSP430G2xxx chips
 * \author
 *         Marcus Lunden <marcus.lunden@gmail.com>
 *         and others
 */

#include <msp430.h>
#include "contiki.h"
#include "dev/watchdog.h"
#include "dev/leds.h"

#if defined(__MSP430__) && defined(__GNUC__)
#define asmv(arg) __asm__ __volatile__(arg)
#endif

/*---------------------------------------------------------------------------*/
#if defined(__MSP430__) && defined(__GNUC__) && MSP430_MEMCPY_WORKAROUND
void *
w_memcpy(void *out, const void *in, size_t n)
{
  uint8_t *src, *dest;
  src = (uint8_t *) in;
  dest = (uint8_t *) out;
  while(n-- > 0) {
    *dest++ = *src++;
  }
  return out;
}
#endif /* __GNUC__ &&  __MSP430__ && MSP430_MEMCPY_WORKAROUND */
/*---------------------------------------------------------------------------*/
#if defined(__MSP430__) && defined(__GNUC__) && MSP430_MEMCPY_WORKAROUND
void *
w_memset(void *out, int value, size_t n)
{
  uint8_t *dest;
  dest = (uint8_t *) out;
  while(n-- > 0) {
    *dest++ = value & 0xff;
  }
  return out;
}
#endif /* __GNUC__ &&  __MSP430__ && MSP430_MEMCPY_WORKAROUND */
/*---------------------------------------------------------------------------*/
/* Init the DCO in the simplest possible way: use the factory calibrated values */
#define XTIN  (1<<6)
#define XTOUT (1<<7)

void
msp430_init_dco(void)
{
  /* 
  this sets the following:
    DCO = 1,4,8,16 MHz
    MCLK source DCO, div suitable for the clock
    SMCLK source DCO, div -> 4 MHz
    ACLK source LFXT1, div 1 -> 32768 Hz
    
    see datasheet p26 and FUG p292 
   */
  P2DIR &=~ (XTIN | XTOUT);
  P2SEL |= (XTIN | XTOUT);


  /* check for erased DCO calibration data */
  if (CALBC1_1MHZ == 0xFF) {
    /* if they are erased, we would end up in a possibly (to the mcu) dangerous
    or unreliable condition if setting bogus values, and running at the default,
    low speed is not feasible or ok. So, we stay here in that case. */
    while(1);
  }

  /* set clock sources, dividers etc. */
  DCOCTL = 0;
#if F_CPU == 1000000ul
  DCOCTL = CALDCO_8MHZ;
  BCSCTL1 = CALBC1_8MHZ;
  BCSCTL2 = SELM_0 | DIVM_3 | DIVS_1;
#elif F_CPU == 4000000ul
  // divide by two
  DCOCTL = CALDCO_8MHZ;
  BCSCTL1 = CALBC1_8MHZ;
  BCSCTL2 = SELM_0 | DIVM_1 | DIVS_1;
#elif F_CPU == 8000000ul
  DCOCTL = CALDCO_8MHZ;
  BCSCTL1 = CALBC1_8MHZ;
  BCSCTL2 = SELM_0 | DIVM_0 | DIVS_1;
#elif F_CPU == 12000000ul
  #error F_CPU 12 MHz not supported due to UART not adapted to that speed yet; choose another speed in platform-conf.h!
/*  DCOCTL = CALDCO_12MHZ;*/
/*  BCSCTL1 = CALBC1_12MHZ;*/
/*  BCSCTL2 = SELM_0 | DIVM_1 | DIVS_2;*/
#elif F_CPU == 16000000ul
  DCOCTL = CALDCO_16MHZ;
  BCSCTL1 = CALBC1_16MHZ;
  BCSCTL2 = SELM_0 | DIVM_0 | DIVS_2;
#else
#error Wrong speed defined (see msp430.c, project-conf.h and platform-conf.h)!
#endif

  
#if 0
  for reference, some settings and registers; 16,12,8,1 MHz factory calibrations
  #define CALDCO_16MHZ_         0x10F8    /* DCOCTL  Calibration Data for 16MHz */
  #define CALBC1_16MHZ_         0x10F9    /* BCSCTL1 Calibration Data for 16MHz */
  #define CALDCO_12MHZ_         0x10FA    /* DCOCTL  Calibration Data for 12MHz */
  #define CALBC1_12MHZ_         0x10FB    /* BCSCTL1 Calibration Data for 12MHz */
  #define CALDCO_8MHZ_          0x10FC    /* DCOCTL  Calibration Data for 8MHz */
  #define CALBC1_8MHZ_          0x10FD    /* BCSCTL1 Calibration Data for 8MHz */
  #define CALDCO_1MHZ_          0x10FE    /* DCOCTL  Calibration Data for 1MHz */
  #define CALBC1_1MHZ_          0x10FF    /* BCSCTL1 Calibration Data for 1MHz */
  DIVS0 = div by 1
  DIVS1 = div by 2
  DIVS2 = div by 4
  DIVS3 = div by 8
  #define DIVM_0              (0x00)   /* MCLK Divider 0: /1 */
  #define DIVM_1              (0x10)   /* MCLK Divider 1: /2 */
  #define DIVM_2              (0x20)   /* MCLK Divider 2: /4 */
  #define DIVM_3              (0x30)   /* MCLK Divider 3: /8 */
#endif
}
/*---------------------------------------------------------------------------*/
static void
init_ports(void)
{
  /* All available pins are outputs, saves the most power */
  P1SEL = 0;
  P1DIR = 0;
  P1OUT = 0;
  P1IE = 0;

  P2SEL = 0;
  P2DIR = 0;
  P2OUT = 0;
  P2IE = 0;
}
/*---------------------------------------------------------------------------*/
/* msp430-ld may align _end incorrectly. Workaround in cpu_init. */
#if defined(__MSP430__) && defined(__GNUC__)
extern int _end;		/* Not in sys/unistd.h */
static char *cur_break = (char *)&_end;
#endif
/*---------------------------------------------------------------------------*/
void
msp430_cpu_init(void)
{
  dint();
  watchdog_init();
  init_ports();
  msp430_init_dco();
  eint();
#if defined(__MSP430__) && defined(__GNUC__)
  if((uintptr_t)cur_break & 1) { /* Workaround for msp430-ld bug! */
    cur_break++;
  }
#endif
}
/*---------------------------------------------------------------------------*/
#if 0
// XXX these are old legacy and not used in this port
/*---------------------------------------------------------------------------*/
/* add/remove_lpm_req - for requiring a specific LPM mode. currently Contiki */
/* jumps to LPM3 to save power, but DMA will not work if DCO is not clocked  */
/* so some modules might need to enter their LPM requirements                */
/* NOTE: currently only works with LPM1 (e.g. DCO) requirements.             */
/*---------------------------------------------------------------------------*/
void
msp430_add_lpm_req(int req)
{
}

/*--------------------------------------------------------------------------*/
void
msp430_remove_lpm_req(int req)
{
}
/*--------------------------------------------------------------------------*/
#define STACK_EXTRA 32

/*
 * Allocate memory from the heap. Check that we don't collide with the
 * stack right now (some other routine might later). A watchdog might
 * be used to check if cur_break and the stack pointer meet during
 * runtime.
 */
#if defined(__MSP430__) && defined(__GNUC__)
void *
sbrk(int incr)
{
  char *stack_pointer;

  asmv("mov r1, %0" : "=r" (stack_pointer));
  stack_pointer -= STACK_EXTRA;
  if(incr > (stack_pointer - cur_break))
    return (void *)-1;		/* ENOMEM */

  void *old_break = cur_break;
  cur_break += incr;
  /*
   * If the stack was never here then [old_break .. cur_break] should
   * be filled with zeros.
  */
  return old_break;
}
#endif
/*---------------------------------------------------------------------------*/
/*
 * Mask all interrupts that can be masked.
 */
int
splhigh_(void)
{
  int sr;
  /* Clear the GIE (General Interrupt Enable) flag. */
#ifdef __IAR_SYSTEMS_ICC__
  sr = __get_SR_register();
  __bic_SR_register(GIE);
#else
  asmv("mov r2, %0" : "=r" (sr));
  asmv("bic %0, r2" : : "i" (GIE));
#endif
  return sr & GIE;		/* Ignore other sr bits. */
}
/*---------------------------------------------------------------------------*/
#ifdef __IAR_SYSTEMS_ICC__
int __low_level_init(void)
{
  /* turn off watchdog so that C-init will run */
  WDTCTL = WDTPW + WDTHOLD;
  /*
   * Return value:
   *
   *  1 - Perform data segment initialization.
   *  0 - Skip data segment initialization.
   */
  return 1;
}
#endif
/*---------------------------------------------------------------------------*/
#endif    /* old, not used */


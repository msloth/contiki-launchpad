/*
 * Copyright (c) 2012
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

/**
 * \file
 *         Platform configuration header file
 * \author
 *         Marcus Lunden <marcus.lunden@gmail.com>
 */

#ifndef __PLATFORM_CONF_H__
#define __PLATFORM_CONF_H__


/*--------------------------------------------------------------------------*/
/* 
 * The following are some defines that can be changed in order to change what
 * is included at compile-time, what CPU speed is used etc. You should copy 
 * this file to your local project folder and change it there, not in the 
 * platform/launchpad folder!
 */

/* CPU speed in Hz; these correspond to the factory calibrated settings */
//#define F_CPU                   18000000uL
//#define F_CPU                   16000000uL
#define F_CPU                   8000000uL
//#define F_CPU                   4000000uL

//#define F_CPU                   12000000uL    // don't use the 12 MHz just yet, not ready (UART etc)

/* is it an 2452 or 2553 mcu? Only these two can be defined for now.
XXX I will fix (later) so that the Makefile automatically choses the right mcu define
but until then you also have to change that in the Makefile.launchpad. You'll
see, if you get lots of "undefined reference to UCSB0..." it's because the wrong
mcu is defined there. This define just removes lots of code that, should you have
a 2452, it will compile anyway. */
#define _MCU_                   2553
//#define _MCU_                   2452

/* 
Here is a short summary of the Launchpad mcu's
                 2211     2231    2452      2553
    ROM             2        2       8        16
    RAM           128      128     256       512
    UART                                       1
    SPI/I2C                  1       1         1
    GPIO           10       10      16        24
    touch-IO                         y         y
    Timers-16       1        1       1         2
    WDT             y        y       y         y
    ADC10                    8       8         8
    ADC12
    PDIP-x         14       14      20        20
    tempsensor               y       y         y
        1.8 V to 3.6 V
        16 MHz
        Wake-Up Less Than 1 µs
          Active Mode: 230 µA at 1 MHz, 2.2 V
          Standby Mode: 0.5 µA
          Off Mode (RAM Retention): 0.1 µA
  For Contiki,
    timer 1 (two CCR's) is used for the two internal clocks.
    Timer 2 for PWM.
    USCI 1 for UART serial/printfs.
    USCI 2 for SPI to radio (blocking so won't go to LPM while communicating).
  Thus we see that 2452 has an SPI USCI but not enough RAM for radio+Rime, nor
    will it accomodate the PWM or serial modules as they use the second timer.
*/

/* use serial port? (printfs); saves space if not */
#define USE_SERIAL              0

/* use the Rime networking stack and a radio driver? */
#define USE_RADIO               1

/*
 * Does the board have an external 32kHz osc? Currently mandatory and this switch
 * won't make a difference, but included for ev future use so clocks and timers
 * can use the DCO instead.
 */
#define HAS_EXT_OSC             1
  
/* 
 * board revision: older boards had a pull-up resistor connected to the button
 * which was removed to conserve power and save money. If the resistor is there,
 * the button needs some extra care behind the curtains, which will happen if this is
 * set to 1. To check if you need to set this to 1, check for a small (ca 1 by
 * 2 mm) rectangular thing in the spot where it says R34, close to the top of
 * the left pin headers (when the USB connector is facing up). If it is there,
 * set this to 1.
 *
 * Note: setting this to 1 will save power on old boards but button will not work
 * on >=rev1.5 boards; if set to 0 the button will work on rev1.3, 1.4 and 1.5 but
 * draw more power on the 1.3 and 1.4 revisions.
 */
#define BOARD_OLD_REVISION      0

/* 
 * LEDs; only change if you are not using the Launchpad board and have LEDs
 * on other pins. You will likely have to change in cpu/msp430/g2xxx/leds-arch.c
 * as well as core/dev/leds.{c, h} if you want to add more LEDs or change pins.
 */
#define LEDS_PORT(type)         P1##type
#define LEDS_PxDIR              P1DIR
#define LEDS_PxOUT              P1OUT
#define LEDS_CONF_RED           (1<<0)
#define LEDS_CONF_GREEN         (1<<6)
#define LEDS_CONF_YELLOW        (1<<6)    /* only here to remove compilation errors */
#define LEDS_CONF_ALL           (LEDS_CONF_GREEN | LEDS_CONF_RED)


/* PWM freq; can be higher and lower; setting to lower will increase accuracy of
  the duty cycle (rounding errors) calculated in pwm_on(). */
#define PWM_CONF_FREQ           128

#ifdef PWM_CONF_FREQ
#define PWM_FREQ                PWM_CONF_FREQ
#else
#define PWM_FREQ                128
#endif

/* P1.3 is the switch 2 on the Launchpad PCB, but other pins can be used. */
//XXX now defined in platform/launchpad/button.h
//#define BUTTON_CONF_PORT        P1
//#define BUTTON_CONF_PIN         (1<<3)

/* use irq/dma with UART to save system resources, otherwise synchrous (blocking) */
// XXX not working yet, is only blocking so leave as 0 now.
#define UART0_CONF_TX_WITH_INTERRUPT  0

/* this is where in memory the node id is stored (must first be written by burn),
  these memory locations are non-volatile and will not be erased by a 'full erase'
  unless explicitly told to, hence we can save some data here that will survive
  reboots and periods without power. Infomem A has some factory calibration data.
  Store node id in infomem C as it is normally empty (A is occupied and D seems
  not empty). 64 B each. Node id is prepended with the magic bytes 0xBEEF. */
#define INFOMEM_D                     ((uint8_t*)0x00001000)
#define INFOMEM_C                     ((uint8_t*)0x00001040)
#define INFOMEM_B                     ((uint8_t*)0x00001080)
#define INFOMEM_A                     ((uint8_t*)0x000010c0)
#define NODEID_INFOMEM_LOCATION       INFOMEM_C

/*--------------------------------------------------------------------------*/
/*
 * The CC2500 definitions - pins, ports, interrupt vector etc - is defined in
 * the following files:
 *    spi.c, spi.h
 *    cc2500.c, cc2500.h, cc2500-arch.c, cc2500-const.h
 */
/*--------------------------------------------------------------------------*/
/* some helping defines that normally should not be changed */
/* Clock resolutions */
#define CLOCK_CONF_SECOND               (128UL)
#define RTIMER_CONF_SECOND              32768

/* Types for clocks and uip_stats */
typedef unsigned short uip_stats_t;
typedef unsigned long clock_time_t;
typedef unsigned long off_t;

/* errors ------------------------------------*/
#if _MCU_ != 2452 && _MCU_ != 2553
#error "Wrong mcu type defined; check your platform-conf.h and/or project-conf.h!"
#endif

#if F_CPU != 16000000uL && F_CPU != 8000000uL && F_CPU != 4000000uL
#error "Unsupported CPU speed; check your platform-conf.h and/or project-conf.h!"
#endif

#if F_CPU == 12000000uL
#error "CPU speed 12 MHz not supported, chose another (platform-conf.h and/or \
project-conf.h)!"
#endif

#if _MCU_ == 2452 && USE_SERIAL
#error "2452 has no hardware UART, either set 2553 or set USE_SERIAL to 0 in platform-conf.h"
#endif

#if HAS_EXT_OSC != 1
#error "You must have the external 32.768 oscillator populated on board."
#endif









/* other various things ----------------------------*/
#define BAUD2UBR(baud) ((F_CPU/baud))
#define CCIF
#define CLIF

#define HAVE_STDINT_H
#define MSP430_MEMCPY_WORKAROUND 1
#if defined(__MSP430__) && defined(__GNUC__) && MSP430_MEMCPY_WORKAROUND
#else /* __GNUC__ &&  __MSP430__ && MSP430_MEMCPY_WORKAROUND */
#define w_memcpy memcpy
#endif /* __GNUC__ &&  __MSP430__ && MSP430_MEMCPY_WORKAROUND */
#include "msp430def.h"

#if 0
/* DCO speed periodic resynchronization for more robust UART, etc. */
#ifndef DCOSYNCH_CONF_ENABLED
#define DCOSYNCH_CONF_ENABLED 0
#endif /* DCOSYNCH_CONF_ENABLED */
#ifndef DCOSYNCH_CONF_PERIOD
#define DCOSYNCH_CONF_PERIOD 30
#endif /* DCOSYNCH_CONF_PERIOD */
#endif
/*--------------------------------------------------------------------------*/



/* USART0 Tx ready? */
//  #define SPI_WAITFOREOTx() while ((U0TCTL & TXEPT) == 0)
///* USART0 Rx ready? */
//  #define SPI_WAITFOREORx() while ((IFG1 & URXIFG0) == 0)
///* USART0 Tx buffer ready? */
//  #define SPI_WAITFORTxREADY() while ((IFG1 & UTXIFG0) == 0)


/* defines related to CFS (file system) and flash; only relevant if we had
 * external storage or want to rewrite flash. */
  //#define ROM_ERASE_UNIT_SIZE  512
  //#define XMEM_ERASE_UNIT_SIZE (64*1024L)
  //#define CFS_CONF_OFFSET_TYPE    long

  /* Use the first 64k of external flash for node configuration */
  //#define NODE_ID_XMEM_OFFSET     (0 * XMEM_ERASE_UNIT_SIZE)

  /* Use the second 64k of external flash for codeprop. */
  //#define EEPROMFS_ADDR_CODEPROP  (1 * XMEM_ERASE_UNIT_SIZE)

  //#define CFS_XMEM_CONF_OFFSET    (2 * XMEM_ERASE_UNIT_SIZE)
  //#define CFS_XMEM_CONF_SIZE      (1 * XMEM_ERASE_UNIT_SIZE)

  //#define CFS_RAM_CONF_SIZE 4096







#endif /* __PLATFORM_CONF_H__ */

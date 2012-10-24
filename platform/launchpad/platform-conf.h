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
// 1 Mhz not used as it is too constraining on UART and timer speeds etc.

/* is it an 2452 or 2553 mcu? Only these two can be defined for now. */
#define _MCU_                   2553
//#define _MCU_                   2452

/* board revision: older boards had a pull-up resistor connected to the button
 * which was removed to conserve power. If the resistor is there, the button
 * needs some extra care behind the curtains, which will happen if this is
 * set to 1. To check if you need to set this to 1, check for a small (ca 1 by
 * 2 mm) rectangular thing in the spot where it says R34, close to the top of
 * the left pin headers (when USB is facing up). If it is there, set this to 1.
 */
#define BOARD_OLD_REVISION      1

/* use serial port? (printfs); saves space if not */
#define USE_SERIAL       1
/* use the Rime networking stack? (have radio?) */
#define USE_RIME                0

/*
 * Does it have an external 32kHz osc? Currently doesn't matter but included for
 * future use so clocks and timers can use the DCO instead.
 */
#define HAS_EXT_OSC             1

/* 
 * LEDs; only change if you are not using the Launchpad board and have LEDs
 * on other pins.
 */
#define LEDS_PORT(type)         P1##type

#define LEDS_PxDIR              P1DIR
#define LEDS_PxOUT              P1OUT
#define LEDS_CONF_RED           (1<<0)
#define LEDS_CONF_GREEN         (1<<6)
#define LEDS_CONF_YELLOW        (LEDS_CONF_GREEN | LEDS_CONF_RED)

/* P1.3 is the switch 2 on the Launchpad PCB, but other pins can be used. */
// XXX not working yet
#define BUTTON_CONF_PORT        P1
#define BUTTON_CONF_PIN         3

/* use irq/dma with UART to save system resources, otherwise synchrous (blocking) */
// XXX not working yet, is only blocking
#define UART1_CONF_TX_WITH_INTERRUPT    0

#define WITH_UIP6               0

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

/* DCO speed periodic resynchronization for more robust UART, etc. */
#ifndef DCOSYNCH_CONF_ENABLED
#define DCOSYNCH_CONF_ENABLED 0
#endif /* DCOSYNCH_CONF_ENABLED */
#ifndef DCOSYNCH_CONF_PERIOD
#define DCOSYNCH_CONF_PERIOD 30
#endif /* DCOSYNCH_CONF_PERIOD */


/*
 * SPI bus configuration--------------------------------------------------------
 */

//SPI pins
  #define SCK           1  /* P3.1 - Output: SPI Serial Clock (SCLK) */
  #define MOSI          2  /* P3.2 - Output: SPI Master out - slave in (MOSI) */
  #define MISO          3  /* P3.3 - Input:  SPI Master in - slave out (MISO) */
/* SPI input/output registers. */
  #define SPI_TXBUF U0TXBUF
  #define SPI_RXBUF U0RXBUF

/* USART0 Tx ready? */
  #define SPI_WAITFOREOTx() while ((U0TCTL & TXEPT) == 0)
/* USART0 Rx ready? */
  #define SPI_WAITFOREORx() while ((IFG1 & URXIFG0) == 0)
/* USART0 Tx buffer ready? */
  #define SPI_WAITFORTxREADY() while ((IFG1 & UTXIFG0) == 0)


// M25P80 external flash configuration SPI
  //#define FLASH_PWR       3       /* P4.3 Output */
  //#define FLASH_CS        4       /* P4.4 Output */
  //#define FLASH_HOLD      7       /* P4.7 Output */

/* Enable/disable flash access to the SPI bus (active low). */
  //#define SPI_FLASH_ENABLE()  ( P4OUT &= ~BV(FLASH_CS) )
  //#define SPI_FLASH_DISABLE() ( P4OUT |=  BV(FLASH_CS) )
  //#define SPI_FLASH_HOLD()                ( P4OUT &= ~BV(FLASH_HOLD) )
  //#define SPI_FLASH_UNHOLD()              ( P4OUT |=  BV(FLASH_HOLD) )










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






// cc2420 stuff; Launchpad has not got one.
#if 0
  /*
   * SPI bus - CC2420 pin configuration.
   */
  #define CC2420_CONF_SYMBOL_LOOP_COUNT 800

  /* P1.0 - Input: FIFOP from CC2420 */
  #define CC2420_FIFOP_PORT(type)   P1##type
  #define CC2420_FIFOP_PIN          0
  /* P1.3 - Input: FIFO from CC2420 */
  #define CC2420_FIFO_PORT(type)     P1##type
  #define CC2420_FIFO_PIN            3
  /* P1.4 - Input: CCA from CC2420 */
  #define CC2420_CCA_PORT(type)      P1##type
  #define CC2420_CCA_PIN             4
  /* P4.1 - Input:  SFD from CC2420 */
  #define CC2420_SFD_PORT(type)      P4##type
  #define CC2420_SFD_PIN             1
  /* P4.2 - Output: SPI Chip Select (CS_N) */
  #define CC2420_CSN_PORT(type)      P4##type
  #define CC2420_CSN_PIN             2
  /* P4.5 - Output: VREG_EN to CC2420 */
  #define CC2420_VREG_PORT(type)     P4##type
  #define CC2420_VREG_PIN            5
  /* P4.6 - Output: RESET_N to CC2420 */
  #define CC2420_RESET_PORT(type)    P4##type
  #define CC2420_RESET_PIN           6

  #define CC2420_IRQ_VECTOR PORT1_VECTOR

  /* Pin status. */
  #define CC2420_FIFOP_IS_1 (!!(CC2420_FIFOP_PORT(IN) & BV(CC2420_FIFOP_PIN)))
  #define CC2420_FIFO_IS_1  (!!(CC2420_FIFO_PORT(IN) & BV(CC2420_FIFO_PIN)))
  #define CC2420_CCA_IS_1   (!!(CC2420_CCA_PORT(IN) & BV(CC2420_CCA_PIN)))
  #define CC2420_SFD_IS_1   (!!(CC2420_SFD_PORT(IN) & BV(CC2420_SFD_PIN)))

  /* The CC2420 reset pin. */
  #define SET_RESET_INACTIVE()   (CC2420_RESET_PORT(OUT) |=  BV(CC2420_RESET_PIN))
  #define SET_RESET_ACTIVE()     (CC2420_RESET_PORT(OUT) &= ~BV(CC2420_RESET_PIN))

  /* CC2420 voltage regulator enable pin. */
  #define SET_VREG_ACTIVE()       (CC2420_VREG_PORT(OUT) |=  BV(CC2420_VREG_PIN))
  #define SET_VREG_INACTIVE()     (CC2420_VREG_PORT(OUT) &= ~BV(CC2420_VREG_PIN))

  /* CC2420 rising edge trigger for external interrupt 0 (FIFOP). */
  #define CC2420_FIFOP_INT_INIT() do {                  \
      CC2420_FIFOP_PORT(IES) &= ~BV(CC2420_FIFOP_PIN);  \
      CC2420_CLEAR_FIFOP_INT();                         \
  } while(0)

  /* FIFOP on external interrupt 0. */
  #define CC2420_ENABLE_FIFOP_INT()  do {CC2420_FIFOP_PORT(IE) |= BV(CC2420_FIFOP_PIN);} while(0)
  #define CC2420_DISABLE_FIFOP_INT() do {CC2420_FIFOP_PORT(IE) &= ~BV(CC2420_FIFOP_PIN);} while(0)
  #define CC2420_CLEAR_FIFOP_INT()   do {CC2420_FIFOP_PORT(IFG) &= ~BV(CC2420_FIFOP_PIN);} while(0)

  /*
   * Enables/disables CC2420 access to the SPI bus (not the bus).
   * (Chip Select)
   */
   /* ENABLE CSn (active low) */
  #define CC2420_SPI_ENABLE()     (CC2420_CSN_PORT(OUT) &= ~BV(CC2420_CSN_PIN))
   /* DISABLE CSn (active low) */
  #define CC2420_SPI_DISABLE()    (CC2420_CSN_PORT(OUT) |=  BV(CC2420_CSN_PIN))
  #define CC2420_SPI_IS_ENABLED() ((CC2420_CSN_PORT(OUT) & BV(CC2420_CSN_PIN)) != BV(CC2420_CSN_PIN))
#endif /* if 0 */


#endif /* __PLATFORM_CONF_H__ */

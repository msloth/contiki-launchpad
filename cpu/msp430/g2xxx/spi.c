/*
 * Copyright (c) 2013, Marcus Linderoth, http://forfunandprof.it
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/**
 * \file
 *         SPI drivers
 * \author
 *         Marcus Linderoth <linderoth.marcus@gmail.com>
 */

#include "contiki-conf.h"
#include "dev/spi.h"

/*--------------------------------------------------------------------------*/
/*
 * Set spi_busy so that interrupt handlers can check if
 * they are allowed to use the SPI bus or not.
 */
/*volatile uint8_t spi_busy = 0;*/
// XXX no, they can instead poll the BUSY flag, saves a byte :)


/*--------------------------------------------------------------------------*/
/*uint8_t*/
/*spi_tx_byte(uint8_t data)*/
/*{*/

/*  UCB0TXBUF = data;*/
/*  // wait for TX*/
/*  while (!(IFG2 & UCB0TXIFG));*/
/*  return UCB0RXBUF;*/

/*}*/
/*--------------------------------------------------------------------------*/
/*
 * Init SPI on USCI UCB0 as UCA0 is used for UART (printf's)
 */
void
spi_init(void)
{
#if _MCU_ == 2553
#if USE_RADIO
  /*
   * From TIs users manual
   * The recommended USCI initialization/re-configuration process is:
   */
  /** 1. Set UCSWRST (BIS.B #UCSWRST,&UCxCTL1)*/
  UCB0CTL1 = UCSWRST;

  /** 2. Initialize all USCI registers with UCSWRST=1 (including UCxCTL1)*/
  UCB0CTL0 |= UCCKPH | UCMSB | UCMST | UCSYNC | UCMODE_0;
  UCB0CTL1 |= UCSSEL_2;

  /** 3. Configure ports*/
  SPI_PORT(SEL)  |= SPI_MISO | SPI_MOSI | SPI_SCL;
  SPI_PORT(SEL2) |= SPI_MISO | SPI_MOSI | SPI_SCL;

  /** 4. Clear UCSWRST via software (BIC.B #UCSWRST,&UCxCTL1)*/
  UCB0CTL1 &= ~UCSWRST;

  /** 5. Enable interrupts (optional) via UCxRXIE and/or UCxTXIE*/
#endif    /* USE_RADIO */
#endif    /* _MCU_ == 2553 */
}
/*--------------------------------------------------------------------------*/

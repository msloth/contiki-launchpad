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

#ifndef __SPI_H__
#define __SPI_H__

/* the only thing the spi.c module should be responsible of is initiating the spi
module with pins, the rest is handled in the driver for that peripheral. */

/*--------------------------------------------------------------------------*/
void spi_init(void);
/*--------------------------------------------------------------------------*/
/* All pins are on port 1 */
#define SPI_PORT(type)          P1##type
#define SPI_MOSI                (1<<7)
/* XXX NB, pin 1.6 (MISO) also for LED2 so remove jumper for that if you use SPI */
#warning "Pin 1.6 is both for SPI MISO and for LED2, remove jumper if SPI is used"
#define SPI_MISO                (1<<6)
#define SPI_SCL                 (1<<5)
/*--------------------------------------------------------------------------*/
#define SPI_TXBUF               UCB0TXBUF
#define SPI_RXBUF               UCB0RXBUF

#define SPI_WAITFOREOTx()       while ((UCB0STAT & UCBUSY) == 0)
#define SPI_WAITFOREORx()       while ((IFG2 & UCB0RXIFG) == 0)
#define SPI_WAITFORTxREADY()    while ((IFG2 & UCB0TXIFG) == 0)

#define SPI_WAITFORTx_BEFORE() SPI_WAITFORTxREADY()
#define SPI_WAITFORTx_AFTER()
#define SPI_WAITFORTx_ENDED() SPI_WAITFOREOTx()
/*--------------------------------------------------------------------------*/
/* wait until there is data in rx buffer; for rx a dummy write to tx is needed */
#define SPI_WAIT_FOR_RECVD()              while((IFG2 & UCB0RXIFG) == 0){}

/* wait until ready to put another byte in tx buffer; nb does not mean done with
    tx of previous */
#define SPI_WAIT_FOR_TX_RDY()             while((IFG2 & UCB0TXIFG) == 0){}

/* wait until SPI is not in use, ie no SPI transfer is happening */
#define SPI_WAIT_WHILE_BUSY()             while(UCB0STAT & UCBUSY){}

/* tx one byte, wait until next tx byte can be put  */
#define SPI_WRITE(x)        do {                                               \
                              SPI_WAIT_FOR_TX_RDY();                           \
                              SPI_TXBUF = x;                                   \
                              SPI_WAIT_WHILE_BUSY();                           \
                            } while(0)

/* tx but don't block (only non-blocking) until tx done; used in burstwriting */
#define SPI_WRITE_FAST(data)    do {                                           \
                                  SPI_WAIT_FOR_TX_RDY();                       \
                                  SPI_TXBUF = data;                            \
                                } while(0)

/* Read one character from SPI; tx a dummy byte */
#define SPI_READ(data)      do {                                               \
                              SPI_WAIT_FOR_TX_RDY();                           \
                              SPI_TXBUF = 0; /*dummy*/                         \
                              SPI_WAIT_WHILE_BUSY();                           \
                              data = SPI_RXBUF;                                \
                            } while(0)
#endif /* __SPI_H__ */

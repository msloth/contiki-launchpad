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
 *         CC2500 driver header file
 * \author
 *         Marcus Linderoth <linderoth.marcus@gmail.com>
 */

#ifndef __CC2500_H__
#define __CC2500_H__

#include "contiki.h"
#include "dev/spi.h"
#include "dev/radio.h"
#include "dev/cc2500-const.h"

/*--------------------------------------------------------------------------*/
#define CC2500_MAX_PACKET_LEN       64
/*
 * The transmission power setting refers to the vector of settings for txp; 
 * their corresponding dBm value is this (starting with 0x50==-30)
 *    {-30, -26, -18, -10, -4, +1}
 * hence the txp is used like this:
 *  packetbuf_attr(TRANSMISSION_POWER, CC2500_TXPOWER_MAX);
 *  packetbuf_attr(TRANSMISSION_POWER, 3);
 */
//const uint8_t cc2500_txp[] = {0x50, 0x44, 0xC0, 0x84, 0x81, 0x46, 0x93, 0x55, 0x8D, 0xC6, 0x97, 0x6E, 0x7F, 0xA9, 0xBB, 0xFE, 0xFF};
//const uint8_t cc2500_txp[] = {0x50, 0xC0, 0x93, 0x97, 0xA9, 0xFF};
//#define CC2500_TXPOWER(x)           cc2500_txp[x]
#define CC2500_TXPOWER_MAX          5
#define CC2500_TXPOWER_MIN          0

/* SPI interface and helper functionality------- */
#define CC2500_SPI_PORT(type)       P1##type      
/* setting/clearing chip select help */
#define CC2500_SPI_ENABLE()         (CC2500_CSN_PORT(OUT) &= ~CC2500_SPI_CSN_PIN)
#define CC2500_SPI_DISABLE()        (CC2500_CSN_PORT(OUT) |=  CC2500_SPI_CSN_PIN)

/* chip select pin */
#define CC2500_CSN_PORT(type)       P2##type
#define CC2500_SPI_CSN_PIN          (1<<3)

/* the interrupt pin */
#define CC2500_GDO_PORT(type)       P2##type
#define CC2500_GDO_PIN              (1<<5)


/*
  this is the minimum required for the radio driver, must be implemented or stubbed:
  cc2500_init,
  cc2500_prepare,
  cc2500_transmit,
  cc2500_send,
  cc2500_read,
  cc2500_cca,
  cc2500_receiving_packet,
  pending_packet,
  cc2500_on,
  cc2500_off,



  RADIO_TX_OK,
  RADIO_TX_ERR,
  RADIO_TX_COLLISION,
  RADIO_TX_NOACK,


*/

/*
  cc2420 functions that are not implemented for cc2500:
    int cc2500_get_txpower(void);
    void cc2500_set_cca_threshold(int value);
    int cc2500_set_channel(int channel);
    int cc2500_get_channel(void);
    int cc2500_rssi(void);
    void cc2500_set_pan_addr(unsigned pan,
                                    unsigned addr,
                                    const uint8_t *ieee_addr);
*/
extern const struct radio_driver cc2500_driver;
/*--------------------------------------------------------------------------*/
int     cc2500_init(void);
void    cc2500_reset(void);
int     cc2500_interrupt(void);
int     cc2500_on(void);
int     cc2500_off(void);
int     cc2500_send(const void *payload, unsigned short payload_len);

void    cc2500_set_channel(uint8_t c);
uint8_t cc2500_strobe(uint8_t strobe);
uint8_t cc2500_read_single(uint8_t adr);
uint8_t cc2500_read_burst(uint8_t adr, uint8_t *dest, uint8_t len);
uint8_t cc2500_write_single(uint8_t adr, uint8_t data);
uint8_t cc2500_write_burst(uint8_t adr, uint8_t *src, uint8_t len);

#endif /* __CC2500_H__ */


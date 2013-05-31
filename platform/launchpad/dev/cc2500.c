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
 *         CC2500 driver, intentionally simple and naive in order to save RAM/ROM
 * \author
 *         Marcus Linderoth <linderoth.marcus@gmail.com>
 */

#include <string.h>
#include "contiki.h"
#include "net/packetbuf.h"
#include "net/netstack.h"
#include "dev/spi.h"
#include "dev/cc2500.h"
#include "dev/cc2500-arch.h"
#include "dev/cc2500-const.h"
#include "dev/cc2500-config.h"
#include "dev/leds.h"
/*---------------------------------------------------------------------------*/
/* configurations */

/* do a clear channel assessment before sending anything */
// XXX not implemented yet
#define WITH_SEND_CCA             0
#define CCA_BEFORE_TX_TIME        (RTIMER_SECOND / 1000)

/*
 * let the radio do destination address filtering in hardware; will conserve
 * power when used with SimpleRDC as unicasts will be ACKed immediately, letting
 * the sender go to sleep and we can use shorter wake-up periods on all nodes.
 */
/* NB: do not enable this, it is not working yet and will cause packet drops. */
#define USE_HW_ADDRESS_FILTER     0
/*---------------------------------------------------------------------------*/
/* debug settings */
#define DEBUG 0

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define LEDS_ON(x) leds_on(x)
#define LEDS_OFF(x) leds_off(x)
#else
#define PRINTF(...) do {} while (0)
#define LEDS_ON(x)
#define LEDS_OFF(x)
#endif
/*---------------------------------------------------------------------------*/
/* useful macros */
#define BUSYWAIT_UNTIL(cond, max_time)                                  \
  do {                                                                  \
    rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time)));   \
  } while(0)

/*
 * only OOK needs more than one field in PATABLE, the others are fine w one so
 * this works for all but OOK.
 */
#define CC2500_SET_TXPOWER(x)       cc2500_write_single(CC2500_PATABLE, x)

/* check radio status */
#define CC2500_STATUS()   ((cc2500_strobe(CC2500_SNOP) & CC2500_STATUSBYTE_STATUSBITS))

/* used in cc2500 read */
#define CC2500_READ_FIFO_BYTE(data)                                     \
  do {                                                                  \
    CC2500_SPI_ENABLE();                                                \
    SPI_WRITE(CC2500_RXFIFO | 0x40);                                    \
    SPI_READ(data);                                                     \
    CC2500_SPI_DISABLE();                                               \
  } while(0)

/* used in cc2500 read */
#define CC2500_READ_FIFO_BUF(buffer, count)                             \
  do {                                                                  \
    uint8_t i;                                                          \
    CC2500_SPI_ENABLE();                                                \
    SPI_WRITE(CC2500_RXFIFO | CC2500_BURSTREAD);                        \
    for(i = 0; i < (count); i++) {                                      \
      SPI_READ(((uint8_t *)(buffer))[i]);                               \
    }                                                                   \
    CC2500_SPI_DISABLE();                                               \
  } while(0)

/* used in cc2500 prepare */
#define CC2500_WRITE_FIFO_BUF(buffer,count)                             \
  do {                                                                  \
    uint8_t i;                                                          \
    CC2500_SPI_ENABLE();                                                \
    SPI_WRITE_FAST(CC2500_TXFIFO);                                      \
    for(i = 0; i < (count); i++) {                                      \
      SPI_WRITE_FAST(((uint8_t *)(buffer))[i]);                         \
    }                                                                   \
    SPI_WAIT_WHILE_BUSY();                                              \
    CC2500_SPI_DISABLE();                                               \
  } while(0)

/* flush the Rx-/Tx-FIFOs */
#define FLUSH_FIFOS()     do {                                          \
                            cc2500_strobe(CC2500_SIDLE);                \
                            cc2500_strobe(CC2500_SFTX);                 \
                            cc2500_strobe(CC2500_SFRX);                 \
                            cc2500_strobe(CC2500_SRX);                  \
                          } while(0);

/* length of an ACK = address + seq# */
#define ACK_LEN   3

/* flags */
static uint8_t is_on = 0;
static uint8_t should_off = 0;

/* the length of rssi, checksum etc bytes appended by radio to packet */
#define FOOTER_LEN        2   // after the packet, two bytes RSSI+LQI+CRC are appended

#define CC2500_DEFAULT_CONFIG_LEN    47
extern const uint8_t cc2500_default_config[];
extern const uint8_t cc2500_txp[];
/*---------------------------------------------------------------------------*/
/* function prototypes for the radio driver */
       int  cc2500_init(void);
static int  cc2500_prepare(const void *data, unsigned short len);
static int  cc2500_transmit(unsigned short len);
       int  cc2500_send(const void *data, unsigned short len);
static int  cc2500_read(void *buf, unsigned short bufsize);
static int  cc2500_channel_clear(void);
static int  cc2500_receiving_packet(void);
static int  cc2500_pending_packet(void);
       int  cc2500_on(void);
       int  cc2500_off(void);

/* define the radio driver */
const struct radio_driver cc2500_driver =
{
  /** init the radio */
  cc2500_init,

  /** Prepare the radio with a packet to be sent. */
  cc2500_prepare,

  /** Send the packet that has previously been prepared. */
  cc2500_transmit,

  /** Prepare & transmit a packet. */
  /* radio return values.
        RADIO_TX_OK,
        RADIO_TX_ERR,
        RADIO_TX_COLLISION,
        RADIO_TX_NOACK,
  */
  cc2500_send,

  /** Read a received packet into a buffer. */
  cc2500_read,

  /** Perform a Clear-Channel Assessment (CCA) to find out if there is
      a packet in the air or not. */
  cc2500_channel_clear,

  /** Check if the radio driver is currently receiving a packet */
  cc2500_receiving_packet,

  /** Check if the radio driver has just received a packet */
  cc2500_pending_packet,

  /** Turn the radio on. */
  cc2500_on,

  /** Turn the radio off. */
  cc2500_off,
};
/* function prototypes for helper functions--------------- */
#if 0
/* check the status of the radio */
#define CC2500_STATUS()   (cc2500_strobe(CC2500_SNOP) & CC2500_STATUSBYTE_STATUSBITS)
#endif
/*---------------------------------------------------------------------------*/
PROCESS(cc2500_process, "CC2500 driver");
/*---------------------------------------------------------------------------*/
/* turn on radio */
static void
on(void)
{
  cc2500_strobe(CC2500_SRX);
  BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_RX, RTIMER_SECOND / 100);
  is_on = 1;
}
/*---------------------------------------------------------------------------*/
/* turn off radio */
static void
off(void)
{
  /* Wait for transmission to end and not receiving §before turning radio off. */
  BUSYWAIT_UNTIL((CC2500_STATUS() != CC2500_STATE_TX), RTIMER_SECOND / 100);
  BUSYWAIT_UNTIL((CC2500_GDO_PORT(IN) & CC2500_GDO_PIN) == 0, RTIMER_SECOND / 100);

  /* might not have finished transmitting here if something is wrong, so we
   * command it into IDLE anyway. */
  cc2500_strobe(CC2500_SIDLE);
  is_on = 0;
}
/*---------------------------------------------------------------------------*/
void
cc2500_reset(void)
{
  uint8_t i;

  /* reset radio core; after a SRES it takes a little while for the core to get
    ready, keep reading out until chip ready bit is de-asserted */
  cc2500_strobe(CC2500_SRES);
  while(cc2500_read_single(0) & 0x80) { }

  /* do basic default setup; see cc2500-config.h */
  for(i = 0; i < CC2500_DEF_CONF_LEN; i += 2) {
    cc2500_write_single(cc2500_default_config[i], cc2500_default_config[i+1]);
  }

  /* calibrate the freq oscillator */
  cc2500_strobe(CC2500_SIDLE);
  cc2500_set_channel(RF_CHANNEL);
  cc2500_strobe(CC2500_SIDLE);

  /* set channel, txp, addr etc */
  CC2500_SET_TXPOWER(CC2500_DEFAULT_TXPOWER);
  cc2500_strobe(CC2500_SCAL);
  BUSYWAIT_UNTIL(CC2500_STATUS() != CC2500_STATE_CAL, RTIMER_SECOND / 100);

#if USE_HW_ADDRESS_FILTER
  /* write node address and address filter on (ADDR, broadcast 0x00, 0xff) */
  cc2500_write_single(CC2500_ADDR, rimeaddr_node_addr.u8[0]);
  cc2500_write_single(CC2500_PKTCTRL1, 0x0B);
#endif /* USE_HW_ADDRESS_FILTER */

  /* start in rx mode */
  cc2500_strobe(CC2500_SRX);
  is_on = 1;
  should_off = 0;
}
/*---------------------------------------------------------------------------*/
int
cc2500_init(void)
{
  /* Initalize ports and SPI. */
  cc2500_arch_init();

  /* do a default setup of the radio */
  cc2500_reset();

  /* start process that will handle interrupts */
  process_start(&cc2500_process, NULL);
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
cc2500_transmit(unsigned short payload_len)
{
  /* set txp according to the packetbuf attribute */
  if(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) > 0) {
    CC2500_SET_TXPOWER(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER));
  } else {
    CC2500_SET_TXPOWER(packetbuf_attr(CC2500_DEFAULT_TXPOWER));
  }

#if WITH_SEND_CCA
  if(CC2500_STATUS() != CC2500_STATE_RX) {
    cc2500_strobe(CC2500_SRX);
    BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_RX, RTIMER_SECOND / 100);
  }
  // XXX wait a little while (?) then do CCA
  BUSYWAIT_UNTIL(0, CCA_BEFORE_TX_TIME);
  if(!CCA) {
    return RADIO_TX_COLLISION;
  }
#endif /* WITH_SEND_CCA */

  /* transmit: strobe Tx, then wait until transmitting, then wait till done */
  cc2500_strobe(CC2500_STX);
  BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_TX, RTIMER_SECOND / 100);
  BUSYWAIT_UNTIL((CC2500_STATUS() != CC2500_STATE_TX), RTIMER_SECOND / 100);
  return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
static int
cc2500_prepare(const void *payload, unsigned short payload_len)
{
  /* Write packet to TX FIFO after flushing it. First byte is total length
    (hdr+data), not including the length byte. */
  cc2500_strobe(CC2500_SIDLE);
  cc2500_strobe(CC2500_SFTX);

  cc2500_write_burst(CC2500_TXFIFO, (uint8_t *)&payload_len, 1);
#if USE_HW_ADDRESS_FILTER
  /* write destination address high byte to FIFO to use HW address filtering */
  {
    rimeaddr_t dest = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);
    cc2500_write_burst(CC2500_TXFIFO, &(dest.u8[0]), 1);
  }
#endif /* USE_HW_ADDRESS_FILTER */
  cc2500_write_burst(CC2500_TXFIFO, (uint8_t*) payload, payload_len);
  return 0;
}
/*---------------------------------------------------------------------------*/
int
cc2500_send(const void *payload, unsigned short payload_len)
{
  cc2500_prepare(payload, payload_len);
  return cc2500_transmit(payload_len);
}
/*---------------------------------------------------------------------------*/
int
cc2500_off(void)
{
  /* Don't do anything if we are already turned off. */
  if(!is_on) {
    return 1;
  }

  /* if we are transmitting or currently recieving, don't turn off */
  if(CC2500_STATUS() == CC2500_STATE_TX || (CC2500_GDO_PORT(IN) & CC2500_GDO_PIN)) {
    should_off = 1;
    return 0;
  }

  off();
  return 1;
}
/*---------------------------------------------------------------------------*/
int
cc2500_on(void)
{
  /* if we are transmitting or currently receiving, don't call on() */
  if(0) {
  // if(is_on) {
/*  if(CC2500_STATUS() != CC2500_STATE_IDLE) {*/
    return 1;
  }

  should_off = 0;
  on();
  return 1;
}
/*---------------------------------------------------------------------------*/
void
cc2500_set_channel(uint8_t c)
{
  /* Wait for any ev transmission to end and any receiving to end. */
  BUSYWAIT_UNTIL(CC2500_STATUS() != CC2500_STATE_TX, RTIMER_SECOND / 100);
  BUSYWAIT_UNTIL((CC2500_GDO_PORT(IN) & CC2500_GDO_PIN) == 0, RTIMER_SECOND / 100);

  /* need to be in Idle or off, stable. */
  if(CC2500_STATUS() != CC2500_STATE_IDLE) {
    cc2500_strobe(CC2500_SIDLE);
  }

  /* write channel setting */
  cc2500_write_single(CC2500_CHANNR, c);

  /* calibrate oscillator for this new freq */
  cc2500_strobe(CC2500_SCAL);
  BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_IDLE, RTIMER_SECOND / 100);

  /* turn on radio if it was on */
  if(is_on) {
    on();
  } else {
    off();
  }
  return;
}
/*---------------------------------------------------------------------------*/
/* this is called from the interrupt service routine; polls the radio process
  which in turn reads the packet from the radio when it runs. The reason for
  this is to avoid blocking the radio from the ISR. */
volatile uint8_t pending_rxfifo = 0;  // XXX
int
cc2500_interrupt(void)
{
  pending_rxfifo++;
  process_poll(&cc2500_process);
  return 1;
}
/*---------------------------------------------------------------------------*/
/*static struct timer radio_check_timer;*/
PROCESS_THREAD(cc2500_process, ev, data)
{
  PROCESS_BEGIN();

/*  timer_set(&radio_check_timer, CLOCK_SECOND);*/
  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    /* We end up here after a radio GDO port ISR -> interrupt handler -> poll process */
    static uint8_t len = 0;
    PRINTF("CC2500 polled\n");
    // pending_rxfifo--;    /* mli: TODO: pendingfix */

    cc2500_read_burst(CC2500_RXBYTES, &len, 1);

    /* overflow in RxFIFO, drop all */
    if(len & 0x80) {
      FLUSH_FIFOS();
      PRINTF("Overflow;F\n");

    } else if(len > 0) {
      /* prepare packetbuffer */
      packetbuf_clear();

      /* read length of packet (first FIFO byte) then pass to higher layers */
      len = cc2500_read(packetbuf_dataptr(), PACKETBUF_SIZE);
      if(len > 0) {
        packetbuf_set_datalen(len);
        NETSTACK_RDC.input();
        /* re-poll the radio process so it can check for any packet received while
          we were handling this one (it will check the rxfifo for data). */
/*          cc2500_interrupt();*/
      } else {
        /* no received data or bad data that was dropped. Do nothing. */
      }
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
static int
cc2500_read(void *buf, unsigned short bufsize)
{
  uint8_t footer[2];
  uint8_t len, dest;

  PRINTF("CC2500:r\n");

  len = cc2500_read_single(CC2500_RXBYTES);
  if(pending_rxfifo == 0) {
    /* we don't have anything in the FIFO (ie no interrupt has fired) */
    if(len > 0) {
      FLUSH_FIFOS();
    }
    return 0;
  }
  pending_rxfifo = 0;   /* TODO: mli pendingfix */

  /* check for FIFO overflow */
  if(len & 0x80) {
    /* overflow in RxFIFO, drop all */
    FLUSH_FIFOS();
    PRINTF("Overflow;F\n");
    return 0;
  } else if(len == 0) {
    /* nothing in buffer */
    return 0;
  }

  /* first byte in FIFO is length of the packet with no appended footer */
  cc2500_read_burst(CC2500_RXFIFO, &len, 1);
  PRINTF("%u B\n", len);

#if USE_HW_ADDRESS_FILTER
  /* get destination address high byte to determine if to be ACKed (later) */
  cc2500_read_burst(CC2500_RXFIFO, &dest, 1);
  PRINTF("%u\n", dest);
#endif /* USE_HW_ADDRESS_FILTER */


  /* Check size; too small (ie no real "data") -> drop it */
  if(len == 0) {
    FLUSH_FIFOS();
    PRINTF("No data;F\n");
    return 0;
  }

  /* corrupt size -> drop it */
  if(len > CC2500_MAX_PACKET_LEN) {
    FLUSH_FIFOS();
    PRINTF("Bad len;F\n");
    return 0;
  }

  /* Check size; too big for buffer -> drop it */
  if(len > bufsize) {
/*  if(len - FOOTER_LEN > bufsize) {*/
    FLUSH_FIFOS();
    PRINTF("Too big(%u);F\n", bufsize);
    return 0;
  }

  /* read the packet data from RxFIFO, put in packetbuf */
  CC2500_READ_FIFO_BUF(buf, len);

  /* read automatically appended data (RSSI, LQI, CRC ok) */
  if(FOOTER_LEN > 0) {
    CC2500_READ_FIFO_BUF(footer, FOOTER_LEN);
    if(footer[1] & FOOTER1_CRC_OK) {
      /* set attributes: RSSI and LQI so they can be read out from packetbuf */
      packetbuf_set_attr(PACKETBUF_ATTR_RSSI, footer[0]);
      packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, footer[1] & FOOTER1_LQI);
    } else {
      /* CRC fail -> drop packet */
      FLUSH_FIFOS();
      PRINTF("CRC fail;F\n");
      return 0;
    }
  }

  /* check for FIFO overflow or if anything else is left in FIFO */
  if(CC2500_STATUS() == CC2500_STATE_RXFIFO_OVERFLOW) {
    FLUSH_FIFOS();
  } else {
    uint8_t len = cc2500_read_single(CC2500_RXBYTES);
    if(len > 0) {
       //XXX another packet in buffer, handle it
      pending_rxfifo++;
      process_poll(&cc2500_process);
    }
  }

#if USE_HW_ADDRESS_FILTER
  /*
   * check if this is a unicast, if so we ACK it (not broadcast or to us are
   * already filtered out by radio HW).
   * Broadcasts are sent to address XX
   *
   *
   * move earlier to save sender energy? if not interferes with reading rxfifo
   * --no, we should do the previous checks to see that it was received properly
   */
  if(dest == rimeaddr_node_addr.u8[0]) {
    uint8_t ab[ACK_LEN];  // ACK-buffer

    ab[0] = rimeaddr_node_addr.u8[0];
    ab[1] = rimeaddr_node_addr.u8[1];
    ab[2] = tx_serial;    // XXX what's here

    // XXX send the ACK
    PRINTF("CC2500 Sent ACK!\n");
  }
#endif /* USE_HW_ADDRESS_FILTER */

  return len;
}
/*---------------------------------------------------------------------------*/
#if 0
void
cc2500_set_txpower(uint8_t power)
{
  cc2500_write_single(CC2500_PATABLE, power);
}
#endif
/*---------------------------------------------------------------------------*/
/* read and return current RSSI */
int
cc2500_rssi(void)
{
  int rssi;
  int radio_was_off = 0;

/*  if(locked) {*/
/*    return 0;*/
/*  }*/

  if(!is_on) {
    radio_was_off = 1;
    cc2500_on();    // XXX on()? no, the cc2500_on will wait for stable on (?)
  }

  rssi = (int)((signed char)cc2500_read_single(CC2500_RSSI));

  if(radio_was_off) {
    cc2500_off();
  }
  return rssi;
}
/*---------------------------------------------------------------------------*/
int
cc2500_channel_clear(void)
{
  volatile uint8_t cca = 0;
  uint8_t radio_was_off = 0;

  /* If the radio is locked by an underlying thread (because we are
     being invoked through an interrupt), we pretend that the coast is
     clear (i.e., no packet is currently being transmitted by a
     neighbor). */
/*  if(locked) {*/
/*    return 1;*/
/*  }*/

  if(!is_on) {
    radio_was_off = 1;
    cc2500_on();
  }

  /* read CCA */
  cca = cc2500_read_single(CC2500_PKTSTATUS);

  if(radio_was_off) {
    cc2500_off();
  }

  /* XXX : bug, was always returning false strangely enough, so manually override it here. */
  /* should read out eg 0x30 where CCA is 1 << 4, and saw this on the logic analyzer too.... */
  if(1) {
  // if(cca & PKTSTATUS_CCA) {
    return 1;
  }
  return 0;

}
/*---------------------------------------------------------------------------*/
int
cc2500_receiving_packet(void)
{
  /* if GDO is high, it means we are either receiving or sending a packet */
  if((CC2500_GDO_PORT(IN) & CC2500_GDO_PIN) && (CC2500_STATUS() != CC2500_STATE_TX)) {
    return 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
cc2500_pending_packet(void)
{
  return (int) cc2500_read_single(CC2500_RXBYTES);
  // return pending_rxfifo;
}
/*--------------------------------------------------------------------------*/
uint8_t
cc2500_strobe(uint8_t strobe)
{
  uint8_t s;
  /* sth (the button?) sets the MISO pin so lets reset all pins we need */
  SPI_PORT(SEL)  |= SPI_MISO | SPI_MOSI | SPI_SCL;
  SPI_PORT(SEL2) |= SPI_MISO | SPI_MOSI | SPI_SCL;

  CC2500_SPI_ENABLE();
  SPI_WRITE(strobe);
  s = SPI_RXBUF;
  CC2500_SPI_DISABLE();
  return s;
}
/*---------------------------------------------------------------------------*/
uint8_t
cc2500_read_single(uint8_t adr)
{
  uint8_t d;
  /* sth (the button?) sets the MISO pin so lets reset all pins we need */
  SPI_PORT(SEL)  |= SPI_MISO | SPI_MOSI | SPI_SCL;
  SPI_PORT(SEL2) |= SPI_MISO | SPI_MOSI | SPI_SCL;

  if(adr >= CC2500_PARTNUM) {
    /* these regs can only be read in burst mode and one at a time */
    cc2500_read_burst(adr, &d, 1);
    return d;
  } else {
    CC2500_SPI_ENABLE();
    SPI_WRITE(adr | CC2500_READ);
    SPI_READ(d);
    CC2500_SPI_DISABLE();
    return d;
  }
}
/*---------------------------------------------------------------------------*/
uint8_t
cc2500_read_burst(uint8_t adr, uint8_t *dest, uint8_t len)
{
  uint8_t s, i;
  /* sth (the button?) sets the MISO pin so lets reset all pins we need */
  SPI_PORT(SEL)  |= SPI_MISO | SPI_MOSI | SPI_SCL;
  SPI_PORT(SEL2) |= SPI_MISO | SPI_MOSI | SPI_SCL;

  CC2500_SPI_ENABLE();
  SPI_WRITE(adr | CC2500_BURSTREAD);
  s = SPI_RXBUF;
  for(i = 0; i < len; i += 1) {
    SPI_READ(dest[i]);
  }
  CC2500_SPI_DISABLE();
  return s;
}
/*---------------------------------------------------------------------------*/
uint8_t
cc2500_write_single(uint8_t adr, uint8_t data)
{
  uint8_t s;
  /* sth (the button?) sets the MISO pin so lets reset all pins we need */
  SPI_PORT(SEL)  |= SPI_MISO | SPI_MOSI | SPI_SCL;
  SPI_PORT(SEL2) |= SPI_MISO | SPI_MOSI | SPI_SCL;

  CC2500_SPI_ENABLE();
  SPI_WRITE(adr | CC2500_WRITE);
  s = SPI_RXBUF;
  SPI_WRITE(data);
  CC2500_SPI_DISABLE();
  return s;
}
/*---------------------------------------------------------------------------*/
uint8_t
cc2500_write_burst(uint8_t adr, uint8_t *src, uint8_t len)
{
  uint8_t s, i;
  /* sth (the button?) sets the MISO pin so lets reset all pins we need */
  SPI_PORT(SEL)  |= SPI_MISO | SPI_MOSI | SPI_SCL;
  SPI_PORT(SEL2) |= SPI_MISO | SPI_MOSI | SPI_SCL;

  CC2500_SPI_ENABLE();
  SPI_WRITE(adr | CC2500_BURSTWRITE);
  s = SPI_RXBUF;
  for(i = 0; i < len; i += 1) {
    SPI_WRITE_FAST(src[i]);
  }
  SPI_WAIT_WHILE_BUSY();
  CC2500_SPI_DISABLE();
  return s;
}
/*---------------------------------------------------------------------------*/
int
cc2500_radio_ok(void)
{
  if(CC2500_STATUS() == CC2500_STATE_RX) {
    return 0;
  }

  return 1;
}
/*---------------------------------------------------------------------------*/


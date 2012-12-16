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
 *         CC2500 driver, intentionally simple and naive in order to save RAM/ROM
 * \author
 *         Marcus Lunden <marcus.lunden@gmail.com>
 */

#include <string.h>
#include "contiki.h"
#include "net/packetbuf.h"
#include "net/netstack.h"
#include "dev/spi.h"
#include "dev/cc2500.h"
#include "dev/cc2500-const.h"
#include "dev/cc2500-config.h"
#include "dev/leds.h"
/*#include "sys/timetable.h"*/

/*---------------------------------------------------------------------------*/
/* conf defines */
/*#define WITH_SEND_CCA 1*/

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

/* flush the Rx-FIFO */
#define FLUSH_FIFOS()     do {                                          \
                            cc2500_strobe(CC2500_SIDLE);                \
                            cc2500_strobe(CC2500_SFTX);                 \
                            cc2500_strobe(CC2500_SFRX);                 \
                            cc2500_strobe(CC2500_SRX);                  \
                          } while(0);

/*---------------------------------------------------------------------------*/
/* function prototypes for the radio driver */
int         cc2500_init(void);
static int  cc2500_prepare(const void *data, unsigned short len);
static int  cc2500_transmit(unsigned short len);
/*static int  cc2500_send(const void *data, unsigned short len);*/
static int  cc2500_read(void *buf, unsigned short bufsize);
static int  cc2500_cca(void);
static int  cc2500_receiving_packet(void);
static int  cc2500_pending_packet(void);
int         cc2500_on(void);
int         cc2500_off(void);

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
      /* radio return values. */
      /* enum {
        RADIO_TX_OK,
        RADIO_TX_ERR,
        RADIO_TX_COLLISION,
        RADIO_TX_NOACK,
      }; */
  cc2500_send,

  /** Read a received packet into a buffer. */
  cc2500_read,

  /** Perform a Clear-Channel Assessment (CCA) to find out if there is
      a packet in the air or not. */
  cc2500_cca,

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
/* set transmission power */
#define CC2500_SET_TXPOWER(x)       cc2500_write_single(CC2500_PATABLE, x)
#endif
/*---------------------------------------------------------------------------*/
/* variables and other stuff*/
/*signed char                 cc2500_last_rssi;*/
/*static volatile uint8_t     pending;*/
/*static volatile uint16_t    last_packet_timestamp;*/
/*static uint8_t              receive_on;*/
/*static int                  channel;*/
/*static uint8_t locked, lock_on, lock_off;*/


/* state flags, uses the same numbering as the CC2500 status byte states:
    #define CC2500_STATE_IDLE             0
    #define CC2500_STATE_RX               1
    #define CC2500_STATE_TX               2
    #define CC2500_STATE_FSTXON           3
    #define CC2500_STATE_CAL              4
    #define CC2500_STATE_SETTLING         5
 */
static uint8_t is_on = 0;         // XXX remove later
static uint8_t should_off = 0;    // XXX remove later
static uint8_t is_state = CC2500_STATE_IDLE;    /* current state */
static uint8_t goto_state = CC2500_STATE_IDLE;  /* state to go to when eg tx is done */
/*--------------------------------------------------------------------------*/
/* the length of rssi, checksum etc bytes appended by radio to packet */
#define FOOTER_LEN        2   // after the packet, two bytes RSSI+LQI+CRC are appended

#define CC2500_DEFAULT_CONFIG_LEN    47
extern const uint8_t cc2500_default_config[];
extern const uint8_t cc2500_txp[];
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
  /* Wait for transmission to end before turning radio off. */
  BUSYWAIT_UNTIL((CC2500_STATUS() != CC2500_STATE_TX), RTIMER_SECOND / 100);
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

  /* set channel, set txp */
  CC2500_SET_TXPOWER(CC2500_DEFAULT_TXPOWER);
  cc2500_strobe(CC2500_SCAL);
  BUSYWAIT_UNTIL(CC2500_STATUS() != CC2500_STATE_CAL, RTIMER_SECOND / 100);

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
  uint8_t total_len = 0;
  /* set txp according to the packetbuf attribute */
  if(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) > 0) {
    CC2500_SET_TXPOWER(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER));
  } else {
    CC2500_SET_TXPOWER(packetbuf_attr(CC2500_DEFAULT_TXPOWER));
  }

  /* do a CCA */
/*  if(CC2500_STATUS() != CC2500_STATE_RX) {*/
/*    cc2500_strobe(CC2500_SRX);*/
/*    BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_RX, RTIMER_SECOND / 100);*/
/*  }*/

  /* transmit: strobe Tx, then wait until transmitting, then wait till done */
  cc2500_strobe(CC2500_STX);
  BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_TX, RTIMER_SECOND / 100);
  BUSYWAIT_UNTIL((CC2500_STATUS() != CC2500_STATE_TX), RTIMER_SECOND / 100);
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
cc2500_prepare(const void *payload, unsigned short payload_len)
{
  uint8_t total_len;
  /* Write packet to TX FIFO after flushing it. First byte is total length
    (hdr+data), not including the lenght byte. */
  cc2500_strobe(CC2500_SIDLE);
  cc2500_strobe(CC2500_SFTX);
  total_len = payload_len;
/*  total_len = payload_len + FOOTER_LEN;*/

  cc2500_write_burst(CC2500_TXFIFO, &total_len, 1);
  cc2500_write_burst(CC2500_TXFIFO, payload, payload_len);
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
  if(is_on) {
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
  /* Wait for any ev transmission to end. */
  BUSYWAIT_UNTIL(CC2500_STATUS() != CC2500_STATE_TX, RTIMER_SECOND / 100);

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
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER();
  PROCESS_BEGIN();
  
/*  timer_set(&radio_check_timer, CLOCK_SECOND);*/
  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
/*    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL || timer_expired(&radio_check_timer));*/
/*    if(timer_expired(&radio_check_timer)) {*/
      /*
       * periodically check the radio for error states and if so reset the radio.
       * This takes ca 8 bytes RAM and 100 bytes ROM so if necessary, remove.
       */
/*      uint8_t state = CC2500_STATUS();*/
/*      if(state == CC2500_STATE_RXFIFO_OVERFLOW || state == CC2500_STATE_TXFIFO_UNDERFLOW) {*/
/*        cc2500_reset();*/
/*      }*/
/*      timer_reset(&radio_check_timer);*/
/*    } else {*/

      /* We end up here after a radio GDO port ISR -> interrupt handler -> poll process */
      static uint8_t len = 0;
      PRINTF("CC2500 polled\n");
      pending_rxfifo--;

      cc2500_read_burst(CC2500_RXBYTES, &len, 1);

      /* overflow in RxFIFO, drop all */
      if(len & 0x80) {
        FLUSH_FIFOS();
        PRINTF("Overflow;F\n");

      /* nothing in buffer */
      } else if(len > 0) {
        /* prepare packetbuffer: clear it and set any attributes eg timestamp */
        packetbuf_clear();
        //packetbuf_set_attr(PACKETBUF_ATTR_TIMESTAMP, last_packet_timestamp);

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

    //XXX
/*    }*/
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
static int
cc2500_read(void *buf, unsigned short bufsize)
{
  uint8_t footer[2];
  uint8_t len;

  PRINTF("CC2500:r\n");

  if(pending_rxfifo == 0) {
    /* we don't have anything in the FIFO (ie no interrupt has fired) */
    return 0;
  }

/*  cc2500_read_burst(CC2500_RXBYTES, &len, 1);*/
#if 0
  if(len & 0x80) {
    /* overflow in RxFIFO, drop all */
    FLUSH_FIFOS();
    PRINTF("Overflow;F\n");
    return 0;
  } else if(len == 0) {
    /* nothing in buffer */
    return 0;
  }
#endif /* if 0; commented out code */

  /* first byte in FIFO is length of the packet with no appended footer */
  cc2500_read_burst(CC2500_RXFIFO, &len, 1);
  PRINTF("%u B\n", len);

  /* Check size; too small (ie no real "data") -> drop it */
/*  if(len <= FOOTER_LEN) {*/
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
/*  CC2500_READ_FIFO_BUF(buf, len - FOOTER_LEN);*/

  /* read automatically appended data (RSSI, LQI, CRC ok) */
  if(FOOTER_LEN > 0) {
    CC2500_READ_FIFO_BUF(footer, FOOTER_LEN);
  }

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

  /* check for FIFO overflow or if anything else is left in FIFO */
  if(CC2500_STATUS() == CC2500_STATE_RXFIFO_OVERFLOW) {
    FLUSH_FIFOS();
  } else {
    uint8_t l = cc2500_read_single(CC2500_RXBYTES);
    if(l > 0) {
       //XXX another packet in buffer, handle it 
      pending_rxfifo++;
      process_poll(&cc2500_process);
    }
  }

#if 0
  if(CC2500_FIFOP_IS_1) {
    if(!CC2500_FIFO_IS_1) {
      /* Clean up in case of FIFO overflow!  This happens for every
       * full length frame and is signaled by FIFOP = 1 and FIFO =
       * 0. */
      flushrx();
    }
  }
#endif

  return len;
/*  return len - FOOTER_LEN;*/
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
static int
cc2500_cca(void)
{
  uint8_t cca;
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
  cca = cc2500_read_single(CC2500_PKTSTATUS) & PKTSTATUS_CCA;

  if(radio_was_off) {
    cc2500_off();
  }
  return cca;
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
  return pending_rxfifo;
}
/*--------------------------------------------------------------------------*/
uint8_t
cc2500_strobe(uint8_t strobe)
{
  uint8_t s;
  /* sth sets the MISO pin (button?) so lets reset all pins we need */
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
  /* sth sets the MISO pin (button?) so lets reset all pins we need */
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
  /* sth sets the MISO pin (button?) so lets reset all pins we need */
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
  /* sth sets the MISO pin (button?) so lets reset all pins we need */
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
  /* sth sets the MISO pin (button?) so lets reset all pins we need */
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


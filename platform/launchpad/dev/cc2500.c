#include <string.h>
#include "contiki.h"

#include "net/packetbuf.h"
#include "net/netstack.h"

#include "dev/spi.h"
#include "dev/cc2500.h"
#include "dev/cc2500-const.h"

#include "dev/leds.h"
#include "sys/timetable.h"

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

/*---------------------------------------------------------------------------*/
/* function prototypes for the radio driver */
int         cc2500_init(void);
static int  cc2500_prepare(const void *data, unsigned short len);
static int  cc2500_transmit(unsigned short len);
static int  cc2500_send(const void *data, unsigned short len);
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
/* send a command strobe; returns the status byte */
uint8_t cc2500_strobe(uint8_t strobe);
/* check the status of the radio */
#define CC2500_STATUS()   (cc2500_strobe(CC2500_SNOP) & CC2500_STATUSBYTE_STATUSBITS)
/* read a single register; returns the read register */
uint8_t cc2500_read_single(uint8_t adr);
/* read multiple registers; returns the status byte */
uint8_t cc2500_read_burst(uint8_t adr, uint8_t *dest, uint8_t len);
/* write a single register; returns the read register */
uint8_t cc2500_write_single(uint8_t adr, uint8_t data);
/* write multiple registers; returns the status byte */
uint8_t cc2500_write_burst(uint8_t adr, uint8_t *src, uint8_t len);
/* set transmission power */
#define CC2500_SET_TXPOWER(x)       cc2500_write_single(CC2500_PATABLE, x)

/*---------------------------------------------------------------------------*/
/* variables and other stuff*/
/*signed char                 cc2500_last_rssi;*/
/*static volatile uint8_t     pending;*/
/*static volatile uint16_t    last_packet_timestamp;*/
/*static uint8_t              receive_on;*/
/*static int                  channel;*/
/*static uint8_t locked, lock_on, lock_off;*/

/*---------------------------------------------------------------------------*/
PROCESS(cc2500_process, "CC2500 driver");
/*---------------------------------------------------------------------------*/
/* turn on radio */
static void
on(void)
{
  cc2500_strobe(CC2500_SRX);
  BUSYWAIT_UNTIL(CC2500_STATUS() & CC2500_STATE_RX, RTIMER_SECOND / 100);
}
/*---------------------------------------------------------------------------*/
/* turn off radio */
static void
off(void)
{
  /* Wait for transmission to end before turning radio off. */
  BUSYWAIT_UNTIL(!(CC2500_STATUS() & CC2500_STATE_TX), RTIMER_SECOND / 10);
  /* might not have finished transmitting here if something is wrong, so we
   * command it into IDLE anyway. */
  cc2500_strobe(CC2500_SIDLE);
}
/*---------------------------------------------------------------------------*/
int
cc2500_init(void)
{
  /* Initalize ports and SPI. */
/*  cc2500_arch_init();*/
  /* do basic setup */
  /* start in off mode */
  /* set address, autoack? */
  /* set channel, set txp */
  /* start process that will handle interrupts */
  process_start(&cc2500_process, NULL);
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
cc2500_transmit(unsigned short payload_len)
{
#if 0
  /* set txp according to the packetbuf attribute */
  if(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) > 0) {
    CC2500_SET_TXPOWER(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER));
  } else {
    CC2500_SET_TXPOWER(packetbuf_attr(RADIO_CC2500_DEFAULT_TXPOWER));
  }

  total_len = payload_len + AUX_LEN;

  /* turn on */
  if(not on) {
    cc2500_strobe(CC2500_SRX);
    BUSYWAIT_UNTIL(CC2500_STATUS() & STATE_RX, RTIMER_SECOND / 10);
  }
  /* tx on CCA */
  cc2500_strobe(CC2500_STXCCA);
  BUSYWAIT_UNTIL(!(CC2500_STATUS() & BV(CC2500_STATE_TX)), RTIMER_SECOND / 10);

  /* should it go into Rx and wait for ACK? depends on if unicast */
  cc2500_strobe(CC2500_SRX);
  BUSYWAIT_UNTIL(!(CC2500_STATUS() & BV(CC2500_STATE_TX)), RTIMER_SECOND / 10);
  if(no ack) {
    return NO_ACK;
  }
#endif
  return 0;   // XXX remove
}
/*---------------------------------------------------------------------------*/
static int
cc2500_prepare(const void *payload, unsigned short payload_len)
{
#if 0
  uint8_t total_len;
  printf("ra tx %uB\n", payload_len);

  /* Write packet to TX FIFO. First byte is total length (hdr+data) */
  cc2500_strobe(CC2500_SFLUSHTX);
  total_len = payload_len + AUX_LEN;
  CC2500_WRITE_FIFO_BUF(&total_len, 1);
  CC2500_WRITE_FIFO_BUF(payload, payload_len);
  return 0;
#endif
  return 0;   // XXX remove
}
/*---------------------------------------------------------------------------*/
static int
cc2500_send(const void *payload, unsigned short payload_len)
{
#if 0
  cc2500_prepare(payload, payload_len);
  return cc2500_transmit(payload_len);
#endif
  return 0;   // XXX remove
}
/*---------------------------------------------------------------------------*/
int
cc2500_off(void)
{
#if 0
  /* Don't do anything if we are already turned off. */
  if(receive_on == 0) {
    return 1;
  }

  /* if we are transmitting or currently recieving, don't turn off */
  if(CC2500_STATUS() & BV(CC2500_STATE_TX) || CC2500_STATUS() & BV(CC2500_RX_ACTIVE)) {
    should_off = 1;
    return 1;
  }
  
  off();
  return 1;
#endif
  return 0;   // XXX remove
}
/*---------------------------------------------------------------------------*/
int
cc2500_on(void)
{
#if 0
  /* if we are transmitting or currently recieving, don't call on */
  if(sladfjsldf) {
    return 1;
  }

  should_off = 0;
  on();
  return 1;
#endif
  return 0;   // XXX remove
}
/*---------------------------------------------------------------------------*/
int
cc2500_get_channel(void)
{
  return 0;   // XXX remove
/*  return channel;*/
}
/*---------------------------------------------------------------------------*/
/* remove this? if needed for saving space... */
int
cc2500_set_channel(int c)
{
  /* Wait for any ev transmission to end. */
  BUSYWAIT_UNTIL(!(CC2500_STATUS() & CC2500_STATE_TX), RTIMER_SECOND / 10);

  /* need to be in Idle or off, stable. (?) */
  if(!(CC2500_STATUS() & CC2500_STATE_IDLE)) {
    cc2500_strobe(CC2500_SIDLE);
  }
  BUSYWAIT_UNTIL(CC2500_STATUS() & CC2500_STATE_TX, RTIMER_SECOND / 10);

  /* write channel setting */
  cc2500_write_single(CC2500_CHANNR, c);

  /* calibrate oscillator */
  cc2500_strobe(CC2500_SCAL);

  /* turn on if necessary XXX */
/*  if(should_off) {*/
/*    off();*/
/*  } else {*/
/*    on();*/
/*  }*/

  return 1;
}
/*---------------------------------------------------------------------------*/
/* this is called from the interrupt service routine; polls the radio process
  which in turn reads the packet from the radio when it runs. The reason for 
  this is to avoid blocking the radio from the ISR. */
int
cc2500_interrupt(void)
{
#if 0
  // XXX where is the irq pin cleared? Here or in ISR?
  process_poll(&cc2500_process);

  pending++;
  return 1;
#endif
  return 0;   // XXX remove
}
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(cc2500_process, ev, data)
{
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER();
  PROCESS_BEGIN();

  while(1) {
    static int len = 0;
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    PRINTF("cc2500_process: calling receiver callback\n");

    packetbuf_clear();
    //packetbuf_set_attr(PACKETBUF_ATTR_TIMESTAMP, last_packet_timestamp);

    len = cc2500_read(packetbuf_dataptr(), PACKETBUF_SIZE);
    packetbuf_set_datalen(len);
    NETSTACK_RDC.input();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
static int
cc2500_read(void *buf, unsigned short bufsize)
{
#if 0
  uint8_t footer[2];
  uint8_t len;

  if(!CC2500_FIFOP_IS_1) {
    return 0;
  }
  /*  if(!pending) {
    return 0;
    }*/
  
  pending = 0;
  CC2500_READ_FIFO_BYTE(&len);

  if(len > CC2500_MAX_PACKET_LEN) {
    /* Oops, we must be out of sync. */
    flushrx();
    return 0;
  }

  if(len <= AUX_LEN) {
    flushrx();
    return 0;
  }

  if(len - AUX_LEN > bufsize) {
    flushrx();
    return 0;
  }

  CC2500_READ_FIFO_BUF(buf, len - AUX_LEN);
  CC2500_READ_FIFO_BUF(footer, FOOTER_LEN);

  if(footer[1] & FOOTER1_CRC_OK) {
    cc2500_last_rssi = footer[0];
    cc2500_last_correlation = footer[1] & FOOTER1_CORRELATION;
    packetbuf_set_attr(PACKETBUF_ATTR_RSSI, cc2500_last_rssi);
    packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, cc2500_last_correlation);
  } else {
    len = AUX_LEN;
  }

  if(CC2500_FIFOP_IS_1) {
    if(!CC2500_FIFO_IS_1) {
      /* Clean up in case of FIFO overflow!  This happens for every
       * full length frame and is signaled by FIFOP = 1 and FIFO =
       * 0. */
      flushrx();
    } else {
      /* Another packet has been received and needs attention. */
      process_poll(&cc2500_process);
    }
  }

  if(len < AUX_LEN) {
    return 0;
  }
  return len - AUX_LEN;
#endif
  return 0;   // XXX remove
}
/*---------------------------------------------------------------------------*/
#if 0
void
cc2500_set_txpower(uint8_t power)
{
/*   only OOK needs more than one field in PATABLE, the others are fine w one. */
  cc2500_write_single(CC2500_PATABLE, power);
}
#endif
/*---------------------------------------------------------------------------*/
/* read and return current RSSI */
int
cc2500_rssi(void)
{
#if 0
  int rssi;
  int radio_was_off = 0;

  if(locked) {
    return 0;
  }
  
  if(!receive_on) {
    radio_was_off = 1;
    cc2500_on();
  }
  BUSYWAIT_UNTIL(CC2500_STATUS() & BV(CC2500_RSSI_VALID), RTIMER_SECOND / 100);

  rssi = (int)((signed char)cc2500_read_single(CC2500_RSSI));

  if(radio_was_off) {
    cc2500_off();
  }
  return rssi;
#endif
  return 0;   // XXX remove
}
/*---------------------------------------------------------------------------*/
static int
cc2500_cca(void)
{
  int cca;
  int radio_was_off = 0;

#if 0
  /* If the radio is locked by an underlying thread (because we are
     being invoked through an interrupt), we pretend that the coast is
     clear (i.e., no packet is currently being transmitted by a
     neighbor). */
  if(locked) {
    return 1;
  }
#endif

/*  if(!receive_on) {*/
/*    radio_was_off = 1;*/
/*    cc2500_on();*/
/*  }*/

  /* Make sure that the radio really got switched on */
/*  if(!receive_on) {*/
/*    if(radio_was_off) {*/
/*      cc2500_off();*/
/*    }*/
/*    return 1;*/
/*  }*/
/*  BUSYWAIT_UNTIL(CC2500_STATUS() & BV(CC2500_RSSI_VALID), RTIMER_SECOND / 100);*/

  /* read CCA */
  // XXX

  if(radio_was_off) {
    cc2500_off();
  }
  return cca;
}
/*---------------------------------------------------------------------------*/
int
cc2500_receiving_packet(void)
{
#if 0
  return CC2500_SFD_IS_1;
#endif
  return 0;   // XXX remove
}
/*---------------------------------------------------------------------------*/
static int
cc2500_pending_packet(void)
{
#if 0
  return CC2500_FIFOP_IS_1;
#endif
  return 0;   // XXX remove
}
/*--------------------------------------------------------------------------*/
uint8_t
cc2500_strobe(uint8_t strobe)
{
  uint8_t s;
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
  uint8_t s;
  CC2500_SPI_ENABLE();
  SPI_WRITE(adr | CC2500_READ);
  SPI_WRITE(0);
  s = SPI_RXBUF;
  CC2500_SPI_DISABLE();
  return s;
}
/*---------------------------------------------------------------------------*/
uint8_t
cc2500_read_burst(uint8_t adr, uint8_t *dest, uint8_t len)
{
  uint8_t s, i;
  CC2500_SPI_ENABLE();
  SPI_WRITE(adr | CC2500_BURSTREAD);
  s = SPI_RXBUF;
  for(i = 0; i < len; i += 1) {
    SPI_WRITE(0);
    dest[i] = SPI_RXBUF;
  }
  CC2500_SPI_DISABLE();
  return s;
}
/*---------------------------------------------------------------------------*/
uint8_t
cc2500_write_single(uint8_t adr, uint8_t data)
{
  uint8_t s;
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
  CC2500_SPI_ENABLE();
  SPI_WRITE(adr | CC2500_BURSTWRITE);
  s = SPI_RXBUF;
  for(i = 0; i < len; i += 1) {
    SPI_WRITE(src[i]);
  }
  CC2500_SPI_DISABLE();
  return s;
}
/*---------------------------------------------------------------------------*/





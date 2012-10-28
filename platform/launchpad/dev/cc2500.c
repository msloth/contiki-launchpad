#include <string.h>
#include "contiki.h"

#include "dev/leds.h"
#include "dev/spi.h"
#include "dev/cc2500.h"
#include "dev/cc2500-const.h"

#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include "net/netstack.h"

#include "sys/timetable.h"

/*---------------------------------------------------------------------------*/
/* conf defines */
#define WITH_SEND_CCA 1

#ifndef CC2500_CONF_AUTOACK
#define CC2500_CONF_AUTOACK 0
#endif /* CC2500_CONF_AUTOACK */

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
/* function prototypes, define the radio driver */

int         cc2500_init(void);
static int  cc2500_prepare(const void *data, unsigned short len);
static int  cc2500_transmit(unsigned short len);
static int  cc2500_send(const void *data, unsigned short len);
static int  cc2500_read(void *buf, unsigned short bufsize);
static int  cc2500_cca(void);
static int  cc2500_receiving_packet(void);
static int  pending_packet(void);
int         cc2500_on(void);
int         cc2500_off(void);

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
  pending_packet,

  /** Turn the radio on. */
  cc2500_on,

  /** Turn the radio off. */
  cc2500_off,
};

/*---------------------------------------------------------------------------*/
/* variables and other stuff*/
signed char                 cc2500_last_rssi;

static uint8_t volatile     pending;
static volatile uint16_t    last_packet_timestamp;
static uint8_t              receive_on;
static int                  channel;

/*---------------------------------------------------------------------------*/
PROCESS(cc2500_process, "CC2500 driver");
/*---------------------------------------------------------------------------*/
#if 0
static void
getrxbyte(uint8_t *byte)
{
  CC2500_READ_FIFO_BYTE(*byte);
}
#endif
/*---------------------------------------------------------------------------*/
static void
flushrx(void)
{
#if 0
  uint8_t dummy;

  CC2500_READ_FIFO_BYTE(dummy);
  CC2500_STROBE(CC2500_SFLUSHRX);
  CC2500_STROBE(CC2500_SFLUSHRX);
#endif
}
/*---------------------------------------------------------------------------*/
static uint16_t
status(void)
{
#if 0
  uint8_t status;
  CC2500_GET_STATUS(status);
  return status;
#endif
  return 0;   // XXX remove
}
/*---------------------------------------------------------------------------*/
static uint8_t locked, lock_on, lock_off;

/* turn on radio */
static void
on(void)
{
#if 0
  CC2500_STROBE(CC2500_SRXON);
  BUSYWAIT_UNTIL(status() & (BV(CC2500_XOSC16M_STABLE)), RTIMER_SECOND / 100);
#endif
}
/*---------------------------------------------------------------------------*/
/* turn off radio */
static void
off(void)
{
#if 0
  /* Wait for transmission to end before turning radio off. */
  BUSYWAIT_UNTIL(!(status() & BV(CC2500_TX_ACTIVE)), RTIMER_SECOND / 10);
  CC2500_STROBE(CC2500_SIDLE);
#endif
}
/*---------------------------------------------------------------------------*/
enum cc2500_register {
  sdfsdf = 0,
};
static uint8_t
getreg(enum cc2500_register regname)
{
#if 0
  uint8_t reg;
  CC2500_READ_REG(regname, reg);
  return reg;
#endif
  return 0;   // XXX remove
}
/*---------------------------------------------------------------------------*/
static void
setreg(enum cc2500_register regname, unsigned value)
{
#if 0
  CC2500_WRITE_REG(regname, value);
#endif
}
/*---------------------------------------------------------------------------*/
static void
set_txpower(uint8_t power)
{
#if 0
  setreg(CC2500_TXCTRL, reg);
#endif
}
/*---------------------------------------------------------------------------*/
int
cc2500_init(void)
{
#if 0
  /* Initalize ports and SPI. */
  cc2500_arch_init();		
  /* do basic setup */
  /* start in off mode */
  /* set address, autoack? */
  /* set channel, set txp */
  /* start process that will handle interrupts */
  process_start(&cc2500_process, NULL);
#endif
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
cc2500_transmit(unsigned short payload_len)
{
#if 0
  /* set txp according to the packetbuf attribute */
  if(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) > 0) {
    set_txpower(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER));
  } else {
    set_txpower(packetbuf_attr(RADIO_CC2500_DEFAULT_TXPOWER));
  }

  total_len = payload_len + AUX_LEN;

  /* turn on */
  if(not on) {
    CC2500_STROBE(ON);
    BUSYWAIT_UNTIL(status() & STATE_RX, RTIMER_SECOND / 10);
  }
  /* tx on CCA */
  CC2500_STROBE(CC2500_STXONCCA);
  BUSYWAIT_UNTIL(!(status() & BV(CC2500_TX_ACTIVE)), RTIMER_SECOND / 10);

  /* should it go into Rx and wait for ACK? depends on if unicast */
  CC2500_STROBE(CC2500_SRXON);
  BUSYWAIT_UNTIL(!(status() & BV(CC2500_TX_ACTIVE)), RTIMER_SECOND / 10);
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
  CC2500_STROBE(CC2500_SFLUSHTX);
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
  if(status() & BV(CC2500_TX_ACTIVE) || status() & BV(CC2500_RX_ACTIVE)) {
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
#if 0
  return channel;
#endif
  return 0;   // XXX remove
}
/*---------------------------------------------------------------------------*/
int
cc2500_set_channel(int c)
{
#if 0
  /* Wait for any ev transmission to end. */
  BUSYWAIT_UNTIL(!(status() & BV(CC2500_TX_ACTIVE)), RTIMER_SECOND / 10);

  /* need to be in Idle or off, stable. (?) */
  if(in on) {
    CC2500_STROBE(idle);
  }
  BUSYWAIT_UNTIL((status() & (BV(CC2500_XOSC16M_STABLE))), RTIMER_SECOND / 10);

  setreg(channel, c);

  /* calibrate osc */
  CC2500_STROBE(SCAL);

  /* turn on if necessary */
  if(should_off) {
    off();
  } else {
    on();
  }

  return 1;
#endif
  return 0;   // XXX remove
}
/*---------------------------------------------------------------------------*/
void
cc2500_set_pan_addr(unsigned pan,
                    unsigned addr,
                    const uint8_t *ieee_addr)
{
#if 0
  uint16_t f = 0;
  uint8_t tmp[2];
  /*
   * Writing RAM requires crystal oscillator to be stable.
   */
  BUSYWAIT_UNTIL(status() & (BV(CC2500_XOSC16M_STABLE)), RTIMER_SECOND / 10);

  tmp[0] = pan & 0xff;
  tmp[1] = pan >> 8;
  CC2500_WRITE_RAM(&tmp, CC2500RAM_PANID, 2);

  tmp[0] = addr & 0xff;
  tmp[1] = addr >> 8;
  CC2500_WRITE_RAM(&tmp, CC2500RAM_SHORTADDR, 2);
  if(ieee_addr != NULL) {
    uint8_t tmp_addr[8];
    /* LSB first, MSB last for 802.15.4 addresses in CC2500 */
    for (f = 0; f < 8; f++) {
      tmp_addr[7 - f] = ieee_addr[f];
    }
    CC2500_WRITE_RAM(tmp_addr, CC2500RAM_IEEEADDR, 8);
  }
#endif
}
/*---------------------------------------------------------------------------*/
int
cc2500_interrupt(void)
{
#if 0
  CC2500_CLEAR_FIFOP_INT();
  process_poll(&cc2500_process);

  last_packet_timestamp = cc2500_sfd_start_time;
  pending++;
  cc2500_packets_seen++;
  return 1;
#endif
  return 0;   // XXX remove
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc2500_process, ev, data)
{
  int len;
  PROCESS_BEGIN();

  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
#if 0    
    PRINTF("cc2500_process: calling receiver callback\n");

    packetbuf_clear();
    packetbuf_set_attr(PACKETBUF_ATTR_TIMESTAMP, last_packet_timestamp);
    len = cc2500_read(packetbuf_dataptr(), PACKETBUF_SIZE);
    
    packetbuf_set_datalen(len);
    
    NETSTACK_RDC.input();
#endif
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
void
cc2500_set_txpower(uint8_t power)
{
#if 0
  set_txpower(power);
#endif
}
/*---------------------------------------------------------------------------*/
int
cc2500_get_txpower(void)
{
#if 0
  int power;
  power = (int)(getreg(CC2500_TXCTRL) & 0x001f);
  return power;
#endif
  return 0;   // XXX remove
}
/*---------------------------------------------------------------------------*/
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
  BUSYWAIT_UNTIL(status() & BV(CC2500_RSSI_VALID), RTIMER_SECOND / 100);

  rssi = (int)((signed char)getreg(CC2500_RSSI));

  if(radio_was_off) {
    cc2500_off();
  }
  return rssi;
#endif
  return 0;   // XXX remove
}
/*---------------------------------------------------------------------------*/
int
cc2500_cca_valid(void)
{
#if 0
  int valid;
  if(locked) {
    return 1;
  }
  valid = !!(status() & BV(CC2500_RSSI_VALID));
  return valid;
#endif
  return 0;   // XXX remove
}
/*---------------------------------------------------------------------------*/
static int
cc2500_cca(void)
{
#if 0
  int cca;
  int radio_was_off = 0;

  /* If the radio is locked by an underlying thread (because we are
     being invoked through an interrupt), we preted that the coast is
     clear (i.e., no packet is currently being transmitted by a
     neighbor). */
  if(locked) {
    return 1;
  }

  if(!receive_on) {
    radio_was_off = 1;
    cc2500_on();
  }

  /* Make sure that the radio really got turned on. */
  if(!receive_on) {
    if(radio_was_off) {
      cc2500_off();
    }
    return 1;
  }

  BUSYWAIT_UNTIL(status() & BV(CC2500_RSSI_VALID), RTIMER_SECOND / 100);

  cca = CC2500_CCA_IS_1;

  if(radio_was_off) {
    cc2500_off();
  }
  return cca;
#endif
  return 0;   // XXX remove
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
pending_packet(void)
{
#if 0
  return CC2500_FIFOP_IS_1;
#endif
  return 0;   // XXX remove
}
/*---------------------------------------------------------------------------*/
void
cc2500_set_cca_threshold(int value)
{
#if 0
  uint16_t shifted = value << 8;
  setreg(CC2500_RSSI, shifted);
#endif
  return;
}
/*---------------------------------------------------------------------------*/

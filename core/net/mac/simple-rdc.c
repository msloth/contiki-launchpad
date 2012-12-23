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
 *         A simple radio duty cycling layer, meant to be a low-complexity and
 *         low-RAM alternative to ContikiMAC.
 *         
 * \author
 *         Marcus Lunden <marcus.lunden@gmail.com>
 */

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
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         Implementation of the ContikiMAC power-saving radio duty cycling protocol
 * \author
 *         Adam Dunkels <adam@sics.se>
 *         Niclas Finne <nfi@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 */


#include <stdio.h>
#include <string.h>
#include "contiki.h"
#include "contiki-conf.h"
#include "net/rime.h"
#include "net/netstack.h"
#include "net/mac/simple-rdc.h"
#include "dev/radio.h"
#include "dev/watchdog.h"
#include "dev/leds.h"
#include "sys/pt.h"
#include "sys/rtimer.h"
#include "sys/clock.h"


#define HEADER 1



#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) PRINTF(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#if 0
    | = on/tx
    - = on/Rx
    _ = off


    Normal duty cycling, to traffic

        ____________+--+____________+--+____________+--+____________
                            tOFF     tON



    Transmitting

        ____________+--+______|_|_|_|_|_|_|_|_|_|___+--+____________
                                               ^tBTx
                              |-------tTX-------|


    tON     SIMPLERDC_ONTIME
    tOFF    deduced from SIMPLERDC_ONTIME and SIMPLERDC_CHECKRATE
    tTX     deduced from SIMPLERDC_CHECKRATE and SIMPLERDC_ONTIME, for sending 
                little longer than just the sleep-period
    tBTx    AFTER_TX_HOLDOFF, the time it takes for a packet to be received and
                ACKed.

#endif /* if 0; commented out code */

/*---------------------------------------------------------------------------*/
/* SimpleRDC simple configuration */
#define SIMPLERDC_CHECKRATE       8                         // in Hz, power of two
#define SIMPLERDC_ONTIME          (CLOCK_SECOND / 64)       // 15 ms

/* deduced configurations and definitions, not to be changed */
#define SIMPLERDC_OFFTIME         (CLOCK_SECOND / SIMPLERDC_CHECKRATE - SIMPLERDC_ONTIME)
#define BETWEEN_TX_TIME           ((7ul * RTIMER_SECOND) / 1000)   // 10 ms as we wait for ACK from the polled process
/*10*(RTIMER_SECOND/1000)*/
// for how long to transmit (broadcast, *casts stop at ACK)
#define TX_PERIOD                 ((CLOCK_SECOND / SIMPLERDC_CHECKRATE) + (2 * SIMPLERDC_ONTIME))

// if in tx and waiting for an ACK, and we detect traffic, we wait this long to see if it is indeed an ACK
#define ACK_DETECT_WAIT_TIME      ((2 * RTIMER_SECOND)/1000)








/* LISTEN_TIME_AFTER_PACKET_DETECTED is the time that we keep checking
   for activity after a potential packet has been detected by a CCA
   check. */
#define LISTEN_TIME_AFTER_PACKET_DETECTED  RTIMER_ARCH_SECOND / 80


/*---------------------------------------------------------------------------*/
#if HEADER
/* SimpleRDC header */
struct hdr {
  rimeaddr_t receiver;
  uint8_t seqnr;
};
#endif /* if 0; commented out code */

// keep a record of the last few received packets, 6 B per sender; for duplicate packet detection
struct seqno {
  rimeaddr_t sender;
  uint8_t seqno;
};

#ifdef NETSTACK_CONF_MAC_SEQNO_HISTORY
#define MAX_SEQNOS NETSTACK_CONF_MAC_SEQNO_HISTORY
#else /* NETSTACK_CONF_MAC_SEQNO_HISTORY */
#define MAX_SEQNOS 2
#endif /* NETSTACK_CONF_MAC_SEQNO_HISTORY */
static struct seqno received_seqnos[MAX_SEQNOS];

#define ACK_LEN 3
/*---------------------------------------------------------------------------*/
static volatile uint8_t simplerdc_is_on = 0;
static volatile uint8_t simplerdc_keep_radio_on = 0;
static volatile uint8_t we_are_sending = 0;
static volatile uint8_t radio_is_on = 0;
/*---------------------------------------------------------------------------*/
#define BUSYWAIT_UNTIL(cond, max_time)                                      \
      do {                                                                  \
        rtimer_clock_t t0;                                                  \
        t0 = RTIMER_NOW();                                                  \
        while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time)));   \
      } while(0)

#ifndef MIN
#define MIN(a, b) ((a) < (b)? (a) : (b))
#endif /* MIN */
/*---------------------------------------------------------------------------*/
// XXX to remove later when sorted out where they are used and for what effect

/* Radio does CSMA and autobackoff */
#ifdef RDC_CONF_HARDWARE_CSMA
#if RDC_CONF_HARDWARE_CSMA
#warning "SimpleRDC assumes no hardware CSMA present, does explicit check. Just FYI."
#endif
#endif

/* Radio returns TX_OK/TX_NOACK after autoack wait */
#ifndef RDC_CONF_HARDWARE_ACK
#define RDC_CONF_HARDWARE_ACK        0
#endif


/* BURST_RECV_TIME is the maximum time a receiver waits for the
   next packet of a burst when FRAME_PENDING is set. */
#define INTER_PACKET_DEADLINE               CLOCK_SECOND / 32

/* CCA_SLEEP_TIME is the time between two successive CCA checks. */
/* Add 1 when rtimer ticks are coarse */
#if RTIMER_ARCH_SECOND > 8000
#define CCA_SLEEP_TIME                     RTIMER_ARCH_SECOND / 2000
#else
#define CCA_SLEEP_TIME                     (RTIMER_ARCH_SECOND / 2000) + 1
#endif

/* Are we currently receiving a burst? */
/*static uint8_t we_are_receiving_burst = 0;*/
/* Has the receiver been awoken by a burst we're sending? */
/*static uint8_t is_receiver_awake = 0;*/

/*---------------------------------------------------------------------------*/
static void
on(void)
{
  if(simplerdc_is_on && radio_is_on == 0) {
    radio_is_on = 1;
    NETSTACK_RADIO.on();
  }
}
/*---------------------------------------------------------------------------*/
static void
off(void)
{
  if(simplerdc_is_on && radio_is_on != 0 &&
     simplerdc_keep_radio_on == 0) {
    radio_is_on = 0;
    NETSTACK_RADIO.off();
  }
}
/*---------------------------------------------------------------------------*/
/* called with, from 'send one packet', int ret = send_packet(sent, ptr, NULL);*/
static int
send_packet(mac_callback_t mac_callback, void *mac_callback_ptr, struct rdc_buf_list *buf_list)
{
  uint8_t is_broadcast = 0;
  uint8_t is_reliable = 0;
  int hdrlen;
  clock_time_t end_of_tx;
  int ret;
#if HEADER
  struct hdr *chdr;
#endif /* if 0; commented out code */

  // a serial number that is used for duplicate packet detection (->drop)
  static uint8_t tx_serial = 1;

  PRINTF("SimpleRDC send\n");

  /* sanity checks ---------------------------------------------------------- */
  /* Exit if RDC and radio were explicitly turned off */
  /* XXX allow send even though RDC &/| radio is off? */
  if (!simplerdc_is_on && !simplerdc_keep_radio_on) {
    PRINTF("simplerdc: radio is turned off\n");
    return MAC_TX_ERR_FATAL;
  }
  /* bad length */
  if(packetbuf_totlen() == 0) {
    PRINTF("simplerdc: send_packet data len 0\n");
    return MAC_TX_ERR_FATAL;
  }

  /* prepare for transmission ----------------------------------------------- */
  packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &rimeaddr_node_addr);

  // is this that packet needs to be ACKed? then yes, for unicasts but that is done automatically so don't use this
/*  packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);*/

//--------------------------------------------------------------- header
  /* simplerdc header, like contikimac header, used to identify packets so they
      can be ACK'ed. Useful for eg unicasts to end a tx-train early and conserve
      some energy */
#if HEADER
/* with contikimac header */
  hdrlen = packetbuf_totlen();
  if(packetbuf_hdralloc(sizeof(struct hdr)) == 0) {
    /* Failed to allocate space for contikimac header */
    PRINTF("simplerdc: send failed, too large header\n");
    return MAC_TX_ERR_FATAL;
  }
  chdr = packetbuf_hdrptr();
  chdr->receiver.u8[0] = packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[0];
  chdr->receiver.u8[1] = packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[1];
  chdr->seqnr = tx_serial;
  tx_serial++;
  
  /* Create the MAC header for the data packet. */
  hdrlen = NETSTACK_FRAMER.create();
  if(hdrlen < 0) {
    /* Failed to send */
    PRINTF("simplerdc: send failed, too large header\n");
    packetbuf_hdr_remove(sizeof(struct hdr));
    return MAC_TX_ERR_FATAL;
  }
  hdrlen += sizeof(struct hdr);
//--------------------------------------------------------------- /header
#else
/*old, before header-stuff*/
  if(NETSTACK_FRAMER.create() < 0) {
    /* Failed to allocate space for headers */
    PRINTF("simplerdc: send failed, too large header\n");
    return MAC_TX_ERR_FATAL;
  }
#endif /* if 0; commented out code */

  /* let radio copy to TXFIFO and do what it needs to do */
  NETSTACK_RADIO.prepare(packetbuf_hdrptr(), packetbuf_totlen());

  /* check to see if broadcast, in which case we won't look for ACKs */
  if(rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER), &rimeaddr_null)) {
    is_broadcast = 1;
    PRINTF("simplerdc: broadcast\n");
  } else {
    is_broadcast = 0;
    PRINTF("simplerdc: unicast to %u.%u\n", packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0], packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[1]);
  }

  if(NETSTACK_RADIO.receiving_packet() || (!is_broadcast && NETSTACK_RADIO.pending_packet())) {
    /*
     * Currently receiving a packet or the radio has packet that needs to be
     * read before sending not-broadcast, as an ACK would overwrite the buffer
     */
    return MAC_TX_COLLISION;
  }

  /* transmit --------------------------------------------------------------- */
  /* make sure the medium is clear */
  if(NETSTACK_RADIO.channel_clear() == 0) {
    return MAC_TX_COLLISION;
  }

  /* transmit the packet repeatedly, and check for ACK if not broadcast */
  end_of_tx = clock_time() + TX_PERIOD;
  while(clock_time() <= end_of_tx) {
/*    volatile rtimer_clock_t tstart;*/
    watchdog_periodic();

    /* transmit */
    ret = NETSTACK_RADIO.transmit(packetbuf_totlen());
    if(ret == RADIO_TX_COLLISION) {
      return MAC_TX_COLLISION;
    } else if(ret == RADIO_TX_ERR) {
      return MAC_TX_ERR;
    }

    /* either turn off to save power, or wait for ACK */
    if(is_broadcast) {
      off();
    } else {
      on();
    }
    
    /* wait between transmissions */
/*    tstart = RTIMER_NOW();*/
/*    while(RTIMER_CLOCK_LT(RTIMER_NOW(), tstart + BETWEEN_TX_TIME)) { }*/
    BUSYWAIT_UNTIL(0, BETWEEN_TX_TIME);

    /* if we are hoping for an ACK, check for that here */
    if(!is_broadcast) {
      if(NETSTACK_RADIO.receiving_packet() || NETSTACK_RADIO.pending_packet() ||
            NETSTACK_RADIO.channel_clear() == 0) {
        if(NETSTACK_RADIO.receiving_packet()) {
          /* wait until any transmissions should be over */
          BUSYWAIT_UNTIL(0, ACK_DETECT_WAIT_TIME);
        }
        
        /* see if it is an ACK to us */
        if(NETSTACK_RADIO.pending_packet()) {
          uint8_t ab[ACK_LEN];  // ACK-buffer
          uint8_t len;
          len = NETSTACK_RADIO.read(ab, ACK_LEN);
          if(len == ACK_LEN && 
                ab[0] == packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0] &&
                ab[1] == packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[1] &&
                ab[2] == tx_serial) {
            /* ACK received */
            PRINTF("Got ACK!\n");
            return MAC_TX_OK;
          } else {
            /* Not an ACK or ACK not correct: collision, ie someone else is transmitting at the same time */
            return MAC_TX_COLLISION;
          }
        }

      }
    } /* /checking for ACK between transmissions */
  }   /* /repeated transmissions */
  
  off();

  return MAC_TX_OK;
}
/*---------------------------------------------------------------------------*/
static void
qsend_packet(mac_callback_t sent, void *ptr)
{
  int ret = send_packet(sent, ptr, NULL);
  PRINTF("SimpleRDC qsend\n");
  if(ret != MAC_TX_DEFERRED) {
    mac_call_sent_callback(sent, ptr, ret, 1);
  }
}
/*---------------------------------------------------------------------------*/
static void
qsend_list(mac_callback_t sent, void *ptr, struct rdc_buf_list *buf_list)
{
  struct rdc_buf_list *curr = buf_list;
  struct rdc_buf_list *next;
  int ret;

  PRINTF("SimpleRDC qsend_list\n");
  if(curr == NULL) {
    return;
  }

  do {
    /* A loop sending a burst of packets from buf_list */
    next = list_item_next(curr);

    /* Prepare the packetbuf */
    queuebuf_to_packetbuf(curr->buf);

    /* Send the current packet */
    ret = send_packet(sent, ptr, curr);
    if(ret != MAC_TX_DEFERRED) {
      mac_call_sent_callback(sent, ptr, ret, 1);
    }

    if(ret == MAC_TX_OK) {
      if(next != NULL) {
        curr = next;
      }
    } else {
      next = NULL;
    }
  } while(next != NULL);
}
/*---------------------------------------------------------------------------*/
static void
input_packet(void)
{

  off();
  PRINTF("Input packet\n");

  if(packetbuf_totlen() > 0 && NETSTACK_FRAMER.parse() >= 0) {

#if HEADER
    struct hdr *chdr;
    chdr = packetbuf_dataptr();
    PRINTF("simplerdc: got header %u %u %u\n", chdr->receiver.u8[0], chdr->receiver.u8[1], chdr->seqnr);

    packetbuf_hdrreduce(sizeof(struct hdr));
    packetbuf_set_datalen(packetbuf_totlen());    // XXX ???
#endif /* if 0; commented out code */


    if(packetbuf_datalen() > 0 && packetbuf_totlen() > 0 &&
       (rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                     &rimeaddr_node_addr) ||
        rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                     &rimeaddr_null))) {
      /* This is a packet to us or a broadcast */

      /* Check for duplicate packet by comparing the sequence number
         of the incoming packet with the last few ones we saw. */
      {
        uint8_t i;

        for(i = 0; i < MAX_SEQNOS; ++i) {
          /* check if duplicate packet, drop if so */
          if(chdr->receiver.u8[0] == received_seqnos[i].sender.u8[0] &&
             chdr->receiver.u8[1] == received_seqnos[i].sender.u8[1] &&
             chdr->seqnr == received_seqnos[i].seqno) {

            /* Drop the packet. */
            PRINTF("Drop duplicate simplerdc layer packet %u\n", packetbuf_attr(PACKETBUF_ATTR_PACKET_ID));
            return;
          }
        }

        /* new packet, add it to the list of seen packets to avoid getting it again */
        for(i = MAX_SEQNOS - 1; i > 0; --i) {
          memcpy(&received_seqnos[i], &received_seqnos[i - 1], sizeof(struct seqno));
        }
        received_seqnos[0].seqno = chdr->seqnr;
        rimeaddr_copy(&received_seqnos[0].sender, packetbuf_addr(PACKETBUF_ADDR_SENDER));

        /* respond to unicasts with ACK so the sender can finish early */
        if(rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER), &rimeaddr_node_addr)) {
          uint8_t ab[ACK_LEN];  // ACK-buffer
          ab[0] = rimeaddr_node_addr.u8[0];
          ab[1] = rimeaddr_node_addr.u8[1];
          ab[2] = packetbuf_attr(PACKETBUF_ATTR_PACKET_ID);
          NETSTACK_RADIO.send(ab, ACK_LEN);
        }
      }

      PRINTF("simplerdc: data (%u)\n", packetbuf_datalen());
      NETSTACK_MAC.input();
      return;



    } else {
      PRINTF("simplerdc: data not for us\n");
    }
  } else {
    PRINTF("simplerdc: failed to parse (%u)\n", packetbuf_totlen());
  }
}
/*---------------------------------------------------------------------------*/
// XXX
int8_t
after_on_check(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
static struct etimer powercycle_timer;
PROCESS(simplerdc_process, "SimpleRDC process");
PROCESS_THREAD(simplerdc_process, ev, data)
{
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER();
  PROCESS_BEGIN();
  while(1) {
    int8_t chk;
    on();
    etimer_set(&powercycle_timer, SIMPLERDC_ONTIME);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&powercycle_timer));
    
    /* check to see if we can turn off the radio or not here */
    /* the following can happen when we get here:
          nothing was received. We end up here and turn off as check returns 0
          sth was received but CRC failed. The radio never got anything more. Check == 0
          sth was received and CRC OK! Check == len. On receive and CRC OK, then it is automatically off() to avoid overwriting ok buffer.
          we are currently receiving or downloading FIFO, then Check must return -1 or sth to distinguish and this postponed a little while
          we are currently transmitting, Check returns -2;
     */
    chk = after_on_check();   // XXX
    if(chk == 0) {
      // here, we have not received anything during on(), or CRC failed.
      off();
    } else if(chk == -1) {
      // downloading FIFO, don't interrupt it
/*      while(after_on_check() == -1) {*/
/*        PROCESS_PAUSE();*/
/*      }*/
    } else if(we_are_sending) {
      // currently txing
    } else if(chk > 0) {
      // there is a packet in the buffer, and we don't touch it; sent up from elsewhere
    }
    
    // check the radio to see that it is in good shape (overflow etc)
    //NETSTACK_RADIO.check();
    etimer_set(&powercycle_timer, SIMPLERDC_OFFTIME);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&powercycle_timer));
  }
  PROCESS_END();
}
 
/*---------------------------------------------------------------------------*/
static void
init(void)
{
  PRINTF("SimpleRDC starting\n");
  radio_is_on = 0;
  process_start(&simplerdc_process, NULL);
  simplerdc_is_on = 1;
}
/*---------------------------------------------------------------------------*/
static int
turn_on(void)
{
  if(simplerdc_is_on == 0) {
    simplerdc_is_on = 1;
    simplerdc_keep_radio_on = 0;
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
turn_off(int keep_radio_on)
{
  simplerdc_is_on = 0;
  if(keep_radio_on > 0) {
    radio_is_on = 1;
    simplerdc_keep_radio_on = 1;
    return NETSTACK_RADIO.on();
  } else {
    radio_is_on = 0;
    simplerdc_keep_radio_on = 0;
    return NETSTACK_RADIO.off();
  }
}
/*---------------------------------------------------------------------------*/
static unsigned short
duty_cycle(void)
{
  return CLOCK_SECOND / SIMPLERDC_CHECKRATE;
}
/*---------------------------------------------------------------------------*/
const struct rdc_driver simplerdc_driver = {
  "SimpleRDC",
  init,
  qsend_packet,
  qsend_list,
  input_packet,
  turn_on,
  turn_off,
  duty_cycle,

#if FOR_REFERENCE_ONLY
/**
 * The structure of a RDC (radio duty cycling) driver in Contiki.
 */
struct rdc_driver {
  char *name;

  /** Initialize the RDC driver */
  void (* init)(void);

  /** Send a packet from the Rime buffer  */
  void (* send)(mac_callback_t sent_callback, void *ptr);

  /** Send a packet list */
  void (* send_list)(mac_callback_t sent_callback, void *ptr, struct rdc_buf_list *list);

  /** Callback for getting notified of incoming packet. */
  void (* input)(void);

  /** Turn the MAC layer on. */
  int (* on)(void);

  /** Turn the MAC layer off. */
  int (* off)(int keep_radio_on);

  /** Returns the channel check interval, expressed in clock_time_t ticks. */
  unsigned short (* channel_check_interval)(void);
};
#endif /* if 0; commented out code */

};
/*---------------------------------------------------------------------------*/
uint16_t
simplerdc_debug_print(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/

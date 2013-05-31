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
 *         A simple radio duty cycling layer, meant to be a low-complexity and
 *         low-RAM alternative to ContikiMAC.
 *
 * \author
 *         Marcus Linderoth <linderoth.marcus@gmail.com>
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
#include "lib/random.h"

/* soon to be deprecated, used during pending-packet radio driver bug. */
#define PENDINGBUG_WORKAROUND   0
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
#if 0
Short explanation of SimpleRDC:
  All nodes with SimpleRDC have their radio off and wake up periodically, turning
  the radio on and listen for traffic for a short period of time. To transmit to
  a destination node, we repeatedly transmit the same packet in full until one
  of two things happen:
    1) a full period is covered
    2) for unicasts, we receive an ACK from the destination, meaning it is
       received and the sender do not have to send any more, thus it can go back
       to sleep early
  The transmissions are timed to the wake-up interval and stay-awake-time so
  that at least two transmissions are performed during a node wake-up. If it
  wakes up and hears traffic, yet do not receive a complete packet within a set
  period, it goes down to sleep again.

  ACKs are sent either by the radio hardware automatically, in the radio driver,
  or from SimpleRDC. The speed (and hence efficiency) of which this happens is,
  from faster to slower, following the same order.

  SimpleRDC is designed to be lean and simple and stems originally from several
  other radio duty cycling protocols: nullrdc, ContikiMAC, Thingsquare Drowsie,
  IEEE 802.15.4e CSL. To reduce size and complexity, lots of functionality was
  removed, such as support for uIP, neighbor tables to track wake-up phases and
  more.

  The radio is assumed to start in Rx mode, and return to Rx after recieving
  or sending data, hence the radio is explicitly turned off when needed to.
  Also, as we assume small packet buffers and simple radio hardware, there is
  no support for streaming data (ie infinite packet length), burst transmissions
  (staying awake to send many packets immediately after eachother).

  SimpleRDC is configured with a few settings, as below. The most basic ones are
    * how often to wake up (channel check rate)
    * how long to stay awake (on-time)
    * do SimpleRDC, the radio driver, or the radio hw send ACKs?
    * how long to wait for an ACK after sending
  The timings dependa on things like radio bitrate, packet sizes, transfer
  time mcu <-> radio etc. The default settings are optimized for a 250kbps, SPI-
  based radio such as the CC2420 or the CC2500 sending less than 64 byte packets.
  Note that some radios or radio drivers by default do filtering or automatic
  handling which may interfere. For CC2420, auto-ack must be turned off.

  Example power consumption:
    a simple setup of one node simply idle listening and one node sending a
    unicast every third second, the radio duty cycle is as follows.
    Conf
      #define NETSTACK_CONF_MAC     nullmac_driver
      #define NETSTACK_CONF_RDC     simplerdc_driver
      #define NETSTACK_CONF_FRAMER  framer_nullmac
    Without unicast ACKs (so the sender can finish early)
        sender      10.7 % (of which ca 1.4 % is in transmission)
        listener     5.9 %
    With unicast ACKs in SimpleRDC
      COOJA failed to produce statistics strangely enough, but on average the overhead
      for sending should be on average half the sending period, and listener little
      more, thus approximately this:
        sender       7.5 %
        listener     6.5 %
    With unicast ACKs in radio hardware
      this should be even less

  Example code size, the application above
    with contikiMAC, phase optimization etc
       text	   data	    bss	    dec	    hex	filename
      18456	    176	   4912	  23544	   5bf8	dumb.sky
    with simpleRDC
       text	   data	    bss	    dec	    hex	filename
      15892	    160	   4074	  20126	   4e9e	dumb.sky


    Normal duty cycling, to traffic

                    +--+            +--+            +--+
        ____________|  |____________|  |____________|  |____________
                            tOFF     tON

    | = on/tx
    - = on/Rx
    _ = off


    Transmitting
                    +--+      | | | | | | | | | |   +--+
        ____________|  |______|_|_|_|_|_|_|_|_|_|___|  |____________
                                               ^tBTx
                              |-------tTX-------|

  /* let radio copy to TXFIFO and do what it needs to do */
  NETSTACK_RADIO.prepare(packetbuf_hdrptr(), packetbuf_totlen());


    tON     SIMPLERDC_ONTIME
    tOFF    deduced from SIMPLERDC_ONTIME and SIMPLERDC_CHECKRATE
    tTX     deduced from SIMPLERDC_CHECKRATE and SIMPLERDC_ONTIME, for sending
                little longer than just the sleep-period
    tBTx    AFTER_TX_HOLDOFF, the time it takes for a packet to be received and
                ACKed.

--------------------------
This is the time it takes to transmit X bytes (total, as seen by radio) at 250 kbaud

    len (B)      t (ms)
    ======       ======
      3           0.20      == length of an ACK
      10          0.42      time to copy to TXFIFO: 30 us
      20          0.74      time to copy to TXFIFO: 52 us
      30          1.06      time to copy to TXFIFO: 73 us
      40          1.38      time to copy to TXFIFO: 95 us
      50          1.70      time to copy to TXFIFO: 116 us
      60          2.02      time to copy to TXFIFO: 138 us

Ie, time to receive a packet, copy ACK->fifo, send ACK, would be < 0.30 ms, but
  do we need to wait for a process to be polled as well? Should be rather fast,
  not the 7.8 ms of a 128 Hz sys clock.

#endif /* if 0; commented out code */
/*---------------------------------------------------------------------------*/
/*
 * Enable unicast ACKs from SimpleRDC as opposed to the radio layer, or not at all.
 *
 * if the radio doesn't ACK (either in hardware or at radio layer), then we can
 * ACK here so that a sender can stop sending early. If ACKs are transmitted at
 * neither place, the sender will keep on sending repeatedly for an entire
 * TX_PERIOD time, hence early unicast ACKs are primarily a way to save energy,
 * not (in the current implementation at least) for adding reliability as
 * the sender do not have any mechanism to retransmit at this layer.
 * This ACK will take longer time than at radio layer, so it's less efficient.
 *
 * Thus, set to 1 if radio layer/hardware does not ACK by itself.
 * Set to 0 if radio handles ACKs, or if you are unsure. It will work, but be
 * less efficient.
 *
 * On CC2420, radio HW ACKs are supported if packet is framed as an 802.15.4
 * hence in contiki-conf.h:
 *    #define NETSTACK_CONF_FRAMER      framer_802154
 *    #define CC2420_CONF_AUTOACK       1
 *
 */
/* note, this is close to become deprecated so don't use it. */
#define RADIO_ACKS_UNICASTS               0

/* ------------------------- SimpleRDC configuration ----------------------- */
/* at what rate to wake up and check for traffic, in Hz, use power of 2 (1,2,4...) */
#define SIMPLERDC_CHECKRATE               8

/* Channel sample period */
#define SIMPLERDC_PERIOD                  (CLOCK_SECOND / SIMPLERDC_CHECKRATE)

/* time in 'on'-mode during a channel sample */
#define SIMPLERDC_BWU_ON                  (4 * (RTIMER_SECOND / 1000))

/*
 * Time waiting for ACK between transmissions before starting the next.
 * When sending, the transmitting device will send the frame repeatedly.
 * This is the time between two such transmissions. It is related to how
 * long time it takes to receive and ACK, and the channel sample time.
 */
#define BETWEEN_TX_WAITACK_TIME           (2 * (RTIMER_SECOND / 1000))

/* for how long to transmit (broadcast, *casts stop at ACK). */
#define TX_GUARDTIME                      (CLOCK_SECOND / 128)
#define TX_PERIOD                         (SIMPLERDC_PERIOD + TX_GUARDTIME)

/* If set, performs a CSMA before transmission (NB no backoffs, just drops if collision) */
#define PERFORM_CSMA_BEFORE_TX            1
#define SIMPLERDC_CSMA_TIME (3 * (RTIMER_SECOND/1000))

/*
 * if in tx and waiting for an ACK, and we detect traffic, we wait this long to
 * see if it is indeed an ACK
 */
#define ACK_TX_DETECTED_WAIT_TIME         ((2ul * RTIMER_SECOND)/1000)

#define SIMPLERDC_POWERCYCLE_RX_TIMEOUT   (10 * (RTIMER_SECOND/1000))

/* Do the radio driver do CSMA and autobackoff? */
#if 0
#ifdef RDC_CONF_HARDWARE_CSMA
#if RDC_CONF_HARDWARE_CSMA
#warning "SimpleRDC assumes no hardware CSMA present, does explicit check. Just FYI."
#endif
#endif
#endif /* if 0; commented out code */
/*---------------------------------------------------------------------------*/
/* serial number for duplicate frame detection */
static volatile uint8_t tx_serial;

/* SimpleRDC header, for ACKs */
struct hdr {
  rimeaddr_t receiver;
  uint8_t seqnr;
};
#define ACK_LEN       3

/* keep a record of the last few received packets, 3 B per sender; for duplicate packet detection. */
struct seqno {
  rimeaddr_t sender;
  uint8_t seqno;
};

/* number of sequence numbers to keep track of */
#ifdef NETSTACK_CONF_MAC_SEQNO_HISTORY
#define MAX_SEQNOS NETSTACK_CONF_MAC_SEQNO_HISTORY
#else /* NETSTACK_CONF_MAC_SEQNO_HISTORY */
#define MAX_SEQNOS 2
#endif /* NETSTACK_CONF_MAC_SEQNO_HISTORY */
static struct seqno received_seqnos[MAX_SEQNOS];
/*---------------------------------------------------------------------------*/
/* some flags */
static volatile uint8_t simplerdc_is_on = 0;
static volatile uint8_t simplerdc_keep_radio_on = 0;
static volatile uint8_t we_are_sending = 0;
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
/* turn on SimpleRDC */
static void
on(void)
{
  if(simplerdc_is_on) {
    NETSTACK_RADIO.on();
  }
}
/*---------------------------------------------------------------------------*/
/* turn off SimpleRDC */
static void
off(void)
{
  if(simplerdc_is_on && simplerdc_keep_radio_on == 0) {
    NETSTACK_RADIO.off();
  }
}
/*---------------------------------------------------------------------------*/
/* Send a packet with SimpleRDC */
static int
send_packet(mac_callback_t mac_callback, void *mac_callback_ptr, struct rdc_buf_list *buf_list)
{
  uint8_t is_broadcast = 0;
  uint8_t is_reliable = 0;
  int hdrlen;
  clock_time_t end_of_tx;
  int ret;
  struct hdr *chdr;
  uint8_t gotack = 0;

  /* store setting and set it so that we can use on() and off() */
  uint8_t simplerdc_was_on = simplerdc_is_on;
  simplerdc_is_on = 1;
  PRINTF("SimpleRDC send\n");

  /* sanity checks ---------------------------------------------------------- */
  /* bad length */
  if(packetbuf_totlen() == 0) {
    PRINTF("simplerdc: send_packet data len 0\n");
    simplerdc_is_on = simplerdc_was_on;
    return MAC_TX_ERR_FATAL;
  }

  /* increase sequence number, used for eg dupe packet detection */
  tx_serial++;
  packetbuf_set_attr(PACKETBUF_ATTR_PACKET_ID, tx_serial);
  packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &rimeaddr_node_addr);

#if RADIO_ACKS_UNICASTS
  /* simplerdc header, like contikimac header, used to identify packets so they
      can be ACK'ed. Useful for eg unicasts to end a tx-train early and conserve
      some energy. Prepended a small header to the frame, that is read by the radio
      driver (which sends ACK) and stripped by SimpleRDC on parsing. */
  if(packetbuf_hdralloc(sizeof(struct hdr)) == 0) {
    /* Failed to allocate space for contikimac header */
    PRINTF("simplerdc: send failed, too large header\n");
    simplerdc_is_on = simplerdc_was_on;
    return MAC_TX_ERR_FATAL;
  }
#endif  /* if 0; commented out code */

  /* check to see if broadcast, in which case we won't look for ACKs */
  if(rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER), &rimeaddr_null)) {
    is_broadcast = 1;
    PRINTF("simplerdc: broadcast\n");
  } else {
    is_broadcast = 0;
    PRINTF("simplerdc: unicast to %u.%u\n", packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0], packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[1]);
  }

  /* Create the MAC header for the data packet. */
  hdrlen = NETSTACK_FRAMER.create();
  if(hdrlen < 0) {
    /* Failed to send */
    PRINTF("simplerdc: send failed, too large header\n");
    simplerdc_is_on = simplerdc_was_on;
    return MAC_TX_ERR_FATAL;
  }

#if RADIO_ACKS_UNICASTS
  /* If the radio ACKs, we add a small header that will be stripped at the receiver radio layer */
  chdr = packetbuf_hdrptr();
  chdr->seqnr = tx_serial;
  if(is_broadcast == 0) {
    chdr->receiver.u8[0] = packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0];
    chdr->receiver.u8[1] = packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[1];
  } else {
    /* use broadcast address */
    chdr->receiver.u8[0] = 0x00;
    chdr->receiver.u8[1] = 0x00;
  }
  hdrlen += sizeof(struct hdr);
#endif

#if DEBUG
  PRINTF("simplerdc: data out (%u): ", packetbuf_datalen());
  {
    int i;
    for(i = 0; i < packetbuf_datalen(); i++) {
      PRINTF("%02x", *((uint8_t*)packetbuf_dataptr() + i));
    }
    PRINTF("\n");
  }
#endif /* DEBUG */

#if PENDINGBUG_WORKAROUND
  if(0) {
#else
  if(NETSTACK_RADIO.receiving_packet() || (is_broadcast == 0 && NETSTACK_RADIO.pending_packet())) {
#endif
    /*
     * Currently receiving a packet or the radio has packet that needs to be
     * read before sending not-broadcast, as an ACK would overwrite the buffer
     */
    return MAC_TX_COLLISION;
  }

  /* transmit --------------------------------------------------------------- */
  /* make sure the medium is clear */
#if PENDINGBUG_WORKAROUND
  /* no implem. */
#else     /* PENDINGBUG_WORKAROUND */
#if PERFORM_CSMA_BEFORE_TX
  on();
  BUSYWAIT_UNTIL(NETSTACK_RADIO.receiving_packet() || NETSTACK_RADIO.pending_packet(), SIMPLERDC_CSMA_TIME);
  if(NETSTACK_RADIO.receiving_packet() || NETSTACK_RADIO.pending_packet()) {
    simplerdc_is_on = simplerdc_was_on;
    return MAC_TX_COLLISION;
  }
  off();
#endif    /* PERFORM_CSMA_BEFORE_TX */
#endif    /* PENDINGBUG_WORKAROUND */

  /* transmit the packet repeatedly, and check for ACK if not broadcast */
  end_of_tx = clock_time() + TX_PERIOD;
  while(clock_time() <= end_of_tx) {
    watchdog_periodic();

    /* transmit; copy to radio TXFIFO and send */
#if RADIO_ACKS_UNICASTS
    NETSTACK_RADIO.prepare(packetbuf_hdrptr(), packetbuf_totlen() + sizeof(struct hdr));
    ret = NETSTACK_RADIO.transmit(packetbuf_totlen() + sizeof(struct hdr));
#else
    NETSTACK_RADIO.prepare(packetbuf_hdrptr(), packetbuf_totlen());
    ret = NETSTACK_RADIO.transmit(packetbuf_totlen());
#endif

    if(ret == RADIO_TX_COLLISION) {
      simplerdc_is_on = simplerdc_was_on;
      return MAC_TX_COLLISION;
    } else if(ret == RADIO_TX_ERR) {
      simplerdc_is_on = simplerdc_was_on;
      return MAC_TX_ERR;
    }

    /* wait between transmissions - either turn off to save power, or wait for ACK */
    if(is_broadcast) {
      off();
      BUSYWAIT_UNTIL(0, BETWEEN_TX_WAITACK_TIME);
    } else {
      on();
#if PENDINGBUG_WORKAROUND
      BUSYWAIT_UNTIL(0, BETWEEN_TX_WAITACK_TIME);
#else
      BUSYWAIT_UNTIL(NETSTACK_RADIO.pending_packet(), BETWEEN_TX_WAITACK_TIME);
#endif
    }

    /* if we are waiting for an ACK, check for that here */
    if(is_broadcast == 0) {
      if(NETSTACK_RADIO.receiving_packet() || NETSTACK_RADIO.pending_packet() ||
            NETSTACK_RADIO.channel_clear() == 0) {
        if(NETSTACK_RADIO.receiving_packet()) {
          /* wait until any transmissions should be over */
          BUSYWAIT_UNTIL(NETSTACK_RADIO.receiving_packet() == 0, ACK_TX_DETECTED_WAIT_TIME);
        }

        off();

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
            simplerdc_is_on = simplerdc_was_on;
            return MAC_TX_OK;
          } else {
            /* Not an ACK or ACK not correct: collision, ie someone else is transmitting at the same time */
            PRINTF("SimpleRDC: ACK was not good ACK. Collision.\n");
            simplerdc_is_on = simplerdc_was_on;
#if PENDINGBUG_WORKAROUND
            // skip returning if error    pendingbug
#else
            return MAC_TX_COLLISION;
#endif
          }
        }
      }
    } /* /checking for ACK between transmissions */
  }   /* /repeated transmissions */

  off();
  simplerdc_is_on = simplerdc_was_on;
  if(is_broadcast == 0) {
    /* if unicast and we end up here, we didn't receive an ACK */
    return MAC_TX_NOACK;
  }
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
/* read from the radio a received packet */
static void
input_packet(void)
{
  int len;
  off();

  if(packetbuf_totlen() == 0) {
    /* nothing to parse */
    return;
  }

  len = NETSTACK_FRAMER.parse();
  if(len <= 0) {
    /* bad frame, drop */
    return;
  }

  if(1) {
#if RADIO_ACKS_UNICASTS
    /* remove the SimpleRDC header read out by the radio */
    packetbuf_hdrreduce(sizeof(struct hdr));
    // packetbuf_set_datalen(packetbuf_totlen());    // XXX ???
#endif  /* RADIO_ACKS_UNICASTS */
    packetbuf_hdrreduce(len);

    if(packetbuf_datalen() > 0 && packetbuf_totlen() > 0 &&
       (rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                     &rimeaddr_node_addr) ||
        rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                     &rimeaddr_null))) {
      /* This is a packet to us or a broadcast */
      {
        /* Check for duplicate packet by comparing the sequence number
           of the incoming packet with the last few ones we saw. */
        uint8_t i;

        /* check if duplicate packet, drop if so, else store seq# and sender */
        PRINTF("SimpleRDC: Gonna check %u.%u: %u\n",
          packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[0],
          packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[1],
          packetbuf_attr(PACKETBUF_ATTR_PACKET_ID));
        for(i = 0; i < MAX_SEQNOS; ++i) {
          PRINTF("..against %u.%u: %u\n",
            received_seqnos[i].sender.u8[0],
            received_seqnos[i].sender.u8[1],
            received_seqnos[i].seqno);
          if(rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_SENDER), &received_seqnos[i].sender) &&
              packetbuf_attr(PACKETBUF_ATTR_PACKET_ID) == received_seqnos[i].seqno) {
            /* Drop the packet. */
            PRINTF("Simplerdc: dupe (%u)->drop.\n", packetbuf_attr(PACKETBUF_ATTR_PACKET_ID));
            return;
          }
        }

        /* new packet, add it to the list of seen packets to avoid getting it again */
        for(i = MAX_SEQNOS - 1; i > 0; --i) {
          memcpy(&received_seqnos[i], &received_seqnos[i - 1], sizeof(struct seqno));
        }
        received_seqnos[0].seqno = packetbuf_attr(PACKETBUF_ATTR_PACKET_ID);
        rimeaddr_copy(&received_seqnos[0].sender, packetbuf_addr(PACKETBUF_ADDR_SENDER));

#if !RADIO_ACKS_UNICASTS
        /* respond to unicasts with ACK so the sender can stop tx train early */
        if(rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER), &rimeaddr_node_addr)) {
          uint8_t ab[ACK_LEN];  // ACK-buffer
          ab[0] = rimeaddr_node_addr.u8[0];
          ab[1] = rimeaddr_node_addr.u8[1];
          ab[2] = packetbuf_attr(PACKETBUF_ATTR_PACKET_ID);
          PRINTF("SimpleRDC: ACKing %u from %u.%u\n", ab[2], ab[0], ab[1]);
          NETSTACK_RADIO.send(ab, ACK_LEN);
        }
#endif /* RADIO_ACKS_UNICASTS */
      }

#if DEBUG
      PRINTF("Simplerdc: data in (%u): ", packetbuf_datalen());
      {
        int i;
        for(i = 0; i < packetbuf_datalen(); i++) {
          PRINTF("%02x", *((uint8_t*)packetbuf_dataptr() + i));
        }
        PRINTF("\n");
      }
#endif
      // {
      //   /* keep stats on how many packets we've received */
      //   static uint16_t received_packets = 0;
      //   received_packets++;
      //   printf("%u\n", received_packets);
      // }

      /* pass the received data to next higher layer */
      NETSTACK_MAC.input();
      return;

    } else {
      PRINTF("Simplerdc: data not for us. (%u.%u -> %u.%u)\n",
          packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[0],
          packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[1],
          packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0],
          packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[1]);
    }
  } else {
    PRINTF("simplerdc: failed to parse (%u)\n", packetbuf_totlen());
  }
}
/*---------------------------------------------------------------------------*/
/* do all necessary checks after the radio has been on, to see if we can turn
  it off again */
static int8_t
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

  etimer_set(&powercycle_timer, SIMPLERDC_PERIOD);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&powercycle_timer));
  while(1) {
    int8_t chk;

    /* reset the timer (instead of setting) to reduce risk of drifting */
    etimer_reset(&powercycle_timer);

    /* perform a channel sample, abort early if we receive something */
    on();
    BUSYWAIT_UNTIL(NETSTACK_RADIO.pending_packet(), SIMPLERDC_BWU_ON);
    if(NETSTACK_RADIO.receiving_packet()) {
      BUSYWAIT_UNTIL(NETSTACK_RADIO.receiving_packet() == 0, SIMPLERDC_POWERCYCLE_RX_TIMEOUT);
    }
    off();

    if(cc2500_radio_ok() == 0) {
      cc2500_reset();
      off();
    }

    #if 0
    chk = after_on_check();
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
    #endif  /* if 0; commented out code */

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&powercycle_timer));
  }
  PROCESS_END();
}

/*---------------------------------------------------------------------------*/

static void
init(void)
{
  PRINTF("SimpleRDC starting\n");

  tx_serial = random_rand();

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
    simplerdc_keep_radio_on = 1;
    return NETSTACK_RADIO.on();
  } else {
    simplerdc_keep_radio_on = 0;
    return NETSTACK_RADIO.off();
  }
}
/*---------------------------------------------------------------------------*/
static unsigned short
channel_check_interval(void)
{
  return CLOCK_SECOND / SIMPLERDC_CHECKRATE;
}
/*---------------------------------------------------------------------------*/
const struct rdc_driver simplerdc_driver = {
  /* Name of the driver */
  "SimpleRDC",
  /** Initialize the RDC driver */
  init,
  /** Send a packet from the Rime buffer  */
  qsend_packet,
  /** Send a packet list */
  qsend_list,
  /** Callback for getting notified of incoming packet. */
  input_packet,
  /** Turn the MAC layer on. */
  turn_on,
  /** Turn the MAC layer off. */
  turn_off,
  /** Returns the channel check interval, expressed in clock_time_t ticks. */
  channel_check_interval,
};
/*---------------------------------------------------------------------------*/
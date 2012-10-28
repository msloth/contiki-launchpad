/**
 * \defgroup launchpad-platform TI MSP430 Launchpad Contiki port.
 *
 * TI MSP430 Launchpad Contiki port.
 *
 * @{
 */

#ifndef CONTIKI_CONF_H
#define CONTIKI_CONF_H

#ifdef PLATFORM_CONF_H
#include PLATFORM_CONF_H
#else
#include "platform-conf.h"
#endif /* PLATFORM_CONF_H */


/* These can be lowered to reduce RAM size, to a certain point-------------- */
// dev/serial-line.h
#define SERIAL_LINE_CONF_BUFSIZE            32

// g2xxxx/uart1.c
#define TX_INTERRUPT_BUFSIZE_CONF           32
#define RX_INTERRUPT_BUFSIZE_CONF           32


/* Others ---------------------------------------------------- */

/* include the project config */
/* PROJECT_CONF_H might be defined in the project Makefile */
#ifdef PROJECT_CONF_H
#include PROJECT_CONF_H
#endif /* PROJECT_CONF_H */


/* -------------------------------------------------------------------------- */
/* These are assumed to not be of use for the Launchpad w g2553 due to high
 * RAM and ROM constraints. They are included for compatibility reasons, to 
 * keep the compiler from complaining. */
#ifndef NETSTACK_CONF_MAC
#define NETSTACK_CONF_MAC     nullmac_driver
//#define NETSTACK_CONF_RDC     lean_nullmac_driver    // XXX coming
#endif /* NETSTACK_CONF_MAC */

#ifndef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC     nullrdc_driver
//#define NETSTACK_CONF_RDC     lean_nullrdc_driver    // XXX coming
#endif /* NETSTACK_CONF_RDC */

#ifndef NETSTACK_CONF_RADIO
#define NETSTACK_CONF_RADIO   cc2500_driver
#endif /* NETSTACK_CONF_RADIO */

#ifndef NETSTACK_CONF_FRAMER
#define NETSTACK_CONF_FRAMER  nullframer_driver
//#define NETSTACK_CONF_FRAMER  lean_nullframer_driver    // XXX coming
#endif /* NETSTACK_CONF_FRAMER */

#if 0
#ifndef CC2420_CONF_AUTOACK
#define CC2420_CONF_AUTOACK              1
#endif /* CC2420_CONF_AUTOACK */
#endif

#ifndef NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE
#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE 8
#endif /* NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE */

#ifndef RF_CHANNEL
#define RF_CHANNEL              1
#endif /* RF_CHANNEL */

#if !WITH_UIP6
  /* Network setup for non-IPv6 (rime). */
  #define NETSTACK_CONF_NETWORK            rime_driver

  #define COLLECT_CONF_ANNOUNCEMENTS       0
  #define CXMAC_CONF_ANNOUNCEMENTS         0
  #define XMAC_CONF_ANNOUNCEMENTS          0
  #define CONTIKIMAC_CONF_ANNOUNCEMENTS    0

  #ifndef COLLECT_NEIGHBOR_CONF_MAX_COLLECT_NEIGHBORS
  #define COLLECT_NEIGHBOR_CONF_MAX_COLLECT_NEIGHBORS     0
  #endif /* COLLECT_NEIGHBOR_CONF_MAX_COLLECT_NEIGHBORS */

  #ifndef QUEUEBUF_CONF_NUM
  #define QUEUEBUF_CONF_NUM                0
  #endif /* QUEUEBUF_CONF_NUM */

  #ifndef TIMESYNCH_CONF_ENABLED
  #define TIMESYNCH_CONF_ENABLED           0
  #endif /* TIMESYNCH_CONF_ENABLED */

  #if TIMESYNCH_CONF_ENABLED
  /* CC2420 SDF timestamps must be on if timesynch is enabled. */
  #undef CC2420_CONF_SFD_TIMESTAMPS
  #define CC2420_CONF_SFD_TIMESTAMPS       0
  #endif /* TIMESYNCH_CONF_ENABLED */

#else /* WITH_UIP6 */
  /* Network setup for IPv6 */
  #define NETSTACK_CONF_NETWORK sicslowpan_driver

  /* Specify a minimum packet size for 6lowpan compression to be
     enabled. This is needed for ContikiMAC, which needs packets to be
     larger than a specified size, if no ContikiMAC header should be
     used. */
  #define SICSLOWPAN_CONF_COMPRESSION_THRESHOLD 63
  #define CONTIKIMAC_CONF_WITH_CONTIKIMAC_HEADER 0

  #define CXMAC_CONF_ANNOUNCEMENTS         0
  #define XMAC_CONF_ANNOUNCEMENTS          0

  #ifndef QUEUEBUF_CONF_NUM
  #define QUEUEBUF_CONF_NUM                8
  #endif
#endif /* WITH_UIP6 */





/* less interesting (now) configurations------------------------------------ */

/* Specify whether the RDC layer should enable
   per-packet power profiling. */
#define CONTIKIMAC_CONF_COMPOWER         0
#define XMAC_CONF_COMPOWER               0
#define CXMAC_CONF_COMPOWER              0

#define PACKETBUF_CONF_ATTRS_INLINE 1
#define CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT 0

#define IEEE802154_CONF_PANID       0xABCD

#define SHELL_VARS_CONF_RAM_BEGIN 0x1100
#define SHELL_VARS_CONF_RAM_END 0x2000

#define PROFILE_CONF_ON 0
#ifndef ENERGEST_CONF_ON
#define ENERGEST_CONF_ON 0
#endif /* ENERGEST_CONF_ON */

#define ELFLOADER_CONF_TEXT_IN_ROM 0
#ifndef ELFLOADER_CONF_DATAMEMORY_SIZE
#define ELFLOADER_CONF_DATAMEMORY_SIZE 0x400
#endif /* ELFLOADER_CONF_DATAMEMORY_SIZE */
#ifndef ELFLOADER_CONF_TEXTMEMORY_SIZE
#define ELFLOADER_CONF_TEXTMEMORY_SIZE 0x800
#endif /* ELFLOADER_CONF_TEXTMEMORY_SIZE */

#define AODV_COMPLIANCE
#define AODV_NUM_RT_ENTRIES 32

#define WITH_ASCII 1

#define PROCESS_CONF_NUMEVENTS 8
#define PROCESS_CONF_STATS 1
/*#define PROCESS_CONF_FASTPOLL    4*/

#define UIP_CONF_ICMP_DEST_UNREACH 1
#define UIP_CONF_DHCP_LIGHT
#define UIP_CONF_LLH_LEN         0
#ifndef  UIP_CONF_RECEIVE_WINDOW
#define UIP_CONF_RECEIVE_WINDOW  48
#endif
#ifndef  UIP_CONF_TCP_MSS
#define UIP_CONF_TCP_MSS         48
#endif
#define UIP_CONF_MAX_CONNECTIONS 4
#define UIP_CONF_MAX_LISTENPORTS 8
#define UIP_CONF_UDP_CONNS       12
#define UIP_CONF_FWCACHE_SIZE    30
#define UIP_CONF_BROADCAST       1
#define UIP_ARCH_IPCHKSUM        1
#define UIP_CONF_UDP             1
#define UIP_CONF_UDP_CHECKSUMS   1
#define UIP_CONF_PINGADDRCONF    0
#define UIP_CONF_LOGGING         0
#define UIP_CONF_TCP_SPLIT       0

#endif /* CONTIKI_CONF_H */
/** @} */

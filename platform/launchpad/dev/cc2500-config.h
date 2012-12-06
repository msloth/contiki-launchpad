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
 *         CC2500 header file: configuration
 * \author
 *         Marcus Lunden <marcus.lunden@gmail.com>
 */

#ifndef __CC2500_CONFIG_H__
#define __CC2500_CONFIG_H__

/* transmission power setting, 0..16 (17 elements) ranging from lowest to highest */
const uint8_t cc2500_txp[] = {0x50, 0x44, 0xC0, 0x84, 0x81, 0x46, 0x93, 0x55, 0x8D, 0xC6, 0x97, 0x6E, 0x7F, 0xA9, 0xBB, 0xFE, 0xFF};
#define CC2500_DEFAULT_TXPOWER    0x97


/* the radio is configured with:
      GFSK modulation
      X MHz freq
      variable packet len
      GDO0 interrupt at sync word received to EOP
      variable packet length
      CRC on, CRC autoflush off
      append two status bytes at end of packet, with RSSI, LQI and CRC ok
      30 of 32 status byte bits are ok
      4 byte preamble
      after tx go to idle
      after rx stay in rx
      only manual freq osc calibration
*/
const uint8_t cc2500_default_config[] = {
//  CC2500_IOCFG2
//  CC2500_IOCFG1
//  CC2500_IOCFG0,    IOCFG_GDO_CFG_PKT_SYNCW_EOP,    // assert on sync word sent, de-assert on end-of-packet
//  CC2500_IOCFG1,    IOCFG_GDO_CFG_PKT_SYNCW_EOP,    // assert on sync word sent, de-assert on end-of-packet
  CC2500_IOCFG2,    IOCFG_GDO_CFG_PKT_SYNCW_EOP,    // assert on sync word sent, de-assert on end-of-packet
  CC2500_FIFOTHR,   0x01,     // 57tx--8rx
  CC2500_SYNC1,     0xbe,
  CC2500_SYNC0,     0xef,
  CC2500_PKTLEN,    0xff,
//  CC2500_PKTCTRL1,  crc autoflush off, append status on
//  CC2500_PKTCTRL0,  crc on, var packet len on
//  CC2500_ADDR,      not used
//  CC2500_CHANNR,    set separately
  CC2500_FSCTRL1,   0x0A,     //smartrf
//  CC2500_FSCTRL0,   not used
  CC2500_FREQ2,     0x5D,     //smartrf
  CC2500_FREQ1,     0x93,     //smartrf
  CC2500_FREQ0,     0xB1,     //smartrf
  CC2500_MDMCFG4,   0x2D,     //smartrf
  CC2500_MDMCFG3,   0x3B,     //smartrf
  CC2500_MDMCFG2,   GFSK | SYNC_MODE_30_32,
//  CC2500_MDMCFG1    FEC off, 4B preamble
//  CC2500_MDMCFG0
//  CC2500_DEVIATN,   0x00,     //smartrf seems to give strange value here...
//  CC2500_MCSM2
  CC2500_MCSM1,     CCA_MODE_3 | TXOFF_IDLE | RXOFF_RX,
  CC2500_MCSM0,     FS_AUTOCAL_NEVER | PO_TIMEOUT_1,
  CC2500_FOCCFG,    0x1D,     //smartrf
  CC2500_BSCFG,     0x1C,     //smartrf
  CC2500_AGCCTRL2,  0xC7,     //smartrf
  CC2500_AGCCTRL1,  0x00,     //smartrf
  CC2500_AGCCTRL0,  0xB0,     //smartrf
//  CC2500_WOREVT1
//  CC2500_WOREVT0
//  CC2500_WORCTRL
  CC2500_FREND1,    0xB6,     //smartrf
//  CC2500_FREND0
  CC2500_FSCAL3,    0xEA,     //smartrf
//  CC2500_FSCAL2
  CC2500_FSCAL1,    0x00,     //smartrf
  CC2500_FSCAL0,    0x11,     //smartrf
//  CC2500_RCCTRL1
//  CC2500_RCCTRL0
//  CC2500_FSTEST     not used
//  CC2500_PTEST      not used
//  CC2500_AGCTEST    not used
//  CC2500_TEST2      not used
//  CC2500_TEST1      not used
//  CC2500_TEST0      not used
};

/* the length of the above default settings = 2*the number of regs being set */
#define CC2500_DEF_CONF_LEN      (2 * 23)

#if 0
  CC2500_IOCFG2
  CC2500_IOCFG1
  CC2500_IOCFG0
  CC2500_FIFOTHR
  CC2500_SYNC1
  CC2500_SYNC0
  CC2500_PKTLEN
  CC2500_PKTCTRL1
  CC2500_PKTCTRL0
  CC2500_ADDR
  CC2500_CHANNR
  CC2500_FSCTRL1
  CC2500_FSCTRL0
  CC2500_FREQ2
  CC2500_FREQ1
  CC2500_FREQ0
  CC2500_MDMCFG4
  CC2500_MDMCFG3
  CC2500_MDMCFG2
  CC2500_MDMCFG1
  CC2500_MDMCFG0
  CC2500_DEVIATN
  CC2500_MCSM2
  CC2500_MCSM1
  CC2500_MCSM0
  CC2500_FOCCFG
  CC2500_BSCFG
  CC2500_AGCCTRL2
  CC2500_AGCCTRL1
  CC2500_AGCCTRL0
  CC2500_WOREVT1
  CC2500_WOREVT0
  CC2500_WORCTRL
  CC2500_FREND1
  CC2500_FREND0
  CC2500_FSCAL3
  CC2500_FSCAL2
  CC2500_FSCAL1
  CC2500_FSCAL0
  CC2500_RCCTRL1
  CC2500_RCCTRL0
  CC2500_FSTEST
  CC2500_PTEST
  CC2500_AGCTEST
  CC2500_TEST2
  CC2500_TEST1
  CC2500_TEST0
#endif


#endif /* __CC2500_CONFIG_H__ */


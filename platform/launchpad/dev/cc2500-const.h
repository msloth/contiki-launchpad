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
 *         CC2500 header file
 * \author
 *         Marcus Linderoth <linderoth.marcus@gmail.com>
 */

#ifndef __CC2500_CONST_H__
#define __CC2500_CONST_H__

/*
 * Definitions for the TI CC2500 2.4 GHz transceiver. See the datasheet.
 */
/* configuration registers, can be read and written in burst. */
#define CC2500_IOCFG2         0x00
#define CC2500_IOCFG1       	0x01
#define CC2500_IOCFG0       	0x02
#define CC2500_FIFOTHR      	0x03
#define CC2500_SYNC1        	0x04
#define CC2500_SYNC0        	0x05
#define CC2500_PKTLEN       	0x06
#define CC2500_PKTCTRL1     	0x07
#define CC2500_PKTCTRL0     	0x08
#define CC2500_ADDR         	0x09
#define CC2500_CHANNR       	0x0A
#define CC2500_FSCTRL1      	0x0B
#define CC2500_FSCTRL0      	0x0C
#define CC2500_FREQ2        	0x0D
#define CC2500_FREQ1        	0x0E
#define CC2500_FREQ0        	0x0F
#define CC2500_MDMCFG4      	0x10
#define CC2500_MDMCFG3      	0x11
#define CC2500_MDMCFG2      	0x12
#define CC2500_MDMCFG1      	0x13
#define CC2500_MDMCFG0      	0x14
#define CC2500_DEVIATN      	0x15
#define CC2500_MCSM2        	0x16
#define CC2500_MCSM1        	0x17
#define CC2500_MCSM0        	0x18
#define CC2500_FOCCFG       	0x19
#define CC2500_BSCFG        	0x1A
#define CC2500_AGCCTRL2     	0x1B
#define CC2500_AGCCTRL1     	0x1C
#define CC2500_AGCCTRL0     	0x1D
#define CC2500_WOREVT1      	0x1E
#define CC2500_WOREVT0      	0x1F
#define CC2500_WORCTRL      	0x20
#define CC2500_FREND1       	0x21
#define CC2500_FREND0       	0x22
#define CC2500_FSCAL3       	0x23
#define CC2500_FSCAL2       	0x24
#define CC2500_FSCAL1       	0x25
#define CC2500_FSCAL0       	0x26
#define CC2500_RCCTRL1      	0x27
#define CC2500_RCCTRL0      	0x28
#define CC2500_FSTEST       	0x29
#define CC2500_PTEST        	0x2A
#define CC2500_AGCTEST      	0x2B
#define CC2500_TEST2        	0x2C
#define CC2500_TEST1        	0x2D
#define CC2500_TEST0        	0x2E    // reg nr 47
/* 0x2F is reserved */

/*
 * status registers; Read with 'burst-read' bit set, like so:
 *    cc2500_read_burst(CC2500_RSSI, &save, 1);
 * or rather
 *    save = cc2500_read_single(CC2500_RSSI);
 * Only 1 reg can be read at a time.
 */
#define CC2500_PARTNUM      	0x30
#define CC2500_VERSION      	0x31
#define CC2500_FREQEST      	0x32
#define CC2500_LQI          	0x33
#define CC2500_RSSI         	0x34
#define CC2500_MARCSTATE    	0x35
#define CC2500_WORTIME1     	0x36
#define CC2500_WORTIME0     	0x37
#define CC2500_PKTSTATUS    	0x38
#define CC2500_VCO_VC_DAC   	0x39
#define CC2500_TXBYTES      	0x3A
#define CC2500_RXBYTES      	0x3B
#define CC2500_RCCTRL1_STATUS 0x3C
#define CC2500_RCCTRL0_STATUS 0x3D
#define CC2500_PATABLE  			0x3E
#define CC2500_TXFIFO   			0x3F
#define CC2500_RXFIFO   			0x3F

/* command strobes */
#define CC2500_SRES     			0x30
#define CC2500_SFSTXON  			0x31
#define CC2500_SXOFF    			0x32
#define CC2500_SCAL     			0x33
#define CC2500_SRX      			0x34
#define CC2500_STX      			0x35
#define CC2500_SIDLE    			0x36
#define CC2500_SWOR     			0x38
#define CC2500_SPWD     			0x39
#define CC2500_SFRX     			0x3A
#define CC2500_SFTX     			0x3B
#define CC2500_SWORRST  			0x3C
#define CC2500_SNOP     			0x3D

/*
 * State definitions if reading SPI statusbyte; several definitions have the same
 * value to be compatible with MARCSTATE definition names. Shifted to correspond
 * with the status byte bits to avoid doing a shift for every check.
 */
#define CC2500_STATE_IDLE             (0)
#define CC2500_STATE_RX               (1<<4)
#define CC2500_STATE_TX               (2<<4)
#define CC2500_STATE_FSTXON           (3<<4)
#define CC2500_STATE_CAL              (4<<4)
#define CC2500_STATE_SETTLING         (5<<4)
#define CC2500_STATE_RXFIFO_OVERFLOW  (6<<4)
#define CC2500_STATE_TXFIFO_UNDERFLOW (7<<4)
#define CC2500_STATUSBYTE_STATUSBITS  0x70

/* SPI addressing modes */
#define CC2500_WRITE          0x00
#define CC2500_BURSTWRITE     0x40
#define CC2500_READ           0x80
#define CC2500_BURSTREAD      0xC0

/* Bit field settings-------------------------------------- */
/* invert assert/de-assert */
#define IOCFG_GDO_CFG_INVERT             (1<<6)
/* drive strength; NB ONLY GDO1 */
#define IOCFG_GDO_CFG_GDO1_DS            (1<<7)
/* enable temp sensor (all other bits in this reg should be 0) NB GDO0 ONLY */
#define IOCFG_GDO_CFG_GDO0_TEMP_EN       (1<<7)
/* CCA bit in PKTSTATUS */
#define PKTSTATUS_CCA                    (1<<4)
/* CS bit in PKTSTATUS */
#define PKTSTATUS_CS                     (1<<6)
/* various bitsettings; cross-check against cc2500-config.h and the datasheet */
#define GFSK              (1<<4)    /* use GFSK modulation */
#define SYNC_MODE_30_32   (3<<0)    /* 30 of 32 bits SYNC word should be correct */
#define CCA_MODE_3        (3<<4)    /* below threshold unless receiving */
#define RXOFF_RX          (3<<2)    /* stay in Rx after received packet */
#define TXOFF_IDLE        (0<<0)    /* goto IDLE after packet sent */
#define TXOFF_RX          (3<<0)    /* goto Rx after packet sent */
#define FS_AUTOCAL_NEVER  (0<<4)    /* only calibrate oscillator manually */
#define PO_TIMEOUT_1      (1<<2)



/* GDO functionality------------------------------------------ */
/* Assert when THR is reached or EOP; de-assert on empty RxFIFO */
#define IOCFG_GDO_CFG_RXFIFO_THR_PKT     1
/* assert on SYNC recv/sent, de-assert on EOP. NB XXX Will also de-assert on over-/underflow so check that! */
#define IOCFG_GDO_CFG_PKT_SYNCW_EOP      6
/* CCA; RSSI < threshold ? assert : de-assert */
#define IOCFG_GDO_CFG_CCA                9
/* Carrier sense; RSSI > threshold ? assert : de-assert; inverted of CCA */
#define IOCFG_GDO_CFG_CS                 14
/* high impedance */
#define IOCFG_GDO_CFG_HIGHZ              46
/*--------------------------------------------------------------------------*/
/* the appended bytes; first one is RSSI, the second is LQI and CRC_OK */
#define FOOTER1_LQI               0x7f
#define FOOTER1_CRC_OK            0x80
/*--------------------------------------------------------------------------*/
/* old and backups */
#if 0
/* MARCSTATE states */
#define CC2500_STATE_SLEEP             0
#define CC2500_STATE_IDLE              1
#define CC2500_STATE_XOFF              2
#define CC2500_STATE_VCOON_MC          3
#define CC2500_STATE_REGON_MC          4
#define CC2500_STATE_MANCAL            5
#define CC2500_STATE_VCOON             6
#define CC2500_STATE_REGON             7
#define CC2500_STATE_STARTCAL          8
#define CC2500_STATE_BWBOOST           9
#define CC2500_STATE_FS_LOCK          10
#define CC2500_STATE_IFADCON          11
#define CC2500_STATE_ENDCAL           12
#define CC2500_STATE_RX               13
#define CC2500_STATE_RX_END           14
#define CC2500_STATE_RX_RST           15
#define CC2500_STATE_TXRX_SWITCH      16
#define CC2500_STATE_RXFIFO_OVERFLOW  17
#define CC2500_STATE_FSTXON           18
#define CC2500_STATE_TX               19
#define CC2500_STATE_TX_END           20
#define CC2500_STATE_RXTX_SWITCH      21
#define CC2500_STATE_TXFIFO_UNDERFLOW 22
#endif



/*
          Checked OK!
  PATable settings; only OOK uses more than PATABLE[0], and [1:7] are lost on SLEEP
  from the datasheet
  dBm -- setting -- current consumption mA
    -30 -- 0x50 -- .9
    -28 -- 0x44 -- .7
    -26 -- 0xC0 -- 0.2
    -24 -- 0x84 -- 0.1
    -22 -- 0x81 -- 0.0
    -20 -- 0x46 -- 0.1
    -18 -- 0x93 -- 1.7
    -16 -- 0x55 -- 0.8
    -14 -- 0x8D -- 2.2
    -12 -- 0xC6 -- 1.1
    -10 -- 0x97 -- 2.2
    -8 -- 0x6E -- 4.1
    -6 -- 0x7F -- 5.0
    -4 -- 0xA9 -- 6.2
    -2 -- 0xBB -- 7.7
    0 -- 0xFE -- 1.2
    +1 -- 0xFF -- 1.5
-30
  -28
-26
  -24
  -22
  -20
-18
  -16
  -14
  -12
-10
  -8
  -6
-4
  -2
  0
+1
*/



//uint8_t cc2500_txp[] = {0x50, 0x44, 0xC0, 0x84, 0x81, 0x46, 0x93, 0x55, 0x8D, 0xC6, 0x97, 0x6E, 0x7F, 0xA9, 0xBB, 0xFE, 0xFF};

#endif /* __CC2500_CONST_H__ */


#ifndef __CC2500_H__
#define __CC2500_H__

/*
 * Definitions for TI CC2500 2.4 GHz transceiver. See the datasheet.
 */

/* configuration registers */
#define CC2500_IOCFG2         0x00
#define CC2500_IOCFG1       	0x01
#define CC2500_IOCFG0D      	0x02
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
#define CC2500_TEST0        	0x2E

/* status registers */
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
#define CC2500_SAFC     			0x37
#define CC2500_SWOR     			0x38
#define CC2500_SPWD     			0x39
#define CC2500_SFRX     			0x3A
#define CC2500_SFTX     			0x3B
#define CC2500_SWORRST  			0x3C
#define CC2500_SNOP     			0x3D

/* MARCSTATE states */
// XXX CHECK!
#if 0
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

/* State definitions if reading SPI statusbyte; several definitions have the same
 * value to be compatible with MARCSTATE definition names */
// XXX CHECK!
#define CC2500_STATE_RX               1
#define CC2500_STATE_RX_END           1
#define CC2500_STATE_TX               2
#define CC2500_STATE_TX_END           2
#define CC2500_STATE_FSTXON           3
#define CC2500_STATE_STARTCAL         4
#define CC2500_STATE_MANCAL           4
#define CC2500_STATE_ENDCAL           4
#define CC2500_STATE_CAL              4
#define CC2500_STATE_SETTLING         5
#define CC2500_STATE_RXFIFO_OVERFLOW  6
#define CC2500_STATE_TXFIFO_UNDERFLOW 7
#define CC2500_STATUSBYTE_STATUSBITS  0x70

/* Bit fields */
// XXX CHECK!
/* invert assert/de-assert */
#define IOCFG_GPIO_CFG_INVERT             BV(6)
/* set pin as "Analog transfer" (==pin not used as GPIO) */
#define IOCFG_GPIO_CFG_ATRAN              BV(7)

/* GPIO functionality */
/* Assert when THR is reached or EOP; de-assert on empty RxFIFO */
#define IOCFG_GPIO_CFG_RXFIFO_THR_PKT     1   
/* assert on SYNC recv/sent, de-assert on EOP */
#define IOCFG_GPIO_CFG_PKT_SYNC_RXTX      6  
/* CS valid ? assert : de-assert */
#define IOCFG_GPIO_CFG_CS_VALID           16  
/* Carrier Sense ? assert : de-assert */
#define IOCFG_GPIO_CFG_CS                 17  
/* assert when in TX, de-assert in Rx/IDLE/settling */ 
#define IOCFG_GPIO_CFG_RXIDLE_OR_TX       26  
/* assert if in rx or tx, de-assert if idle/settling (MARC_2PIN_STATUS[0]) */ 
#define IOCFG_GPIO_CFG_RXTX_OR_IDLE       38
/* high impedance */
#define IOCFG_GPIO_CFG_HIGHZ              48  








#endif /* __CC2500_H__ */


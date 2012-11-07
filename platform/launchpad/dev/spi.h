#ifndef __SPI_H__
#define __SPI_H__

/* the only thing the spi.c module should be responsible of is initiating the spi
module with pins, the rest is handled in the driver for that peripheral. */

/* All pins are on port 1 */
#define SPI_PORT(type)          P1##type
#define SPI_MOSI                (1<<7)
#define SPI_MISO                (1<<6)
#define SPI_SCL                 (1<<5)
/* NB, pin 1.6 (MISO) also for LED2 so remove jumper for that if you use SPI */
#warning "Pin 1.6 is both for SPI MISO and for LED2, remove jumper if SPI is used"

#define SPI_WAITFOREOTx()       while ((U0TCTL & TXEPT) == 0)
#define SPI_WAITFOREORx()       while ((IFG2 & UCB0RXIFG) == 0)
#define SPI_WAITFORTxREADY()    while ((IFG2 & UCB0TXIFG) == 0)

#define SPI_TXBUF               UCB0TXBUF
#define SPI_RXBUF               UCB0RXBUF

//#define SPI_WRITE(x)          (UCB0TXBUF = x)
#define SPI_WRITE(x)              do {                                        \
                                    SPI_TXBUF = x;                         \
                                    while (!(IFG2 & UCB0TXIFG));              \
                                  } while(0)
      


void spi_init(void);

#endif /* __SPI_H__ */


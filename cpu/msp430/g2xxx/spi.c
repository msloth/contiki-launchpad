
#include "contiki-conf.h"
#include "dev/spi.h"

/*--------------------------------------------------------------------------*/
/*
 * Set spi_busy so that interrupt handlers can check if
 * they are allowed to use the SPI bus or not.
 */
/*volatile uint8_t spi_busy = 0;*/
// XXX no, they can instead poll the BUSY flag, saves a byte :)


/*--------------------------------------------------------------------------*/
/*uint8_t*/
/*spi_tx_byte(uint8_t data)*/
/*{*/

/*  UCB0TXBUF = data;*/
/*  // wait for TX*/
/*  while (!(IFG2 & UCB0TXIFG));*/
/*  return UCB0RXBUF;*/

/*}*/
/*--------------------------------------------------------------------------*/
/*
 * Init SPI on USCI UCB0 as UCA0 is used for UART (printf's)
 */
void
spi_init(void)
{
  /*
   * From TIs users manual
   * The recommended USCI initialization/re-configuration process is:
   */
  /** 1. Set UCSWRST (BIS.B #UCSWRST,&UCxCTL1)*/
  UCB0CTL1 = UCSWRST;

  /** 2. Initialize all USCI registers with UCSWRST=1 (including UCxCTL1)*/
  UCB0CTL0 |= UCCKPH | UCMSB | UCMST | UCSYNC | UCMODE_0;
  UCB0CTL1 |= UCSSEL_2;

  /** 3. Configure ports*/
  SPI_PORT(SEL)  |= SPI_MISO | SPI_MOSI | SPI_SCL;
  SPI_PORT(SEL2) |= SPI_MISO | SPI_MOSI | SPI_SCL;

  /** 4. Clear UCSWRST via software (BIC.B #UCSWRST,&UCxCTL1)*/
  UCB0CTL1 &= ~UCSWRST;
  /** 5. Enable interrupts (optional) via UCxRXIE and/or UCxTXIE*/
  /* this step is done through cc2500_arch_init() */
}
/*--------------------------------------------------------------------------*/

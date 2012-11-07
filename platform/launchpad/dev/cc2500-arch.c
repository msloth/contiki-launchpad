
#include "contiki.h"
#include "contiki-net.h"
#include "dev/spi.h"
#include "dev/cc2500.h"
#include "isr_compat.h"
/*
 * This file handles two things: the ISR for the radio and initiating the radio
 * on mcu (SPI, pins, irq). The cc2500.c handles initiating the actual radio
 * with the proper settings.
 */
/*--------------------------------------------------------------------------*/
/* The interrupt service routine for when the radio signals received a packet */
#if 1
ISR(PORT1, cc2500_port1_interrupt)
{
  /* check for a valid packet, and if there is one, we wake up the mcu */
  if(cc2500_interrupt()) {
    LPM4_EXIT;
  }
}
#endif
/*--------------------------------------------------------------------------*/
void
cc2500_arch_init(void)
{
#if 1
  /* init irq and spi enable pins; spi pins are inited in spi_init() */

  /* init spi */
  spi_init();
  CC2500_SPI_DISABLE();                /* Unselect radio. */

  /* init irq */

#endif
}
/*--------------------------------------------------------------------------*/



#include "contiki.h"
#include "contiki-net.h"

#include "dev/spi.h"
#include "dev/cc2500.h"
#include "isr_compat.h"

/*---------------------------------------------------------------------------*/
#if 0
ISR(CC2500_IRQ, cc2500_port1_interrupt)
{
  if(cc2500_interrupt()) {
    LPM4_EXIT;
  }
  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
#endif
/*---------------------------------------------------------------------------*/
void
cc2500_arch_init(void)
{
#if 0
  /* init ports, pins */
  /* init spi */
  spi_init();
  CC2500_SPI_DISABLE();                /* Unselect radio. */
  /* init irq */
#endif
}
/*---------------------------------------------------------------------------*/

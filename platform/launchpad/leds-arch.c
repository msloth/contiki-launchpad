/**
 * \file
 *         A leds implementation for the TI Launchpad platform
 * \author
 *         Marcus Lunden <marcus.lunden@gmail.com>
 */

#include "contiki.h"
#include "dev/leds.h"

/*---------------------------------------------------------------------------*/
void
leds_arch_init(void)
{
  LEDS_PORT(SEL) &= ~(LEDS_ALL);
  LEDS_PORT(SEL2) &= ~(LEDS_ALL);
  LEDS_PORT(DIR) |= (LEDS_ALL);
  LEDS_PORT(OUT) &= ~(LEDS_ALL);
}
/*---------------------------------------------------------------------------*/
unsigned char
leds_arch_get(void)
{
  return ~LEDS_PORT(OUT);
}
/*---------------------------------------------------------------------------*/
void
leds_arch_set(unsigned char leds)
{
  LEDS_PORT(OUT) = ~leds;
}
/*---------------------------------------------------------------------------*/

/**
 * \addtogroup launchpad-examples
 *
 * @{
 */

/**
 * \file
 *    readadc.c
 * \author
 *    Marcus Lunden <marcus.lunden@gmail.com>
 * \brief
 *    ADC reading example. One process periodically reads the ADC on one channel
 *    and updates a blink speed variable. Another sets a LED blink according to
 *    that variable.
 *    
 */

#include <stdio.h>
#include "contiki.h"
#include "dev/leds.h"
#include "dev/adc.h"
static uint16_t adcval;
static struct etimer et;
/*--------------------------------------------------------------------------*/
PROCESS(adc_reading_process, "ADC reading process");
AUTOSTART_PROCESSES(&adc_reading_process);
/* -------------------------------------------------------------------------- */
static void
adc_pollhandler(void)
{
  printf("ADC: %u\n", adcval);
}
/*--------------------------------------------------------------------------*/
PROCESS_THREAD(adc_reading_process, ev, data)
{
  PROCESS_POLLHANDLER(adc_pollhandler());
  PROCESS_BEGIN();
  while(1) {
    adc_irqpoll_get(A5, &adcval, PROCESS_CURRENT());
    etimer_set(&et, CLOCK_SECOND/8);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/** @} */

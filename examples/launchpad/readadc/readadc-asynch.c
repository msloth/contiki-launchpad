/**
 * \addtogroup launchpad-examples
 *
 * @{
 */

/**
 * \file
 *    sdf
 * \author
 *    Marcus Lunden <marcus.lunden@gmail.com>
 * \brief
 *    sdf
 *    
 */

#include <stdio.h>
#include "contiki.h"
#include "dev/leds.h"
#include "dev/adc.h"

static volatile uint16_t adc = 0;
/*--------------------------------------------------------------------------*/
PROCESS(adc_reading_process, "ADC reading process");
AUTOSTART_PROCESSES(&adc_reading_process);
/*--------------------------------------------------------------------------*/
static struct etimer et;
PROCESS_THREAD(adc_reading_process, ev, data)
{
  PROCESS_BEGIN();
  while(1) {
    adc_asynch_get(A5, &adc);
    etimer_set(&et, CLOCK_SECOND/8);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    printf("ADC: %u\n", adc);
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/** @} */

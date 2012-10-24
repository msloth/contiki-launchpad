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
 *    ADC reading example.
 *    
 */

#include <stdio.h>
#include "contiki.h"
#include "dev/leds.h"
#include "dev/adc.h"

/*--------------------------------------------------------------------------*/
PROCESS(adc_reading_process, "ADC reading process");
AUTOSTART_PROCESSES(&adc_reading_process);
/* -------------------------------------------------------------------------- */
PROCESS_THREAD(adc_reading_process, ev, data)
{
  PROCESS_BEGIN();
  while(1) {
    adc_irqevent_get(A5, PROCESS_CURRENT());
    PROCESS_WAIT_EVENT_UNTIL(ev == adc_event);
    if(data != NULL) {
      printf("ADC: %u\n", *((uint16_t*)data));
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/** @} */

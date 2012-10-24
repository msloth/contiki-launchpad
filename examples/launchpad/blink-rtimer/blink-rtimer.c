
/* Blink with rtimer example application,
 * Author: Marcus Lunden <marcus.lunden@gmail.com>
 * expected result from running it:
 *   red LED lighting up for 1/4 second every second, the green toggling every s.
 */

#include <stdio.h>
#include "contiki.h"
#include "dev/leds.h"

/*---------------------------------------------------------------------------*/
PROCESS(blink_process, "Blink");
AUTOSTART_PROCESSES(&blink_process);
/* -------------------------------------------------------------------------- */
static void
rtcb(struct rtimer *r, void *d)
{
  leds_off(LEDS_RED);
} 
/*--------------------------------------------------------------------------*/
static struct rtimer rt;
static struct etimer et;

PROCESS_THREAD(blink_process, ev, data)
{
  PROCESS_BEGIN();
  while(1) {
    leds_on(LEDS_RED);
    /* start an rtimer; the 1 signifies how long time in rtimer ticks the task
     * will take (ie the callback) but is currently not implemented on a deeper
     * level. So, use just any value for now. */
    rtimer_set(&rt, RTIMER_NOW() + RTIMER_SECOND/4, 1, rtcb, NULL);

    leds_toggle(LEDS_GREEN);
    etimer_set(&et, CLOCK_SECOND/2);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/


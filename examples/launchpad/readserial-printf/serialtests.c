
/* Blink with rtimer example application,
 * Author: Marcus Lunden <marcus.lunden@gmail.com>
 * usage
 *    compile, upload to LP, login over serial
 *    requires a 2553 due to hw UART; remember to adjust the rxtx headers on 
 *    the board
 * expected result when running it:
 *    will print out the time every second, will repeat back anything sent to it
 *    and if sent "red" or "green", it will toggle the corresponding LED
 */

#include <stdio.h>
#include <string.h>
#include "contiki.h"
#include "dev/leds.h"
#include "dev/serial-line.h"

/*---------------------------------------------------------------------------*/
PROCESS(blink_process, "Blink");
PROCESS(serial_read_process, "Serial Reader");
AUTOSTART_PROCESSES(&blink_process, &serial_read_process);
/*--------------------------------------------------------------------------*/
/* will repeatedly wait for serial data and then repeat back what it received.
 * If the data is "red" or "green", it will toggle the corresponding LED */
PROCESS_THREAD(serial_read_process, ev, data) {
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER();
  PROCESS_BEGIN();
  while (1) {
    char* buf;
    PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message);
    buf = data;

    if(!strncmp(buf, "red", 3)) {
      leds_toggle(LEDS_RED);
    } else if (!strncmp(buf, "green", 5)) {
      leds_toggle(LEDS_GREEN);
    } else if (!strncmp(buf, "off", 3)) {
      leds_off(LEDS_GREEN | LEDS_RED);
    }
    
    printf("Got:%s\n", buf);
  }
  PROCESS_END();
}
 
/*--------------------------------------------------------------------------*/
/* repeatedly printing out the time over the serial port (9600 baud, 8,n,1) */
static struct etimer et;

PROCESS_THREAD(blink_process, ev, data)
{
  PROCESS_BEGIN();
  while(1) {
    printf("Time since bootup: %lu\n", clock_seconds());
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/



/* Testing button example
 * Author: Marcus Lunden <marcus.lunden@gmail.com>
 * expected result from running it:
 *   When pressing the button, the red LED will toggle until enough presses,
 *   then the green blinks stops and the button becomes unresponsive.
 */

#include <stdio.h>
#include "contiki.h"
#include "contiki-net.h"
#include "dev/leds.h"
#include "button.h"

/*
 * how to read a Contiki program: start by finding AUTOSTART_PROCESSES(...).
 * The first process in that list is started first. Then, if it starts a new
 * process by process_start() execution flow continues in that process until
 * any PROCESS_YIELD_* or PROCESS_WAIT_*.   */
/* -------------------------------------------------------------------------- */
PROCESS(button_process, "Button process");
PROCESS(blink_process, "Blink process");
AUTOSTART_PROCESSES(&blink_process, &button_process);
/*---------------------------------------------------------------------------*/
/* Broadcast receive callback */
static uint8_t buf[15];
static void
bcr(struct broadcast_conn *c, const rimeaddr_t *f)
{
  memcpy(buf, packetbuf_dataptr(), packetbuf_datalen());
  buf[packetbuf_datalen()] = 0; // null-terminate string
  printf("[%u] Broadcast Received from %u.%u:%s\n", clock_seconds(), f->u8[0], f->u8[1], buf);
}
/*---------------------------------------------------------------------------*/
static struct broadcast_conn bc;
static struct broadcast_callbacks bccb = {bcr, NULL};
/*---------------------------------------------------------------------------*/
/*#define SWITCH_2  (1<<3) // LP button*/

PROCESS_THREAD(button_process, ev, data)
{
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER();
  PROCESS_BEGIN();
  broadcast_open(&bc, 2001, &bccb);
  while (1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == button_event && (*((uint8_t *)data) & SWITCH_2));
    leds_toggle(LEDS_RED);
    packetbuf_copyfrom("Hello", sizeof("Hello"));
    broadcast_send(&bc);
  }
  PROCESS_END();
}
/*--------------------------------------------------------------------------*/
/* this process periodically blinks an LED */
static struct etimer et;

PROCESS_THREAD(blink_process, ev, data)
{
  PROCESS_BEGIN();
  while(1) {
    leds_toggle(LEDS_GREEN);
    etimer_set(&et, CLOCK_SECOND/8);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/


/* Testing button example
 * Author: Marcus Lunden <marcus.lunden@gmail.com>
 * expected result from running it:
 *   When pressing the button, the red LED will toggle until enough presses,
 *   then the green blinks stops and the button becomes unresponsive.
 */

#include <stdio.h>
#include "contiki.h"
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
static struct etimer etr;

PROCESS_THREAD(button_process, ev, data)
{
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER();
  PROCESS_BEGIN();
  static uint8_t btnpresses = 0;
  while (1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == button_event && (*data & SwITCH_2));
    leds_toggle(LEDS_RED);
    btnpresses++;
    if (btnpresses == 10) {
      process_exit(&blink_process);
    }
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
#if 0

/*AUTOSTART_PROCESSES(&blink_process, &button_process, &batterycheck_process);*/
/*PROCESS(batterycheck_process, "Voltage checker");*/
/* periodically read out supply voltage and print it out on the serial port */
static struct etimer voltagecheck_timer;
static uint16_t battval;

PROCESS_THREAD(batterycheck_process, ev, data) {
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER();
  PROCESS_BEGIN();

  SENSORS_ACTIVATE(battery_sensor);

  while (1) {
    etimer_set(&voltagecheck_timer, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&voltagecheck_timer));
    /* read voltage (10bit = 0..1024), convert to mV */
    battval = (uint16_t)((battery_sensor.value(0) * 1000 * 5) / (1024*3));
    printf("[%u] %u mV\n", (uint16_t)clock_seconds(), battval);
  }
  
  PROCESS_END();
}
#endif



#if 0
/*---------------------------------------------------------------------------*/
static struct etimer etr;
static uint8_t btnpresses = 0;
PROCESS_THREAD(button_process, ev, data)
{
  PROCESS_POLLHANDLER();
  PROCESS_BEGIN();
  SENSORS_ACTIVATE(button_sensor);
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event && data == &button_sensor);
    leds_toggle(LEDS_GREEN);
    btnpresses++;
    if (btnpresses == 10) {
      process_exit(&blink_process);
    }
  }
  PROCESS_END();
}

#endif


/* PWM example application,
 * Author: Marcus Lunden <marcus.lunden@gmail.com>
 * Demonstrates using the PWM API to dim an LED.
 * expected result from running it:
 *   red and green LEDs fading in and out, out of phase from eachother.
 */

#include <stdio.h>
#include "contiki.h"
#include "dev/leds.h"
#include "dev/pwm.h"
#include "dev/button.h"

#define LEDRED_PIN       (0)   // P1.0
#define LEDGRN_PIN       (6)   // P1.6
/*---------------------------------------------------------------------------*/
PROCESS(button_process, "Button reader");
PROCESS(pwmled_process, "PWM LED process");
AUTOSTART_PROCESSES(&pwmled_process, &button_process);
/*---------------------------------------------------------------------------*/ 
static struct etimer etr;

PROCESS_THREAD(pwmled_process, ev, data)
{
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER();
  static uint8_t i = 1;
  static uint8_t up = 1;
  PROCESS_BEGIN();
  while(1) {
    pwm_on(0, LEDGRN_PIN, 100-i);
    pwm_on(1, LEDRED_PIN, i);
    if(up) {
      i++;
      if(i == 100) {
        up = 0;
        i = 99;
      }
    } else {
      i--;
      if(i == 0) {
        up = 1;
        i = 1;
      }
    }
    etimer_set(&etr, CLOCK_SECOND/32);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
  }
  PROCESS_END();
}
/*--------------------------------------------------------------------------.*/ 
/* Kill the PWM'ing with the push of the button. */
PROCESS_THREAD(button_process, ev, data)
{
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER();
  PROCESS_BEGIN();
  PROCESS_WAIT_EVENT_UNTIL(ev == button_event);
  pwm_off(0);
  pwm_off(1);
  process_exit(&pwmled_process);
  PROCESS_END();
}
/* -------------------------------------------------------------------------- */
 










#include <stdio.h>
#include "contiki.h"
#include "isr_compat.h"
#include "pwm.h"

/*

  XXX todo/suggestions
  * change clock and rtimer to TimerA1? This as TimerA0 has output (PWM) pins on port 1 and TimerA1 on port 2.
  * invert pin mode/dc=255-dc
  * smoothly change PWM over time, PI-controller
      --build separate module for this
  * servo-motor module
      --build separate module for this
      --cannot co-exist with PWM as not enough hardware timers (messy otherwise)
        and 20 ms period time is 50Hz/655.26 ticks @ 32768.
        One tick is 0.0305 ms, too slow for servo arm position fine resolution?
        50Hz is ok for LED, is it good enough for servos?
 */

/*
 * The PWM module; starts and handles PWM on pins. The duty cycle can be chosen 
 * from [1..255] or off.
 * 
 * This is kept as simple as possible to keep RAM usage down. 
 * 
 */


/*---------------------------------------------------------------------------*/
/* keep settings for a maximum of 8 pins = one port. Seems like the most space
efficient way of doing it. */
#define MAX_PWM_PINS  8
static uint8_t pwmsettings[MAX_PWM_PINS];
/*---------------------------------------------------------------------------*/
/* Init the PWM */
void
pwm_init(void)
{
  uint8_t i;
  for (i = 0; i < MAX_PWM_PINS; i += 1) {
    pwmsettings[i] = 0;
  }

  dint();
  /* Timer is in cont up mode, irq when counter TAR == CCR0 */
  /* using ACLK, which is 32768 ext osc; div 1 --> 32.768kHz; with a 256 period this is 128 Hz,
  enough for LEDs but not for servo motors. */
  TA1CTL = TASSEL_1 | ID_0;
  TA1CCR0 = ;
  TA1CCTL0 = CCIE;
  TA1CTL |= MC_2;
  count = 0;
  eint();
}
/*--------------------------------------------------------------------------*/
/* turn PWM on on a pin; dc is duty cycle expressed in [1..255] where 255 is max
and pin is pin [0..7] on port 1 to run PWM on. */
void
pwm_on(uint8_t dc, uint8_t pin)
{
  if(pin > 7 || dc == 0) {
    return;
  }
  pwmsettings[pin] = dc;
  TA1CCR1 = dc;
}
/*--------------------------------------------------------------------------*/
/* returns the current duty cycle on the pin [0..7] on port1 */
uint8_t
pwm_get(uint8_t pin)
{
  if(pin > 7) {
    /* illegal pin, OOB */
    return 0;
  }
  return pwmsettings[pin];
}
/*--------------------------------------------------------------------------*/
void
pwm_all_off(void)
{
  for (i = 0; i < MAX_PWM_PINS; i += 1) {
    pwmsettings[i] = 0;
  }
}
/*--------------------------------------------------------------------------*/
/* ISR for CCR0 compare match */
ISR(TIMER1_A0, pwm_periodstart_ta1ccr0_isr)
{
  ;
}
/*---------------------------------------------------------------------------*/
/* ISR for CCR1..2 compare match */
ISR(TIMER1_A1, pwm_ccrmatch_ta1ccrX_isr)
{
  ;
}
/*---------------------------------------------------------------------------*/
#if 0
  #define TIMER0_A1_VECTOR    (0x0010)  /* 0xFFF0 Timer0_A CC1, TA0 */
  #define TIMER0_A0_VECTOR    (0x0012)  /* 0xFFF2 Timer0_A CC0 */
  #define TIMER1_A1_VECTOR    (0x0018) /* 0xFFF8 Timer1_A CC1-4, TA1 */
  #define TIMER1_A0_VECTOR    (0x001A) /* 0xFFFA Timer1_A CC0 */

  #define TACCTL0             TA0CCTL0  /* Timer A Capture/Compare Control 0 */
  #define TACCTL1             TA0CCTL1  /* Timer A Capture/Compare Control 1 */
  #define TACCTL2             TA0CCTL2  /* Timer A Capture/Compare Control 2 */
#endif
/*--------------------------------------------------------------------------*/
#if NOTINUSE
#define MCU_CLOCK                       1000000
#define PWM_FREQUENCY                   46              // In Hertz, ideally 50Hz.

unsigned int PWM_Period         = (MCU_CLOCK / PWM_FREQUENCY);  // PWM Period
unsigned int PWM_Duty           = 0;                                                    // %

void main (void){
  TACCTL1 = OUTMOD_7;            // TACCR1 reset/set
  TACTL   = TASSEL_2 + MC_1;     // SMCLK, upmode
  TACCR0  = PWM_Period-1;        // PWM Period
  TACCR1  = PWM_Duty;            // TACCR1 PWM Duty Cycle

  P1DIR   |= BIT2;               // P1.2 = output
  P1SEL   |= BIT2;               // P1.2 = TA1 output

  while (1){
    PROCESS_WAIT_EVENT_UNTIL(ev == servo_change_event);
    pwm_set(SERVO_PWM_PIN, servo_lut[45]);
  }
}
#endif










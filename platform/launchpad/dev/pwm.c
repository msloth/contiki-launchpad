/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include "contiki.h"
#include "pwm.h"

#define MAX_PWM_PINS  10
static uint16_t pwmsettings[MAX_PWM_PINS];

/*--------------------------------------------------------------------------*/
void
pwm_init(void)
{
  uint8_t i;
  for (i = 0; i < MAX_PWM_PINS; i += 1) {
    pwmsettings[i] = 0;
  }
}
/*--------------------------------------------------------------------------*/
void
pwm_set(uint16_t dc, uint8_t pin)
{
  pwmsettings[pin] = dc;
}

/*--------------------------------------------------------------------------*/
uint16_t
pwm_get(uint8_t pin)
{
  return pwmsettings[pin];
}

/*--------------------------------------------------------------------------*/
void
pwm_all_off(void)
{
  
}

/*--------------------------------------------------------------------------*/
ISR(TIMER0_A1, timerA0ccr_interrupt)
{
}

#if 0
  #define TIMER0_A1_VECTOR    (0x0010)  /* 0xFFF0 Timer0_A CC1, TA0 */
  #define TIMER0_A0_VECTOR    (0x0012)  /* 0xFFF2 Timer0_A CC0 */
  #define WDT_VECTOR          (0x0014) /* 0xFFF4 Watchdog Timer */
  #define COMPARATORA_VECTOR  (0x0016) /* 0xFFF6 Comparator A */
  #define TIMER1_A1_VECTOR    (0x0018) /* 0xFFF8 Timer1_A CC1-4, TA1 */
  #define TIMER1_A0_VECTOR    (0x001A) /* 0xFFFA Timer1_A CC0 */

  #define TACCTL0             TA0CCTL0  /* Timer A Capture/Compare Control 0 */
  #define TACCTL1             TA0CCTL1  /* Timer A Capture/Compare Control 1 */
  #define TACCTL2             TA0CCTL2  /* Timer A Capture/Compare Control 2 */
#endif

/*--------------------------------------------------------------------------*/
#if oijsfoisjdf
#define MCU_CLOCK                       1000000
#define PWM_FREQUENCY           46              // In Hertz, ideally 50Hz.

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
    TACCR1 = servo_lut[45];
    // delay...
  }
}
#endif










/*
 * Copyright (c) 2012
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

/**
 * \file
 *         Launchpad PWM drivers
 * \author
 *         Marcus Lunden <marcus.lunden@gmail.com>
 */

#include "contiki.h"
#include "isr_compat.h"
#include "pwm.h"
#include "dev/leds.h"


/*
 * The PWM module; starts and handles PWM on pins. 
 * 
 * Assumes PWM freq of no slower than 128 Hz (7.8 ms period due to otherwise
 * variables have to be 16-bit so we save some RAM this way.
 * 
 * There are checks for MCU being an MSP430G2553 as the '2452 does not have two
 * hardware timers, which this module needs.
 */

/*---------------------------------------------------------------------------*/
/*
  XXX todo/suggestions
  * change clock and rtimer to TimerA1? This as TimerA0 has output (PWM) pins on port 1 and TimerA1 on port 2.
  * smoothly change PWM over time, PI-controller
      --ie set eg rise time and 'should'-value
      --build separate module for this
  * servo-motor module
      --build separate module for this
      --One tick is 0.0305 ms, too low res for servo arm position fine resolution?
        ->ca 5.5 deg resolution
      --no, this can be used for that if freq set to 50..100 Hz and on_time set
        to ca 25..65 which gives an angular resolution of ca 5.5 degrees.
        For finer control, the timer must be sourced from DCO which means that
        the microcontroller can't go to LPM3.
*/
/*---------------------------------------------------------------------------*/

/* PWM freq; can be higher and lower; setting to lower will increase accuracy of
  the duty cycle (rounding errors) calculated in pwm_on(). */
#ifdef PWM_CONF_FREQ
#define PWM_FREQ          PWM_CONF_FREQ
#else
#define PWM_FREQ          128
#endif

/* PWM module is sourced from external crystal osc @ 32.768 kHz, hence do not change this */
#define PWM_CLOCK_SPEED   32768

/*
 * a PWM-device is here a combination of a pin and an time for which to be in 
 * 'on'-state. If time is zero, then the device is off. All PWM-devices has a
 * common period. Each PWM-device correspond to a CCR-register, for which the 
 * 2553 has three, but one is for the common period. 
 */
#define PWMDEVS   2     /* number of PWM devices; must not be changed */
struct pwm_s {
  /* pin is pin on port 1 */
  uint8_t pin;
  /* on_time is time in on-state; duty cycle == on_time/period */
  uint8_t on_time;

  /* possible enhancement 1: store port as well to enable PWM on several ports */
/*  char *port;*/
  
  /* possible enhancement 2: allows for more a more generic approach with more/less
  PWM-devices, but takes more RAM: store pointers to registers */
/*  char *reg_ctl;*/
/*  char *reg_ccr;*/

} pwms[PWMDEVS];

const uint16_t period = (PWM_CLOCK_SPEED / PWM_FREQ) - 1;    /* don't change */
/*---------------------------------------------------------------------------*/
/* Init the PWM; up mode to CCR0, PWM periods CCR1, CCR2, clock source XT1 @ 32.768 */
void
pwm_init(void)
{
#if _MCU_ == 2553
  dint();

  /* null out both PWM-devices */
  pwms[0].pin = 0;
  pwms[0].on_time = 0;
  pwms[1].pin = 0;
  pwms[1].on_time = 0;

  /* count up to CCR0 == period, clock source external crystal @ 32.768 kHz */
  TA1CCTL0 = CCIE;
  TA1CCR0 = period;
  TA1CTL = TASSEL_1 | ID_0;
  TA1CTL |= MC_1;

  eint();
#endif    /* _MCU_ == 2553 */
}
/*--------------------------------------------------------------------------*/
/* Turn on PWM on a PWM-device, for a certain pin on port 1, at a duty cycle dc;
note, this makes an calculation with a division so it might be slow. Use 
pwm_on_fine if possible/necessary/fun. */
void
pwm_on(uint8_t pwmdevice, uint8_t pin, uint8_t dc)
{
#if _MCU_ == 2553
  if(pin > 7 || dc == 0 || dc > 100 || pwmdevice >= PWMDEVS) {
    return;

  } else {
    /* convert duty cycle to PWM clock ticks */
    uint8_t finetime = (dc * period) / 100;

    /* set PWM accordingly */
    if(pwmdevice == 0) {
      TA1CCR1 = finetime;
      TA1CCTL1 = CCIE;
    } else {
      TA1CCR2 = finetime;
      TA1CCTL2 = CCIE;
    }

    /* set up a compare match for the next PWM period */
    pwms[pwmdevice].pin = pin;
    pwms[pwmdevice].on_time = finetime;
  }
#endif    /* _MCU_ == 2553 */
}
/*--------------------------------------------------------------------------*/
/* For finer control, this function can be used to set duty time in ticks instead
of a duty cycle in percent, eg for servo motor control. */
void
pwm_on_fine(uint8_t pwmdevice, uint8_t pin, uint8_t finetime)
{
#if _MCU_ == 2553
  /* sanity checks */
  if(pin > 7 || finetime == 0 || finetime > period || pwmdevice >= PWMDEVS) {
    return;
  }

  /* set up a compare match for the next PWM period */
  if(pwmdevice == 0) {
    TA1CCR1 = finetime;
    TA1CCTL1 = CCIE;
  } else {
    TA1CCR2 = finetime;
    TA1CCTL2 = CCIE;
  }
  pwms[pwmdevice].pin = pin;
  pwms[pwmdevice].on_time = finetime;
#endif    /* _MCU_ == 2553 */
}
/*--------------------------------------------------------------------------*/
/* get the period in ticks sourced at 32768 Hz. */
uint16_t
pwm_period(void)
{
#if _MCU_ == 2553
  return period;
#else    /* _MCU_ == 2553 */
  return 0;
#endif    /* _MCU_ == 2553 */
}
/*--------------------------------------------------------------------------*/
void
pwm_off(uint8_t pwmdevice)
{
#if _MCU_ == 2553
  if(pwmdevice >= PWMDEVS) {
    return;
  }

  if(pwmdevice == 0) {
    TA1CCTL1 &= ~(CCIE);
  } else {
    TA1CCTL2 &= ~(CCIE);
  }
  P1OUT &= ~(1 << pwms[pwmdevice].pin);
  pwms[pwmdevice].on_time = 0;
#endif    /* _MCU_ == 2553 */
}
/*--------------------------------------------------------------------------*/
/* returns the current duty cycle of a PWM device */
// XXX incorrect, currently returns the on-time, not dc.
uint8_t
pwm_dc(uint8_t pwmdevice)
{
#if _MCU_ == 2553
  if(pwmdevice >= PWMDEVS) {
    return 0;
  }
  return pwms[pwmdevice].on_time;
#else    /* _MCU_ == 2553 */
  return 0;
#endif    /* _MCU_ == 2553 */
}
/*--------------------------------------------------------------------------*/
/* Switch off all PWM-devices */
void
pwm_all_off(void)
{
#if _MCU_ == 2553
  if(pwms[0].on_time != 0) {
    P1OUT &= ~(1 << pwms[0].pin);
    pwms[0].on_time = 0;
  }
  TA1CCTL1 &= ~(CCIE);

  if(pwms[1].on_time != 0) {
    P1OUT &= ~(1 << pwms[1].pin);
    pwms[1].on_time = 0;
  }
  TA1CCTL2 &= ~(CCIE);
#endif    /* _MCU_ == 2553 */
}
/*--------------------------------------------------------------------------*/
/* ISR for CCR0 compare match ie end of a period */
#if _MCU_ == 2553
ISR(TIMER1_A0, pwm_periodstart_ta1ccr0_isr)
{
  /* ensure next period interrupt happens at correct time */
  TA1CCTL0 &= ~CCIFG;

  /* clear PWM-devices output pins */
  if(pwms[0].on_time != 0) {
    P1OUT &= ~(1 << pwms[0].pin);
  }
  if(pwms[1].on_time != 0) {
    P1OUT &= ~(1 << pwms[1].pin);
  }
}
#endif    /* _MCU_ == 2553 */
/*---------------------------------------------------------------------------*/
/* ISR for CCR1, CCR2 compare match ie end of a PWM on period */
#if _MCU_ == 2553
ISR(TIMER1_A1, pwm_ccrmatch_ta1ccrX_isr)
{
  /* store the IV register as any read/write resets interrupt flags */
  uint16_t ivreg = TA1IV;
  if(ivreg & TA1IV_TACCR1) {
    /* ensure next PWM interrupt happens at correct time */
    TA1CCTL1 &= ~CCIFG;
    /* set output */
    if(pwms[0].on_time != 0) {
      P1OUT |= (1 << pwms[0].pin);
    }

  } else if(ivreg & TA1IV_TACCR2) {
    /* ensure next PWM interrupt happens at correct time */
    TA1CCTL2 &= ~CCIFG;
    /* set output */
    if(pwms[1].on_time != 0) {
      P1OUT |= (1 << pwms[1].pin);
    }
  }
}
#endif    /* _MCU_ == 2553 */
/*---------------------------------------------------------------------------*/

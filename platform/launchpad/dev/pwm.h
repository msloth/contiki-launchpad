
#ifndef __PWM_H__
#define __PWM_H__

void      pwm_init(void);
void      pwm_on(uint8_t pwmdevice, uint8_t pin, uint8_t dc);
void      pwm_on_fine(uint8_t pwmdevice, uint8_t pin, uint8_t finetime);
void      pwm_off(uint8_t pwmdevice);
uint8_t   pwm_dc(uint8_t pwmdevice);
void      pwm_all_off(void);
uint16_t  pwm_period(void);

#endif /* __PWM_H__ */



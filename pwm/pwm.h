/*
 * pwm.h
 *
 *  Created on: Oct 13, 2014
 *      Author: jcobb
 */

#ifndef PWM_H_
#define PWM_H_

void pwm_terminal_in_cb(uint8_t c);
void pwm_init();
void pwm_tick();
void pwm_setval(uint16_t val, uint16_t channel);

#endif /* PWM_H_ */

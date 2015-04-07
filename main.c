/*
 * main.c
 *
 *  Created on: Sep 11, 2014
 *      Author: jcobb
 */

#define F_CPU	8000000

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include "i2c/i2c.h"
#include "util/log.h"
#include "util/clock.h"
#include "util/config.h"
#include "imu/imu.h"
#include "imu/gyro.h"
#include "eeprom/eeprom.h"
#include "gimbal/gimbal.h"
#include "pwm/pwm.h"
#include "cli/cli.h"

// log debugging
static const char _tag[] PROGMEM = "main: ";
volatile char term_in = 0;
float center_val = 0.0f;





// pid variables
clock_time_t pid_timer = 0;
clock_time_t last_pid_read = 0;
float pid_val_x = 0.0f;
float pid_val_y = 0.0f;
//servo on channel 4 & 5
#define duty_min 600 //.6 ms
#define duty_max 2400 //2.4 ms
#define duty_mid 1500 //1.5ms

uint16_t pwm_out_x = duty_mid;
uint16_t pwm_out_y = duty_mid;
// local porototypes
void balance();

// https://github.com/TKJElectronics/KalmanFilter/tree/master/examples
// https://github.com/TKJElectronics/BalancingRobotArduino/blob/master/BalancingRobotArduino.ino
// https://github.com/TKJElectronics/KalmanFilter


void terminal_in_cb(uint8_t c)
{
	//term_in = c;
	cli_put_char(c);

}

int main()
{
	// terminal callback for command line input
	debug_init(terminal_in_cb);

	/*
	 * initialize command line interpreter
	 */
	cli_init();

	// initialize clock
	clock_init();
	// enable interrupts
	sei();

	// seed the pid loop
	last_pid_read = clock_time();

	LOG("\r\n\r\navr_328p_gimbal starting...\r\n");

	/*
	 * load configuration
	 */
	LOG("config_init...\r\n");
	config_init();


	/*
	 * i2c/gyro/imu initialization
	 */
	// First we need to join the I2C bus
	LOG("joining i2c bus...\r\n");
	i2c_begin();


	/*
	 * load configuration
	 */
	LOG("gyro_init...\r\n");
	gyro_init();


	/*
	 * gimbal is the main state machine
	 * for processing stabilization
	 */
	LOG("gimbal_init...\r\n");
	gimbal_init();

	center_val = kal_angle_x;
	pwm_init();

	LOG("starting gimbal loop...\r\n");

	int16_t axis_rotation[3];

	while(1)
	{
		cli_tick();

//		gimbal_tick();
//		balance();
	}

	return 0;

}

void balance()
{
	pid_val_x = (center_val - gimbal_angle.X);
	pid_val_y = (center_val - gimbal_angle.Y);

	pwm_out_x = duty_mid + (int)(pid_val_x*10);

	pwm_out_y = duty_mid + (int)(pid_val_y*10);

	if(pwm_out_x > 2400)
		pwm_out_x = 2400;
	if(pwm_out_x < 600)
		pwm_out_x = 600;

	if(pwm_out_y > 2400)
		pwm_out_y = 2400;
	if(pwm_out_y < 600)
		pwm_out_y = 600;

	//LOG("PWM OUT: %d\r\n", pwm_out_x);
	pwm_setval(pwm_out_x, 4);
	pwm_setval(pwm_out_y, 5);
}







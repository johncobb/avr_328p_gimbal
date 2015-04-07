/*
 * config.h
 *
 *  Created on: Oct 24, 2014
 *      Author: jcobb
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <avr/pgmspace.h>


// types of config parameters
enum conf_type {
  BOOL,
  INT8,
  INT16,
  INT32,
  UINT8,
  UINT16,
  UINT32,
  FLOAT
};

typedef uint8_t crc;

typedef struct
{
	int32_t		foo;
	int32_t		bar;
	float		foobar;
	int32_t		abc;
	int32_t		abc123;
	int32_t 	gyro_pitch_kp;
	int32_t		gyro_pitch_ki;
	int32_t		gyro_pitch_kd;
	int32_t 	gyro_roll_kp;
	int32_t		gyro_roll_ki;
	int32_t		gyro_roll_kd;
	int16_t 	acc_time_constant;
	int16_t 	angle_offset_pitch;   // angle offset, deg*100
	int16_t 	angle_offset_roll;
	int8_t		dir_motor_pitch;
	int8_t		dir_motor_roll;
	uint8_t		motor_number_pitch;
	uint8_t		motor_number_roll;
	int8_t		max_pwm_motor_pitch;
	int8_t		max_pwm_motor_roll;
	uint16_t	ref_voltage_bat;
	uint16_t	cuttoff_voltage;
	bool 		motor_power_scale;
	bool		enable_gyro;
	bool		enable_acc;
	bool 		axis_reverse_z;
	bool		axis_swap_xy;
	bool		fpv_freeze_pitch;
	bool		fpv_freeze_roll;
	uint8_t 	max_pwm_fpv_pitch;
	uint8_t 	max_pwm_fpv_roll;
	bool 		gyro_calibrate;
	int16_t		gyro_offset_x;
	int16_t		gyro_offset_y;
	int16_t		gyro_offset_z;
	int16_t 	acc_offset_x;
	int16_t 	acc_offset_y;
	int16_t		acc_offset_z;
	uint8_t 	crc8;
} config_t;


extern config_t config;

extern float resolution_divider;


#define CONFIG_NAME_MAXLEN 20
typedef struct cfg_def {
	char name[CONFIG_NAME_MAXLEN];
	int32_t type;
	void * parm_address;
	void (* update)(void);
} t_config_def;

t_config_def config_def;

typedef union {
	t_config_def c;
	char		 bytes[sizeof(t_config_def)];
} t_config_union;

t_config_union config_union;

int8_t init_foo();
int8_t init_bar();
int8_t init_foobar();







void config_init();
void load_config();

void config_test();
void config_set(char *entry, char *parm);
void write_config(t_config_def * def, int32_t val);
void write_config_f(t_config_def *def, float val);
//void write_config();

#endif /* CONFIG_H_ */

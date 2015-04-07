/*
 * config.c
 *
 *  Created on: Oct 24, 2014
 *      Author: jcobb
 */

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <avr/pgmspace.h>
#include "config.h"
#include "../util/log.h"
#include "../eeprom/eeprom.h"
#include "../math/fast_math.h"

static const char _tag[] PROGMEM = "cli: ";

config_t config;
float resolution_divider;

const t_config_def PROGMEM config_list_pgm[] = {
		{"foo", INT8, &config.foo, &init_foo},
		{"bar", INT8, &config.bar, &init_bar},
		{"foobar", FLOAT, &config.foobar, &init_foobar},
		{"", BOOL, NULL, NULL}
};

void get_pgm_string(PGM_P s, char *d, int num_bytes);
t_config_def * get_config_def(char *name);


void config_init()
{
	config.foo = 0;
	config.bar = 0;
	config.foobar = 0.000f;
	config.gyro_pitch_kp = 20000;
	config.gyro_pitch_ki = 10000;
	config.gyro_pitch_kd = 40000;
	config.gyro_roll_kp = 20000;
	config.gyro_roll_ki = 8000;
	config.gyro_roll_kd = 30000;
	config.acc_time_constant = 7;
	config.angle_offset_pitch = 0;
	config.angle_offset_roll = 0;
	config.dir_motor_pitch = 1;
	config.dir_motor_roll = -1;
	config.motor_number_pitch = 0;
	config.motor_number_roll = 1;
	config.max_pwm_motor_pitch = 80;
	config.max_pwm_motor_roll = 80;
	config.ref_voltage_bat = 800;
	config.cuttoff_voltage = 600;
	config.motor_power_scale = 0;
	config.enable_gyro = true;
	config.enable_acc = true;
	config.axis_reverse_z = true;
	config.axis_swap_xy = false;
	config.fpv_freeze_pitch = false;
	config.fpv_freeze_roll = false;
	config.max_pwm_fpv_pitch = 80;
	config.max_pwm_fpv_roll = 80;
	config.gyro_calibrate = true;
}


void load_config()
{
	//eeprom_read(0, config);

	if (config.crc8 == crc_slow((crc *)&config, sizeof(config)-1))
	{
		//updateAllParameters();
	} else {
		// crc failed intialize directly here, as readEEPROM is void
		//printMessage(MSG_WARNING, F("EEPROM CRC failed, initialize to default"));
		//setDefaultParameters();
		config.crc8 = crc_slow((crc *)&config, sizeof(config)-1); // set proper CRC
		//eeprom_write(0, config);
	}
}


void get_pgm_string(PGM_P s, char *d, int num_bytes)
{
	for(int i=0; i<num_bytes; i++) {
		*d++ = pgm_read_byte(s++);
	}
}

void config_test()
{
	t_config_def * def = get_config_def("abc");

	if(def != NULL)
		write_config(def, 123);

}

//void config_set(char *entry, int32_t parm)
void config_set(char * entry, char * parm)
{
	t_config_def * def = get_config_def(entry);

	if(def != NULL)
		if(def->type == FLOAT)
			write_config_f(def, atof(parm));
		else
			write_config(def, atoi(parm));
	else
		LOG("entry not found\r\n");
}

t_config_def * get_config_def(char *name)
{
	void *addr = NULL;
	bool found = false;

	t_config_def * p = (t_config_def *) config_list_pgm;

	while(true) {
		get_pgm_string((PGM_P)p, config_union.bytes, sizeof(config_def));
		if(config_union.c.parm_address == NULL) break;
		if(strncmp(config_union.c.name, name, CONFIG_NAME_MAXLEN) == 0) {
			addr = config_union.c.parm_address;
			found = true;
			break;
		}
		p++;
	}

	if(found)
		return &config_union.c;
	else
		return NULL;
}

void write_config(t_config_def * def, int32_t val)
{
	if(def == NULL) {
		LOG("illegal parameter\r\n");
		return;
	}

	switch(def->type) {
		case BOOL	: *(bool *) (def->parm_address) 	= val; break;
		case UINT8	: *(uint8_t *) (def->parm_address) 	= val; break;
		case UINT16	: *(uint16_t *) (def->parm_address) = val; break;
		case UINT32	: *(uint32_t *) (def->parm_address) = val; break;
		case INT8	: *(int8_t *) (def->parm_address) 	= val; break;
		case INT16	: *(int16_t *) (def->parm_address) 	= val; break;
		case INT32	: *(int32_t *) (def->parm_address) 	= val; break;
	}

	if(def->update != NULL) def->update();

}

void write_config_f(t_config_def *def, float val)
{
	if(def == NULL) {
		LOG("illegal parameter\r\n");
		return;
	}

	 *(float *) (def->parm_address) = val;

	 if(def->update != NULL) def->update();
}

int8_t init_foo() {
	LOG("foo=%d\r\n", config.foo);
}

int8_t init_bar() {
	LOG("bar=%d\r\n", config.bar);
}

int8_t init_foobar(){
	LOG("foobar=%f\r\n", config.foobar);

}

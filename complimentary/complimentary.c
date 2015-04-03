/*
 * complimentary.c
 *
 *  Created on: Mar 17, 2015
 *      Author: Will
 */
#include "../util/clock.h"
#include <math.h>
#include "complimentary.h"

const float gyro_sensitivity = 131.0f;
clock_time_t delta_t, t_now, last_t = 0;

void ComplimentaryFilter (int16_t acc[3], int16_t gyro[3], float *pitch, float *roll)
{
	float pitch_acc, roll_acc;

	if (last_t == 0)
	{
		last_t = clock_time();
		return;
	}

	t_now = clock_time();
	delta_t = t_now - last_t;
	float dt = ((float)delta_t)/1000.0f;

	last_t = t_now;

	*pitch += ((float)gyro[0] / gyro_sensitivity) * dt;
	*roll -= ((float)gyro[1] / gyro_sensitivity) * dt;

	int acc_magnitude = abs(acc[0]) + abs(acc[1]) + abs(acc[2]);

	if (acc_magnitude > 8192 && acc_magnitude < 32768)
	{
		pitch_acc = atan2f((float)acc[1], (float)acc[2]) * 180/MY_PI;
		//HERE IS WHERE YOU TUNE THE COMPLIMENTARY FILTER FOR PITCH!!!
		*pitch = (*pitch * GYRO_WEIGHT) + (pitch_acc * ACC_WEIGHT);

		roll_acc = atan2f((float)acc[0], (float)acc[2]) * 180/MY_PI;
		//HERE IS WHERE YOU TUNE THE COMPLIMENTARY FILTER FOR ROLL!!!
		*roll = (*roll * GYRO_WEIGHT) + (roll_acc * ACC_WEIGHT);
	}
}

/*

 * gimbal.c
 *
 *  Created on: Oct 28, 2014
 *      Author: jcobb
 */
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include "../util/config.h"
#include "../util/clock.h"
#include "../util/log.h"
#include "../math/fast_math.h"
#include "../imu/imu.h"
#include "../imu/gyro.h"
#include "../kalman/kalman2.h"


//#include "../kalman/kalman.h"
#include "gimbal.h"

// https://github.com/sparkfun/MPU-9150_Breakout/blob/master/firmware/MPU6050/Examples/MPU9150_AHRS.ino

static const char _tag[] PROGMEM = "gimbal: ";

volatile int8_t gimbal_state = GIM_IDLE;



// state timeout management
static volatile clock_time_t future = 0;
static bool timeout();
static void set_timer(clock_time_t timeout);

// local prototypes
//Here was the drift error source. Variable declarations were off.
void read_sensor_data(int16_t* gX, int16_t* gY, int16_t* gZ, int16_t* mX, int16_t* mY, int16_t* mZ);

void gimbal_kalman_angle();

uint16_t mag_count = 0; // used to control display output rate
uint8_t mag_rate = 10;     // read rate for magnetometer data

pid_data_t pitch_pid_par;
pid_data_t roll_pid_par;

clock_time_t _last_time_read = 0;

float _last_angle_x = 0.0f;
float _last_angle_y = 0.0f;
float _last_angle_z = 0.0f;
float _last_gyro_angle_x = 0.0f;
float _last_gyro_angle_y = 0.0f;
float _last_gyro_angle_z = 0.0f;
kalman_state kalman_roll;
kalman_state kalman_pitch;

//ACC Variables
const float accel_alpha = 0.5f;
int16_t aX = 0;
int16_t aY = 0;
int16_t aZ = 0;

const float g_sensitivity = 131.0f; // for 250 deg/s, check datasheet

t_fp_angle_t gimble_angle = {0.0f, 0.0f, 0.0f};

float kal_angle_x = 0.0f;
float kal_angle_y = 0.0f;

enum gimbal_task {
	READACC = 0,
	UPDATEACC = 1,
	VOLTAGECOMP = 2
};


void gimbal_init()
{

	/*
	 * init pid parameters
	 */
	LOG("init_pids...\r\n");
	init_pids();

	//kalman_init2(&kalman_roll, 0.001f, 0.003f,  0.03f, 0.0f, 0.0f);
	//kalman_init();

	// initial tunable variables
	kalman_roll.q_angle = 0.1f;
	kalman_roll.q_bias = 0.03f;
	kalman_roll.r_measure = 0.03f;

	kalman_roll.k_angle = 0.0f; // reset the angle
	kalman_roll.bias = 0.0f; // reset bias

	kalman_pitch.q_angle = 0.1f;
	kalman_pitch.q_bias = 0.03f;
	kalman_pitch.r_measure = 0.03f;

	kalman_pitch.k_angle = 0.0f; // reset the angle
	kalman_pitch.bias = 0.0f; // reset bias


	// Since we assume that the bias is 0 and we know the starting angle (use setAngle),
	// the error covariance matrix is set like so -
	// see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical

	kalman_roll.P[0][0] = 0.0f;
	kalman_roll.P[0][1] = 0.0f;
	kalman_roll.P[1][0] = 0.0f;
	kalman_roll.P[1][1] = 0.0f;

	kalman_pitch.P[0][0] = 0.0f;
	kalman_pitch.P[0][1] = 0.0f;
	kalman_pitch.P[1][0] = 0.0f;
	kalman_pitch.P[1][1] = 0.0f;

	_last_time_read = clock_time();

}

void gimbal_tick()
{
	gimbal_kalman_angle();
}


void read_sensor_data(int16_t* gX, int16_t* gY, int16_t* gZ, int16_t* mX, int16_t* mY, int16_t* mZ)
{

	int16_t temp_X, temp_Y, temp_Z;

	mag_count++;
	// *** ACCEL ***

	imu_get_acceleration(&temp_X, &temp_Y, &temp_Z);
	//LOG("ax: %d\r\n", *aX);


	// apply calibration offsets
	/*
	temp_X = (temp_X - config.acc_offset_x);
	temp_Y = (temp_Y - config.acc_offset_y);
	temp_Z = (temp_Z - config.acc_offset_z);
	*/

	//Use low-pass filter to help stabilize readings
	//Low Pass Filter
	aX = accel_alpha*temp_X - (aX)*(1.0 - accel_alpha);
	aY = accel_alpha*temp_Y - (aY)*(1.0 - accel_alpha);
	aZ = accel_alpha*temp_Z - (aZ)*(1.0 - accel_alpha);


	imu_get_rotation(gX, gY, gZ);
	//Since the variables were declared/passed wrong. This step was never applied and we were getting raw Gyro data back.
	*gX = (*gX - config.gyro_offset_x) / g_sensitivity;
	*gY = (*gY - config.gyro_offset_y) / g_sensitivity;
	*gZ = (*gZ - config.gyro_offset_z) / g_sensitivity;

	//LOG("gyro_x, acc_x: %d %d\r\n", gX, aX);


	if (mag_count > 1000/mag_rate) {
		imu_get_mag(mX, mY, mZ);
		*mX = (*mX)*10*1229/4096 + 18; // milliGauss (1229 microTesla per 2^12 bits, 10 mG per microTesla)
		*mY = (*mY)*10*1229/4096 + 70; // apply calibration offsets in mG that correspond to your environment and magnetometer
		*mZ = (*mZ)*10*1229/4096 + 270;
		mag_count = 0;
	}

}

#define RESTRICT_PITCH

void gimbal_kalman_angle()
{
	int16_t gX=0, gY=0, gZ=0, mX=0, mY=0, mZ=0;

	read_sensor_data(&gX, &gY, &gZ, &mX, &mY, &mZ);

	clock_time_t t_now = clock_time();


	clock_time_t delta_t = (t_now-_last_time_read);

	LOG("ax, ay, az, gx, gy, gz: %d %d %d %d %d %d\r\n", aX, aY, aZ, gX, gY, gZ);

	float dt = ((float)delta_t)/1000.0f;

	float accel_x = (float)aX;
	float accel_y = (float)aY;
	float accel_z = (float)aZ;

	float gyro_x = (float)gX;
	float gyro_y = (float)gY;
	float gyro_z = (float)gZ;

	gyro_x = ((float) gX)*DEG_TO_RAD;
	gyro_y = ((float) gY)*DEG_TO_RAD;
	gyro_z = ((float) gZ)*DEG_TO_RAD;


	// restricting pitch
#ifdef RESTRICT_PITCH
	float roll = atan2(accel_y, accel_z) * RAD_TO_DEG;
	float pitch = atan(-accel_x / sqrt(pow(accel_y, 2) + pow(accel_z, 2))) * RAD_TO_DEG;

	if((roll < -90.0f && kal_angle_x > 90.0f) || (roll > 90.0f && kal_angle_x < -90.0f)) {
		kalman_roll.k_angle = roll;
		kal_angle_x = roll;

	} else {
		kal_angle_x = get_angle2(&kalman_roll, roll, gyro_x, dt);
	}

	if(abs(kal_angle_x) > 90)
		gyro_y = -gyro_y; // invert rate so it fits the restricted accelerometer reading

	kal_angle_y = get_angle2(&kalman_pitch, pitch, gyro_y, dt);
#else
	// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	if((pitch < -90 && kal_angle_y > 90) || (pitch > 90 && kal_angle_y < -90)) {
		kalman_pitch.k_angle = pitch;
		kalman_angle_y = pitch;
	} else {
		kal_angle_y = get_angle2(&kalman_pitch, pitch, gyro_y, dt);
	}

	if(abs(kal_angle_y) > 90)
		gyro_x = -gyro_x;

	kal_angle_x = get_angle2(&kalman_roll, gyro_x, dt);

#endif

	// reset when gyro accumulates too much drift
	if(gyro_x < -180 || gyro_x > 180)
		gyro_x = kal_angle_x;
	if(gyro_y < -180 || gyro_y > 180)
		gyro_y = kal_angle_y;


	_last_time_read = t_now;

	gimbal_angle.X = kal_angle_x;
	gimbal_angle.Y = kal_angle_y;
	LOG("roll/pitch/yaw %f:%f:%f\r\n", kal_angle_x, kal_angle_y, 0.0f);

}

static void set_timer(clock_time_t timeout)
{
	future = clock_time() + timeout;
}

// timeout routine to demonstrate clock_time
// being kept by pwm isr interrupt
static bool timeout()
{
	bool timeout = false;

	if(clock_time() >= future)
	{
		set_timer(1000);
		timeout = true;

	}

	return timeout;
}



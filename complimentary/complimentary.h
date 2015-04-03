/*
 * complimentary.h
 *
 *  Created on: Mar 17, 2015
 *      Author: Will
 */

#ifndef COMPLIMENTARY_H_
#define COMPLIMENTARY_H_


#define MY_PI 3.14159265359
//The weights must sum to 1
#define GYRO_WEIGHT 0.98
#define ACC_WEIGHT 0.02

void ComplimentaryFilter (int16_t acc[3], int16_t gyro[3], float *pitch, float *roll);



#endif /* COMPLIMENTARY_H_ */

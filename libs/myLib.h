/**
 * myLib.h
 *
 *  Created on: 20. 4. 2015.
 *      Author: Vedran
 */

#ifndef MYLIB_H_
#define MYLIB_H_

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

//  Common error codes
#define STATUS_OK 					0
#define STATUS_ARG_ERR			    1
#define STATUS_PROG_ERR             2

#define PI_CONST 	3.14159265f
#define GRAVITY_CONST   9.80665f //  m/s^2

#ifdef __cplusplus
extern "C"
{
#endif

/*		Math-related function		*/
int32_t interpolate(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t _x);
float   finterpolatef(float x1, float y1, float x2, float y2, float _x);
int32_t min(int32_t arg1, int32_t arg2);

/*		Functions for converting string to number		*/
float   stof (uint8_t *nums, uint8_t strLen);
int32_t stoi (uint8_t *nums, uint8_t strLen);
int32_t stoiv (volatile uint8_t *nums, volatile uint8_t strLen);

/*      Functions to convert number to string           */
void    itoa (int32_t num, uint8_t *str);

#ifdef __cplusplus
}
#endif

#endif /* MYLIB_H_ */

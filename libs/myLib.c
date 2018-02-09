/**
 * myLib.c
 *
 *  Created on: 29. 5. 2016.
 *      Author: Vedran
 */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "myLib.h"

/**
 * Interpret sensor distance based on the interval
 * @param (x1,y1), (x2,y2) points that form a line
 * @param _x point on the line for which we search y value
 * @return y value from line for corresponding _x
 */
int32_t interpolate(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t _x)
{
	return ( (y2-y1) * (_x-x1) / (x2-x1) + y1 );
}

float finterpolatef(float x1, float y1, float x2, float y2, float _x)
{
    return ( (y2-y1) * (_x-x1) / (x2-x1) + y1 );
}


/**
 * Returns value of smaller of the two passed arguments
 * @param arg1
 * @param arg2
 * @return
 */
int32_t min(int32_t arg1, int32_t arg2)
{
	return ((arg1 < arg2) ? arg1 : arg2);
}

/**
 * For a given string containing a number, this function will attempt to
 * 	convert that string to a corresponding float value and return it when done.
 * 	@param nums string containing a number
 * 	@param strLen length of nums variable
 * 	@return float value of number in nums string
 */
float stof (uint8_t *nums, uint8_t strLen)
{
	float retVal = 0, 		//Value to return
		  multiplier = 1;	//Digit multiplier
	uint8_t itB = 0, 		//Beginning of a whole part, iterator
			itE = 0;		//End of whole part, iterator
	int8_t i;				//Loop variable

	//  Check if we're dealing with negative number, and adjust beginning iterator
	if (nums[0] == '-') itB = 1;

	//  Find position of decimal dot, and save it in ending iterator variable
	while((nums[itE] != '.') && (nums[itE] != '\0') && (itE < strLen)) itE++;

	/*
	 * Loop from the dot to the beginning of string (to the left) taking each
	 * 	digit and adding it to the float after applying correct multiplier.
	 * 	Increase multiplier every time iterator move a digit to the left.
	 */
	for (i = (itE - 1); i >= itB; i--)
	{
		//  Precaution
		if ((nums[i] < 48) || (nums[i] > 58)) continue;

		retVal += ((int)nums[i] - 48) * multiplier;
		multiplier *= 10;
	}

	//  Prepare multiplier for calculating decimal part
	multiplier = 0.1;

	/*
	 * Loop from the dot to the end of the string (to the right) taking each
	 * 	digit and adding it to the float after applying correct multiplier.
	 * 	Decrease multiplier every time iterator moves a digit to the right.
	 */
	for (i = (itE + 1); i < strLen; i++)
	{
		//  Precaution
		if ((nums[i] < 48) || (nums[i] > 58)) continue;

		retVal += (float)((int32_t)nums[i] - 48) * multiplier;
		multiplier /= 10;
	}

	//  Apply correct sign to the float variable
	if (itB == 1) retVal *= (-1);

	return retVal;
}

/**
 * For a given string containing a number, this function will attempt to
 * 	convert that string to a corresponding integer value and return it when done.
 * 	@param nums string containing a number
 * 	@param strLen length of nums variable
 * 	@return int32_t value of number in nums string
 */
int32_t stoi (uint8_t *nums, uint8_t strLen)
{
	int32_t retVal = 0, 		//Value to return
			 multiplier = 1;	//Digit multiplier
	uint8_t itB = 0; 			//Beginning of a number, iterator
	int8_t i;					//Loop variable

	//  Check if we're dealing with negative number, and adjust beginning iterator
	if (nums[0] == '-') itB = 1;

	/*
	 * Loop from the dot to the beginning of string (to the left) taking each
	 * 	digit and adding it to the float after applying correct multiplier.
	 * 	Increase multiplier every time iterator move a digit to the left.
	 */
	for (i = (strLen - 1); i >= itB; i--)
	{
		//  Precaution
		if ((nums[i] < 48) || (nums[i] > 58)) continue;

		retVal += ((int32_t)nums[i] - 48) * multiplier;
		multiplier *= 10;
	}

	//  Apply correct sign to the variable
	if (itB == 1) retVal *= (-1);

	return retVal;
}

int32_t stoiv (volatile uint8_t *nums, volatile uint8_t strLen)
{
    int32_t retVal = 0,         //  Value to return
             multiplier = 1;    //  Digit multiplier
    uint8_t itB = 0;            //  Beginning of a number, iterator
    int8_t i;                   //  Loop variable

    //  Check if we're dealing with negative number, and adjust beginning iterator
    if (nums[0] == '-') itB = 1;

    /*
     * Loop from the dot to the beginning of string (to the left) taking each
     *  digit and adding it to the float after applying correct multiplier.
     *  Increase multiplier every time iterator move a digit to the left.
     */
    for (i = (strLen - 1); i >= itB; i--)
    {
        //  Precaution
        if ((nums[i] < 48) || (nums[i] > 58)) continue;

        retVal += ((int32_t)nums[i] - 48) * multiplier;
        multiplier *= 10;
    }

    //  Apply correct sign to the variable
    if (itB == 1) retVal *= (-1);

    return retVal;
}
/**
 * Convert any integer number to string
 * @param num input number to convert
 * @param str char array to store convert integer to
 */
void itoa (int32_t num, uint8_t *str)
{
    uint8_t it = 0;

    if (num < 0)
    {
        str[it++]= '-';
        num = labs(num);
    }

    uint8_t digits = 1;

    //  Find length of the string
    while ( (num / ((int32_t)powf(10.0f, (float)digits))) > 0)
        digits++;

    //  We'll start adding digits from the right to the left of the number,
    //  adjust iterator accordingly
    it += (digits -1);
    //  Convert digits to string (48 is ASCII offset for digit)
    while ((digits--) > 0)
    {
        str[it--] = (uint8_t)(48 + num % 10);
        num /=10;
    }
}


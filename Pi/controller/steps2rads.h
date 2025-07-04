// Filename : steps2rads.h 
// Authors : Luis Moreno (s3608255), Luca Provenzano (s3487636)
// Group : 43
// License : N.A. or open source license like LGPL
// Description : Header for helper function for converting steps to radiants
//==============================================================

#ifndef STEPS2RADS_HPP
#define STEPS2RADS_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <math.h>
#include "xxtypes.h"

#define PITCH_RANGE_RAD M_PI/2.0
#define YAW_RANGE_RAD M_PI

/*********************************************
* @brief Converts a number of steps to radians based on the maximum steps and total angle in radians.
* 
* @param [in] steps             Steps to be converted
* @param [in] max_steps         Steps counted in a total angle
* @param [in] total_angle_rad   Total angle in radiants
* 
* @return input steps in radiants
*********************************************/
XXDouble steps2rads(int32_t steps, int32_t max_steps, XXDouble total_angle_rad)
{
    // Handle division by zero
    if (max_steps == 0) {
        return 0.0;
    }
    // Calculate the ratio of travel (0.0 to 1.0) and scale by the total angle
    return ((XXDouble)steps / (XXDouble)max_steps) * total_angle_rad;
}

#ifdef __cplusplus
}
#endif

#endif
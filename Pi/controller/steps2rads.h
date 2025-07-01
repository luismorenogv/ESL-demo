// Filename : steps2rads.h 
// Authors : 
// Group : 
// License : N.A. or open source license like LGPL
// Description : 
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

// Converts a number of steps to radians based on the maximum steps and total angle in radians.
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
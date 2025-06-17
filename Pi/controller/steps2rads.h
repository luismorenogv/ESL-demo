// Filename : steps2rads.h 
// Authors : 
// Group : 
// License : N.A. or open source license like LGPL
// Description : 
//==============================================================

#ifndef STEPS2RADS_HPP
#define STEPS2RADS_HPP

#define MAX_YAW     2454
#define MAX_PITCH   740

#include <stdint.h>
#include <math.h>
#include "xxtypes.h"

XXDouble steps2rads(int32_t steps, int32_t max_steps)
{
    return (2*M_PI*steps)/max_steps;
}

XXDouble pitch2rads(int32_t pitch)
{
    return steps2rads(pitch, MAX_PITCH);
}

XXDouble yaw2rads(int32_t yaw)
{
    return steps2rads(yaw, MAX_YAW);
}

#endif
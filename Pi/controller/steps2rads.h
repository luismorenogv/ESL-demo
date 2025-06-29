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

XXDouble steps2rads(int32_t steps, int32_t max_steps)
{
    return (2*M_PI*steps)/max_steps;
}

#ifdef __cplusplus
}
#endif

#endif
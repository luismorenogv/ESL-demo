// Filename : homing.h 
// Authors : 
// Group : 
// License : N.A. or open source license like LGPL
// Description : description
//==============================================================

#ifndef HOMING_HPP
#define HOMING_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void HomeBothAxes(/*int spi_fd, */int32_t* pitch_offset_out, int32_t* yaw_offset_out,
                                uint32_t* pitch_max_steps, uint32_t* yaw_max_steps);

#ifdef __cplusplus
}
#endif

#endif
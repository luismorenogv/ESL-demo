// motor_control.hpp

#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include <cstdint>
#include "controller/common/xxtypes.h" // For XXDouble


// Finds the physical limits of the gimbal axes and sets the zero offset.
void HomeBothAxes(int spi_fd, int32_t* pitch_offset_out, int32_t* yaw_offset_out,
                  uint32_t* pitch_max_steps, uint32_t* yaw_max_steps);

// The main loop for the high-frequency motor control thread.
void control_thread_func(int spi_fd, int32_t pitch_offset, int32_t yaw_offset, 
                         uint32_t pitch_max_steps, uint32_t yaw_max_steps);

#endif
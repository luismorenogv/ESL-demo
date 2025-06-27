// motor_control.cpp

#include "motor_control.hpp"

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

#include "spi_comm.h"
#include "controller/controller.h"
#include "controller/steps2rads.h"
#include "img_proc.hpp" // Shared data: g_run, g_target_data, g_target_mutex

// Constants for control loop
#define LOOP_HZ         1000000
#define PERIOD_NS       (1000000000L / LOOP_HZ)
#define ENCODER_ERROR_TOLERANCE  2
#define HOMING_STALL_THRESHOLD  50
#define MAX_SAFE_DUTY  ((uint16_t)(0.2 * ((1 << 12) - 1)))

void HomeBothAxes(int spi_fd, int32_t* pitch_offset_out, int32_t* yaw_offset_out,
                                uint32_t* pitch_max_steps, uint32_t* yaw_max_steps) {
    printf("Homing both axes simultaneously...\n");

    SendAllPwmCmd(spi_fd, 0, 0, 0, 0, 0, 0); // Stop motors before homing
    
    uint16_t homing_duty = (uint16_t)(0.15 * ((1 << 12) - 1));
    
    int32_t current_pitch = 0, current_yaw = 0;
    if (ReadPositionCmd(spi_fd, UnitAll, &current_pitch, &current_yaw) < 0) {
        fprintf(stderr, "Homing: Failed initial read.\n");
    }

    int32_t last_pitch = current_pitch;
    int32_t last_yaw = current_yaw;
    
    int pitch_stall_counter = 0;
    int yaw_stall_counter = 0;
    
    bool pitch_homed = false;
    bool yaw_homed = false;

    for (uint8_t i = 0; i < 2; i++) {
        while (!pitch_homed || !yaw_homed) {
            // Enable only the motors that still need homing
            uint16_t pitch_drive_duty = pitch_homed ? 0 : homing_duty;
            uint16_t yaw_drive_duty = yaw_homed ? 0 : homing_duty;
            uint8_t dir = i == 0 ? 1 : 0; // First pass forward, second pass reverse

            SendAllPwmCmd(spi_fd, pitch_drive_duty, !pitch_homed, dir, 
                                yaw_drive_duty, !yaw_homed, dir);

            if (ReadPositionCmd(spi_fd, UnitAll, &current_pitch, &current_yaw) < 0) {
                fprintf(stderr, "Homing: Failed to read position, retrying...\n");
                usleep(10000);
                continue;
            }

            // Detect pitch stall
            if (!pitch_homed) {
                pitch_stall_counter = (abs(current_pitch - last_pitch) < ENCODER_ERROR_TOLERANCE)
                    ? pitch_stall_counter + 1 : 0;
                last_pitch = current_pitch;

                if (pitch_stall_counter >= HOMING_STALL_THRESHOLD) {
                    pitch_homed = true;
                    if (i == 0)
                        *pitch_offset_out = current_pitch;
                    else
                        *pitch_max_steps = abs(current_pitch - *pitch_offset_out);
                }
            }

            // Detect yaw stall
            if (!yaw_homed) {
                yaw_stall_counter = (abs(current_yaw - last_yaw) < ENCODER_ERROR_TOLERANCE)
                    ? yaw_stall_counter + 1 : 0;
                last_yaw = current_yaw;

                if (yaw_stall_counter >= HOMING_STALL_THRESHOLD) {
                    yaw_homed = true;
                    if (i == 0)
                        *yaw_offset_out = current_yaw;
                    else
                        *yaw_max_steps = abs(current_yaw - *yaw_offset_out);
                }
            }

            usleep(10000);
        }

        // Prepare for next homing pass
        pitch_homed = false;
        yaw_homed = false;
        pitch_stall_counter = 0;
        yaw_stall_counter = 0;
    }

    printf("Homing complete for both axes.\n");
    SendAllPwmCmd(spi_fd, 0, 0, 0, 0, 0, 0); // Stop motors
}

void control_thread_func(int spi_fd, int32_t pitch_offset, int32_t yaw_offset, 
                         uint32_t pitch_max_steps, uint32_t yaw_max_steps) {
    printf("Control thread started.\n");

    XXDouble pitch_max_rad = steps2rads((int32_t)pitch_max_steps, (int32_t)pitch_max_steps);
    XXDouble yaw_max_rad   = steps2rads((int32_t)yaw_max_steps, (int32_t)yaw_max_steps);

    XXDouble dt = (XXDouble)PERIOD_NS / 1000000000.0;

    // Forward declaration of loop variables
    int32_t raw_p, raw_y, abs_p, abs_y; 
    XXDouble pitch_curr_pos_rad, yaw_curr_pos_rad, pitch_dst_rad, yaw_dst_rad, pan_out, tilt_out;
    TargetData current_target;
    uint16_t pan_duty, tlt_duty;
    uint8_t pan_dir, tlt_dir;

    // Initialize destination positions to the middle of the range
    pitch_dst_rad = steps2rads((int32_t)pitch_max_steps/2, (int32_t)pitch_max_steps);
    yaw_dst_rad   = steps2rads((int32_t)yaw_max_steps/2, (int32_t)yaw_max_steps);

    // Initialize timing
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    struct timespec last_step, now;
    clock_gettime(CLOCK_MONOTONIC, &last_step);

    while (g_run) {
        {
            std::lock_guard<std::mutex> lock(g_target_mutex);
            current_target = g_target_data;
                if (g_target_data.new_frame) {
                    // Reset the new frame flag
                    g_target_data.new_frame = false;
            }
        }

        if (ReadPositionCmd(spi_fd, UnitAll, &raw_p, &raw_y) < 0) {
            fprintf(stderr, "Error: Failed to read position in control thread.\n");
            g_run = false;
            continue;
        }

        // Convert encoder readings to radians
        abs_p = raw_p - pitch_offset;
        abs_y = raw_y - yaw_offset;
        pitch_curr_pos_rad = steps2rads(abs_p, (int32_t)pitch_max_steps);
        yaw_curr_pos_rad   = steps2rads(abs_y, (int32_t)yaw_max_steps);

        // Set destination based on object size
        if (current_target.new_frame) {
            if (current_target.obj_size <= MIN_OBJ_SIZE) {
                pitch_dst_rad = pitch_curr_pos_rad;
                yaw_dst_rad   = yaw_curr_pos_rad;
            } else {
                yaw_dst_rad   = fmax(0.0, fmin(yaw_curr_pos_rad + current_target.x_offset_rad, yaw_max_rad));
                pitch_dst_rad = fmax(0.0, fmin(pitch_curr_pos_rad + current_target.y_offset_rad, pitch_max_rad));
            }
        }
        // dt calculation
        clock_gettime(CLOCK_MONOTONIC, &now);
        dt = (XXDouble)(now.tv_sec - last_step.tv_sec) + 
             (XXDouble)(now.tv_nsec - last_step.tv_nsec) / 1000000000.0;
        last_step = now;
        ControllerStep(pitch_curr_pos_rad, pitch_dst_rad, yaw_curr_pos_rad, yaw_dst_rad, dt);

        // Get controller outputs
        pan_out  = getPanOut();
        tilt_out = getTiltOut();

        // Convert to PWM signals
        pan_duty = (uint16_t)(fmin(fabs(pan_out), 1.0) * MAX_SAFE_DUTY);
        pan_dir  = (pan_out >= 0.0) ? 0 : 1;
        tlt_duty = (uint16_t)(fmin(fabs(tilt_out), 1.0) * MAX_SAFE_DUTY);
        tlt_dir  = (tilt_out >= 0.0) ? 0 : 1;

        // Send PWM command
        SendAllPwmCmd(spi_fd, tlt_duty, 1, tlt_dir, pan_duty, 1, pan_dir);

        // Wait until next cycle
        next_time.tv_nsec += PERIOD_NS;
        if (next_time.tv_nsec >= 1000000000L) {
            next_time.tv_sec++;
            next_time.tv_nsec -= 1000000000L;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    }

    printf("Control thread finished.\n");
}


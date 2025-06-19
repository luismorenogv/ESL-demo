// main.c
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <stdbool.h> // Include for bool type

#include "spi_comm.h"
#include "controller/controller.h"
#include "controller/steps2rads.h"
#include "img_proc/img_proc.hpp"

#define LOOP_HZ         1000
#define PERIOD_NS       (1000000000L / LOOP_HZ)
#define DEADBAND_CNT    5
#define ENCODER_ERROR_TOLERANCE  2
#define HOMING_STALL_THRESHOLD  50
#define RAD_TOLERANCE    (0.1)
#define MAX_SAFE_DUTY  ((uint16_t)(0.2 * ((1 << 12) - 1)))

#define MIN_OBJ_SIZE 150

// Forward declaration
int error(const char *msg, const int e_code, int fd);

void home_both_axes(int spi_fd, int32_t* pitch_offset_out, int32_t* yaw_offset_out) {
    printf("Homing both axes simultaneously...\n");
    
    uint16_t homing_duty = (uint16_t)(0.15 * ((1 << 12) - 1));
    uint8_t homing_dir = 1; // Move in the negative direction
    
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

    uint16_t pitch_drive_duty;
    uint16_t yaw_drive_duty;

    while (!pitch_homed || !yaw_homed) {
        pitch_drive_duty = pitch_homed ? 0 : homing_duty;
        yaw_drive_duty = yaw_homed ? 0 : homing_duty;

        SendAllPwmCmd(spi_fd, pitch_drive_duty, !pitch_homed, homing_dir, 
                              yaw_drive_duty, !yaw_homed, homing_dir);
    
        if (ReadPositionCmd(spi_fd, UnitAll, &current_pitch, &current_yaw) < 0) {
            fprintf(stderr, "Homing: Failed to read position, retrying...\n");
            usleep(10000);
            continue;
        }

        if (!pitch_homed) {
            if (abs(current_pitch - last_pitch) < ENCODER_ERROR_TOLERANCE) {
                pitch_stall_counter++;
            } else {
                pitch_stall_counter = 0;
            }
            last_pitch = current_pitch;

            if (pitch_stall_counter >= HOMING_STALL_THRESHOLD) {
                pitch_homed = true;
                *pitch_offset_out = current_pitch;
                printf("Pitch axis homed at encoder value: %d\n", *pitch_offset_out);
            }
        }

        if (!yaw_homed) {
            if (abs(current_yaw - last_yaw) < ENCODER_ERROR_TOLERANCE) {
                yaw_stall_counter++;
            } else {
                yaw_stall_counter = 0;
            }
            last_yaw = current_yaw;

            if (yaw_stall_counter >= HOMING_STALL_THRESHOLD) {
                yaw_homed = true;
                *yaw_offset_out = current_yaw;
                printf("Yaw axis homed at encoder value: %d\n", *yaw_offset_out);
            }
        }
        usleep(10000);
    }
    printf("Homing complete for both axes.\n");
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <pitch_counts> <yaw_counts>\n", argv[0]);
        return 1;
    }
    GstElement *pipeline, *sink;
    
    bool pitch_target_met = false;
    bool yaw_target_met = false;

    // 1) parse targets
    int32_t pitch_steps_target;// = strtol(argv[1], NULL, 0);
    int32_t yaw_steps_target;// = strtol(argv[2], NULL, 0);

    // Check for validness against the actual physical range
    /*if (pitch_steps_target > MAX_PITCH || pitch_steps_target < 0) {
        fprintf(stderr, "Warning: Pitch target out of range [0, %d]. Clamping.\n", MAX_PITCH);
        pitch_steps_target = (pitch_steps_target < 0) ? 0 : MAX_PITCH;
    }
    if (yaw_steps_target > MAX_YAW || yaw_steps_target < 0) {
        fprintf(stderr, "Warning: Yaw target out of range [0, %d]. Clamping.\n", MAX_YAW);
        yaw_steps_target = (yaw_steps_target < 0) ? 0 : MAX_YAW;
    }*/

    // pitch and yaw destination
    XXDouble pitch_ref;// = pitch2rads(pitch_steps_target);
    XXDouble yaw_ref;//   = yaw2rads(yaw_steps_target);
    printf("Target: Pitch=%.2f rad (%d steps), Yaw=%.2f rad (%d steps)\n", 
           pitch_ref, pitch_steps_target, yaw_ref, yaw_steps_target);

    // 2) open SPI
    int fd = SpiOpen(SPI_CHANNEL, SPI_SPEED_HZ, SPI_MODE);
    if (fd < 0) return 1;

    // 3) Homing procedure
    SendAllPwmCmd(fd, 0, 0, 0, 0, 0, 0);
    usleep(100000);

    int32_t pitch_offset, yaw_offset;
    double obj_size;
    home_both_axes(fd, &pitch_offset, &yaw_offset);

    SendAllPwmCmd(fd, 0, 0, 0, 0, 0, 0);

    // 4) init your C controller
    ControllerInitialize();
    if (init_gstreamer_pipeline(argv[1], &pipeline, &sink) != 0) return -1;

    // 6) main loop
    while (1) {
        int32_t raw_p, raw_y;
        if (ReadPositionCmd(fd, UnitAll, &raw_p, &raw_y) < 0)
            return error("Failed to read position", 2, fd);
        
        // Calculate absolute position in steps, relative to the homed zero
        int32_t abs_p = raw_p - pitch_offset;
        int32_t abs_y = raw_y - yaw_offset;

        printf("Current Position: Pitch=%d steps, Yaw=%d steps\n", abs_p, abs_y);
        printf("Difference to target: Pitch=%d steps, Yaw=%d steps\n",
               abs_p - pitch_steps_target, abs_y - yaw_steps_target);

        // to radians
        XXDouble pitch_rad = pitch2rads(abs_p);
        XXDouble yaw_rad   = yaw2rads(abs_y);

        process_one_frame(sink, yaw_ref, pitch_ref, obj_size);

        //Object too small, no real object in sight! (TODO: make it better)
        if(obj_size <= MIN_OBJ_SIZE)
        {
            yaw_ref = yaw_rad;
            pitch_ref = pitch_rad;
        }

        // step the C controller
        ControllerStep(pitch_rad, pitch_ref, yaw_rad, yaw_ref);

        // read back
        XXDouble pan_out  = getPanOut();  // Corresponds to Yaw
        XXDouble tilt_out = getTiltOut(); // Corresponds to Pitch

        printf("Controller Output: Pitch=%.2f rad, Yaw=%.2f rad\n",
               tilt_out, pan_out);

        // pack PWM
        uint16_t pan_duty = yaw_target_met ? 0 : (uint16_t)(fmin(fabs(pan_out),1.0) * MAX_SAFE_DUTY);
        uint8_t  pan_dir  = (pan_out >= 0.0) ? 0 : 1;
        uint16_t tlt_duty = pitch_target_met ? 0 : (uint16_t)(fmin(fabs(tilt_out),1.0) * MAX_SAFE_DUTY);
        uint8_t  tlt_dir  = (tilt_out >= 0.0) ? 0 : 1;

        printf("PWM: Pitch=%d (dir=%d), Yaw=%d (dir=%d)\n",
               tlt_duty, tlt_dir, pan_duty, pan_dir);

        // check convergence
        if (fabs(pitch_ref - pitch_rad) < RAD_TOLERANCE){
            pitch_target_met = true;
        }
        if (fabs(yaw_ref   - yaw_rad) < RAD_TOLERANCE) {
            yaw_target_met = true;
        }

        if (pitch_target_met && yaw_target_met) {
            printf("Both targets met. Exiting loop.\n");
            break;
        }
        // send PWM
        SendAllPwmCmd(fd,
                      tlt_duty, /*en=*/1, tlt_dir,  // Pitch (Tilt)
                      pan_duty, /*en=*/1, pan_dir); // Yaw (Pan)

    }

    // 7) stop & close
    printf("Stopping motors and closing SPI.\n");
    SendAllPwmCmd(fd, 0,0,0, 0,0,0);
    SpiClose(fd);
    cleanup_gstreamer_pipeline(pipeline);
    return 0;
}

int error(const char *msg, const int e_code, int fd) {
    fprintf(stderr, "Error %i: %s\n", e_code, msg);
    if (fd >= 0) SpiClose(fd);
    return e_code;
}
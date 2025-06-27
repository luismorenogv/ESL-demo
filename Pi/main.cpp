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
#include "homing.h"


#define ENCODER_ERROR_TOLERANCE  2
#define HOMING_STALL_THRESHOLD  50
//#define MAX_SAFE_DUTY  ((uint16_t)(0.2 * ((1 << 12) - 1)))

#define MIN_OBJ_SIZE 2000

// Forward declaration



int main(int argc, char *argv[]) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <source_file>\n", argv[0]);
        return 1;
    }
    GstElement *pipeline, *sink;

    int32_t pitch_steps_target;
    int32_t yaw_steps_target;

    // pitch and yaw destination
    XXDouble pitch_dst_rad;
    XXDouble yaw_dst_rad;

    // Initialize offsets in radians
    XXDouble x_ball_offset_rad = 0.0, y_ball_offset_rad = 0.0;

    // 2) open SPI
    int fd = SpiOpen(SPI_CHANNEL, SPI_SPEED_HZ, SPI_MODE);
    if (fd < 0) return 1;

    // 3) Homing procedure
    int32_t pitch_offset, yaw_offset;
    uint32_t pitch_max_steps, yaw_max_steps;
    HomeBothAxes(/*fd,*/ &pitch_offset, &yaw_offset, 
                   &pitch_max_steps, &yaw_max_steps);

    // Pre-compute useful encoder positions
    /*XXDouble pitch_middle_rad = steps2rads((int32_t)(pitch_max_steps/2), 
                                            (int32_t)pitch_max_steps);
    XXDouble yaw_middle_rad = steps2rads((int32_t)(yaw_max_steps/2), 
                                          (int32_t)yaw_max_steps);*/
    XXDouble pitch_max_rad = steps2rads((int32_t)pitch_max_steps, 
                                        (int32_t)pitch_max_steps);
    XXDouble yaw_max_rad = steps2rads((int32_t)yaw_max_steps, 
                                      (int32_t)yaw_max_steps);

    double obj_size;
    // 4) init your C controller
    ControllerInitialize();
    if (InitGstreamerPipeline(argv[1], &pipeline, &sink) != 0) return -1;
    
    // 5) Timing setup
    struct timespec last_time, current_time;
    clock_gettime(CLOCK_MONOTONIC, &last_time); // Get the starting time
    XXDouble dt = 0.0;

    // 6) main loop
    while (1) {
        // Timing calculation
        clock_gettime(CLOCK_MONOTONIC, &current_time);
        dt = (current_time.tv_sec - last_time.tv_sec) + 
             (current_time.tv_nsec - last_time.tv_nsec) / 1000000000.0;
        last_time = current_time;

        // printf("Loop time: %.3f seconds\n", dt);
        int32_t raw_p, raw_y;
        if (ReadPositionCmd(fd, UnitAll, &raw_p, &raw_y) < 0)
            return error("Failed to read position", 2, fd);
        
        // Calculate absolute position in steps, relative to the homed zero
        int32_t abs_p = raw_p - pitch_offset;
        int32_t abs_y = raw_y - yaw_offset;

        // Current position in radians
        XXDouble pitch_curr_pos_rad = steps2rads(abs_p, (int32_t)pitch_max_steps);
        XXDouble yaw_curr_pos_rad   = steps2rads(abs_y, (int32_t)yaw_max_steps);

        // Check for new frame
        if (ProcessOneFrame(sink, x_ball_offset_rad, y_ball_offset_rad, obj_size)){
            if(obj_size <= MIN_OBJ_SIZE)
            {
                // No object, command is to hold current position
                yaw_dst_rad = yaw_curr_pos_rad;
                pitch_dst_rad = pitch_curr_pos_rad;
            }
            else
            {
                // Object found, new destination is current position + offset
                XXDouble target_yaw = yaw_curr_pos_rad + x_ball_offset_rad;
                yaw_dst_rad = fmax(0.0, fmin(target_yaw, yaw_max_rad)); //TODO shouldnt this be the offset?

                XXDouble target_pitch = pitch_curr_pos_rad - y_ball_offset_rad;
                pitch_dst_rad = fmax(0.0, fmin(target_pitch, pitch_max_rad));
            }
        }

        ControllerStep(pitch_curr_pos_rad, pitch_dst_rad, yaw_curr_pos_rad, yaw_dst_rad, dt);

        // read back
        //XXDouble pan_out  = getPanOut();  // Corresponds to Yaw
        //XXDouble tilt_out = getTiltOut(); // Corresponds to Pitch
        //printf("Tilt Out: %.2f\n", tilt_out);
        uint16_t pan_duty;
        uint8_t  pan_dir;
        uint16_t tilt_duty;
        uint8_t  tilt_dir;

        getPanOut(&pan_duty, &pan_dir);
        getTiltOut(&tilt_duty, &tilt_dir);
        // printf("Controller Output: Pitch=%.2f rad, Yaw=%.2f rad\n", tilt_out, pan_out);

        // pack PWM
        /*uint16_t pan_duty = (uint16_t)(fmin(fabs(pan_out),1.0) * MAX_SAFE_DUTY);
        uint8_t  pan_dir  = (pan_out >= 0.0) ? 0 : 1;
        uint16_t tlt_duty = (uint16_t)(fmin(fabs(tilt_out),1.0) * MAX_SAFE_DUTY);
        uint8_t  tlt_dir  = (tilt_out >= 0.0) ? 0 : 1;*/

        //printf("\nPWM: Pitch=%d (dir=%d), Yaw=%d (dir=%d)\n", tlt_duty, tlt_dir, pan_duty, pan_dir);

        // send PWM
        SendAllPwmCmd(fd,
                      tilt_duty, /*en=*/1, tilt_dir,  // Pitch (Tilt)
                      pan_duty, /*en=*/1, pan_dir); // Yaw (Pan)

    }

    // 7) stop & close
    printf("Stopping motors and closing SPI.\n");
    //SendAllPwmCmd(fd, 0,0,0, 0,0,0);
    StopMotors();
    SpiClose(fd);
    CleanupGstreamerPipeline(pipeline);
    return 0;
}


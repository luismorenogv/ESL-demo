
#include "homing.h"

void HomeBothAxes(/*int spi_fd, */int32_t* pitch_offset_out, int32_t* yaw_offset_out,
                                uint32_t* pitch_max_steps, uint32_t* yaw_max_steps) {
    printf("Homing both axes simultaneously...\n");

    SendAllPwmCmd(/*spi_fd, */0, 0, 0, 0, 0, 0); // Stop motors before homing
    
    uint16_t homing_duty = (uint16_t)(0.15 * ((1 << 12) - 1));
    
    int32_t current_pitch = 0, current_yaw = 0;
    
    if (ReadPositionCmd(/*spi_fd, */UnitAll, &current_pitch, &current_yaw) < 0) {
        fprintf(stderr, "Homing: Failed initial read.\n");
    }
    int32_t last_pitch = current_pitch;
    int32_t last_yaw = current_yaw;
    
    int pitch_stall_counter = 0;
    int yaw_stall_counter = 0;
    
    bool pitch_homed = false;
    bool yaw_homed = false;

    for (uint8_t i = 0; i < 2; i++){
        while (!pitch_homed || !yaw_homed) {
            uint16_t pitch_drive_duty = pitch_homed ? 0 : homing_duty;
            uint16_t yaw_drive_duty = yaw_homed ? 0 : homing_duty;
            uint8_t dir = i == 0 ? 1 : 0; // Reverse direction on start

            SendAllPwmCmd(/*spi_fd, */pitch_drive_duty, !pitch_homed, dir, 
                                yaw_drive_duty, !yaw_homed, dir);
        
            if (ReadPositionCmd(/*spi_fd, */UnitAll, &current_pitch, &current_yaw) < 0) {
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
                    if (i == 0){
                        *pitch_offset_out = current_pitch;
                        printf("Pitch axis homed at encoder value: %d\n", *pitch_offset_out);
                    } else {
                        *pitch_max_steps = abs(current_pitch - *pitch_offset_out);
                    }
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
                    if (i == 0){
                        *yaw_offset_out = current_yaw;
                        printf("Yaw axis homed at encoder value: %d\n", *yaw_offset_out);
                    } else {
                        *yaw_max_steps = abs(current_yaw - *yaw_offset_out);
                    }
                }
            }
            usleep(10000);
        }
        // Reset values
        yaw_homed = false;
        pitch_homed = false;
        pitch_stall_counter = 0;
        yaw_stall_counter = 0;
    }
    printf("Homing complete for both axes.\n");
    //SendAllPwmCmd(spi_fd, 0, 0, 0, 0, 0, 0); // Stop motors
    StopMotors();
}

// Filename : spi_comm.h 
// Authors : Luis Moreno (s3608255), Luca Provenzano (s3487636)
// Group : 43
// License : N.A. or open source license like LGPL
// Description : header file for SPI communication module
//==============================================================

#ifndef SPI_COMM_H
#define SPI_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define SPI_CHANNEL       1
#define SPI_SPEED_HZ      10000000 // 10 MHz
#define SPI_MODE          0
#define SPI_BITS_PER_WORD 8

typedef enum {
    UnitPitch = 0,
    UnitYaw   = 1,
    UnitAll   = 2
} encoder_t;

typedef struct PwmStatus {
    uint8_t enable, dir;
    uint16_t duty;
} PwmStatus;

// Initializes the SPI interface.
int SpiOpen(unsigned spi_chan, unsigned spi_baud, unsigned spi_flags);

// Cleans up the SPI interface.
int SpiClose(int fd);

// Send new PWM values for a specific encoder (pitch or yaw). 
int SendPwmCmd(int fd, encoder_t unit, uint16_t duty, uint8_t enable, uint8_t dir);

// Send new PWM values for both encoders (pitch and yaw) at once.
int SendAllPwmCmd(int fd, uint16_t pitch_duty, uint8_t pitch_enable, uint8_t pitch_dir,
                  uint16_t yaw_duty, uint8_t yaw_enable, uint8_t yaw_dir);

// Reads the current position of the specified encoder/s (pitch, yaw or both).
int ReadPositionCmd(int fd, encoder_t unit, int32_t *pitch_pos, int32_t *yaw_pos);

// Reads the current status of the PWM for both encoders (pitch and yaw).
int CheckPwmStatus(int fd, PwmStatus *pitch_status, PwmStatus *yaw_status);

#ifdef __cplusplus
}
#endif

#endif
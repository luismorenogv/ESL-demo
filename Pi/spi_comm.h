//spi_comm.h


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
} unit_t;

typedef struct PwmStatus {
    uint8_t enable, dir;
    uint16_t duty;
} PwmStatus;

int SpiOpen(unsigned spi_chan, unsigned spi_baud, unsigned spi_flags);
int SpiClose(int fd);
int SendPwmCmd(int fd, unit_t unit, uint16_t duty, uint8_t enable, uint8_t dir);
int SendAllPwmCmd(int fd, uint16_t pitch_duty, uint8_t pitch_enable, uint8_t pitch_dir,
                  uint16_t yaw_duty, uint8_t yaw_enable, uint8_t yaw_dir);
int ReadPositionCmd(int fd, unit_t unit, int32_t *pitch_pos, int32_t *yaw_pos);
int CheckPwmStatus(int fd, PwmStatus *pitch_status, PwmStatus *yaw_status);

#ifdef __cplusplus
}
#endif

#endif

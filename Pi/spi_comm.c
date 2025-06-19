//spi_comm.c
#include "spi_comm.h"
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define CMD_WRITE_PITCH_PWM 0x10
#define CMD_WRITE_YAW_PWM   0x11
#define CMD_WRITE_ALL_PWM   0x12
#define CMD_READ_PITCH_POS  0x20
#define CMD_READ_YAW_POS    0x21
#define CMD_READ_ALL_POSITIONS 0x22
#define CHECK_PWM_STATUS 0x30

static int SpiXfer(int fd, unsigned speed, uint8_t *tx_buf, uint8_t *rx_buf, unsigned count) {
    struct spi_ioc_transfer tr = {
        .tx_buf        = (unsigned long)tx_buf,
        .rx_buf        = (unsigned long)rx_buf,
        .len           = count,
        .speed_hz      = speed,
        .delay_usecs   = 0,
        .bits_per_word = SPI_BITS_PER_WORD,
        .cs_change     = 0,
    };
    int err = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (err < 0) perror("SPI_IOC_MESSAGE");
    return err;
}

int SpiOpen(unsigned spi_chan, unsigned spi_baud, unsigned spi_flags) {
    int fd;
    char dev[32];
    char mode = spi_flags & 0x03;
    char bits = SPI_BITS_PER_WORD;

    snprintf(dev, sizeof(dev), "/dev/spidev0.%u", spi_chan);
    fd = open(dev, O_RDWR);
    if (fd < 0) {
        perror("open(spidev)");
        return -1;
    }
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_baud) < 0) {
        perror("spi setup");
        close(fd);
        return -2;
    }
    return fd;
}

int SpiClose(int fd) {
    return close(fd);
}

int SendPwmCmd(int fd, unit_t unit, uint16_t duty, uint8_t enable, uint8_t dir) {
    uint8_t tx[3], rx[3];
    uint8_t cmd = (unit == UnitPitch) ? CMD_WRITE_PITCH_PWM : CMD_WRITE_YAW_PWM;

    tx[0] = cmd;
    tx[1] = (uint8_t)(duty & 0xFF);
    tx[2] = (uint8_t)(((enable & 0x1) << 7) | ((dir & 0x1) << 6) | (((duty >> 8) & 0x0F) << 2));
    memset(rx, 0, sizeof(rx));
    return SpiXfer(fd, SPI_SPEED_HZ, tx, rx, 3);
}

int SendAllPwmCmd(int fd, uint16_t pitch_duty, uint8_t pitch_enable, uint8_t pitch_dir,
                  uint16_t yaw_duty, uint8_t yaw_enable, uint8_t yaw_dir) {
    uint8_t tx[5], rx[5];
    tx[0] = CMD_WRITE_ALL_PWM;
    tx[1] = (uint8_t)(pitch_duty & 0xFF);
    tx[2] = (uint8_t)(((pitch_enable & 0x1) << 7) | ((pitch_dir & 0x1) << 6) | (((pitch_duty >> 8) & 0x0F) << 2));
    tx[3] = (uint8_t)(yaw_duty & 0xFF);
    tx[4] = (uint8_t)(((yaw_enable & 0x1) << 7) | ((yaw_dir & 0x1) << 6) | (((yaw_duty >> 8) & 0x0F) << 2));
    memset(rx, 0, sizeof(rx));
    return SpiXfer(fd, SPI_SPEED_HZ, tx, rx, 5);
}

int ReadPositionCmd(int fd, unit_t unit, int32_t *pitch_pos, int32_t *yaw_pos) {
    int32_t *out_pos = (unit == UnitPitch) ? pitch_pos : (unit == UnitYaw) ? yaw_pos : NULL;
    uint8_t byte_size = (unit == UnitAll) ? 9 : 5;
    uint8_t tx[byte_size], rx[byte_size];
    memset(tx, 0, sizeof(tx));
    memset(rx, 0, sizeof(rx));
    tx[0] = (unit == UnitPitch) ? CMD_READ_PITCH_POS : (unit == UnitYaw) ? CMD_READ_YAW_POS : CMD_READ_ALL_POSITIONS;

    int err = SpiXfer(fd, SPI_SPEED_HZ, tx, rx, byte_size);
    if (err < 0) return err;

    if (unit == UnitAll) {
        *pitch_pos = ((int32_t)rx[1] << 24) | ((int32_t)rx[2] << 16) | ((int32_t)rx[3] << 8) | (int32_t)rx[4];
        *yaw_pos   = ((int32_t)rx[5] << 24) | ((int32_t)rx[6] << 16) | ((int32_t)rx[7] << 8) | (int32_t)rx[8];
        return 0;
    }

    if (out_pos != NULL) {
        *out_pos = ((int32_t)rx[1] << 24) | ((int32_t)rx[2] << 16) | ((int32_t)rx[3] << 8) | (int32_t)rx[4];
    }
    return 0;
}

int CheckPwmStatus(int fd, PwmStatus *pitch_status, PwmStatus *yaw_status) {
    uint8_t tx[5] = { CHECK_PWM_STATUS }, rx[5] = {0};
    int err = SpiXfer(fd, SPI_SPEED_HZ, tx, rx, 5);
    if (err < 0) return err;

    pitch_status->enable = (rx[1] >> 7) & 0x01;
    pitch_status->dir    = (rx[1] >> 6) & 0x01;
    pitch_status->duty   = ((rx[1] >> 2) & 0x0F) << 8 | rx[2];
    yaw_status->enable   = (rx[3] >> 7) & 0x01;
    yaw_status->dir      = (rx[3] >> 6) & 0x01;
    yaw_status->duty     = ((rx[3] >> 2) & 0x0F) << 8 | rx[4];
    return 0;
}

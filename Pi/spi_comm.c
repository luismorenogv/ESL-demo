// Filename : spi_comm.c 
// Authors : Luis Moreno (s3608255), Luca Provenzano (s3487636)
// Group : 43
// License : N.A. or open source license like LGPL
// Description : Spi communication module
//==============================================================
#include "spi_comm.h"
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

// Defines for the custom codes for SPI communication protocol with the FPGA.
#define CMD_WRITE_PITCH_PWM 0x10
#define CMD_WRITE_YAW_PWM   0x11
#define CMD_WRITE_ALL_PWM   0x12
#define CMD_READ_PITCH_POS  0x20
#define CMD_READ_YAW_POS    0x21
#define CMD_READ_ALL_POSITIONS 0x22
#define CHECK_PWM_STATUS 0x30

/*********************************************
* @brief Low-level helper to perform a generic SPI transaction using ioctl.
* 
* @param [in]  fd       SPI communication handle
* @param [in]  speed    SPI communication frequency
* @param [in]  tx_buf   Buffer to be transmitted
* @param [out] rx_buf   Buffer to be received
* @param [out] count    Buffer length
* 
* @return error code, err >= 0: no error
*********************************************/
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

/*********************************************
* @brief Opens SPI communication
* 
* @param [in] spi_chan SPI channel number
* @param [in] spi_baud SPI baud rate
* @param [in] spi_flags SPI flags
* 
* @return SPI opened succesfully -> returns SPI handle; -1: error in opening the SPI channel; -2: error in setting up the SPI mode 
*********************************************/
int SpiOpen(unsigned spi_chan, unsigned spi_baud, unsigned spi_flags) {
    int fd;
    char dev[32];
    char mode = spi_flags & 0x03;
    char bits = SPI_BITS_PER_WORD;

    // Construct the device path (e.g., /dev/spidev0.1) and open it.
    snprintf(dev, sizeof(dev), "/dev/spidev0.%u", spi_chan);
    fd = open(dev, O_RDWR);
    if (fd < 0) {
        perror("open(spidev)");
        return -1;
    }
    // Configure SPI mode, bits per word, and max speed.
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_baud) < 0) {
        perror("spi setup");
        close(fd);
        return -2;
    }
    return fd;
}


/*********************************************
* @brief Close SPI communication
* 
* @param [in] fd SPI communication handle
* 
* @return None.
*********************************************/
int SpiClose(int fd) {
    return close(fd);
}


/*********************************************
* @brief Sends the PWM command over a specific
* 
* @param [in] parameter parameter description
* 
* @return SpiXfer return value
*********************************************/
int SendPwmCmd(int fd, encoder_t unit, uint16_t duty, uint8_t enable, uint8_t dir) {
    uint8_t tx[3], rx[3];
    uint8_t cmd = (unit == UnitPitch) ? CMD_WRITE_PITCH_PWM : CMD_WRITE_YAW_PWM;

    tx[0] = cmd;
    // Pack the 12-bit duty cycle and flags into two bytes.
    tx[1] = (uint8_t)(duty & 0xFF);
    tx[2] = (uint8_t)(((enable & 0x1) << 7) | ((dir & 0x1) << 6) | (((duty >> 8) & 0x0F) << 2));
    memset(rx, 0, sizeof(rx));
    return SpiXfer(fd, SPI_SPEED_HZ, tx, rx, 3);
}


/*********************************************
* @brief Sends the PWM command for pitch and yaw
* 
* @param [in] fd            SPI communication handle
* @param [in] pitch_duty    pitch duty cicle
* @param [in] pitch_enable  pitch pwm enable bit
* @param [in] pitch_dir     pitch pwm direction bit
* @param [in] yaw_duty      yaw duty cicle
* @param [in] yaw_enable    yaw pwm enable bit
* @param [in] yaw_dir       yaw pwm direction bit
* 
* @return Return value of SpiXfer function
*********************************************/
int SendAllPwmCmd(int fd, uint16_t pitch_duty, uint8_t pitch_enable, uint8_t pitch_dir,
                  uint16_t yaw_duty, uint8_t yaw_enable, uint8_t yaw_dir) {
    uint8_t tx[5], rx[5];
    tx[0] = CMD_WRITE_ALL_PWM;

    // Byte 1-2: Pitch PWM data.
    tx[1] = (uint8_t)(pitch_duty & 0xFF);
    tx[2] = (uint8_t)(((pitch_enable & 0x1) << 7) | ((pitch_dir & 0x1) << 6) | (((pitch_duty >> 8) & 0x0F) << 2));

    // Byte 3-4: Yaw PWM data.
    tx[3] = (uint8_t)(yaw_duty & 0xFF);
    tx[4] = (uint8_t)(((yaw_enable & 0x1) << 7) | ((yaw_dir & 0x1) << 6) | (((yaw_duty >> 8) & 0x0F) << 2));

    memset(rx, 0, sizeof(rx));
    return SpiXfer(fd, SPI_SPEED_HZ, tx, rx, 5);
}


/*********************************************
* @brief Reads from SPI the device position in steps
* 
* @param [in] fd SPI communication channel
* @param [in] unit determines the unit read (Yaw, Pitch, both)
* @param [out] pitch_pos pitch steps position
* @param [out] yaw_pos yaw steps position
* 
* @return 0: Position read succesfully; < 0: returns code error
*********************************************/
int ReadPositionCmd(int fd, encoder_t unit, int32_t *pitch_pos, int32_t *yaw_pos) {
    // Ternary operator to set up single-axis or dual-axis read.
    int32_t *out_pos = (unit == UnitPitch) ? pitch_pos : (unit == UnitYaw) ? yaw_pos : NULL;
    uint8_t byte_size = (unit == UnitAll) ? 9 : 5;
    uint8_t tx[byte_size], rx[byte_size];

    // The first byte sent is the command code. The rest are dummy bytes (0x00).
    memset(tx, 0, sizeof(tx));
    memset(rx, 0, sizeof(rx));
    tx[0] = (unit == UnitPitch) ? CMD_READ_PITCH_POS : (unit == UnitYaw) ? CMD_READ_YAW_POS : CMD_READ_ALL_POSITIONS;

    int err = SpiXfer(fd, SPI_SPEED_HZ, tx, rx, byte_size);
    if (err < 0) return err;

    // Reconstruct the 32-bit integer position values from the received bytes.
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

/*********************************************
* @brief Checks the PWM status
* 
* @param [in]  fd           SPI communication handle
* @param [out] pitch_status pitch status struct
* @param [out] yaw_status   yaw status struct
* 
* @return 0: No error; < 0: error code
*********************************************/
int CheckPwmStatus(int fd, PwmStatus *pitch_status, PwmStatus *yaw_status) {
    uint8_t tx[5] = { CHECK_PWM_STATUS }, rx[5] = {0};
    int err = SpiXfer(fd, SPI_SPEED_HZ, tx, rx, 5);
    if (err < 0) return err;
    
    // Unpack the received bytes into the status structs.
    pitch_status->enable = (rx[1] >> 7) & 0x01;
    pitch_status->dir    = (rx[1] >> 6) & 0x01;
    pitch_status->duty   = ((rx[1] >> 2) & 0x0F) << 8 | rx[2];

    yaw_status->enable   = (rx[3] >> 7) & 0x01;
    yaw_status->dir      = (rx[3] >> 6) & 0x01;
    yaw_status->duty     = ((rx[3] >> 2) & 0x0F) << 8 | rx[4];
    return 0;
}

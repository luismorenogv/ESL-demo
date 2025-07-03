#ifndef MOCK_SPIDEV_H
#define MOCK_SPIDEV_H

// #define SPI_IOC_WR_MODE           0x0
// #define SPI_IOC_WR_BITS_PER_WORD 0x1
// #define SPI_IOC_WR_MAX_SPEED_HZ  0x2
// #define SPI_IOC_MESSAGE(n)       (n+100)
#define SPI_BITS_PER_WORD        8

struct spi_ioc_transfer {
    unsigned long tx_buf;
    unsigned long rx_buf;
    unsigned len;
    unsigned speed_hz;
    unsigned short delay_usecs;
    unsigned char bits_per_word;
    unsigned char cs_change;
};

#endif

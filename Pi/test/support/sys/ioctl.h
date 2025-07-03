#ifndef _MOCK_SYS_IOCTL_H
#define _MOCK_SYS_IOCTL_H

#define SPI_IOC_WR_MODE 0
#define SPI_IOC_WR_BITS_PER_WORD 1
#define SPI_IOC_WR_MAX_SPEED_HZ 2
#define SPI_IOC_MESSAGE(N) (100 + (N))

int ioctl(int fd, int request, void* mode);

#endif

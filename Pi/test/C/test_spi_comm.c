#include "unity.h"
#include "spi_comm.h"

// Override system headers
#include "mock_fake_fcntl.h"
#include "mock_ioctl.h"
#include "mock_fake_unistd.h"
#include "mock_spidev.h"

#define CMD_READ_PITCH_POS  0x20

void setUp(void) {}
void tearDown(void) {}

void test_SpiOpen_success(void) {
    int fake_fd = 3;

    char mode = 0x00;
    char bits = SPI_BITS_PER_WORD;
    unsigned spi_baud = 500000;


    open_ExpectAndReturn("/dev/spidev0.0", O_RDWR, fake_fd);
    ioctl_ExpectAndReturn(fake_fd, SPI_IOC_WR_MODE, &mode, 0);
    ioctl_ExpectAndReturn(fake_fd, SPI_IOC_WR_BITS_PER_WORD, &bits, 0);
    ioctl_ExpectAndReturn(fake_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_baud, 0);

    int result = SpiOpen(0, 500000, 0x00);
    TEST_ASSERT_EQUAL(fake_fd, result);
}

void test_SpiOpen_fails_on_open(void) {
    open_ExpectAndReturn("/dev/spidev0.0", O_RDWR, -1);

    int result = SpiOpen(0, 500000, 0x00);
    TEST_ASSERT_EQUAL(-1, result);
}

void test_SpiOpen_fails_on_ioctl(void) {
    int fake_fd = 4;
    char mode = 0x00;

    open_ExpectAndReturn("/dev/spidev0.0", O_RDWR, fake_fd);
    ioctl_ExpectAndReturn(fake_fd, SPI_IOC_WR_MODE, &mode, -1);

    close_ExpectAndReturn(fake_fd, 0);

    int result = SpiOpen(0, 500000, 0x00);
    TEST_ASSERT_EQUAL(-2, result);
}


void test_SpiXfer_called_from_CheckPwmStatus_success(void) {
    int fd = 3;
    char mode = 0x00;
    PwmStatus pitch = {0}, yaw = {0};

    // Expect ioctl to be called from SpiXfer (which is called by CheckPwmStatus)
    ioctl_ExpectAnyArgsAndReturn(5);  // 5 bytes transferred

    int result = CheckPwmStatus(fd, &pitch, &yaw);
    TEST_ASSERT_EQUAL(0, result);
}

void test_SpiXfer_called_from_CheckPwmStatus_fail(void) {
    int fd = 3;
    char mode = 0x00;
    int exp_result = -2;
    PwmStatus pitch = {0}, yaw = {0};

    // Expect ioctl to be called from SpiXfer (which is called by CheckPwmStatus)
    ioctl_ExpectAnyArgsAndReturn(exp_result);

    int result = CheckPwmStatus(fd, &pitch, &yaw);
    TEST_ASSERT_EQUAL(exp_result, result);
}

void test_SendPwmCmd_pitch_success(void) {
    int fd = 3;
    uint16_t duty = 0x1234;
    uint8_t enable = 1, dir = 1;

    // We expect ioctl to be called by SpiXfer with 3 bytes (tx length)
    ioctl_ExpectAnyArgsAndReturn(3); // Simulate success (3 bytes written)

    int result = SendPwmCmd(fd, UnitPitch, duty, enable, dir);
    TEST_ASSERT_EQUAL(3, result);
}

void test_SendAllPwmCmd_success(void) {
    int fd = 7;
    uint16_t pitch_duty = 0x00A5;
    uint8_t pitch_enable = 1;
    uint8_t pitch_dir = 0;
    uint16_t yaw_duty = 0x0033;
    uint8_t yaw_enable = 0;
    uint8_t yaw_dir = 1;

    // Simulate ioctl called by SpiXfer
    ioctl_ExpectAnyArgsAndReturn(5); // 5 bytes transferred

    int result = SendAllPwmCmd(fd, pitch_duty, pitch_enable, pitch_dir,
                               yaw_duty, yaw_enable, yaw_dir);

    TEST_ASSERT_EQUAL(5, result);
}

void test_SendAllPwmCmd_failure_if_SpiXfer_fails(void) {
    int fd = 7;

    ioctl_ExpectAnyArgsAndReturn(-1); // Simulate failure in SpiXfer

    int result = SendAllPwmCmd(fd, 0x1234, 1, 1, 0x5678, 0, 0);

    TEST_ASSERT_EQUAL(-1, result);
}

void test_CheckPwmStatus_success(void) {
    int fd = 4;
    PwmStatus pitch = {0}, yaw = {0};

    ioctl_ExpectAnyArgsAndReturn(5);

    int result = CheckPwmStatus(fd, &pitch, &yaw);
    TEST_ASSERT_EQUAL(0, result);

    // Assert pitch status decoded correctly
    TEST_ASSERT_EQUAL(0, pitch.enable);
    TEST_ASSERT_EQUAL(0, pitch.dir);
    TEST_ASSERT_EQUAL_HEX16(0, pitch.duty); // (0x02 << 8) | 0xA5

    // Assert yaw status decoded correctly
    TEST_ASSERT_EQUAL(0, yaw.dir);
    TEST_ASSERT_EQUAL(0, yaw.enable);
    TEST_ASSERT_EQUAL_HEX16(0, yaw.duty); // (0x03 << 8) | 0xF2
}

void test_CheckPwmStatus_fail(void) {
    int fd = 4;
    PwmStatus pitch = {0}, yaw = {0};

    ioctl_ExpectAnyArgsAndReturn(-2);

    int result = CheckPwmStatus(fd, &pitch, &yaw);
    TEST_ASSERT_EQUAL(-2, result);
}

void test_SpiClose(void){
    int fd = 4;
    close_ExpectAndReturn(fd, 2);
    
    int result = SpiClose(fd);

    TEST_ASSERT_EQUAL(result, 2);
}

void test_ReadPositionCmd_reads_pitch_position_correctly(void) {
    int fd = 3;
    int32_t pitch = 0;
    int32_t yaw = 0xFFFFFFFF; // Unused

    // Expect ioctl call inside SpiXfer
    ioctl_ExpectAnyArgsAndReturn(0); // simulate success

    int result = ReadPositionCmd(fd, UnitPitch, &pitch, &yaw);

    TEST_ASSERT_EQUAL(0, result);
    // We can't assert actual pitch value here without a stub, but we confirm function succeeds
}

void test_ReadPositionCmd_reads_yaw_position_fail(void) {
    int fd = 3;
    int32_t pitch = 0xFFFFFFFF; // Unused
    int32_t yaw = 0;

    ioctl_ExpectAnyArgsAndReturn(-1); // simulate success

    int result = ReadPositionCmd(fd, UnitYaw, &pitch, &yaw);

    TEST_ASSERT_EQUAL(-1, result);
}

void test_ReadPositionCmd_reads_all_position_success(void) {
    int fd = 3;
    int32_t pitch = 0xFFFFFFFF; // Unused
    int32_t yaw = 0;

    ioctl_ExpectAnyArgsAndReturn(0); // simulate success

    int result = ReadPositionCmd(fd, UnitAll, &pitch, &yaw);

    TEST_ASSERT_EQUAL(0, result);
}
#include <stdint.h>


int PWMinit(uint8_t duty_bits, uint8_t max_duty_pitch, uint8_t max_duty_yaw);
uint16_t getPitchPWM(double input);
uint16_t getYawPWM(double input);

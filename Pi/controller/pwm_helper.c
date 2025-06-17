#include "pwm_helper.h"
#include <math.h>

#define MAX_DUTY_BITS 16

uint32_t max_duty_val_pitch;
uint32_t max_duty_val_yaw;

int PWMinit(uint8_t duty_bits, uint8_t max_duty_pitch, uint8_t max_duty_yaw)
{
    if(duty_bits>MAX_DUTY_BITS)
    {
        printf("Too many bits for duty cycle, maximum of %d", MAX_DUTY_BITS);
        return -1;
    }
    max_duty_val_pitch = (pow(2, duty_bits)-1)/100*max_duty_pitch;
    max_duty_val_yaw = (pow(2, duty_bits)-1)/100*max_duty_yaw;
}


uint16_t getPitchPWM(double input)
{
    return (uint16_t) (input * max_duty_val_pitch);
}

uint16_t getYawPWM(double input)
{
    return (uint16_t) (input * max_duty_val_yaw);
}
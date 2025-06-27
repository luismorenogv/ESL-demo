// controller.c

#include "controller.h"

#include "pan/pan_xxsubmod.h"
#include "tilt/tilt_xxsubmod.h"
#include <stdbool.h>

#define DUTY_BITS 12  //Bits for duty cycle in verilog
#define MAX_DUTY_PERC 0.2 //Percentage of duty cycle (from 0.00 to 1.00)
#define MAX_SAFE_DUTY ((uint16_t)(MAX_DUTY_PERC * ((1 << DUTY_BITS) - 1)))

// Global variables to hold the I/O for each controller
static XXDouble pan_inputs[2];
static XXDouble pan_outputs[2];
static XXDouble tilt_inputs[3];
static XXDouble tilt_outputs[1];

// Global state for time
static XXDouble g_pan_time = 0.0;
static XXDouble g_tilt_time = 0.0;

// Flag to ensure initialization is only called once
static bool g_is_initialized = false;


void ControllerInitialize(void) {
    // Initialize Pan with zero inputs/outputs at time 0
    pan_inputs[0] = 0.0; pan_inputs[1] = 0.0;
    PanInitializeSubmodel(pan_inputs, pan_outputs, 0.0);
    
    // Initialize Tilt with zero inputs/outputs at time 0
    tilt_inputs[0] = 0.0; tilt_inputs[1] = 0.0; tilt_inputs[2] = 0.0;
    TiltInitializeSubmodel(tilt_inputs, tilt_outputs, 0.0);
    
    g_is_initialized = true;
}

void ControllerStep(XXDouble tiltPos, XXDouble tiltDst,
                    XXDouble panPos,  XXDouble panDst,
                    XXDouble dt) {

    if (!g_is_initialized) {
        ControllerInitialize();
    }
    
    // Pan Controller
    
    // The submodel wrapper handles time and step size internally
    // Update step size
    extern XXDouble pan_step_size;
    pan_step_size = dt;
    g_pan_time += dt;

    // Prepare inputs for the pan model
    pan_inputs[0] = panDst;    // "in"
    pan_inputs[1] = panPos;    // "position"
    
    // Calculate one step
    PanCalculateSubmodel(pan_inputs, pan_outputs, g_pan_time);
    
    // Tilt Controller
    // Update step size
    extern XXDouble tilt_step_size;
    tilt_step_size = dt;
    g_tilt_time += dt;

    // Prepare inputs for the tilt model
    tilt_inputs[0] = pan_outputs[0]; // corr value from pan controller
    tilt_inputs[1] = tiltDst;        // "in"
    tilt_inputs[2] = tiltPos;        // "position"
    
    // Calculate one step
    TiltCalculateSubmodel(tilt_inputs, tilt_outputs, g_tilt_time);
}

void ControllerTerminate(void) {
    PanTerminateSubmodel(pan_inputs, pan_outputs, g_pan_time);
    TiltTerminateSubmodel(tilt_inputs, tilt_outputs, g_tilt_time);
}

// Getter functions
void getPanOut(uint16_t* pan_duty, uint8_t* pan_dir) {
    *pan_duty = (uint16_t)(fmin(fabs(pan_outputs[1]),1.0) * MAX_SAFE_DUTY);//pan_outputs[1];
    *pan_dir  = (pan_outputs[1] >= 0.0) ? 0 : 1;
}

void getTiltOut(uint16_t* tilt_duty, uint8_t* tilt_dir) {
    *tilt_duty = (uint16_t)(fmin(fabs(tilt_outputs[0]),1.0) * MAX_SAFE_DUTY);//tilt_outputs[0];
    *tilt_dir  = (tilt_outputs[0] >= 0.0) ? 0 : 1;
}
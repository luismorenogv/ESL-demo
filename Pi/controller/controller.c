// Filename : controller.c
// Assumes 20-sim generated functions/variables have been renamed with Pan/Tilt prefixes.

#include "controller.h"

// Include the renamed submodel headers
#include "pan/pan_xxsubmod.h"
#include "tilt/tilt_xxsubmod.h"
#include <stdbool.h>

// --- Global variables to hold the I/O for each controller ---
static XXDouble pan_inputs[2];
static XXDouble pan_outputs[2];
static XXDouble tilt_inputs[3];
static XXDouble tilt_outputs[1];

// --- Global state for time ---
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
    
    // --- Step 1: Pan Controller ---
    
    // The submodel wrapper handles time and step size internally now.
    // We just need to pass the current time and the model will use its step size.
    // We must, however, update the global step size variable before calling.
    extern XXDouble pan_step_size; // This global is defined in the renamed pan_xxmodel.c
    pan_step_size = dt;
    g_pan_time += dt;

    // Prepare inputs for the pan model
    pan_inputs[0] = panDst;    // "in"
    pan_inputs[1] = panPos;    // "position"
    
    // Calculate one step
    PanCalculateSubmodel(pan_inputs, pan_outputs, g_pan_time);
    
    // --- Step 2: Tilt Controller ---
    
    extern XXDouble tilt_step_size; // This global is defined in the renamed tilt_xxmodel.c
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

// --- Getter Functions ---
XXDouble getPanOut(void) {
    // From pan_xxsubmod.c, output[0] is "corr", output[1] is "out"
    return pan_outputs[1];
}

XXDouble getTiltOut(void) {
    // From tilt_xxsubmod.c, output[0] is "out"
    return tilt_outputs[0];
}
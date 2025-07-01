#include "unity.h"
#include "controller.h"

#include "mock_pan_xxsubmod.h"
#include "mock_tilt_xxsubmod.h"

double pan_step_size = 0.01;
double tilt_step_size = 0.01;

void setUp(void) {
    // This function is run before each test
}

void tearDown(void) {
    // This function is run after each test
}

void test_ControllerInitialize_Should_InitializeSubmodelsAndSetFlag(void) {
    // Expect calls to submodel initialization functions with 0.0 and proper arrays

    XXDouble dummy_pan_inputs[2] = {0.0, 0.0};
    XXDouble dummy_pan_outputs[2] = {0.0, 0.0};
    XXDouble dummy_tilt_inputs[3] = {0.0, 0.0, 0.0};
    XXDouble dummy_tilt_outputs[1] = {0.0};

    PanInitializeSubmodel_Expect(dummy_pan_inputs, dummy_pan_outputs, 0.0);
    TiltInitializeSubmodel_Expect(dummy_tilt_inputs, dummy_tilt_outputs, 0.0);

    ControllerInitialize();
}
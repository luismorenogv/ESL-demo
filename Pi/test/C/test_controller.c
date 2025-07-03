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

void test_ControllerStep_WithInit(void) {
    XXDouble tiltPos = 10.0;
    XXDouble tiltDst = 15.0;
    XXDouble tiltIn [3] = {0.0, tiltDst, tiltPos};
    XXDouble dummy_tilt_outputs[1] = {0.0};
    XXDouble panPos = 20.0;
    XXDouble panDst = 25.0;
    XXDouble panIn [2] = {panDst, panPos};
    XXDouble dummy_pan_outputs[2] = {0.0, 0.0};
    XXDouble dt = 0.1;

    // Expect initialization functions to be called
    //Proves that ControllerInitialize has been called
    PanInitializeSubmodel_ExpectAnyArgs();      
    TiltInitializeSubmodel_ExpectAnyArgs();

    // First, ControllerStep will trigger initialization
    // Now expect the actual stepping logic
    PanCalculateSubmodel_Expect(panIn, dummy_pan_outputs, dt);
    TiltCalculateSubmodel_Expect(tiltIn, dummy_tilt_outputs, dt); //2*dt, at every call of ControllerStep this increases

    ControllerStep(tiltPos, tiltDst, panPos, panDst, dt);
}

void test_ControllerInitialize_Positive(void) {
    // Expect calls to submodel initialization functions with 0.0 and proper arrays

    XXDouble dummy_pan_inputs[2] = {0.0, 0.0};
    XXDouble dummy_pan_outputs[2] = {0.0, 0.0};
    XXDouble dummy_tilt_inputs[3] = {0.0, 0.0, 0.0};
    XXDouble dummy_tilt_outputs[1] = {0.0};

    PanInitializeSubmodel_Expect(dummy_pan_inputs, dummy_pan_outputs, 0.0);
    TiltInitializeSubmodel_Expect(dummy_tilt_inputs, dummy_tilt_outputs, 0.0);

    ControllerInitialize();
}

void test_ControllerStep_WithoutInit(void) {
    XXDouble tiltPos = 10.0;
    XXDouble tiltDst = 15.0;
    XXDouble tiltIn [3] = {0.0, tiltDst, tiltPos};
    XXDouble dummy_tilt_outputs[1] = {0.0};
    XXDouble panPos = 20.0;
    XXDouble panDst = 25.0;
    XXDouble panIn [2] = {panDst, panPos};
    XXDouble dummy_pan_outputs[2] = {0.0, 0.0};
    XXDouble dt = 0.1;

    // First, ControllerStep will trigger initialization
    // Now expect the actual stepping logic
    PanCalculateSubmodel_Expect(panIn, dummy_pan_outputs, 2*dt);
    TiltCalculateSubmodel_Expect(tiltIn, dummy_tilt_outputs, 2*dt); //2*dt, at every call of ControllerStep this increases

    ControllerStep(tiltPos, tiltDst, panPos, panDst, dt);
}

void test_ControllerTerminate_Positive(void) {
    PanTerminateSubmodel_ExpectAnyArgs();
    TiltTerminateSubmodel_ExpectAnyArgs();

    ControllerTerminate();
}

void test_getPanOut_Positive_Null(void) {
    XXDouble retVal = getPanOut();
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, retVal);
}

void test_getTiltOut_Positive_Null(void) {
    XXDouble retVal = getTiltOut();
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, retVal);
}




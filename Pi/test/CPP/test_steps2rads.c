#include "../../controller/steps2rads.h"
#include <math.h>
#include <gtest/gtest.h>

void setUp(void) {}
void tearDown(void) {}


void test_steps2rads_zero_steps_returns_zero(void)
{
    int32_t steps = 0;
    int32_t max_steps = 1000;
    XXDouble result = steps2rads(steps, max_steps);

    EXPECT_EQ(0.0, result);
}


void test_steps2rads_half_circle(void)
{
    int32_t steps = 500;
    int32_t max_steps = 1000;
    XXDouble result = steps2rads(steps, max_steps);

    EXPECT_NEAR(result, M_PI/2. 1e-6);
}

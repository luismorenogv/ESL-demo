#include <gtest/gtest.h>
#include "../../controller/steps2rads.h"
#include <math.h>

void setUp(void) {}
void tearDown(void) {}


TEST(steps2rads_test, nullTest)
{
    int32_t steps = 0;
    int32_t max_steps = 1000;
    XXDouble result = steps2rads(steps, max_steps, M_PI);

    EXPECT_EQ(0.0, result);
}


TEST(steps2rads_test, halfwayTest)
{
    int32_t steps = 500;
    int32_t max_steps = 1000;
    XXDouble result = steps2rads(steps, max_steps, M_PI);

    EXPECT_NEAR(result, M_PI/2, 1e-6);
}

TEST(steps2rads_test, divisionByZero)
{
    int32_t steps = 500;
    XXDouble result = steps2rads(steps, 0, M_PI);

    EXPECT_NEAR(result, 0, 1e-6);
}
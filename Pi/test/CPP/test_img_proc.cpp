#include <gst/app/gstappsink.h>
#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gtest/gtest.h>
#include "../../img_proc.hpp"

// GStreamer function called flag
extern int set_state_called;
extern int unref_called;


void reset_mock_counters() {
    set_state_called = 0;
    unref_called = 0;
}


TEST(CleanupGstreamerPipelineTest, CallsSetStateAndUnref) {
    reset_mock_counters();

    CleanupGstreamerPipeline(fake_pipeline);

    EXPECT_EQ(set_state_called, 1);
    EXPECT_EQ(unref_called, 1);
}

// ---------------------------------------------------------------------

TEST(ComputeAnglesTest, ComputesCorrectOffsets) {
    double x_offset_rad = 0;
    double y_offset_rad = 0;

    // Image center would be 320x240, simulate a point to the right and below center
    ComputeAngles(420, 280, 640, 480, x_offset_rad, y_offset_rad);

    EXPECT_NEAR(x_offset_rad, ((420 - 320) * (48.808 * M_PI / 180.0f)) / 640, 1e-6);
    EXPECT_NEAR(y_offset_rad, ((280 - 240) * (28.634 * M_PI / 180.0f)) / 480, 1e-6);
}


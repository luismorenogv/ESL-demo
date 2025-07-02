#include <gst/app/gstappsink.h>
#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gtest/gtest.h>
#include "../../img_proc.hpp"

// GStreamer mocks (wrap-based)
extern int gst_init_called;
extern int pipeline_new_called;
extern int factory_make_called;
extern int caps_from_string_called;
extern int g_object_set_called;
extern int caps_unref_called;
extern int element_link_called;
extern int set_state_called;
extern int unref_called;
extern int try_pull_sample_called;
extern int get_buffer_called;
extern int get_caps_called;
extern int get_structure_called;
extern int structure_get_int_called;
extern int buffer_map_called;
extern int buffer_unmap_called;
extern int sample_unref_called;

extern GstElement* fake_pipeline;
extern GstElement* fake_element;
extern GstSample* fake_sample;

// OpenCV mocks
extern int cvtColor_called;
extern int inRange_called;
extern int findContours_called;
extern int contourArea_called;
extern int boundingRect_called;


void reset_mock_counters() {
    set_state_called = 0;
    unref_called = 0;
}

// ---------------------------------------------------------------------

// ---------------------------------------------------------------------

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

// ---------------------------------------------------------------------
/*
TEST(ProcessOneFrameTest, ReturnsTrueAndSetsOutputs) {
    reset_mock_counters();

    double x_rad, y_rad, obj_size;
    bool result = ProcessOneFrame(fake_element, x_rad, y_rad, obj_size);

    EXPECT_TRUE(result);
    EXPECT_EQ(try_pull_sample_called, 1);
    EXPECT_EQ(get_buffer_called, 1);
    EXPECT_EQ(get_caps_called, 1);
    EXPECT_EQ(get_structure_called, 1);
    EXPECT_EQ(structure_get_int_called, 2);
    EXPECT_EQ(buffer_map_called, 1);
    EXPECT_EQ(cvtColor_called, 2); // BGR + HSV
    EXPECT_EQ(inRange_called, 1);
    EXPECT_EQ(findContours_called, 1);
    EXPECT_EQ(contourArea_called, 1);
    EXPECT_EQ(boundingRect_called, 1);
    EXPECT_EQ(buffer_unmap_called, 1);
    EXPECT_EQ(sample_unref_called, 1);

    EXPECT_GT(obj_size, 0.0);
}
*/
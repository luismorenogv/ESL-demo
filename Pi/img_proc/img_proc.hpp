// Filename : img_proc.h 
// Authors : 
// Group :
// License : N.A. or open source license like LGPL
// Description : 
//==============================================================

#ifndef IMG_PROC_HPP
#define IMG_PROC_HPP

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <opencv2/opencv.hpp>

typedef struct {
    int x;
    int y;
} Point;

int init_gstreamer_pipeline(const char* device_path, GstElement** pipeline_out, GstElement** appsink_out);

void cleanup_gstreamer_pipeline(GstElement* pipeline);

bool process_one_frame(GstElement* appsink, double& x_offset_rad, double& y_offset_rad, double& obj_size);

void computeAngles(int x_actual, int y_actual, int width, int height, double& x_offset_rad, double& y_offset_rad);

#endif
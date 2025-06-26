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

int InitGstreamerPipeline(const char* device_path, GstElement** pipeline_out, GstElement** appsink_out);

void CleanupGstreamerPipeline(GstElement* pipeline);

bool ProcessOneFrame(GstElement* appsink, double& x_offset_rad, double& y_offset_rad, double& obj_size);

void ComputeAngles(int x_actual, int y_actual, int width, int height, double& x_offset_rad, double& y_offset_rad);

#endif
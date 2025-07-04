// Filename : img_proc.hpp 
// Authors : Luis Moreno (s3608255), Luca Provenzano (s3487636)
// Group : 43
// License : N.A. or open source license like LGPL
// Description : Header file for image processor
//==============================================================

#ifndef IMG_PROC_HPP
#define IMG_PROC_HPP

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <opencv2/opencv.hpp>

// For threading
#include <thread>
#include <mutex>
#include <atomic>

#include "controller/common/xxtypes.h" // For XXDouble

#define MIN_OBJ_SIZE    2000

// Shared data structure between threads
struct TargetData {
    XXDouble x_offset_rad = 0.0;
    XXDouble y_offset_rad = 0.0;
    double obj_size = 0.0;
    bool new_frame = false;
};

// Global variables for thread communication
extern std::atomic<bool> g_run;
extern TargetData g_target_data;
extern std::mutex g_target_mutex;


// Initializes the GStreamer pipeline.
int InitGstreamerPipeline(const char* device_path, GstElement** pipeline_out, GstElement** appsink_out);

// Cleans up the GStreamer pipeline.
void CleanupGstreamerPipeline(GstElement* pipeline);

// The main loop for the vision thread.
void vision_thread_func(GstElement *sink);

// Internal processing functions
bool ProcessOneFrame(GstElement* appsink, double& x_offset_rad, double& y_offset_rad, double& obj_size);
void ComputeAngles(int x_actual, int y_actual, int width, int height, double& x_offset_rad, double& y_offset_rad);

#endif
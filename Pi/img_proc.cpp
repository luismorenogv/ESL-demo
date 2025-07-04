// Filename : img_proc.cpp 
// Authors : Luis Moreno (s3608255), Luca Provenzano (s3487636)
// Group : 43
// License : N.A. or open source license like LGPL
// Description : Manages the gstreamer pipeline, operates OpenCV and the camera output
//==============================================================

#include "img_proc.hpp"
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#define DFOV_DEG    55 //Obtained from camera manufacturer website
#define HFOV_DEG    48.808 //Calculated
#define VFOV_DEG    28.634 //Calculated

#define HFOV_RAD    HFOV_DEG * M_PI / 180.0f
#define VFOV_RAD    VFOV_DEG * M_PI / 180.0f

// Definitions for the global shared variables
std::atomic<bool> g_run(true);
TargetData g_target_data;
std::mutex g_target_mutex;


/*********************************************
* @brief Vision thread loop function
* 
* @param [inout] sink sink element of the pipeline
* 
* @return None.
*********************************************/
void vision_thread_func(GstElement *sink) {
    printf("Vision thread started.\n");
    
    double x_offset, y_offset, obj_size;
    
    while(g_run) {
        // Attempt to process a new frame. 
        if (ProcessOneFrame(sink, x_offset, y_offset, obj_size)){
            // Update the shared data.
            {
                std::lock_guard<std::mutex> lock(g_target_mutex);
                g_target_data.x_offset_rad = x_offset;
                g_target_data.y_offset_rad = y_offset;
                g_target_data.obj_size = obj_size;
                g_target_data.new_frame = true; // Indicate that a new frame was processed
            }
        }

        usleep(10); // Sleep for 0.01ms
    }
    printf("Vision thread finished.\n");
}

/*********************************************
* @brief Gstreamer Pipeline initilizer
* 
* @param [in] device_path path to the camera
* @param [out] pipeline_out address to return the pipeline
* @param [out] appsink_out address to return the appsink
* 
* @return 0: init succesful; -1: init failed
*********************************************/
int InitGstreamerPipeline(const char* device_path, GstElement** pipeline_out, GstElement** appsink_out) {
    // Initialize GStreamer library
    gst_init(NULL, NULL);

    // Create pipeline and elements: camera source, capsfilter, and appsink
    GstElement *pipeline = gst_pipeline_new("video-pipeline");
    GstElement *source = gst_element_factory_make("v4l2src", "source");
    GstElement *capsfilter = gst_element_factory_make("capsfilter", "capsfilter");
    GstElement *sink = gst_element_factory_make("appsink", "sink");

    if (!pipeline || !source || !capsfilter || !sink) {
        g_printerr("Failed to create elements\n");
        return -1;
    }

    // Set the video device path
    g_object_set(G_OBJECT(source), "device", device_path, NULL);

    // Set appsink properties
    g_object_set(G_OBJECT(sink),
                 "emit-signals", FALSE,
                 "sync", FALSE,
                 "max-buffers", 1,
                 "drop", TRUE,
                 NULL);

    // Set caps (format)
    GstCaps *caps = gst_caps_from_string("video/x-raw,format=YUY2,width=640,height=480,framerate=30/1");
    g_object_set(G_OBJECT(capsfilter), "caps", caps, NULL);
    gst_caps_unref(caps);

    // Build pipeline
    gst_bin_add_many(GST_BIN(pipeline), source, capsfilter, sink, NULL);
    if (!gst_element_link_many(source, capsfilter, sink, NULL)) {
        g_printerr("Pipeline linking failed\n");
        return -1;
    }

    // Start the pipeline
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    // Output the created pipeline and appsink
    *pipeline_out = pipeline;
    *appsink_out = sink;

    return 0;
}


/*********************************************
* @brief Cleans the Gstreamer Pipeline before closure
* 
* @param [inout] pipeline pipeline to be closed
* 
* @return None.
*********************************************/
void CleanupGstreamerPipeline(GstElement* pipeline) {
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
}


/*********************************************
* @brief Function to compute angular position from image center in radians
* 
* @param [in]   x_actual        X coordinate read from the camera
* @param [in]   y_actual        Y coordinate read from the camera
* @param [in]   width           Camera width in pixels
* @param [in]   height          Camera height in pixels
* @param [out]  x_offset_rad    Objects distance on x in radiants from center of camera
* @param [out]  y_offset_rad    Objects distance on y in radiants from center of camera
* 
* @return None.
*********************************************/
void ComputeAngles(int x_actual, int y_actual, int width, int height, double& x_offset_rad, double& y_offset_rad) {

    // Compute offset from center
    double x_offset = x_actual - (width/2.0f); // Horizontal distance from center
    double y_offset = y_actual - (height/2.0f); // Vertical distance from center

    // Compute radians per pixel
    double rad_per_px_x = HFOV_RAD / width;
    double rad_per_px_y = VFOV_RAD / height;

    // Compute angle offsets from center
    x_offset_rad = x_offset * rad_per_px_x;
    y_offset_rad = y_offset * rad_per_px_y;
}


/*********************************************
* @brief Pulls a frame (if available), process it with openCV
* 
* @param [in]   appsink         Sink from which it takes the frame
* @param [out]  x_offset_rad    Objects distance on x in radiants from center of camera
* @param [out]  y_offset_rad    Objects distance on y in radiants from center of camera
* @param [out]  obj_size        Objects size
* 
* @return false: processing failed OR no new sample available; true: processing succesful
*********************************************/
bool ProcessOneFrame(GstElement* appsink, double& x_offset_rad, double& y_offset_rad, double& obj_size) {
    // Try to get a new video frame from the GStreamer pipeline. Non-blocking.
    GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink), 0);

    // If no new frame is available, do nothing and report failure.
    if (!sample) {
        return false;
    }

    // A new frame is available, proceed with processing.
    GstBuffer* buffer = gst_sample_get_buffer(sample);
    GstCaps* caps = gst_sample_get_caps(sample);
    GstStructure* s = gst_caps_get_structure(caps, 0);
    GstMapInfo map;
    
    static int width = 0, height = 0;
    if (width == 0) { // Get dimensions once
        gst_structure_get_int(s, "width", &width);
        gst_structure_get_int(s, "height", &height);
    }
    
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        gst_sample_unref(sample);
        return false;
    }

    // Convert the YUY2 format into a standard BGR image for OpenCV.
    cv::Mat yuy2(height, width, CV_8UC2, (char*)map.data);
    static cv::Mat bgr;
    cv::cvtColor(yuy2, bgr, cv::COLOR_YUV2BGR_YUY2);

    // Convert the image from BGR to HSV color space.
    static cv::Mat inputImageHSV, maskedImage;
    cv::cvtColor(bgr, inputImageHSV, cv::COLOR_BGR2HSV);

    // Create a binary "mask" to isolate the green color
    static const cv::Scalar green_lower_threshold(35, 50, 50);
    static const cv::Scalar green_upper_threshold(85, 255, 255);
    cv::inRange(inputImageHSV, green_lower_threshold, green_upper_threshold, maskedImage);
    
    // Find the contours of all the green objects
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(maskedImage, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    int bBoxCenterX = 0;
    int bBoxCenterY = 0;
    double max_area = 0.0;

    if (!contours.empty()) {
        // Take largest area
        double area = 0;
        int largeContIndex = 0;
        for (size_t i = 0; i < contours.size(); ++i) {
            area = cv::contourArea(contours[i]);
            if (area < MIN_OBJ_SIZE) {
                continue; // Skip small contours
            }
            if (area > max_area) {
                max_area = area;
                largeContIndex = i;
            }
        }
        
        // Create a bounding box around green object
        cv::Rect boundingBox = cv::boundingRect(contours[largeContIndex]);
        // Take central coordinates of the bounding box
        bBoxCenterX = boundingBox.x + boundingBox.width / 2;
        bBoxCenterY = boundingBox.y + boundingBox.height / 2;
    }
    
    // Store the object's area and calculate its angular offset from the center.
    obj_size = max_area;
    ComputeAngles(bBoxCenterX, bBoxCenterY, width, height, x_offset_rad, y_offset_rad);
    
    // Clean up memory.
    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    
    // Report that a new frame was processed successfully.
    return true;
}




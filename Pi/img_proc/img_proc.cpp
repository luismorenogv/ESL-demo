#include "img_proc.hpp"
#include <stdio.h>
#include <math.h>

#define DFOV_DEG    55 //Obtained from camera manufacturer website
#define HFOV_DEG    48.808 //Calculated
#define VFOV_DEG    28.634 //Calculated

#define HFOV_RAD    HFOV_DEG * M_PI / 180.0f
#define VFOV_RAD    VFOV_DEG * M_PI / 180.0f

/* PROTOTYPES */
//void computeAngles(int x_actual, int y_actual, int width, int height, double& x_offset_rad, double& y_offset_rad);


int init_gstreamer_pipeline(const char* device_path, GstElement** pipeline_out, GstElement** appsink_out) {
    gst_init(NULL, NULL);

    GstElement *pipeline = gst_pipeline_new("video-pipeline");
    GstElement *source = gst_element_factory_make("v4l2src", "source");
    GstElement *capsfilter = gst_element_factory_make("capsfilter", "capsfilter");
    GstElement *sink = gst_element_factory_make("appsink", "sink");

    if (!pipeline || !source || !capsfilter || !sink) {
        g_printerr("Failed to create elements\n");
        return -1;
    }

    // Set camera device
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

    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    *pipeline_out = pipeline;
    *appsink_out = sink;

    return 0;
}


void cleanup_gstreamer_pipeline(GstElement* pipeline) {
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
}


// Function to compute angular position from image center in radians
void computeAngles(int x_actual, int y_actual, int width, int height, double& x_offset_rad, double& y_offset_rad) {

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

bool process_one_frame(GstElement* appsink, double& x_offset_rad, double& y_offset_rad, double& obj_size) {
    GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink),  0 /*GST_SECOND / 10*/);  // non-blocking
    int width, height;
    static double old_x_offset_rad = 0;
    static double old_y_offset_rad = 0;
    static double old_obj_size = 0;
    if (!sample) {
        x_offset_rad = old_x_offset_rad;
        y_offset_rad = old_y_offset_rad;
        obj_size     = old_obj_size;
        return false;
    }
    
    
    GstBuffer* buffer = gst_sample_get_buffer(sample);
    GstCaps* caps = gst_sample_get_caps(sample);
    GstStructure* s = gst_caps_get_structure(caps, 0);
    GstMapInfo map;
    gst_structure_get_int(s, "width", &width);
    gst_structure_get_int(s, "height", &height);
    
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        gst_sample_unref(sample);
        return false;
    }

    // Convert to OpenCV format
    cv::Mat yuy2(height, width, CV_8UC2, (char*)map.data);
    cv::Mat bgr;
    cv::cvtColor(yuy2, bgr, cv::COLOR_YUV2BGR_YUY2);

    // Green tracking
    cv::Mat inputImageHSV, maskedImage;
    std::vector<std::vector<cv::Point>> contours;
    int largeContIndex = 0;
    double max_area = 0.0;

    // Green thresholds
    cv::Scalar green_lower_threshold(35, 50, 50);
    cv::Scalar green_upper_threshold(85, 255, 255);

    // Convert to HSV
    cv::cvtColor(bgr, inputImageHSV, cv::COLOR_BGR2HSV);
    cv::inRange(inputImageHSV, green_lower_threshold, green_upper_threshold, maskedImage);
    cv::findContours(maskedImage, contours, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

    int bBoxCenterX;
    int bBoxCenterY;

    if (!contours.empty()) {
        // Take biggest contour
        for (size_t i = 0; i < contours.size(); ++i) {
            double area = cv::contourArea(contours[i]);
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
    } else {
        max_area = 0;
        bBoxCenterX = 0;
        bBoxCenterY = 0;
    }

    obj_size = max_area;
    //printf("Object size: %.2f, Center: (%d, %d)\n", obj_size, bBoxCenterX, bBoxCenterY);
    computeAngles(bBoxCenterX, bBoxCenterY, width, height, x_offset_rad, y_offset_rad);
    //if(obj_size > 2000) printf("y_offset: %.2f\n", x_offset_rad, y_offset_rad);
    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);


    old_x_offset_rad = x_offset_rad;
    old_y_offset_rad = y_offset_rad;
    old_obj_size = obj_size;
    return true;
}
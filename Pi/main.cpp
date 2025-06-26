// main.cpp

#include <stdio.h>
#include <stdlib.h>
#include <csignal>

#include "spi_comm.h"
#include "controller/controller.h"
#include "img_proc.hpp"
#include "motor_control.hpp"

// Signal handler to stop the threads gracefully
void signal_handler(int signum) {
    printf("\nInterrupt signal (%d) received. Shutting down.\n", signum);
    g_run = false;
}

// Error function specific to main setup
int error(const char *msg, const int e_code, int fd) {
    fprintf(stderr, "Error %i: %s\n", e_code, msg);
    if (fd >= 0) SpiClose(fd);
    return e_code;
}

int main(int argc, char *argv[]) {
    // Register signal handler for Ctrl+C
    signal(SIGINT, signal_handler);

    if (argc != 2) {
        fprintf(stderr, "Usage: %s <source_file>\n", argv[0]);
        return 1;
    }
    
    GstElement *pipeline, *sink;

    // 1) Open SPI
    int fd = SpiOpen(SPI_CHANNEL, SPI_SPEED_HZ, SPI_MODE);
    if (fd < 0) return 1;

    // 2) Homing procedure
    int32_t pitch_offset, yaw_offset;
    uint32_t pitch_max_steps, yaw_max_steps;
    HomeBothAxes(fd, &pitch_offset, &yaw_offset, &pitch_max_steps, &yaw_max_steps);

    if (!g_run) { // Check if Ctrl+C was pressed during homing
        printf("Shutdown signal received during homing. Exiting.\n");
        SpiClose(fd);
        return 0;
    }

    // 3) Init Controller and GStreamer
    ControllerInitialize();
    if (InitGstreamerPipeline(argv[1], &pipeline, &sink) != 0) {
        SpiClose(fd);
        return -1;
    }
    
    // 4) Start Threads
    printf("Starting threads...\n");
    std::thread control_thr(control_thread_func, fd, pitch_offset, yaw_offset, pitch_max_steps, yaw_max_steps);
    std::thread vision_thr(vision_thread_func, sink);

    // 5) Wait for threads to finish
    // The threads will run until g_run is set to false (e.g., by Ctrl+C)
    control_thr.join();
    vision_thr.join();

    // 6) Stop & close
    printf("Stopping motors and closing SPI.\n");
    SendAllPwmCmd(fd, 0,0,0, 0,0,0);
    SpiClose(fd);
    CleanupGstreamerPipeline(pipeline);
    
    printf("Program finished gracefully.\n");
    return 0;
}
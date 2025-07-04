ESL Final Demo: Vision Tracker Readme
=======================================

This document contains commands to build, run, and test the project.

--------------------
1. Prerequisites
--------------------

This single command will install all required system dependencies from the
Ubuntu repositories, including the C++/FPGA toolchains and testing tools.

sudo apt update && sudo apt install -y \
    build-essential \
    g++ \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good \
    libopencv-dev \
    yosys \
    nextpnr-ice40 \
    fpga-icestorm \
    libgtest-dev \
    gcovr


-------------------------
2. Build and Run Demo
-------------------------

These commands compile the FPGA and C++ code, program the FPGA, and run the final application.
They assume the project is at `~/ESL-demo/` and `icoprog` is at `~/icoprog/`.

# --- Build, Program FPGA, and Compile C++ ---
cd ~/ESL-demo/FPGA && \
yosys -p 'synth_ice40 -top TopEntity -json ice40.json' TopEntity.v PWM.v QuadratureEncoder.v && \
nextpnr-ice40 --hx8k --json ice40.json --pcf ico-jiwy.pcf --asc ice40.asc && \
icepack ice40.asc ice40.bin && \
sudo modprobe spi-bcm2835 -r && \
(cd ~/icoprog && ./icoprog -R && ./icoprog -p < ~/ESL-demo/FPGA/ice40.bin) && \
sudo modprobe spi-bcm2835 && \
cd ../Pi && \
g++ main.cpp motor_control.cpp img_proc.cpp spi_comm.c \
    controller/controller.c \
    controller/common/xxfuncs.c \
    controller/pan/pan_integ.c \
    controller/pan/pan_xxmodel.c \
    controller/pan/pan_xxsubmod.c \
    controller/tilt/tilt_integ.c \
    controller/tilt/tilt_xxmodel.c \
    controller/tilt/tilt_xxsubmod.c \
    -I./ -I./controller/common \
    `pkg-config --cflags --libs opencv4 gstreamer-1.0 gstreamer-app-1.0` \
    -lm -lpthread -lstdc++ -Wall \
    -o gimbal_tracker


# --- How to Run ---
# Find your camera device (e.g., /dev/video1)
ls /dev/video*

# Execute the tracker
cd ~/ESL-demo/Pi && ./gimbal_tracker /dev/video1


--------------------------------
3. Unit Tests
--------------------------------

g++ ./CPP/test_img_proc.cpp ./CPP/gstreamer_mocks.cpp ../img_proc.cpp     -O0 -g --coverage     `pkg-config --cflags --libs opencv4 gstreamer-1.0 gstreamer-app-1.0`   -lgtest -lgtest_main -pthread -o test_runner
gcovr -r .. --html --html-details -o ./html/coverage.html
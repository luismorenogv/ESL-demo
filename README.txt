ESL Final Demo: Vision Tracker Readme
=======================================

This document contains commands to build, run, and test the project.

--------------------
1. Prerequisites
--------------------

This single command will install all required system dependencies from the
Ubuntu repositories, including the C++/FPGA toolchains and testing tools (excluded C tests, 
for those, check dedicated section).

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
    libgtest-dev googletest \ 
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

# --- For C++ ---

## Testing test_img_proc.cpp

### Compiling test_img_proc.cpp
cd ./Pi

g++ ./test/CPP/test_img_proc.cpp ./test/CPP/gstreamer_mocks.cpp ./img_proc.cpp  \
    -O0 -g --coverage  `pkg-config --cflags --libs opencv4 gstreamer-1.0 gstreamer-app-1.0` \
    -lgtest -lgtest_main -pthread -o test_runner

### Running test_img_proc.cpp
./test_runner

### Generating the html code coverage files for img_proc.cpp
mkdir ./test/results/html_img_proc

gcovr test_runner-img_proc.gcda test_runner-img_proc.gcno \
      --html --html-details -o ./test/results/html_img_proc/coverage.html

## Testing test_steps2rads.cpp

### Compiling test_steps2rads.cpp
cd ./Pi

g++ ./test/CPP/test_steps2rads.cpp    -I controller/common/  -O0 -g --coverage  `pkg-config --cflags \
    --libs opencv4 gstreamer-1.0 gstreamer-app-1.0`     -lgtest -lgtest_main -pthread -o test_runner

### Running test_steps2rads.cpp, this will print test results on terminal
./test_runner

### Generating the html code coverage files for img_proc.cpp
mkdir ./test/results/html_steps2rads

gcovr ./test_runner-test_steps2rads.gcda ./test_runner-test_steps2rads.gcno --html --html-details  \
        -o ./test/results/html_steps2rads/coverage.html


# --- For C --- (Only on Windows!)
We tried the same pipeline on Linux, but the test cases crash when launched, this is because on Linux, 
ceedling is most probably not capable of succesfully mocking libraries like spidev and ioctl.  

For C testing we used Ceedling, which is a framework based on Ruby, to use this, Ruby must be installed

## Install Ruby (more info here https://www.ruby-lang.org/en/documentation/installation/#winget)
### Linux command
sudo apt-get install ruby-full

## (On Windows) Open a command prompt with Ruby and install Ceedling (more info here: https://www.throwtheswitch.org/ceedling#get-ceedling-section)
gem install ceedling

## (On Linux)
sudo gem install ceedling

## On the command prompt (on Windows it must have been opened with Ruby)
cd <to-project-folder>/Pi

## Launching only tests
ceedling test:all

### This will print results on terminal

## Launching tests AND code generating code coverage reports
### Is important to remove artifacts from previous C++ tests, especially .gcda and .gcno files
rm *.gcda
rm *.gcno

ceedling gcov:all

### This will print results on terminal
### The tests results html and code coverage reports will be im build/artifacts/gcov

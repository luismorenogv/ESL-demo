cd ~/ESL-demo/FPGA && \
yosys -p 'synth_ice40 -top TopEntity -json ice40.json' TopEntity.v PWM.v QuadratureEncoder.v && \
nextpnr-ice40 --hx8k --json ice40.json --pcf ico-jiwy.pcf --asc ice40.asc && \
sudo modprobe spi-bcm2835 -r && \
icepack ice40.asc ice40.bin && \
mv -f ice40.bin ~/icoprog/ && \
cd ~/icoprog && \
./icoprog -R && \
./icoprog -p < ice40.bin && \
sudo modprobe spi-bcm2835 && \
cd ../ESL-demo/Pi


g++ main.cpp motor_control.cpp img_proc.cpp spi_comm.c \
    controller/controller.c \
    controller/common/xxfuncs.c \
    controller/pan/pan_integ.c \
    controller/pan/pan_xxmodel.c \
    controller/pan/pan_xxsubmod.c \
    controller/tilt/tilt_integ.c \
    controller/tilt/tilt_xxmodel.c \
    controller/tilt/tilt_xxsubmod.c \
    -I./ \
    -I./controller/common \
    `pkg-config --cflags --libs opencv4 gstreamer-1.0 gstreamer-app-1.0` \
    -lm -lpthread -lstdc++ -Wall \
    -o gimbal_tracker


Test commands for gtest
g++ ./CPP/test_img_proc.cpp ./CPP/gstreamer_mocks.cpp ../img_proc.cpp     -O0 -g --coverage     `pkg-config --cflags --libs opencv4 gstreamer-1.0 gstreamer-app-1.0`   -lgtest -lgtest_main -pthread -o test_runner
gcovr -r .. --html --html-details -o ./html/coverage.html
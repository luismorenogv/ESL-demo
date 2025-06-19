cd ~/Assignment_13/FPGA && \
yosys -p 'synth_ice40 -top TopEntity -json ice40.json' TopEntity.v PWM.v QuadratureEncoder.v && \
nextpnr-ice40 --hx8k --json ice40.json --pcf ico-jiwy.pcf --asc ice40.asc && \
sudo modprobe spi-bcm2835 -r && \
icepack ice40.asc ice40.bin && \
mv -f ice40.bin ~/icoprog/ && \
cd ~/icoprog && \
./icoprog -R && \
./icoprog -p < ice40.bin && \
sudo modprobe spi-bcm2835 && \
cd ../Assignment_13/Pi


gcc -o controllerPi main.c spi_comm.c controller/controller.c controller/pan_model.c controller/tilt_model.c -lm -Wall
`timescale 1ns / 1ps

module TopEntity_tb;

    // Testbench parameters
    parameter CLK_FREQ  = 25000000;
    parameter PWM_FREQ  = 20000;
    parameter COUNTER_W = 12;

    // Simulation timing constants
    localparam CLK_PERIOD_NS     = 1000000000/CLK_FREQ; // 25 MHz FPGA clock
    localparam SPI_CLK_PERIOD_NS = 400; // 25 MHz SPI clock

    // DUT I/O
    reg clk = 0;
    reg btn1;
    reg SPI_CLK = 0;
    reg SPI_PICO;
    reg SPI_CS;
    reg PITCH_ENC_A;
    reg PITCH_ENC_B;
    reg YAW_ENC_A;
    reg YAW_ENC_B;
    wire SPI_POCI;
    wire PITCH_DIRA, PITCH_DIRB, PITCH_PWM_VAL;
    wire YAW_DIRA, YAW_DIRB, YAW_PWM_VAL;
    wire led1, led2, led3;

    // Instantiate the DUT
    TopEntity #(
        .CLK_FREQ(CLK_FREQ),
        .PWM_FREQ(PWM_FREQ),
        .COUNTER_W(COUNTER_W)
    ) dut (
        .clk(clk), .btn1(btn1), .SPI_CLK(SPI_CLK), .SPI_PICO(SPI_PICO),
        .SPI_CS(SPI_CS), .SPI_POCI(SPI_POCI), .PITCH_ENC_A(PITCH_ENC_A),
        .PITCH_ENC_B(PITCH_ENC_B), .PITCH_DIRA(PITCH_DIRA), .PITCH_DIRB(PITCH_DIRB),
        .PITCH_PWM_VAL(PITCH_PWM_VAL), .YAW_ENC_A(YAW_ENC_A), .YAW_ENC_B(YAW_ENC_B),
        .YAW_DIRA(YAW_DIRA), .YAW_DIRB(YAW_DIRB), .YAW_PWM_VAL(YAW_PWM_VAL),
        .led1(led1), .led2(led2), .led3(led3)
    );

    // Clock generator
    initial begin
        forever #(CLK_PERIOD_NS / 2) clk = ~clk;
    end
    
    // Testbench variables for SPI and results
    reg [7:0] tb_tx_packet [0:8];
    reg [7:0] tb_rx_packet [0:8];
    integer received_pitch;
    integer received_yaw;
    integer i;
    integer k;

    // SPI Master transaction task
    task spi_transaction;
        input integer num_bytes;
        integer i, j;
        reg [7:0] received_byte;
        begin
            // Assert CS low to start transaction
            SPI_CS <= 1'b0; 
            SPI_CLK <= 1'b0;
            #(SPI_CLK_PERIOD_NS); 
            for (i = 0; i < num_bytes; i = i + 1) begin
                received_byte = 8'h00;
                for (j = 7; j >= 0; j = j - 1) begin
                    // Drive PICO wire
                    SPI_PICO <= tb_tx_packet[i][j];
                    #(SPI_CLK_PERIOD_NS/2);
                    // Rising edge
                    SPI_CLK <= 1'b1; 
                    #(10); 
                    // Read POCI right after the rising edge
                    received_byte[j] = SPI_POCI;
                    #((SPI_CLK_PERIOD_NS/2) - 10); 
                    // Falling edge
                    SPI_CLK <= 1'b0;
                end
                // Save full byte
                tb_rx_packet[i] = received_byte;
            end
            // Deassert CS to end transaction
            #(SPI_CLK_PERIOD_NS/2);
            SPI_CS <= 1'b1;
            #(CLK_PERIOD_NS * 10); 
        end
    endtask

    // Main Test Sequence
    initial begin

        $display("Starting TopEntity Testbench...");
        $dumpfile("top_entity_signals.vcd");
        $dumpvars(0, TopEntity_tb);

        // Initialization and reset
        btn1 = 1'b1; 
        SPI_CS = 1'b1; SPI_PICO = 1'b0;
        PITCH_ENC_A = 1'b0; PITCH_ENC_B = 1'b0;
        YAW_ENC_A = 1'b0; YAW_ENC_B = 1'b0;
        #(CLK_PERIOD_NS * 20);
        btn1 = 1'b0;
        #(CLK_PERIOD_NS * 20);

        // Test 1: Write PWM values and verify by reading status
        $display("TEST 1: Write and Verify PWM Settings");
        tb_tx_packet[0] = 8'h12; // Write All PWM
        tb_tx_packet[1] = 8'h00; tb_tx_packet[2] = 8'hA0; // Pitch: duty=0x800, en=1, dir=0
        tb_tx_packet[3] = 8'h00; tb_tx_packet[4] = 8'hD0; // Yaw:   duty=0x400, en=1, dir=1
        spi_transaction(5);
        
        tb_tx_packet[0] = 8'h30; // Check PWM Status
        spi_transaction(5);

        if ({tb_rx_packet[1], tb_rx_packet[2]} == {8'hA0, 8'h00} && {tb_rx_packet[3], tb_rx_packet[4]} == {8'hD0, 8'h00})
            $display("PASSED: PWM Status matches written values.");
        else
            $display("FAILED: PWM Status mismatch.");

        // Test 2: Simulate encoder movement and verify positions
        #(CLK_PERIOD_NS * 100);
        $display("TEST 2: Simulate and Verify Encoder Positions");

        // Simulate +123 cycles on pitch encoder (CW) (4 steps each cycle   )
        for (i = 0; i < 123; i = i + 1) begin
            {PITCH_ENC_A, PITCH_ENC_B} <= 2'b10; #(CLK_PERIOD_NS * 10);
            {PITCH_ENC_A, PITCH_ENC_B} <= 2'b11; #(CLK_PERIOD_NS * 10);
            {PITCH_ENC_A, PITCH_ENC_B} <= 2'b01; #(CLK_PERIOD_NS * 10);
            {PITCH_ENC_A, PITCH_ENC_B} <= 2'b00; #(CLK_PERIOD_NS * 10);
        end

        // Simulate -456 cycles on yaw encoder (CCW) (4 steps each cycle)
        for (i = 0; i < 456; i = i + 1) begin
            {YAW_ENC_A, YAW_ENC_B} <= 2'b01; #(CLK_PERIOD_NS * 10);
            {YAW_ENC_A, YAW_ENC_B} <= 2'b11; #(CLK_PERIOD_NS * 10);
            {YAW_ENC_A, YAW_ENC_B} <= 2'b10; #(CLK_PERIOD_NS * 10);
            {YAW_ENC_A, YAW_ENC_B} <= 2'b00; #(CLK_PERIOD_NS * 10);
        end

        #(CLK_PERIOD_NS * 100);
        
        tb_tx_packet[0] = 8'h22; // Read All Positions
        spi_transaction(9);
        
        received_pitch = $signed({tb_rx_packet[1], tb_rx_packet[2], tb_rx_packet[3], tb_rx_packet[4]});
        received_yaw   = $signed({tb_rx_packet[5], tb_rx_packet[6], tb_rx_packet[7], tb_rx_packet[8]});

        // Check against the decoded value (4 counts per simulated cycle)
        if (received_pitch == (123 * 4) && received_yaw == (-456 * 4)) begin
            $display("PASSED: Encoder positions match simulated movement.");
        end else begin
            $display("FAILED: Encoder position mismatch. Expected P:%d Y:%d, Got P:%d Y:%d",
                (123*4), (-456*4), received_pitch, received_yaw);
        end

        #(CLK_PERIOD_NS * 100);
        $display("All tests finished.");
        $finish;
    end

endmodule
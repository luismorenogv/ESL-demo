// TopEntity.v
module TopEntity #(
    parameter CLK_FREQ  = 12_000_000,   // 12 MHz
    parameter PWM_FREQ  = 20_000,       // 20 kHz
    parameter COUNTER_W = 12            // 12-bit resolution
  )
  (
    input  wire         clk,
    input  wire         btn1,          // btn1 used as reset
    // SPI Interface
    input  wire         SPI_CLK,        // SCLK from Pi
    input  wire         SPI_PICO,       // MOSI
    input  wire         SPI_CS,         // CS (active-low)
    output wire         SPI_POCI,       // MISO
    // Encoders & PWM signals
    input  wire         PITCH_ENC_A,
    input  wire         PITCH_ENC_B,
    output  wire         PITCH_DIRA,
    output  wire         PITCH_DIRB,
    output wire         PITCH_PWM_VAL,
    input  wire         YAW_ENC_A,
    input  wire         YAW_ENC_B,
    output  wire         YAW_DIRA,
    output  wire         YAW_DIRB,
    output wire         YAW_PWM_VAL,
    output reg          led1 = 1'b0,
    output reg          led2 = 1'b0,
    output reg          led3 = 1'b0
  );

  // Instantiate Encoders and PWM
  wire [1:0] dir_pitch, dir_yaw;
  wire signed [31:0] position_pitch, position_yaw;

  QuadratureEncoder pitch_encoder (
    .clk(clk), .reset(btn1),
    .ENCA_raw(PITCH_ENC_A), .ENCB_raw(PITCH_ENC_B),
    .DIR(dir_pitch), .position(position_pitch)
  );

  QuadratureEncoder yaw_encoder (
    .clk(clk), .reset(btn1),
    .ENCA_raw(YAW_ENC_A), .ENCB_raw(YAW_ENC_B),
    .DIR(dir_yaw), .position(position_yaw)
  );

  reg                  enable_pitch      = 1'b0;
  reg                  direction_pitch   = 1'b0;
  reg [COUNTER_W-1:0]  duty_cycle_pitch  = {COUNTER_W{1'b0}};

  reg                  enable_yaw        = 1'b0;
  reg                  direction_yaw     = 1'b0;
  reg [COUNTER_W-1:0]  duty_cycle_yaw    = {COUNTER_W{1'b0}};

  PWM #(
    .CLK_FREQ(CLK_FREQ), .PWM_FREQ(PWM_FREQ), .COUNTER_W(COUNTER_W)
  ) pitch_pwm (
    .clk(clk), .reset(btn1),
    .enable(enable_pitch),
    .duty_cycle(duty_cycle_pitch),
    .direction(direction_pitch),
    .ina(PITCH_DIRA), .inb(PITCH_DIRB),
    .pwm_out(PITCH_PWM_VAL)
  );

  PWM #(
    .CLK_FREQ(CLK_FREQ), .PWM_FREQ(PWM_FREQ), .COUNTER_W(COUNTER_W)
  ) yaw_pwm (
    .clk(clk), .reset(btn1),
    .enable(enable_yaw),
    .duty_cycle(duty_cycle_yaw),
    .direction(direction_yaw),
    .ina(YAW_DIRA), .inb(YAW_DIRB),
    .pwm_out(YAW_PWM_VAL)
  );

  // SPI Slave Mechanism

  // edge detectors
  reg [2:0] SPI_CLKr, SPI_CSr;
  always @(posedge clk) begin
    SPI_CLKr <= {SPI_CLKr[1:0], SPI_CLK};
    SPI_CSr  <= {SPI_CSr[1:0], SPI_CS};
  end
  wire SPI_CLK_risingedge  = (SPI_CLKr[2:1] == 2'b01);
  wire SPI_CLK_fallingedge = (SPI_CLKr[2:1] == 2'b10);
  wire SPI_CS_active       = ~SPI_CSr[1];            // low = active
  wire SPI_CS_startmessage = (SPI_CSr[2:1] == 2'b10);
  wire SPI_CS_endmessage   = (SPI_CSr[2:1] == 2'b01);

  // synchronize MOSI to avoid metastability
  reg [1:0] SPI_PICOr;
  always @(posedge clk)
    SPI_PICOr <= {SPI_PICOr[0], SPI_PICO};
  wire SPI_PICO_data = SPI_PICOr[1];

  // buffers & counters
  localparam integer MAX_BYTES = 10;
  reg [7:0] rx_buf[0:MAX_BYTES-1], tx_buf[0:MAX_BYTES-1];
  reg [2:0]  bit_cnt   = 3'b000;   // 0..7
  reg [3:0]  byte_cnt  = 4'h0;     // 0..MAX_BYTES-1
  reg [7:0] rx_shift, tx_shift;

  assign SPI_POCI = tx_shift[7];

  // unified SPI handling
  always @(posedge clk) begin

    // end‐of‐transaction: handle writes
    if (SPI_CS_endmessage) begin
      case (rx_buf[0])
        8'h10: begin
          led2 <= 1'b1; // indicate we received a write command
          enable_pitch     <= rx_buf[2][7];
          direction_pitch  <= rx_buf[2][6];
          duty_cycle_pitch <= {rx_buf[2][5:2], rx_buf[1]};
        end
        8'h11: begin
          led1 <= 1'b1; // indicate we received a write command
          enable_yaw       <= rx_buf[2][7];
          direction_yaw    <= rx_buf[2][6];
          duty_cycle_yaw   <= {rx_buf[2][5:2], rx_buf[1]};
        end
        8'h12: begin
          enable_pitch     <= rx_buf[2][7];
          direction_pitch  <= rx_buf[2][6];
          duty_cycle_pitch <= {rx_buf[2][5:2], rx_buf[1]};
          enable_yaw       <= rx_buf[4][7];
          direction_yaw    <= rx_buf[4][6];
          duty_cycle_yaw   <= {rx_buf[4][5:2], rx_buf[3]};
        end
        default: ; // read‐only commands already handled
      endcase
      byte_cnt <= 4'h0;
    end

    if (~SPI_CS_active) begin
      // Reset on CS deassert
      bit_cnt   <= 3'b000;
      byte_cnt  <= 4'h0;
      rx_shift  <= 8'h00;
      tx_shift  <= 8'h00;
    end else begin
      // Start of a new transaction
      if (SPI_CS_startmessage) begin
        tx_buf[0] <= 8'h01; // send dummy byte first
      end

      // SHIFT IN rx_shift on rising edge
      if (SPI_CLK_risingedge) begin
        bit_cnt  <= bit_cnt + 1;
        rx_shift <= {rx_shift[6:0], SPI_PICO_data};

        if (bit_cnt == 3'b111) begin
          // got a full byte
          rx_buf[byte_cnt] <= {rx_shift[6:0], SPI_PICO_data};
          byte_cnt         <= byte_cnt + 1;

          // decode first byte immediately for read commands
          if (byte_cnt == 4'h0) begin
            case ({rx_shift[6:0], SPI_PICO_data})
              8'h20: begin
                tx_buf[1] <= position_pitch[31:24];
                tx_buf[2] <= position_pitch[23:16];
                tx_buf[3] <= position_pitch[15: 8];
                tx_buf[4] <= position_pitch[ 7: 0];
              end
              8'h21: begin
                tx_buf[1] <= position_yaw[31:24];
                tx_buf[2] <= position_yaw[23:16];
                tx_buf[3] <= position_yaw[15: 8];
                tx_buf[4] <= position_yaw[ 7: 0];
              end
              8'h22: begin
                tx_buf[1]  <= position_pitch[31:24];
                tx_buf[2]  <= position_pitch[23:16];
                tx_buf[3]  <= position_pitch[15: 8];
                tx_buf[4]  <= position_pitch[ 7: 0];
                tx_buf[5]  <= position_yaw[31:24];
                tx_buf[6]  <= position_yaw[23:16];
                tx_buf[7]  <= position_yaw[15: 8];
                tx_buf[8]  <= position_yaw[ 7: 0];
              end
              8'h30: begin
                tx_buf[1]  <= {enable_pitch, direction_pitch, duty_cycle_pitch[11:8], /* don't care */ 2'b00};
                tx_buf[2]  <= duty_cycle_pitch[7:0];
                tx_buf[3]  <= {enable_yaw, direction_yaw, duty_cycle_yaw[11:8], /* don't care */ 2'b00};
                tx_buf[4]  <= duty_cycle_yaw[7:0];
              end
              default: begin
                // no‐care defaults
                tx_buf[1] <= 8'h00;
                tx_buf[2] <= 8'h00;
                tx_buf[3] <= 8'h00;
                tx_buf[4] <= 8'h00;
              end
            endcase
          end
        end
      end

      // SHIFT OUT tx_shift on falling edge
      if (SPI_CLK_fallingedge) begin
        if (bit_cnt == 3'b000)
          tx_shift <= tx_buf[byte_cnt];
        else
          tx_shift <= {tx_shift[6:0], 1'b0};
      end
    end
  end

  // 3) led3: 1 Hz blink to show core is alive
  reg [31:0] led3_counter = 32'd0;
  always @(posedge clk) begin
    if (btn1) begin
      led3_counter <= 0;
      led3         <= 0;
    end else begin
      led3_counter <= led3_counter + 1;
      if (led3_counter >= CLK_FREQ) begin
        led3         <= ~led3;
        led3_counter <= 0;
      end
    end
  end

endmodule
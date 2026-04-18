`timescale 1ns / 1ps
`default_nettype none

module tick_gen_100hz #(
    parameter int unsigned CLK_HZ = 50_000_000
) (
    input  wire clk,
    input  wire rst_n,
    output logic tick_100hz
);
    localparam int unsigned DIV = CLK_HZ / 100;
    logic [$clog2(DIV)-1:0] cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt        <= '0;
            tick_100hz <= 1'b0;
        end else begin
            if (cnt == DIV-1) begin
                cnt        <= '0;
                tick_100hz <= 1'b1;
            end else begin
                cnt        <= cnt + 1'b1;
                tick_100hz <= 1'b0;
            end
        end
    end
endmodule

module top #(
    parameter int unsigned CLK_HZ          = 50_000_000,
    parameter int unsigned I2C_HZ          = 100_000,
    parameter int unsigned TOF_SENSOR_KIND = 0
) (
    input  wire       CLK,
    input  wire [3:0] KEY_SW,

    output logic [3:0] LED,
    output wire  [7:0] SEG,
    output wire  [3:0] DIG,

    inout  wire       i2c_scl,
    inout  wire       i2c_sda
);
    localparam int unsigned W_DIGIT = 4;
    localparam signed [8:0] DRIVE_SPEED = 9'sd50;

    wire clk   = CLK;
    wire rst_n = KEY_SW[3];
    wire rst   = ~rst_n;

    logic [3:0] key_sw_d;
    logic tick_100hz;

    logic tof_init_start;
    logic tof_sample_start;

    logic [15:0] tof_distance_mm;
    logic        tof_distance_valid;
    logic        tof_busy;
    logic        tof_done;
    logic        tof_error;
    logic        tof_nack;

    logic        tof_txn_req_valid;
    logic        tof_txn_req_ready;
    logic        tof_txn_req_is_read;
    logic [6:0]  tof_txn_req_dev_addr;
    logic [15:0] tof_txn_req_reg_addr;
    logic        tof_txn_req_reg_addr_16b;
    logic [7:0]  tof_txn_req_wr_data;
    logic [1:0]  tof_txn_req_rd_len;

    logic        tof_txn_rsp_done;
    logic        tof_txn_rsp_error;
    logic        tof_txn_rsp_nack;
    logic [15:0] tof_txn_rsp_rd_data;

    logic        rover_cmd_valid;
    logic signed [8:0] rover_m1;
    logic signed [8:0] rover_m2;
    logic signed [8:0] rover_m3;
    logic signed [8:0] rover_m4;
    logic signed [8:0] rover_m1_sent;
    logic signed [8:0] rover_m2_sent;
    logic signed [8:0] rover_m3_sent;
    logic signed [8:0] rover_m4_sent;
    logic        rover_busy;
    logic        rover_done;
    logic        rover_error;
    logic        rover_nack;

    logic        rover_txn_req_valid;
    logic        rover_txn_req_ready;
    logic        rover_txn_req_is_read;
    logic [6:0]  rover_txn_req_dev_addr;
    logic [15:0] rover_txn_req_reg_addr;
    logic        rover_txn_req_reg_addr_16b;
    logic [7:0]  rover_txn_req_wr_data;
    logic [1:0]  rover_txn_req_rd_len;

    logic        rover_txn_rsp_done;
    logic        rover_txn_rsp_error;
    logic        rover_txn_rsp_nack;
    logic [15:0] rover_txn_rsp_rd_data;

    logic        txn_core_start;
    logic        txn_core_is_read;
    logic [6:0]  txn_core_dev_addr;
    logic [15:0] txn_core_reg_addr;
    logic        txn_core_reg_addr_16b;
    logic [7:0]  txn_core_wr_data;
    logic [1:0]  txn_core_rd_len;
    logic [15:0] txn_core_rd_data;
    logic        txn_core_busy;
    logic        txn_core_done;
    logic        txn_core_error;
    logic        txn_core_nack;

    logic        scl_drive_low;
    logic        sda_drive_low;
    logic [15:0] disp_value;

    typedef enum logic [0:0] {
        BR_IDLE = 1'b0,
        BR_WAIT = 1'b1
    } bridge_state_t;

    typedef enum logic [1:0] {
        SRC_NONE  = 2'd0,
        SRC_TOF   = 2'd1,
        SRC_ROVER = 2'd2
    } bridge_src_t;

    bridge_state_t bridge_state;
    bridge_src_t   bridge_src;

    assign i2c_scl = scl_drive_low ? 1'b0 : 1'bz;
    assign i2c_sda = sda_drive_low ? 1'b0 : 1'bz;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            key_sw_d <= 4'b0000;
        else
            key_sw_d <= KEY_SW;
    end

    tick_gen_100hz #(
        .CLK_HZ(CLK_HZ)
    ) u_tick_100hz (
        .clk        (clk),
        .rst_n      (rst_n),
        .tick_100hz (tick_100hz)
    );

    assign tof_init_start   = KEY_SW[0] & ~key_sw_d[0];
    assign tof_sample_start = tick_100hz;

    always_comb begin
        rover_m1 = 9'sd0;
        rover_m2 = 9'sd0;
        rover_m3 = 9'sd0;
        rover_m4 = 9'sd0;

        if (KEY_SW[1] && !KEY_SW[2]) begin
            rover_m1 =  DRIVE_SPEED;
            rover_m2 =  DRIVE_SPEED;
            rover_m3 =  DRIVE_SPEED;
            rover_m4 =  DRIVE_SPEED;
        end else if (KEY_SW[2] && !KEY_SW[1]) begin
            rover_m1 = -DRIVE_SPEED;
            rover_m2 = -DRIVE_SPEED;
            rover_m3 = -DRIVE_SPEED;
            rover_m4 = -DRIVE_SPEED;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rover_cmd_valid <= 1'b0;
            rover_m1_sent   <= 9'sd0;
            rover_m2_sent   <= 9'sd0;
            rover_m3_sent   <= 9'sd0;
            rover_m4_sent   <= 9'sd0;
        end else begin
            rover_cmd_valid <= 1'b0;

            if (!rover_busy && tick_100hz &&
                ((rover_m1 != rover_m1_sent) ||
                 (rover_m2 != rover_m2_sent) ||
                 (rover_m3 != rover_m3_sent) ||
                 (rover_m4 != rover_m4_sent))) begin
                rover_cmd_valid <= 1'b1;
                rover_m1_sent   <= rover_m1;
                rover_m2_sent   <= rover_m2;
                rover_m3_sent   <= rover_m3;
                rover_m4_sent   <= rover_m4;
            end
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bridge_state          <= BR_IDLE;
            bridge_src            <= SRC_NONE;

            txn_core_start        <= 1'b0;
            txn_core_is_read      <= 1'b0;
            txn_core_dev_addr     <= 7'h00;
            txn_core_reg_addr     <= 16'h0000;
            txn_core_reg_addr_16b <= 1'b0;
            txn_core_wr_data      <= 8'h00;
            txn_core_rd_len       <= 2'd0;

            tof_txn_req_ready     <= 1'b0;
            tof_txn_rsp_done      <= 1'b0;
            tof_txn_rsp_error     <= 1'b0;
            tof_txn_rsp_nack      <= 1'b0;
            tof_txn_rsp_rd_data   <= 16'h0000;

            rover_txn_req_ready   <= 1'b0;
            rover_txn_rsp_done    <= 1'b0;
            rover_txn_rsp_error   <= 1'b0;
            rover_txn_rsp_nack    <= 1'b0;
            rover_txn_rsp_rd_data <= 16'h0000;
        end else begin
            txn_core_start      <= 1'b0;
            tof_txn_req_ready   <= 1'b0;
            tof_txn_rsp_done    <= 1'b0;
            rover_txn_req_ready <= 1'b0;
            rover_txn_rsp_done  <= 1'b0;

            case (bridge_state)
                BR_IDLE: begin
                    if (rover_txn_req_valid && !txn_core_busy) begin
                        bridge_src            <= SRC_ROVER;
                        txn_core_is_read      <= rover_txn_req_is_read;
                        txn_core_dev_addr     <= rover_txn_req_dev_addr;
                        txn_core_reg_addr     <= rover_txn_req_reg_addr;
                        txn_core_reg_addr_16b <= rover_txn_req_reg_addr_16b;
                        txn_core_wr_data      <= rover_txn_req_wr_data;
                        txn_core_rd_len       <= rover_txn_req_rd_len;
                        txn_core_start        <= 1'b1;
                        rover_txn_req_ready   <= 1'b1;
                        bridge_state          <= BR_WAIT;
                    end else if (tof_txn_req_valid && !txn_core_busy) begin
                        bridge_src            <= SRC_TOF;
                        txn_core_is_read      <= tof_txn_req_is_read;
                        txn_core_dev_addr     <= tof_txn_req_dev_addr;
                        txn_core_reg_addr     <= tof_txn_req_reg_addr;
                        txn_core_reg_addr_16b <= tof_txn_req_reg_addr_16b;
                        txn_core_wr_data      <= tof_txn_req_wr_data;
                        txn_core_rd_len       <= tof_txn_req_rd_len;
                        txn_core_start        <= 1'b1;
                        tof_txn_req_ready     <= 1'b1;
                        bridge_state          <= BR_WAIT;
                    end
                end

                BR_WAIT: begin
                    if (txn_core_done) begin
                        case (bridge_src)
                            SRC_TOF: begin
                                tof_txn_rsp_done    <= 1'b1;
                                tof_txn_rsp_error   <= txn_core_error;
                                tof_txn_rsp_nack    <= txn_core_nack;
                                tof_txn_rsp_rd_data <= txn_core_rd_data;
                            end
                            SRC_ROVER: begin
                                rover_txn_rsp_done    <= 1'b1;
                                rover_txn_rsp_error   <= txn_core_error;
                                rover_txn_rsp_nack    <= txn_core_nack;
                                rover_txn_rsp_rd_data <= txn_core_rd_data;
                            end
                            default: begin
                            end
                        endcase

                        bridge_src   <= SRC_NONE;
                        bridge_state <= BR_IDLE;
                    end
                end

                default: begin
                    bridge_src   <= SRC_NONE;
                    bridge_state <= BR_IDLE;
                end
            endcase
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            disp_value <= 16'h0000;
        else if (tof_distance_valid)
            disp_value <= tof_distance_mm;
    end

    always_comb begin
        LED[0] = tof_busy;
        LED[1] = rover_busy;
        LED[2] = tof_error | tof_nack | rover_error | rover_nack;
        LED[3] = tof_distance_valid;
    end

    i2c_master #(
        .CLK_HZ(CLK_HZ),
        .I2C_HZ(I2C_HZ)
    ) u_i2c_master (
        .clk           (clk),
        .rst_n         (rst_n),
        .start         (txn_core_start),
        .is_read       (txn_core_is_read),
        .dev_addr      (txn_core_dev_addr),
        .reg_addr      (txn_core_reg_addr),
        .reg_addr_16b  (txn_core_reg_addr_16b),
        .wr_data       (txn_core_wr_data),
        .rd_len        (txn_core_rd_len),
        .rd_data       (txn_core_rd_data),
        .busy          (txn_core_busy),
        .done          (txn_core_done),
        .error         (txn_core_error),
        .nack          (txn_core_nack),
        .scl_drive_low (scl_drive_low),
        .sda_drive_low (sda_drive_low),
        .scl_in        (i2c_scl),
        .sda_in        (i2c_sda)
    );

    tof_ctrl #(
        .SENSOR_KIND(TOF_SENSOR_KIND),
        .CLK_HZ(CLK_HZ)
    ) u_tof_ctrl (
        .clk                  (clk),
        .rst_n                (rst_n),
        .init_start           (tof_init_start),
        .sample_start         (tof_sample_start),
        .distance_mm          (tof_distance_mm),
        .distance_valid       (tof_distance_valid),
        .busy                 (tof_busy),
        .done                 (tof_done),
        .error                (tof_error),
        .nack                 (tof_nack),
        .txn_req_valid        (tof_txn_req_valid),
        .txn_req_ready        (tof_txn_req_ready),
        .txn_req_is_read      (tof_txn_req_is_read),
        .txn_req_dev_addr     (tof_txn_req_dev_addr),
        .txn_req_reg_addr     (tof_txn_req_reg_addr),
        .txn_req_reg_addr_16b (tof_txn_req_reg_addr_16b),
        .txn_req_wr_data      (tof_txn_req_wr_data),
        .txn_req_rd_len       (tof_txn_req_rd_len),
        .txn_rsp_done         (tof_txn_rsp_done),
        .txn_rsp_error        (tof_txn_rsp_error),
        .txn_rsp_nack         (tof_txn_rsp_nack),
        .txn_rsp_rd_data      (tof_txn_rsp_rd_data)
    );

    roverc_pro_api u_roverc_pro_api (
        .clk                  (clk),
        .rst_n                (rst_n),
        .cmd_valid            (rover_cmd_valid),
        .motor1_speed         (rover_m1),
        .motor2_speed         (rover_m2),
        .motor3_speed         (rover_m3),
        .motor4_speed         (rover_m4),
        .busy                 (rover_busy),
        .done                 (rover_done),
        .error                (rover_error),
        .nack                 (rover_nack),
        .txn_req_valid        (rover_txn_req_valid),
        .txn_req_ready        (rover_txn_req_ready),
        .txn_req_is_read      (rover_txn_req_is_read),
        .txn_req_dev_addr     (rover_txn_req_dev_addr),
        .txn_req_reg_addr     (rover_txn_req_reg_addr),
        .txn_req_reg_addr_16b (rover_txn_req_reg_addr_16b),
        .txn_req_wr_data      (rover_txn_req_wr_data),
        .txn_req_rd_len       (rover_txn_req_rd_len),
        .txn_rsp_done         (rover_txn_rsp_done),
        .txn_rsp_error        (rover_txn_rsp_error),
        .txn_rsp_nack         (rover_txn_rsp_nack),
        .txn_rsp_rd_data      (rover_txn_rsp_rd_data)
    );

    seven_segment_display #(
        .w_digit   (W_DIGIT),
        .clk_mhz   (CLK_HZ / 1_000_000),
        .update_hz (120)
    ) u_seven_segment_display (
        .clk      (clk),
        .rst      (rst),
        .number   (disp_value),
        .dots     ({W_DIGIT{1'b0}}),
        .abcdefgh (SEG),
        .digit    (DIG)
    );
endmodule

`default_nettype wire

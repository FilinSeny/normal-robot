`timescale 1ns / 1ps
`default_nettype none

module top #(
    parameter int unsigned CLK_HZ = 50_000_000,
    parameter int unsigned I2C_HZ = 100_000
)(
    input  wire CLK,
    input  wire [3:0] KEY_SW,

    output logic [3:0] LED,

    inout  wire i2c_scl,
    inout  wire i2c_sda
);

    // ===============================
    // RESET
    // ===============================
    wire clk   = CLK;
    wire rst_n = KEY_SW[3];

    // ===============================
    // I2C master interface
    // ===============================
    logic        start;
    logic        is_read;
    logic [6:0]  dev_addr;
    logic [15:0] reg_addr;
    logic        reg_addr_16b;
    logic [7:0]  wr_data;
    logic [1:0]  rd_len;
    logic [15:0] rd_data;
    logic        busy;
    logic        done;
    logic        error;
    logic        nack;

    logic scl_drive_low;
    logic sda_drive_low;

    assign i2c_scl = scl_drive_low ? 1'b0 : 1'bz;
    assign i2c_sda = sda_drive_low ? 1'b0 : 1'bz;

    // ===============================
    // I2C master
    // ===============================
    i2c_master #(
        .CLK_HZ(CLK_HZ),
        .I2C_HZ(I2C_HZ)
    ) u_i2c_master (
        .clk           (clk),
        .rst_n         (rst_n),
        .start         (start),
        .is_read       (is_read),
        .dev_addr      (dev_addr),
        .reg_addr      (reg_addr),
        .reg_addr_16b  (reg_addr_16b),
        .wr_data       (wr_data),
        .rd_len        (rd_len),
        .rd_data       (rd_data),
        .busy          (busy),
        .done          (done),
        .error         (error),
        .nack          (nack),
        .scl_drive_low (scl_drive_low),
        .sda_drive_low (sda_drive_low),
        .scl_in        (i2c_scl),
        .sda_in        (i2c_sda)
    );

    // ===============================
    // FSM управления моторами
    // ===============================

    localparam [6:0] ROVER_ADDR = 7'h38;
    localparam signed [7:0] SPEED = 8'd60; // скорость вперёд

    typedef enum logic [3:0] {
        IDLE,
        WAIT_POWER,
        M1_START, M1_WAIT,
        M2_START, M2_WAIT,
        M3_START, M3_WAIT,
        M4_START, M4_WAIT,
        DONE
    } state_t;

    state_t state;

    // задержка после старта (~0.1 сек)
    logic [22:0] delay_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= IDLE;
            delay_cnt   <= 0;

            start       <= 0;
            is_read     <= 0;
            dev_addr    <= 0;
            reg_addr    <= 0;
            reg_addr_16b<= 0;
            wr_data     <= 0;
            rd_len      <= 0;
        end else begin
            start <= 0;

            case (state)

                // -------------------
                IDLE:
                    state <= WAIT_POWER;

                // -------------------
                WAIT_POWER:
                    if (delay_cnt < CLK_HZ / 10) begin
                        delay_cnt <= delay_cnt + 1;
                    end else begin
                        state <= M1_START;
                    end

                // ===================
                // MOTOR 1
                M1_START: begin
                    dev_addr <= ROVER_ADDR;
                    reg_addr <= 8'h00;
                    wr_data  <= SPEED;
                    is_read  <= 0;

                    start <= 1;
                    state <= M1_WAIT;
                end

                M1_WAIT:
                    if (done) state <= M2_START;

                // ===================
                // MOTOR 2
                M2_START: begin
                    reg_addr <= 8'h01;
                    start <= 1;
                    state <= M2_WAIT;
                end

                M2_WAIT:
                    if (done) state <= M3_START;

                // ===================
                // MOTOR 3
                M3_START: begin
                    reg_addr <= 8'h02;
                    start <= 1;
                    state <= M3_WAIT;
                end

                M3_WAIT:
                    if (done) state <= M4_START;

                // ===================
                // MOTOR 4
                M4_START: begin
                    reg_addr <= 8'h03;
                    start <= 1;
                    state <= M4_WAIT;
                end

                M4_WAIT:
                    if (done) state <= DONE;

                // -------------------
                DONE:
                    state <= DONE;

            endcase
        end
    end

    // ===============================
    // LED индикация
    // ===============================
    always_comb begin
        LED[0] = busy;
        LED[1] = done;
        LED[2] = error | nack;
        LED[3] = (state == DONE);
    end

endmodule

`default_nettype wire

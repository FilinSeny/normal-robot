`timescale 1ns / 1ps
`default_nettype none

module top #(
    parameter int unsigned CLK_HZ = 50_000_000,
    parameter int unsigned I2C_HZ = 100_000
)(
    input  wire       CLK,
    input  wire [3:0] KEY_SW,
    output logic [3:0] LED,
    inout  wire       i2c_scl,
    inout  wire       i2c_sda
);

    wire clk   = CLK;
    wire rst_n = KEY_SW[3];

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

    localparam [6:0] ROVER_ADDR = 7'h38;
    localparam [7:0] SPEED_FWD  = 8'd70;

    typedef enum logic [3:0] {
        ST_PWRUP_WAIT,
        ST_M1_START, ST_M1_WAIT,
        ST_M2_START, ST_M2_WAIT,
        ST_M3_START, ST_M3_WAIT,
        ST_M4_START, ST_M4_WAIT,
        ST_GAP
    } state_t;

    state_t state;

    logic [24:0] wait_cnt;
    logic        err_latched;
    logic        nack_latched;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= ST_PWRUP_WAIT;
            wait_cnt      <= '0;
            err_latched   <= 1'b0;
            nack_latched  <= 1'b0;

            start         <= 1'b0;
            is_read       <= 1'b0;
            dev_addr      <= ROVER_ADDR;
            reg_addr      <= 16'h0000;
            reg_addr_16b  <= 1'b0;
            wr_data       <= 8'h00;
            rd_len        <= 2'd0;
        end else begin
            start <= 1'b0;

            if (error)
                err_latched <= 1'b1;
            if (nack)
                nack_latched <= 1'b1;

            case (state)
                ST_PWRUP_WAIT: begin
                    if (wait_cnt < CLK_HZ/5) begin
                        wait_cnt <= wait_cnt + 1'b1; // ~200 ms
                    end else begin
                        wait_cnt <= '0;
                        state    <= ST_M1_START;
                    end
                end

                ST_M1_START: begin
                    dev_addr     <= ROVER_ADDR;
                    reg_addr     <= 16'h0000;
                    reg_addr_16b <= 1'b0;
                    wr_data      <= SPEED_FWD;
                    is_read      <= 1'b0;
                    rd_len       <= 2'd0;
                    start        <= 1'b1;
                    state        <= ST_M1_WAIT;
                end

                ST_M1_WAIT: begin
                    if (done) begin
                        state <= ST_M2_START;
                    end else if (error || nack) begin
                        state <= ST_GAP;
                    end
                end

                ST_M2_START: begin
                    reg_addr <= 16'h0001;
                    wr_data  <= SPEED_FWD;
                    start    <= 1'b1;
                    state    <= ST_M2_WAIT;
                end

                ST_M2_WAIT: begin
                    if (done) begin
                        state <= ST_M3_START;
                    end else if (error || nack) begin
                        state <= ST_GAP;
                    end
                end

                ST_M3_START: begin
                    reg_addr <= 16'h0002;
                    wr_data  <= SPEED_FWD;
                    start    <= 1'b1;
                    state    <= ST_M3_WAIT;
                end

                ST_M3_WAIT: begin
                    if (done) begin
                        state <= ST_M4_START;
                    end else if (error || nack) begin
                        state <= ST_GAP;
                    end
                end

                ST_M4_START: begin
                    reg_addr <= 16'h0003;
                    wr_data  <= SPEED_FWD;
                    start    <= 1'b1;
                    state    <= ST_M4_WAIT;
                end

                ST_M4_WAIT: begin
                    if (done) begin
                        wait_cnt <= '0;
                        state    <= ST_GAP;
                    end else if (error || nack) begin
                        wait_cnt <= '0;
                        state    <= ST_GAP;
                    end
                end

                ST_GAP: begin
                    if (wait_cnt < CLK_HZ/20) begin
                        wait_cnt <= wait_cnt + 1'b1; // ~50 ms
                    end else begin
                        wait_cnt <= '0;
                        state    <= ST_M1_START;
                    end
                end

                default: begin
                    state <= ST_PWRUP_WAIT;
                end
            endcase
        end
    end

    always_comb begin
        LED[0] = busy;                         // I2C занят
        LED[1] = (state != ST_PWRUP_WAIT);     // FSM активна
        LED[2] = err_latched | nack_latched;   // была ошибка/NACK
        LED[3] = (state == ST_GAP);            // цикл команд завершён
    end

endmodule

`default_nettype wire

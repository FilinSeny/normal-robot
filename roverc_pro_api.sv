`timescale 1ns / 1ps
`default_nettype none

// RoverC-Pro wheel control API for the existing project architecture.
// Uses the same transaction handshake as tof_ctrl, so it can share one I2C master.
//
// RoverC-Pro I2C protocol:
//   device address = 0x38
//   reg 0x00 = motor1 speed (-127..127)
//   reg 0x01 = motor2 speed (-127..127)
//   reg 0x02 = motor3 speed (-127..127)
//   reg 0x03 = motor4 speed (-127..127)
//
// The module accepts a one-cycle cmd_valid pulse together with 4 signed wheel speeds.
// It then performs 4 I2C byte writes and raises done for one clock on completion.
//
module roverc_pro_api #(
    parameter int unsigned I2C_ADDR = 7'h38
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        cmd_valid,
    input  wire signed [8:0] motor1_speed,
    input  wire signed [8:0] motor2_speed,
    input  wire signed [8:0] motor3_speed,
    input  wire signed [8:0] motor4_speed,

    output logic       busy,
    output logic       done,
    output logic       error,
    output logic       nack,

    output logic       txn_req_valid,
    input  wire        txn_req_ready,
    output logic       txn_req_is_read,
    output logic [6:0] txn_req_dev_addr,
    output logic [15:0] txn_req_reg_addr,
    output logic       txn_req_reg_addr_16b,
    output logic [7:0] txn_req_wr_data,
    output logic [1:0] txn_req_rd_len,

    input  wire        txn_rsp_done,
    input  wire        txn_rsp_error,
    input  wire        txn_rsp_nack,
    input  wire [15:0] txn_rsp_rd_data
);

    typedef enum logic [3:0] {
        ST_IDLE     = 4'd0,
        ST_REQ_M1   = 4'd1,
        ST_WAIT_M1  = 4'd2,
        ST_REQ_M2   = 4'd3,
        ST_WAIT_M2  = 4'd4,
        ST_REQ_M3   = 4'd5,
        ST_WAIT_M3  = 4'd6,
        ST_REQ_M4   = 4'd7,
        ST_WAIT_M4  = 4'd8,
        ST_FINISH   = 4'd9,
        ST_FAIL     = 4'd10
    } state_t;

    state_t state;

    logic cmd_valid_d;
    logic signed [7:0] m1_l;
    logic signed [7:0] m2_l;
    logic signed [7:0] m3_l;
    logic signed [7:0] m4_l;

    function automatic signed [7:0] sat_speed(input signed [8:0] v);
        begin
            if (v > 9'sd127)
                sat_speed = 8'sd127;
            else if (v < -9'sd127)
                sat_speed = -8'sd127;
            else
                sat_speed = v[7:0];
        end
    endfunction

    always_comb begin
        txn_req_valid        = 1'b0;
        txn_req_is_read      = 1'b0;
        txn_req_dev_addr     = I2C_ADDR[6:0];
        txn_req_reg_addr     = 16'h0000;
        txn_req_reg_addr_16b = 1'b0;
        txn_req_wr_data      = 8'h00;
        txn_req_rd_len       = 2'd0;

        case (state)
            ST_REQ_M1: begin
                txn_req_valid    = 1'b1;
                txn_req_reg_addr = 16'h0000;
                txn_req_wr_data  = m1_l[7:0];
            end
            ST_REQ_M2: begin
                txn_req_valid    = 1'b1;
                txn_req_reg_addr = 16'h0001;
                txn_req_wr_data  = m2_l[7:0];
            end
            ST_REQ_M3: begin
                txn_req_valid    = 1'b1;
                txn_req_reg_addr = 16'h0002;
                txn_req_wr_data  = m3_l[7:0];
            end
            ST_REQ_M4: begin
                txn_req_valid    = 1'b1;
                txn_req_reg_addr = 16'h0003;
                txn_req_wr_data  = m4_l[7:0];
            end
            default: begin
            end
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_IDLE;
            cmd_valid_d <= 1'b0;
            m1_l        <= 8'sd0;
            m2_l        <= 8'sd0;
            m3_l        <= 8'sd0;
            m4_l        <= 8'sd0;
            busy        <= 1'b0;
            done        <= 1'b0;
            error       <= 1'b0;
            nack        <= 1'b0;
        end else begin
            cmd_valid_d <= cmd_valid;
            done        <= 1'b0;

            case (state)
                ST_IDLE: begin
                    busy  <= 1'b0;
                    error <= 1'b0;
                    nack  <= 1'b0;

                    if (cmd_valid && !cmd_valid_d) begin
                        m1_l <= sat_speed(motor1_speed);
                        m2_l <= sat_speed(motor2_speed);
                        m3_l <= sat_speed(motor3_speed);
                        m4_l <= sat_speed(motor4_speed);
                        busy <= 1'b1;
                        state <= ST_REQ_M1;
                    end
                end

                ST_REQ_M1: if (txn_req_ready) state <= ST_WAIT_M1;
                ST_REQ_M2: if (txn_req_ready) state <= ST_WAIT_M2;
                ST_REQ_M3: if (txn_req_ready) state <= ST_WAIT_M3;
                ST_REQ_M4: if (txn_req_ready) state <= ST_WAIT_M4;

                ST_WAIT_M1: begin
                    if (txn_rsp_done) begin
                        if (txn_rsp_error || txn_rsp_nack) begin
                            error <= txn_rsp_error;
                            nack  <= txn_rsp_nack;
                            state <= ST_FAIL;
                        end else begin
                            state <= ST_REQ_M2;
                        end
                    end
                end

                ST_WAIT_M2: begin
                    if (txn_rsp_done) begin
                        if (txn_rsp_error || txn_rsp_nack) begin
                            error <= txn_rsp_error;
                            nack  <= txn_rsp_nack;
                            state <= ST_FAIL;
                        end else begin
                            state <= ST_REQ_M3;
                        end
                    end
                end

                ST_WAIT_M3: begin
                    if (txn_rsp_done) begin
                        if (txn_rsp_error || txn_rsp_nack) begin
                            error <= txn_rsp_error;
                            nack  <= txn_rsp_nack;
                            state <= ST_FAIL;
                        end else begin
                            state <= ST_REQ_M4;
                        end
                    end
                end

                ST_WAIT_M4: begin
                    if (txn_rsp_done) begin
                        if (txn_rsp_error || txn_rsp_nack) begin
                            error <= txn_rsp_error;
                            nack  <= txn_rsp_nack;
                            state <= ST_FAIL;
                        end else begin
                            state <= ST_FINISH;
                        end
                    end
                end

                ST_FINISH: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    state <= ST_IDLE;
                end

                ST_FAIL: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    state <= ST_IDLE;
                end

                default: begin
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    error <= 1'b1;
                    nack  <= 1'b0;
                    state <= ST_IDLE;
                end
            endcase
        end
    end

    // Unused for write-only API.
    wire _unused_ok = &{1'b0, txn_rsp_rd_data[0]};

endmodule

`default_nettype wire

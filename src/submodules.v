/*
 * Copyright (c) 2026 Tejas Patel, Pandit Deendayal Energy University
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// Submodule 1: Hamming(13,8) Extended Encoder
module hamming13_encoder (
    input  wire [7:0]  data_in,
    output wire [12:0] code_out
);
    wire p1 = data_in[0] ^ data_in[1] ^ data_in[3] ^ data_in[4] ^ data_in[6];
    wire p2 = data_in[0] ^ data_in[2] ^ data_in[3] ^ data_in[5] ^ data_in[6];
    wire p4 = data_in[1] ^ data_in[2] ^ data_in[3] ^ data_in[7];
    wire p8 = data_in[4] ^ data_in[5] ^ data_in[6] ^ data_in[7];

    wire [11:0] hamming12 = {
        data_in[7], data_in[6], data_in[5], data_in[4], p8,
        data_in[3], data_in[2], data_in[1], p4,
        data_in[0], p2, p1
    };

    wire p0 = ^hamming12;
    assign code_out = {hamming12, p0};
endmodule


// Submodule 2: 0 / 1 / 2-Bit Fault Injector
module fault_injector (
    input  wire [12:0] code_in,
    input  wire [3:0]  err_pos,
    input  wire        err_double,
    output wire [12:0] code_out
);
    reg [12:0] mask1;
    reg [12:0] mask2;

    always @(*) begin
        mask1 = 13'd0;
        mask2 = 13'd0;
        case (err_pos)
            4'd1:  begin mask1 = 13'h0001; mask2 = err_double ? 13'h0002 : 13'd0; end
            4'd2:  begin mask1 = 13'h0002; mask2 = err_double ? 13'h0004 : 13'd0; end
            4'd3:  begin mask1 = 13'h0004; mask2 = err_double ? 13'h0008 : 13'd0; end
            4'd4:  begin mask1 = 13'h0008; mask2 = err_double ? 13'h0010 : 13'd0; end
            4'd5:  begin mask1 = 13'h0010; mask2 = err_double ? 13'h0020 : 13'd0; end
            4'd6:  begin mask1 = 13'h0020; mask2 = err_double ? 13'h0040 : 13'd0; end
            4'd7:  begin mask1 = 13'h0040; mask2 = err_double ? 13'h0080 : 13'd0; end
            4'd8:  begin mask1 = 13'h0080; mask2 = err_double ? 13'h0100 : 13'd0; end
            4'd9:  begin mask1 = 13'h0100; mask2 = err_double ? 13'h0200 : 13'd0; end
            4'd10: begin mask1 = 13'h0200; mask2 = err_double ? 13'h0400 : 13'd0; end
            4'd11: begin mask1 = 13'h0400; mask2 = err_double ? 13'h0800 : 13'd0; end
            4'd12: begin mask1 = 13'h0800; mask2 = err_double ? 13'h1000 : 13'd0; end
            4'd13: begin mask1 = 13'h1000; mask2 = err_double ? 13'h0001 : 13'd0; end
            default: begin
                // err_pos = 0, 14, 15: no fault unless err_double is asserted
                mask1 = 13'd0;
                mask2 = err_double ? 13'h0003 : 13'd0;
            end
        endcase
    end

    assign code_out = code_in ^ mask1 ^ mask2;
endmodule


// Submodule 3: Syndrome & Overall Parity Calculator
module syndrome_calc (
    input  wire [12:0] code_in,
    output wire [3:0]  syndrome,
    output wire        p_tot
);
    assign syndrome[0] = code_in[1] ^ code_in[3] ^ code_in[5] ^ code_in[7] ^ code_in[9]  ^ code_in[11];
    assign syndrome[1] = code_in[2] ^ code_in[3] ^ code_in[6] ^ code_in[7] ^ code_in[10] ^ code_in[11];
    assign syndrome[2] = code_in[4] ^ code_in[5] ^ code_in[6] ^ code_in[7] ^ code_in[12];
    assign syndrome[3] = code_in[8] ^ code_in[9] ^ code_in[10] ^ code_in[11] ^ code_in[12];
    assign p_tot       = ^code_in;
endmodule


// Submodule 4: SECDED Corrector & Flag Generator
module secded_corrector (
    input  wire [12:0] code_in,
    input  wire [3:0]  syndrome,
    input  wire        p_tot,
    output wire [7:0]  data_out,
    output wire [12:0] fixed_code,
    output wire        sec_flag,
    output wire        ded_flag,
    output wire        parity_ok
);
    reg [12:0] corr_mask;

    assign sec_flag  = p_tot;
    assign ded_flag  = (~p_tot) & (|syndrome);
    assign parity_ok = (~p_tot) & (~|syndrome);

    always @(*) begin
        case (syndrome)
            4'd0:  corr_mask = 13'h0001; // Error in overall parity bit P0
            4'd1:  corr_mask = 13'h0002;
            4'd2:  corr_mask = 13'h0004;
            4'd3:  corr_mask = 13'h0008;
            4'd4:  corr_mask = 13'h0010;
            4'd5:  corr_mask = 13'h0020;
            4'd6:  corr_mask = 13'h0040;
            4'd7:  corr_mask = 13'h0080;
            4'd8:  corr_mask = 13'h0100;
            4'd9:  corr_mask = 13'h0200;
            4'd10: corr_mask = 13'h0400;
            4'd11: corr_mask = 13'h0800;
            4'd12: corr_mask = 13'h1000;
            default: corr_mask = 13'd0;
        endcase
    end

    assign fixed_code = code_in ^ (sec_flag ? corr_mask : 13'd0);

    assign data_out = {
        fixed_code[12], fixed_code[11], fixed_code[10], fixed_code[9],
        fixed_code[7],  fixed_code[6],  fixed_code[5],  fixed_code[3]
    };    
endmodule

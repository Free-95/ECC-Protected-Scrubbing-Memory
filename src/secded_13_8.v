/*
 * Copyright (c) 2026 Tejas Patel, Pandit Deendayal Energy University
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_secded_13_8 (
    input  wire [7:0] ui_in,    // Dedicated inputs: data_in[7:0]
    output wire [7:0] uo_out,   // Dedicated outputs: corrected data_out[7:0]
    input  wire [7:0] uio_in,   // IOs: [3:0] err_pos, [4] err_double
    output wire [7:0] uio_out,  // IOs: [5] sec_flag, [6] ded_flag, [7] parity_ok
    output wire [7:0] uio_oe,   // IOs: Enable path (1=output, 0=input)
    input  wire       ena,      // always 1 when powered
    input  wire       clk,      // clock
    input  wire       rst_n     // active-low reset
);

    wire [12:0] encoded_code;
    wire [12:0] corrupted_code;
    wire [3:0]  syndrome;
    wire        p_tot;

    wire [7:0]  data_out_comb;
    wire        sec_flag_comb;
    wire        ded_flag_comb;
    wire        parity_ok_comb;

    reg  [7:0]  data_out_reg;
    reg         sec_flag_reg;
    reg         ded_flag_reg;
    reg         parity_ok_reg;

    // Hamming(13,8) SECDED Encoder
    hamming13_encoder u_enc (
        .data_in  (ui_in),
        .code_out (encoded_code)
    );

    // On-Chip DIP-Switch Fault Injector
    fault_injector u_inj (
        .code_in    (encoded_code),
        .err_pos    (uio_in[3:0]),
        .err_double (uio_in[4]),
        .code_out   (corrupted_code)
    );

    // Syndrome & Overall Parity Calculator
    syndrome_calc u_syn (
        .code_in  (corrupted_code),
        .syndrome (syndrome),
        .p_tot    (p_tot)
    );

    // SECDED Corrector & Status Flags
    secded_corrector u_corr (
        .code_in   (corrupted_code),
        .syndrome  (syndrome),
        .p_tot     (p_tot),
        .data_out  (data_out_comb),
        .sec_flag  (sec_flag_comb),
        .ded_flag  (ded_flag_comb),
        .parity_ok (parity_ok_comb)
    );

    // Output Register Stage
    always @(posedge clk) begin
        if (!rst_n) begin
            data_out_reg  <= 8'd0;
            sec_flag_reg  <= 1'b0;
            ded_flag_reg  <= 1'b0;
            parity_ok_reg <= 1'b0;
        end else begin
            data_out_reg  <= data_out_comb;
            sec_flag_reg  <= sec_flag_comb;
            ded_flag_reg  <= ded_flag_comb;
            parity_ok_reg <= parity_ok_comb;
        end
    end

    // Tiny Tapeout Pin Mapping
    assign uo_out  = data_out_reg;
    assign uio_out = {parity_ok_reg, ded_flag_reg, sec_flag_reg, 5'b00000};
    assign uio_oe  = 8'b1110_0000;

    // Suppress unused inputs to avoid lint warnings
    wire _unused = &{ena, uio_in[7:5], 1'b0};

endmodule

// -----------------------------------------------------------------------------
// Submodule 1: Hamming(13,8) Extended Encoder
// -----------------------------------------------------------------------------
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

// -----------------------------------------------------------------------------
// Submodule 2: 0 / 1 / 2-Bit Fault Injector
// -----------------------------------------------------------------------------
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

// -----------------------------------------------------------------------------
// Submodule 3: Syndrome & Overall Parity Calculator
// -----------------------------------------------------------------------------
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

// -----------------------------------------------------------------------------
// Submodule 4: SECDED Corrector & Flag Generator
// -----------------------------------------------------------------------------
module secded_corrector (
    input  wire [12:0] code_in,
    input  wire [3:0]  syndrome,
    input  wire        p_tot,
    output wire [7:0]  data_out,
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

    wire [12:0] fixed = code_in ^ (sec_flag ? corr_mask : 13'd0);

    assign data_out = {
        fixed[12], fixed[11], fixed[10], fixed[9],
        fixed[7],  fixed[6],  fixed[5],  fixed[3]
    };
    
    wire _unused_fixed = &{fixed[8], fixed[4], fixed[2:0], 1'b0};
endmodule


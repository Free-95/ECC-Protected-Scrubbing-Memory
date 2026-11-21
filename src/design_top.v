/*
 * Copyright (c) 2026 Tejas Patel, Pandit Deendayal Energy University
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_ecc_prot_scrub_mem (
    input  wire [7:0] ui_in,    // Dedicated inputs: Write data[7:0] or Inject {3'b0, err_double, err_pos[3:0]}
    output wire [7:0] uo_out,   // Dedicated outputs: Healed read data[7:0] or Telemetry byte[7:0]
    input  wire [7:0] uio_in,   // IOs: [3:0] addr, [5:4] cmd, [6] scrub_en
    output wire [7:0] uio_out,  // IOs: [7] err_alert (sticky)
    output wire [7:0] uio_oe,   // IOs: Enable path (1=output, 0=input)
    input  wire       ena,      // always 1 when powered
    input  wire       clk,      // clock
    input  wire       rst_n     // active-low synchronous reset
);

    localparam [1:0] CMD_READ   = 2'b00;
    localparam [1:0] CMD_WRITE  = 2'b01;
    localparam [1:0] CMD_INJECT = 2'b10;
    localparam [1:0] CMD_TELEM  = 2'b11;

    // Decode control inputs
    wire [3:0] addr       = uio_in[3:0];
    wire [1:0] cmd        = uio_in[5:4];
    wire       scrub_en   = uio_in[6];
    wire       telem_clr  = (cmd == CMD_TELEM) && addr[3];

    // 16 x 13-bit ECC-protected register file
    // Reset value 13'd0 is the exact valid Hamming(13,8) codeword for 8'h00.
    reg [12:0] mem [0:15];

    // Scrubber & Telemetry state
    reg [3:0] scrub_ptr;
    reg [3:0] sec_cnt;
    reg [3:0] ded_cnt;
    reg [3:0] last_err_addr;
    reg [3:0] last_syndrome;
    reg       last_sec;
    reg       last_ded;
    reg       err_alert_reg;
    reg [7:0] uo_out_reg;

    // -------------------------------------------------------------------------
    // Host Write Encoder & Targeted Memory Fault Injector
    // -------------------------------------------------------------------------
    wire [12:0] wr_encoded_code;
    wire [12:0] host_raw_code = mem[addr];
    wire [12:0] inj_corrupted_code;

    hamming13_encoder u_enc (
        .data_in  (ui_in),
        .code_out (wr_encoded_code)
    );

    fault_injector u_inj (
        .code_in    (host_raw_code),
        .err_pos    (ui_in[3:0]),
        .err_double (ui_in[4]),
        .code_out   (inj_corrupted_code)
    );

    // -------------------------------------------------------------------------
    // Host Read Path: On-the-Fly SECDED Decoder
    // -------------------------------------------------------------------------
    wire [3:0]  read_syndrome;
    wire        read_p_tot;
    wire [7:0]  read_data_out;
    wire [12:0] read_corrected_code;
    wire        read_sec;
    wire        read_ded;
    wire        read_parity_ok;

    syndrome_calc u_read_syn (
        .code_in  (host_raw_code),
        .syndrome (read_syndrome),
        .p_tot    (read_p_tot)
    );

    secded_corrector u_read_corr (
        .code_in        (host_raw_code),
        .syndrome       (read_syndrome),
        .p_tot          (read_p_tot),
        .data_out       (read_data_out),
        .corrected_code (read_corrected_code),
        .sec_flag       (read_sec),
        .ded_flag       (read_ded),
        .parity_ok      (read_parity_ok)
    );

    // -------------------------------------------------------------------------
    // Autonomous Background Scrubber Path: Dedicated SECDED Decoder
    // -------------------------------------------------------------------------
    wire [12:0] scrub_raw_code = mem[scrub_ptr];
    wire [3:0]  scrub_syndrome;
    wire        scrub_p_tot;
    wire [7:0]  scrub_data_out;
    wire [12:0] scrub_corrected_code;
    wire        scrub_sec;
    wire        scrub_ded;
    wire        scrub_parity_ok;

    syndrome_calc u_scrub_syn (
        .code_in  (scrub_raw_code),
        .syndrome (scrub_syndrome),
        .p_tot    (scrub_p_tot)
    );

    secded_corrector u_scrub_corr (
        .code_in        (scrub_raw_code),
        .syndrome       (scrub_syndrome),
        .p_tot          (scrub_p_tot),
        .data_out       (scrub_data_out),
        .corrected_code (scrub_corrected_code),
        .sec_flag       (scrub_sec),
        .ded_flag       (scrub_ded),
        .parity_ok      (scrub_parity_ok)
    );

    // Scrubber runs when enabled and host is not writing/injecting/clearing telemetry
    wire scrub_active = scrub_en && ((cmd == CMD_READ) || ((cmd == CMD_TELEM) && !addr[3]));

    // Telemetry Readback Multiplexer
    reg [7:0] telem_byte;
    always @(*) begin
        case (addr[1:0])
            2'b00:   telem_byte = {ded_cnt, sec_cnt};
            2'b01:   telem_byte = {scrub_ptr, last_err_addr};
            2'b10:   telem_byte = {last_ded, last_sec, 2'b00, last_syndrome};
            default: telem_byte = {7'b0000000, err_alert_reg};
        endcase
    end

    // -------------------------------------------------------------------------
    // Sequential Memory, Scrubber, Telemetry & Output Registers
    // -------------------------------------------------------------------------
    integer i;
    always @(posedge clk) begin
        if (!rst_n) begin
            for (i = 0; i < 16; i = i + 1) begin
                mem[i] <= 13'd0;
            end
            scrub_ptr     <= 4'd0;
            sec_cnt       <= 4'd0;
            ded_cnt       <= 4'd0;
            last_err_addr <= 4'd0;
            last_syndrome <= 4'd0;
            last_sec      <= 1'b0;
            last_ded      <= 1'b0;
            err_alert_reg <= 1'b0;
            uo_out_reg    <= 8'd0;
        end else begin
            // 1. Host Command Execution & Output Register
            case (cmd)
                CMD_READ: begin
                    uo_out_reg <= read_data_out;
                    if (read_sec || read_ded) begin
                        err_alert_reg <= 1'b1;
                        if (!(scrub_active && (scrub_sec || scrub_ded))) begin
                            last_err_addr <= addr;
                            last_syndrome <= read_syndrome;
                            last_sec      <= read_sec;
                            last_ded      <= read_ded;
                        end
                    end
                end

                CMD_WRITE: begin
                    mem[addr]  <= wr_encoded_code;
                    uo_out_reg <= ui_in;
                end

                CMD_INJECT: begin
                    mem[addr]  <= inj_corrupted_code;
                    uo_out_reg <= read_data_out;
                end

                CMD_TELEM: begin
                    uo_out_reg <= telem_byte;
                    if (telem_clr) begin
                        sec_cnt       <= 4'd0;
                        ded_cnt       <= 4'd0;
                        last_err_addr <= 4'd0;
                        last_syndrome <= 4'd0;
                        last_sec      <= 1'b0;
                        last_ded      <= 1'b0;
                        err_alert_reg <= 1'b0;
                    end
                end
            endcase

            // 2. Autonomous Background Scrubber
            if (scrub_active) begin
                scrub_ptr <= scrub_ptr + 4'd1;

                if (scrub_sec) begin
                    mem[scrub_ptr] <= scrub_corrected_code;
                    if (sec_cnt != 4'hF) begin
                        sec_cnt <= sec_cnt + 4'd1;
                    end
                    last_err_addr <= scrub_ptr;
                    last_syndrome <= scrub_syndrome;
                    last_sec      <= 1'b1;
                    last_ded      <= 1'b0;
                    err_alert_reg <= 1'b1;
                end else if (scrub_ded) begin
                    // Do not overwrite memory on uncorrectable double-bit error
                    if (ded_cnt != 4'hF) begin
                        ded_cnt <= ded_cnt + 4'd1;
                    end
                    last_err_addr <= scrub_ptr;
                    last_syndrome <= scrub_syndrome;
                    last_sec      <= 1'b0;
                    last_ded      <= 1'b1;
                    err_alert_reg <= 1'b1;
                end
            end
        end
    end

    // Tiny Tapeout Pin Mapping
    assign uo_out  = uo_out_reg;
    assign uio_out = {err_alert_reg, 7'b0000000};
    assign uio_oe  = 8'b1000_0000;

    // Suppress unused signals to avoid lint warnings
    wire _unused = &{
        ena,
        uio_in[7],
        read_corrected_code,
        read_parity_ok,
        scrub_data_out,
        scrub_parity_ok,
        1'b0
    };

endmodule

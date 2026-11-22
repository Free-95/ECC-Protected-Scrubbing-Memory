/*
- Copyright (c) 2026 Tejas Patel, Pandit Deendayal Energy University
- SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_ecc_prot_scrub_mem (
    input  wire [7:0] ui_in,    // Dedicated Inputs: Write data / Fault cfg / CSR scrub_en (bit 0)
    output wire [7:0] uo_out,   // Dedicated Outputs: Healed read data[7:0] or Telemetry byte[7:0]
    input  wire [7:0] uio_in,   // IOs (in): [2:0] addr / [3:0] telem_ctrl, [5:4] cmd
    output wire [7:0] uio_out,  // IOs (out): [6] stall, [7] err_alert (sticky)
    output wire [7:0] uio_oe,   // IOs (enable path): 1=output, 0=input
    input  wire       ena,      // enable input: always 1 when powered
    input  wire       clk,      // clock
    input  wire       rst_n     // active-low synchronous reset
);

    localparam [1:0] CMD_READ   = 2'b00;
    localparam [1:0] CMD_WRITE  = 2'b01;
    localparam [1:0] CMD_INJECT = 2'b10;
    localparam [1:0] CMD_TELEM  = 2'b11;

    // Decode host inputs
    wire [2:0] addr        = uio_in[2:0];
    wire [1:0] cmd         = uio_in[5:4];
    wire       host_mem_op = (cmd != CMD_TELEM);
    wire       csr_we      = (cmd == CMD_TELEM) && addr[2];
    wire       telem_clr   = (cmd == CMD_TELEM) && uio_in[3];

    // 8 x 13-bit Single-Port ECC-protected register file
    reg [12:0] mem [0:7];

    // Arbiter, Scrubber, CSR & Telemetry registers
    reg        scrub_en_reg;
    reg [3:0]  starve_cnt;
    reg [2:0]  scrub_ptr;
    reg [3:0]  sec_cnt;
    reg [3:0]  ded_cnt;
    reg [2:0]  last_err_addr;
    reg [3:0]  last_syndrome;
    reg        last_sec;
    reg        last_ded;
    reg        err_alert_reg;
    reg        stall_reg;
    reg [7:0]  uo_out_reg;

    // --- 3-Tier Starvation-Preventing Cycle-Stealing Arbiter ---
    wire scrub_forced = scrub_en_reg && (starve_cnt == 4'hF) && host_mem_op;
    wire scrub_idle   = scrub_en_reg && (cmd == CMD_TELEM) && !telem_clr;
    wire scrub_grant  = scrub_forced || scrub_idle;
    wire host_grant   = host_mem_op && !scrub_forced;

    // --- Shared Single-Port RAM Read Path & Single SECDED Decoder ---
    wire [2:0]  port_addr     = scrub_grant ? scrub_ptr : addr;
    wire [12:0] port_raw_code = mem[port_addr];

    wire [12:0] wr_encoded_code;
    wire [12:0] inj_corrupted_code;

    wire [3:0]  dec_syndrome;
    wire        dec_p_tot;
    wire [7:0]  dec_data_out;
    wire [12:0] dec_fixed_code;
    wire        dec_sec;
    wire        dec_ded;
    wire        dec_parity_ok;

    hamming13_encoder u_enc (
        .data_in  (ui_in),
        .code_out (wr_encoded_code)
    );

    fault_injector u_inj (
        .code_in    (port_raw_code),
        .err_pos    (ui_in[3:0]),
        .err_double (ui_in[4]),
        .code_out   (inj_corrupted_code)
    );

    syndrome_calc u_syn (
        .code_in  (port_raw_code),
        .syndrome (dec_syndrome),
        .p_tot    (dec_p_tot)
    );

    secded_corrector u_corr (
        .code_in    (port_raw_code),
        .syndrome   (dec_syndrome),
        .p_tot      (dec_p_tot),
        .data_out   (dec_data_out),
        .fixed_code (dec_fixed_code),
        .sec_flag   (dec_sec),
        .ded_flag   (dec_ded),
        .parity_ok  (dec_parity_ok)
    );

    // --- Shared Single-Port RAM Write Enable & Write Data Mux ---
    reg        mem_we;
    reg [12:0] mem_wdata;

    always @(*) begin
        mem_we    = 1'b0;
        mem_wdata = 13'd0;

        if (scrub_grant && dec_sec) begin
            mem_we    = 1'b1;
            mem_wdata = dec_fixed_code;
        end else if (host_grant && (cmd == CMD_WRITE)) begin
            mem_we    = 1'b1;
            mem_wdata = wr_encoded_code;
        end else if (host_grant && (cmd == CMD_INJECT)) begin
            mem_we    = 1'b1;
            mem_wdata = inj_corrupted_code;
        end
    end

    // --- Telemetry Readback Multiplexer ---
    reg [7:0] telem_byte;
    always @(*) begin
        case (addr[1:0])
            2'b00:   telem_byte = {ded_cnt, sec_cnt};
            2'b01:   telem_byte = {1'b0, scrub_ptr, 1'b0, last_err_addr};
            2'b10:   telem_byte = {last_ded, last_sec, 2'b00, last_syndrome};
            default: telem_byte = {starve_cnt, 2'b00, scrub_en_reg, err_alert_reg};
        endcase
    end

    // --- Sequential Memory, Arbiter, Scrubber & Telemetry FSM ---
    integer i;
    always @(posedge clk) begin
        if (!rst_n) begin
            for (i = 0; i < 8; i = i + 1) begin
                mem[i] <= 13'd0;    // Reset value 13'd0 is the exact valid Hamming(13,8) codeword for 8'h00.
            end
            scrub_en_reg  <= 1'b1;  // Autonomous scrubbing enabled by default
            starve_cnt    <= 4'd0;
            scrub_ptr     <= 3'd0;
            sec_cnt       <= 4'd0;
            ded_cnt       <= 4'd0;
            last_err_addr <= 3'd0;
            last_syndrome <= 4'd0;
            last_sec      <= 1'b0;
            last_ded      <= 1'b0;
            err_alert_reg <= 1'b0;
            stall_reg     <= 1'b0;
            uo_out_reg    <= 8'd0;
        end else begin
            // 1. Single-Port RAM Write
            if (mem_we) begin
                mem[port_addr] <= mem_wdata;
            end

            // 2. Stall Output & Watchdog Starvation Counter
            stall_reg <= scrub_forced;

            if (!scrub_en_reg || scrub_grant) begin
                starve_cnt <= 4'd0;
            end else if (host_grant && (starve_cnt != 4'hF)) begin
                starve_cnt <= starve_cnt + 4'd1;
            end

            // 3. Host Command Execution (when not stalled)
            if (cmd == CMD_TELEM) begin
                uo_out_reg <= telem_byte;
                if (csr_we) begin
                    scrub_en_reg <= ui_in[0];
                end
                if (telem_clr) begin
                    sec_cnt       <= 4'd0;
                    ded_cnt       <= 4'd0;
                    last_err_addr <= 3'd0;
                    last_syndrome <= 4'd0;
                    last_sec      <= 1'b0;
                    last_ded      <= 1'b0;
                    err_alert_reg <= 1'b0;
                end
            end else if (host_grant) begin
                case (cmd)
                    CMD_READ: begin
                        uo_out_reg <= dec_data_out;
                        if (dec_sec || dec_ded) begin
                            err_alert_reg <= 1'b1;
                            last_err_addr <= addr;
                            last_syndrome <= dec_syndrome;
                            last_sec      <= dec_sec;
                            last_ded      <= dec_ded;
                        end
                    end

                    CMD_WRITE: begin
                        uo_out_reg <= ui_in;
                    end

                    CMD_INJECT: begin
                        uo_out_reg <= dec_data_out;
                    end

                    default: begin
                        uo_out_reg <= uo_out_reg;
                    end
                endcase
            end

            // 4. Scrubber Execution (Mandatory Cycle-Steal or Opportunistic Idle)
            if (scrub_grant) begin
                scrub_ptr <= scrub_ptr + 3'd1;

                if (dec_sec) begin
                    if (sec_cnt != 4'hF) begin
                        sec_cnt <= sec_cnt + 4'd1;
                    end
                    last_err_addr <= scrub_ptr;
                    last_syndrome <= dec_syndrome;
                    last_sec      <= 1'b1;
                    last_ded      <= 1'b0;
                    err_alert_reg <= 1'b1;
                end else if (dec_ded) begin
                    // Do not overwrite memory on uncorrectable double-bit error
                    if (ded_cnt != 4'hF) begin
                        ded_cnt <= ded_cnt + 4'd1;
                    end
                    last_err_addr <= scrub_ptr;
                    last_syndrome <= dec_syndrome;
                    last_sec      <= 1'b0;
                    last_ded      <= 1'b1;
                    err_alert_reg <= 1'b1;
                end
            end
        end
    end

    // Tiny Tapeout Pin Mapping
    assign uo_out  = uo_out_reg;
    assign uio_out = {err_alert_reg, stall_reg, 6'b000000};
    assign uio_oe  = 8'b1100_0000;

    // Suppress unused signals to avoid lint warnings
    wire _unused = &{
        ena,
        uio_in[7:6],
        dec_parity_ok,
        1'b0
    };

endmodule

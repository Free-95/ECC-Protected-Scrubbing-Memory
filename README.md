![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/fpga/badge.svg)

# 16×13-bit Hamming(13,8) SECDED ECC-Protected Scrubbing Memory

A digital ASIC implementing a **16-byte Hamming(13,8) Single-Error-Correcting, Double-Error-Detecting (SECDED) register memory** with targeted Single-Event Upset (SEU) fault injection, an autonomous background scrubber, and live diagnostic telemetry.

- [Read the full project datasheet](docs/info.md)

## Overview

This design packs a self-healing ECC memory subsystem onto a single Tiny Tapeout tile (`tt_um_ecc_prot_scrub_mem`):

1. **16×13-bit ECC Register File:** Stores 16 bytes encoded as 13-bit extended Hamming codewords, resetting cleanly to `13'd0` (valid codeword for `0x00`).
2. **Targeted SEU Fault Injector:** Intentionally flips 0, 1, or 2 bits directly inside any stored memory word (`mem[addr]`) to simulate radiation-induced bit flips in real time.
3. **Dual SECDED Decoder Paths:**
   - **Host Read Path:** Heals single-bit errors on the fly during `CMD_READ` and flags uncorrectable double-bit errors without miscorrecting data.
   - **Autonomous Background Scrubber:** Sweeps `scrub_ptr` across addresses `0x0`–`0xF` at 1 word/cycle during Read and non-clearing Telemetry cycles, permanently writing healed codewords back to RAM.
4. **Diagnostic Telemetry & Sticky Alert:** Tracks 4-bit saturating `sec_cnt` and `ded_cnt` counters, `scrub_ptr`, `last_err_addr`, `last_syndrome`, error classification flags, and a sticky `err_alert` output pin (`uio_out[7]`).

## Architecture

```text
ui_in[7:0] (data_in / fault_cfg)       uio_in[3:0] (addr), uio_in[5:4] (cmd), uio_in[6] (scrub_en)
   │                                                            │
   ├──► [hamming13_encoder] ──(13b)──►┐                         │
   │                                  ▼                         ▼
   └──► [fault_injector] ◄──(13b)──► ┌─────────────────────────────────────┐
            (corrupts mem[addr])     │   16 × 13-bit ECC Register File     │
                                     │   (208 DFFs, resets to valid 0x00)  │
                                     └──────┬──────────────────────┬───────┘
                                  mem[addr] │                      │ mem[scrub_ptr]
                                            ▼                      ▼
                                   ┌─────────────────┐    ┌─────────────────┐
                                   │ Host Read Path  │    │ Scrubber Path   │
                                   │ syndrome_calc + │    │ syndrome_calc + │
                                   │ secded_corrector│    │ secded_corrector│
                                   └────────┬────────┘    └────────┬────────┘
                                            │                      ├──► Healed 13-bit writeback
                                            │                      └──► Telemetry & Counters
                                            ▼                           (sec_cnt, ded_cnt, last_*)
                                   ┌───────────────────────────────────────┐
                                   │   Output & Sticky Alert Registers     │ (1-cycle latency)
                                   └────────┬──────────────────────┬───────┘
                                            ▼                      ▼
                                       uo_out[7:0]             uio_out[7]
                                   (data_out / telem)          (err_alert)
```

## Pinout

| Port | Direction | Signal | Description |
| --- | --- | --- | --- |
| `ui_in[7:0]` | Input | `data_in` / `fault_cfg` | Write byte (`CMD_WRITE`) or `{3'b0, err_double, err_pos[3:0]}` (`CMD_INJECT`) |
| `uo_out[7:0]` | Output | `data_out` / `telem` | Healed read byte (`CMD_READ`), write-through echo, or telemetry byte (`CMD_TELEM`) |
| `uio[3:0]` | Input | `addr[3:0]` | Memory address (`0x0`–`0xF`). During `CMD_TELEM`: `[1:0]` selects reg, `[3]=1` clears counters |
| `uio[5:4]` | Input | `cmd[1:0]` | `00` = Read, `01` = Write, `10` = Inject Fault, `11` = Read/Clear Telemetry |
| `uio[6]` | Input | `scrub_en` | `1` = enable autonomous background scrubbing during Read and non-clearing Telemetry |
| `uio[7]` | Output | `err_alert` | Sticky alert flag (asserts on any SEC/DED event; cleared by `rst_n` or `addr[3]=1` telemetry clear) |

Bidirectional pin direction mask: `uio_oe = 8'b1000_0000`

## Telemetry Register Map (`cmd = 2'b11`)

| `addr[1:0]` | `uo_out[7:4]` | `uo_out[3:0]` | Description |
| --- | --- | --- | --- |
| `00` | `ded_cnt[3:0]` | `sec_cnt[3:0]` | 4-bit saturating scrubber error counters (cap at `15`) |
| `01` | `scrub_ptr[3:0]` | `last_err_addr[3:0]` | Current scrubber address pointer and last faulted address |
| `10` | `{last_ded, last_sec, 2'b00}` | `last_syndrome[3:0]` | Most recent error classification and 4-bit syndrome |
| `11` | `4'b0000` | `{3'b000, err_alert}` | Live sticky error alert status bit |

*Setting `addr[3] = 1` on any telemetry read returns the pre-clear snapshot and atomically resets all counters, diagnostic latches, and `err_alert` to `0`.*

## Verification

The design is verified using a sign-off Cocotb testbench (`test/test_scrub_mem.py`) checked cycle-by-cycle against an independent Python golden scoreboard, covering:

* Synchronous reset override under active writes/injections and `uio_oe` direction checks
* All 16 `err_pos` values (`0`–`15`) for both `err_double = 0` and `err_double = 1` (including out-of-range and `err_pos = 13` wrap-around)
* Cumulative multi-cycle fault injection (same-bit cancellation and distinct-bit escalation to DED)
* Scrubber pointer advance/pause rules across all commands and same-cycle host/scrubber collision priority
* Both `sec_cnt` and `ded_cnt` saturation at `15` (`4'hF`), atomic telemetry clear, mid-operation reset, and unused-pin immunity
* 2,000-cycle deterministic pseudo-random stress test with periodic full-RAM and full-telemetry audits

**Commands:**

```sh
cd test
make -B             # Run RTL simulation (Icarus Verilog + Cocotb)
make -B GATES=yes   # Run Gate-Level Simulation (requires gate_level_netlist.v)
gtkwave tb.fst tb.gtkw
```

## Repository Layout

* `docs/info.md` — Datasheet documentation
* `src/design_top.v` — Top-level RTL (`tt_um_ecc_prot_scrub_mem`: 16×13-bit RAM, scrubber FSM, and telemetry)
* `src/submodules.v` — Hamming(13,8) encoder, fault injector, syndrome calculator, and SECDED corrector
* `src/config.json` — LibreLane hardening configuration
* `test/Makefile` — Makefile for compiling and running RTL simulation & GLS
* `test/test_scrub_mem.py` — Sign-off Cocotb verification suite with golden scoreboard
* `test/tb.v` — Verilog testbench wrapper (RTL + GLS power-pin support)
* `info.yaml` — Tiny Tapeout shuttle metadata and pinout

## About Tiny Tapeout

Built for [Tiny Tapeout](https://tinytapeout.com) using [LibreLane](https://www.zerotoasiccourse.com/terminology/librelane/).

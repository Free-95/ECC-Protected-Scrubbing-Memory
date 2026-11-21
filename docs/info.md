## How it works

This project implements a **16×13-bit Hamming(13,8) Single-Error-Correcting, Double-Error-Detecting (SECDED) ECC-Protected Scrubbing Memory** with targeted Single-Event Upset (SEU) fault injection and diagnostic telemetry. It models the radiation-hardened ECC memory architecture used in spacecraft flight computers and server DRAM—combining on-the-fly error correction during host reads with an autonomous background scrubber that permanently heals bit flips before a second upset can turn them into an uncorrectable error.

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

### Codeword Bit Mapping (`code[12:0]`)

Every 8-bit byte written to memory is encoded into a 13-bit extended Hamming codeword with parity bits at power-of-two positions plus overall even parity (`P0`) at bit `0`, allowing the 4-bit syndrome to index any single-bit error directly:

| **Bit Index** | 12 | 11 | 10 | 9 | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| **Field** | `D7` | `D6` | `D5` | `D4` | `P8` | `D3` | `D2` | `D1` | `P4` | `D0` | `P2` | `P1` | `P0` |

### Command Interface (`uio_in[5:4]`)

| `cmd[1:0]` | Operation | `ui_in[7:0]` Meaning | `uo_out[7:0]` Output | Scrubber (`scrub_en=1`) |
| --- | --- | --- | --- | --- |
| `00` | Read (`CMD_READ`) | Ignored | On-the-fly healed byte from `mem[addr]` | Active (advances `scrub_ptr`) |
| `01` | Write (`CMD_WRITE`) | 8-bit `data_in` to encode & store | Write-through echo of `ui_in` | Paused (host owns write port) |
| `10` | Inject Fault (`CMD_INJECT`) | `{3'b0, err_double, err_pos[3:0]}` | Pre-injection healed byte | Paused (host owns write port) |
| `11` | Telemetry (`CMD_TELEM`) | Ignored | Selected telemetry byte (`addr[1:0]`) | Active if `addr[3]=0`; paused if `addr[3]=1` |

* **Fault Injection (`CMD_INJECT`):** `err_pos` (`ui_in[3:0]`) values `1`–`13` flip codeword bit `err_pos - 1`. Setting `err_double` (`ui_in[4]`) to `1` flips an additional adjacent bit (wrapping `13` to bit `0`) to create an uncorrectable 2-bit error. Values `0`, `14`, and `15` inject no fault when `err_double = 0`, or flip bits `[1:0]` when `err_double = 1`.
* **Autonomous Background Scrubber (`uio_in[6] = 1`):** Sweeps `scrub_ptr` through addresses `0x0`–`0xF` at 1 word/cycle during Read and non-clearing Telemetry cycles. Single-bit errors (SEC) are written back healed and increment `sec_cnt`; double-bit errors (DED) increment `ded_cnt` and raise `err_alert` without overwriting memory.

### Telemetry Register Map (`cmd = 2'b11`)

Setting `addr[3] = 1` on any telemetry read returns the selected register's snapshot and atomically resets all counters, diagnostic latches, and `err_alert` (`uio_out[7]`) to `0`.

| `addr[1:0]` | `uo_out[7:4]` | `uo_out[3:0]` | Description |
| --- | --- | --- | --- |
| `00` | `ded_cnt[3:0]` | `sec_cnt[3:0]` | 4-bit saturating scrubber error counters (cap at `15`) |
| `01` | `scrub_ptr[3:0]` | `last_err_addr[3:0]` | Current scrubber pointer and last faulted address |
| `10` | `{last_ded, last_sec, 2'b00}` | `last_syndrome[3:0]` | Most recent error classification and 4-bit syndrome |
| `11` | `4'b0000` | `{3'b000, err_alert}` | Sticky error alert status bit |

---

## How to test

1. **Clock and Reset:** Apply a system clock (25 MHz nominal, timing-closed to 50 MHz) and pulse `rst_n` low. All 16 memory words reset to `13'd0` (valid codeword for `0x00`), and `uo_out` and `err_alert` (`uio_out[7]`) clear to `0`.
2. **Write & Clean Readback:** Set `scrub_en = 0`, `cmd = 01` (`CMD_WRITE`), `addr = 0x3`, and `ui_in = 0xA5`. Clock once, then switch to `cmd = 00` (`CMD_READ`) at `addr = 0x3`. Verify `uo_out == 0xA5` and `err_alert == 0`.
3. **Single-Bit Fault Injection & On-the-Fly Healing:** Keep `scrub_en = 0`. Set `cmd = 10` (`CMD_INJECT`), `addr = 0x3`, and `ui_in = 0x05` (`err_pos = 5`, `err_double = 0`) and clock once to corrupt bit `4` inside `mem[3]`. Read `addr = 0x3` (`cmd = 00`): verify `uo_out` still returns the healed byte `0xA5` while `err_alert` goes high.
4. **Autonomous Background Scrubbing:** Set `scrub_en = 1` and hold `cmd = 00` (`CMD_READ`) for 16 clock cycles so the scrubber sweeps all 16 slots. Read telemetry register `00` (`cmd = 11`, `addr = 0x0`) and verify `sec_cnt == 1`. Clear telemetry (`cmd = 11`, `addr = 0x8`), then read `addr = 0x3` with `scrub_en = 0`—verify `uo_out == 0xA5` and `err_alert == 0`, proving the RAM word was permanently healed.
5. **Double-Bit Error Detection (DED):** Inject a 2-bit fault at `addr = 0x6` (`cmd = 10`, `ui_in = 0x15` for `err_double = 1`, `err_pos = 5`). Sweep 16 cycles with `scrub_en = 1`, then read telemetry registers `00` and `10` to confirm `ded_cnt` incremented and `last_ded = 1` without miscorrecting memory.

---

## External hardware

No external hardware is required—the design can be exercised directly via the Tiny Tapeout carrier board's RP2040 controller, onboard switches, and output display/LEDs. For manual bench demonstrations, optional switches on `uio[6:0]` and an alert LED on `uio[7]` can be connected to the bidirectional Pmod header.

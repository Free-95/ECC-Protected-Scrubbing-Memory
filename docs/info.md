## How it works

This project implements an **8×13-bit Single-Port Hamming(13,8) Single-Error-Correcting, Double-Error-Detecting (SECDED) ECC-Protected Scrubbing Memory** with a **3-tier starvation-preventing cycle-stealing arbiter**, targeted Single-Event Upset (SEU) fault injection, and diagnostic telemetry. It models the radiation-hardened ECC memory controller architecture used in server DRAM and spacecraft flight computers—combining a compact single-port register file and shared SECDED decoder with an autonomous scrubber that permanently heals bit flips before a second upset can escalate into an uncorrectable error.

```text
ui_in[7:0] (data_in / fault_cfg / csr_val)      uio_in[2:0] (addr), uio_in[3] (clr), uio_in[5:4] (cmd)
   │                                                                   │
   ├──► [hamming13_encoder] ──────────────────────┐                    ▼
   │                                              │       ┌─────────────────────────┐
   │  ┌─[fault_injector]◄──(port_raw_code)──┐     │       │ 3-Tier Watchdog Arbiter │
   │  │                                     │     │       │ (starve_cnt, scrub_ptr, │
   │  ▼                                     │     ▼       │  internal scrub_en CSR) │
   │  └────────► [Single-Port Write Mux] ───┴───► ┌───────────────────────────┐     │
   │                                              │  8 × 13-bit Single-Port   │◄────┘
   │                                              │     ECC Register File     │
   │                                              │ (104 DFFs, resets to 0x00)│
   │                                              └─────────────┬─────────────┘
   │                                                            │ port_raw_code (mem[port_addr])
   │                                                            ▼
   │                                              ┌───────────────────────────┐
   │                                              │   Shared SECDED Decoder   │
   │                                              │     (syndrome_calc +      │
   │                                              │     secded_corrector)     │
   │                                              └──────┬────────────┬───────┘
   │                                                     │            ├──► Healed 13b writeback
   │                                                     │            └──► Telemetry & Counters
   │                                                     ▼                 (sec_cnt, ded_cnt, last_*)
   │                                              ┌────────────────────────────────────┐
   │                                              │ Output, Stall & Alert Registers    │
   │                                              └───────┬────────────────────┬───────┘
   │                                                      ▼                    ▼
   │                                                 uo_out[7:0]          uio_out[7:6]
   │                                             (data_out / telem)   (err_alert, stall)
```

### Codeword Bit Mapping

Every 8-bit byte written to memory is encoded into a 13-bit extended Hamming codeword with parity bits at power-of-two positions plus overall even parity (`P0`) at bit `0`, allowing the 4-bit syndrome to index any single-bit error directly:

| **Bit Index** | 12 | 11 | 10 | 9 | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| **Field** | `D7` | `D6` | `D5` | `D4` | `P8` | `D3` | `D2` | `D1` | `P4` | `D0` | `P2` | `P1` | `P0` |

### Command Interface (`uio_in[5:4]`)

| `cmd[1:0]` | Operation | `ui_in[7:0]` Meaning | `uo_out[7:0]` Output | Arbiter & Scrubber (`scrub_en = 1`) |
| --- | --- | --- | --- | --- |
| `00` | Read (`CMD_READ`) | Ignored | On-the-fly healed byte from `mem[addr]` | Host op (increments `starve_cnt`; stalls if `starve_cnt == 15`) |
| `01` | Write (`CMD_WRITE`) | 8-bit `data_in` to encode & store | Write-through echo of `ui_in` | Host op (increments `starve_cnt`; stalls if `starve_cnt == 15`) |
| `10` | Inject Fault (`CMD_INJECT`) | `{3'b0, err_double, err_pos[3:0]}` | Pre-injection healed byte | Host op (increments `starve_cnt`; stalls if `starve_cnt == 15`) |
| `11` | Telemetry / CSR (`CMD_TELEM`) | `ui_in[0] = scrub_en` if `uio_in[2] = 1` | Selected telemetry byte (`uio_in[1:0]`) | Opportunistic idle scrub if `uio_in[3] = 0`; paused if `uio_in[3] = 1` |

* **Fault Injection (`CMD_INJECT`):** `err_pos` (`ui_in[3:0]`) values `1`–`13` flip codeword bit `err_pos - 1` inside `mem[addr]`. Setting `err_double` (`ui_in[4]`) to `1` flips an additional adjacent bit (wrapping `13` to bit `0`) to create an uncorrectable 2-bit error. Values `0`, `14`, and `15` inject no fault when `err_double = 0`, or flip bits `[1:0]` when `err_double = 1`.
* **3-Tier Watchdog Arbiter & `stall` Handshake (`uio_out[6]`):** Because the RAM array and SECDED decoder are single-ported, an internal arbiter prevents scrubber starvation without dropping host transactions:
1. **Mandatory Cycle-Steal (Highest Priority):** When `scrub_en = 1` (enabled by default after reset), each consecutive host memory operation increments `starve_cnt`. When `starve_cnt == 15`, the arbiter preempts the next host memory op for 1 cycle to scrub `mem[scrub_ptr]`, resets `starve_cnt` to `0`, holds `uo_out`, and asserts `stall` (`uio_out[6] = 1`) so the host re-issues the command on the following cycle.
2. **Normal Host Access:** When `starve_cnt < 15` (or `scrub_en = 0`), host memory operations execute immediately with `stall = 0`.
3. **Opportunistic Idle Scrubbing:** During non-clearing `CMD_TELEM` cycles (`uio_in[3] = 0`) while `scrub_en = 1`, the scrubber uses the idle RAM port for free (`stall = 0`), resets `starve_cnt` to `0`, heals any 1-bit error in place, and advances `scrub_ptr` (`0x0`–`0x7`).



### Telemetry & CSR Register Map (`cmd = 2'b11`)

* **`uio_in[1:0]`:** Selects the telemetry register to read on `uo_out[7:0]`.
* **`uio_in[2]` (`csr_we`):** Writes `ui_in[0]` into the internal `scrub_en` control register (`1` = enabled, `0` = disabled).
* **`uio_in[3]` (`telem_clr`):** Returns the pre-clear snapshot of the selected register and atomically resets `sec_cnt`, `ded_cnt`, `last_*` latches, and `err_alert` (`uio_out[7]`) to `0` (while pausing the scrubber for that cycle).

| `uio_in[1:0]` | `uo_out[7:4]` | `uo_out[3:0]` | Description |
| --- | --- | --- | --- |
| `00` | `ded_cnt[3:0]` | `sec_cnt[3:0]` | 4-bit saturating scrubber error counters (cap at `15`) |
| `01` | `{1'b0, scrub_ptr[2:0]}` | `{1'b0, last_err_addr[2:0]}` | Current scrubber pointer (`0`–`7`) and last faulted address (`0`–`7`) |
| `10` | `{last_ded, last_sec, 2'b00}` | `last_syndrome[3:0]` | Most recent error classification and 4-bit syndrome |
| `11` | `starve_cnt[3:0]` | `{2'b00, scrub_en, err_alert}` | Watchdog starvation counter, internal `scrub_en` CSR, and sticky alert |

---

## How to test

1. **Clock and Reset:** Apply a system clock (25 MHz nominal, timing-closed to 50 MHz) and pulse `rst_n` low. All 8 memory words reset to `13'd0` (valid codeword for `0x00`), `scrub_en` defaults to `1`, and `uo_out`, `stall` (`uio_out[6]`), and `err_alert` (`uio_out[7]`) clear to `0`.
2. **Pause Scrubber & Write/Read:** To inspect faults manually before scrubbing, disable `scrub_en` via CSR write: set `cmd = 11`, `uio_in[3:0] = 4'b1111` (`telem_clr = 1`, `csr_we = 1`, reg `3`), and `ui_in = 0x00`, then clock once. Write `0xA5` to address `0x3` (`cmd = 01`, `uio_in[2:0] = 3'd3`, `ui_in = 0xA5`), then read back (`cmd = 00`, `uio_in[2:0] = 3'd3`). Verify `uo_out == 0xA5`, `stall == 0`, and `err_alert == 0`.
3. **Single-Bit Fault Injection & On-the-Fly Healing:** Keep `scrub_en = 0`. Issue `cmd = 10` (`CMD_INJECT`) at `addr = 0x3` with `ui_in = 0x05` (`err_pos = 5`, `err_double = 0`) and clock once to flip bit `4` inside `mem[3]`. Read `addr = 0x3` (`cmd = 00`): verify `uo_out` still returns the healed byte `0xA5` while `err_alert` goes high.
4. **Opportunistic Background Scrubbing:** Re-enable `scrub_en` (`cmd = 11`, `uio_in[3:0] = 4'b0111`, `ui_in = 0x01`), then hold `cmd = 11`, `uio_in[3:0] = 4'b0000` for 8 clock cycles so the scrubber sweeps all 8 slots with `stall == 0`. Read telemetry register `00` and verify `sec_cnt == 1`. Disable scrubbing and clear telemetry (`cmd = 11`, `uio_in[3:0] = 4'b1100`, `ui_in = 0x00`), then read `addr = 0x3` (`cmd = 00`)—verify `uo_out == 0xA5` and `err_alert == 0`, proving `mem[3]` was permanently healed in RAM.
5. **Watchdog Cycle-Stealing (`stall` Handshake):** With `scrub_en = 1`, issue 16 consecutive host memory ops (`CMD_READ`/`CMD_WRITE`) without idle telemetry cycles. On the 16th op, verify `stall` (`uio_out[6]`) pulses high for 1 cycle while the arbiter forces a mandatory scrub and holds `uo_out`; re-issue the host op on cycle 17 and verify it completes with `stall == 0`.
6. **Double-Bit Error Detection (DED):** Inject a 2-bit fault at `addr = 0x6` (`cmd = 10`, `ui_in = 0x15` for `err_double = 1`, `err_pos = 5`). Run an 8-cycle telemetry sweep with `scrub_en = 1`, then read telemetry registers `00` and `10` to confirm `ded_cnt` incremented and `last_ded = 1` without overwriting memory.

---

## External hardware

No external hardware is required—the design can be exercised directly via the Tiny Tapeout carrier board's RP2040 controller, onboard switches, and output display/LEDs. For manual bench demonstrations, optional switches on `uio[5:0]` and two status LEDs on `uio[7:6]` (`err_alert` and `stall`) can be connected to the bidirectional Pmod header.

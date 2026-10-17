# Dual brushed DC motor driver firmware

This is firmware for the custom STM32F103C8T6 board with 4× BTN8962 half-bridges (schematic `btn7971pcb_layoutrevision`). It drives two H-bridge motors in closed loop, with a cascaded position and velocity PID per motor, current limiting from the IS pins, latched faults, an ASCII protocol on UART3, and an ODrive 0.5.x compatible protocol on CAN, so it can share a bus with ODrives.

A desktop GUI (`tools/gui`) controls the motors and edits the settings over either link, through a USB-UART or USB-CAN adapter. Settings changed at runtime can be saved to flash.

**Framework:** PlatformIO with STM32Cube HAL (`framework = stm32cube`). The HAL handles initialisation. The interrupt hot paths (TIM2 valley, ADC trigger, UART ring buffers, EXTI) touch registers directly to keep latency predictable. I found no reason to recommend anything else. Arduino-STM32 would hide the timer and ADC details this board depends on, and bare CMSIS would just mean rewriting HAL init code.

```
dual-motor-fw/
├── platformio.ini            4 envs: ENC1_CONFIG × USE_CURRENT_SENSE
├── include/
│   ├── config.h              every option, limit, gain and constant
│   ├── pins.h                every pin, named after the schematic nets
│   └── stm32f1xx_hal_conf.h  HAL module selection
├── src/
│   ├── main.c                init order, main loop, command watchdog, LEDs, IWDG
│   ├── stm32f1xx_it.c        SysTick + fault handlers (fault => both INH low)
│   ├── drivers/              clock, board (INH/LED/GPIO), pwm, adc_is, encoder,
│   │                         limits, ppm, uart, can_bus
│   ├── drivers/flash_store.c settings page in flash
│   ├── control/              pid, motor (cascade, limits, faults), params (runtime settings)
│   └── comms/                protocol (ASCII), can_proto, fmt
└── tools/
    ├── gui/                  desktop control + configuration GUI (Python/Tkinter)
    └── board_sim/            the firmware's control + protocol code on a PC, with a
                              motor model and fake UART / USB-CAN ports
```

## Pin map

LQFP48 pin numbers. Net names are as drawn on the schematic. Motor numbering is fixed as drawn: motor 1 = U6/U8 (OUT1/OUT2) with encoder 1 on CN1, motor 2 = U2/U3 (OUT3/OUT4) with encoder 2 on CN2.

| Net | MCU pin | Pin # | Function in firmware | Notes |
|---|---|---|---|---|
| P1 | PA3 | 13 | TIM2_CH4 PWM → R2 → U6 IN | Motor 1 side A (OUT1) |
| P2 | PA2 | 12 | TIM2_CH3 PWM → R25 → U8 IN | Motor 1 side B (OUT2) |
| P3 | PA1 | 11 | TIM2_CH2 PWM → R49 → U2 IN | Motor 2 side A (OUT3) |
| P4 | PA0 | 10 | TIM2_CH1 PWM → R50 → U3 IN | Motor 2 side B (OUT4) |
| E1 | PC13 | 2 | GPIO out, static | U6+U8 INH, 10k pull-down |
| E2 | PC14 | 3 | GPIO out, static | U2+U3 INH, 10k pull-down |
| IS1 | PA4 | 14 | ADC1_IN4 | U6 IS, R11 3.3k |
| IS2 | PA5 | 15 | ADC1_IN5 | U8 IS, R14 3.3k |
| IS3 | PB0 | 18 | ADC1_IN8 | U2 IS, R17 3.3k |
| IS4 | PB1 | 19 | ADC1_IN9 | U3 IS, R20 3.3k |
| ENC_A1 | PA6 | 16 | TIM3_CH1 (config B) / unused (config A) | CN1-2, **not 5 V tolerant** |
| ENC_B1 | PA7 | 17 | TIM3_CH2 (config B) / unused (config A) | CN1-3, **not 5 V tolerant** |
| ENC_Z1 | PA8 | 29 | TIM1_CH1 = encoder 1 A (config A) / free or Z index (config B) | CN1-1, 5 V tolerant |
| RS485_TX | PA9 | 30 | TIM1_CH2 = encoder 1 B (config A) / unused (config B) | U1 pad 4 only |
| RS485_RX | PA10 | 31 | unused (analog) | |
| RS485_DE/RE | PB12 | 25 | unused (analog) | |
| ENC_A2 | PB6 | 42 | TIM4_CH1 | CN2-2 |
| ENC_B2 | PB7 | 43 | TIM4_CH2 | CN2-3 |
| ENC_Z2 | PB9 | 46 | unused | CN2-1 |
| LIMIT1 | PB15 | 28 | input, pull-up, active low | U13, motor 1 by default (`LIM1_MOTOR`) |
| LIMIT2 | PB2 | 20 | input, pull-up, active low | U15, motor 2 by default (`LIM2_MOTOR`); pin is also BOOT1 |
| STATUS | PB8 | 45 | GPIO out, **active low** | U18 green |
| ERROR | PB5 | 41 | GPIO out, **active low** | U19 red |
| UART_TX | PB10 | 21 | USART3_TX | U4-3 |
| UART_RX | PB11 | 22 | USART3_RX | U4-2 |
| CAN_RX | PA11 | 32 | CAN_RX | U14 R |
| CAN_TX | PA12 | 33 | CAN_TX | U14 D |
| PPM1 | PB13 | 26 | EXTI13, both edges | U17-2 |
| PPM2 | PB14 | 27 | EXTI14, both edges | U17-1 |
| SWDIO / SWCLK | PA13 / PA14 | 34 / 37 | SWD | H1; JTAG disabled |
| — | PA15, PB3, PB4, PC15 | | unused (analog) | JTAG pins released |

### Timer and resource plan

| Resource | Config A (`ENC1_ON_TIM1`) | Config B (`ENC1_ON_TIM3_3V3`) |
|---|---|---|
| TIM2 | PWM ×4, centre-aligned, 20 kHz; drives ADC + control tick | same |
| TIM1 | encoder 1 (PA8/PA9) | free |
| TIM3 | free | encoder 1 (PA6/PA7) |
| TIM4 | encoder 2 (PB6/PB7) | same |
| DWT cycle counter | PPM pulse width (64 MHz free-running) | same |
| ADC1 + DMA1 Ch1 | IS1..IS4 scan, circular buffer | same |

The PPM pins are output-only (TIM1_CH1N/CH2N), so they are read with EXTI. Pulse widths are timed with the core's DWT cycle counter rather than a spare timer. Config A and config B leave different timers free, so a spare timer would have made this code differ between them.

### How the timing works

1. TIM2 runs centre-aligned in PWM mode 1, so every on-pulse is centred on CNT = 0 (the valley).
2. At each valley, the TIM2 update interrupt starts one 4-channel ADC scan (7.5 µs). The STM32F1 ADC has no regular-group trigger from TIM2 apart from CC2, and CC2 is a motor output here. So the scan is started by software inside the interrupt, with under 1 µs of jitter.
3. DMA writes the scan into a circular buffer that holds the last `IS_AVG_SAMPLES` PWM periods. The control loop averages that buffer.
4. Every `PWM_FREQ_HZ / CTRL_LOOP_HZ` valleys, the interrupt pends PendSV, which runs the 1 kHz control loop at lower priority. The loop is therefore phase-locked to the PWM and the current samples.

## Build options

### `ENC1_CONFIG`

Pick the matching environment, or pass `-DENC1_CONFIG=...`:

```
pio run -e enc1_tim1            # config A (default)
pio run -e enc1_tim3_3v3        # config B
pio run -e enc1_tim1 -t upload  # flash with ST-Link
```

Only `pins.h` (the `ENC1_*` block and the unused-pin mask) and the timer instance in `encoder.c` change between the two configs. All other code is shared.

**Config A, `ENC1_ON_TIM1` (default).** This needs board rework:
- Encoder A goes to **CN1 pin 1** (net ENC_Z1 → PA8).
- Encoder B needs a **bodge wire to PA9**. Net RS485_TX only reaches U1 pad 4 (MAX3485 DI); it is not on any connector. U1 must not be fitted.
- **CN1 pins 2 and 3 must stay disconnected.** PA6/PA7 are not 5 V tolerant and CN1 supplies +5 V to the encoder.
- No Z index is available.

**Config B, `ENC1_ON_TIM3_3V3`.** The encoder uses CN1 pins 2/3 as drawn. CN1 pin 4 is hard-wired to +5 V, so the encoder's supply must be moved to 3.3 V on the board. PA8 (CN1 pin 1) stays free. Set `ENC1_INDEX_ENABLE=1` to latch the encoder count on a Z rising edge; the latched value is shown by `GET 1` as `IDX=`.

### `USE_CURRENT_SENSE`

| | `1` (default) | `0` (`*_nocs` envs) |
|---|---|---|
| IS ADC/DMA | initialised; zero offset calibrated at boot | never initialised |
| Current limit | integrating duty ceiling above `CURRENT_LIMIT_A` | none |
| OVERCURRENT fault | above `CURRENT_TRIP_A` for `CURRENT_TRIP_MS` | none |
| DRIVER fault | IS at ADC full scale (BTN8962 fault signal) | none |
| STALL fault | current ≥ `STALL_CURRENT_A` with no encoder movement for `STALL_TIMEOUT_MS` | \|duty\| ≥ `STALL_DUTY_MIN` with no encoder movement for `STALL_TIMEOUT_MS` |
| Duty cap / ramp | `Mx_DUTY_MAX` 0.95, 20 /s | `Mx_DUTY_MAX` 0.60, 2 /s |
| Current in telemetry | mA | `NA` (never 0) |

Current-sense constants in `config.h`:

- `IS_R_SENSE_OHMS` (3300)
- `IS_KILIS` (19500, the BTN8962 typical value)
- `ADC_VREF_V`
- `IS_AMPS_PER_COUNT`, derived from the above (about 4.76 mA per count, about 19.5 A full scale)
- `CURRENT_LIMIT_A` (5 A) and `CURRENT_TRIP_A` (8 A)

## Runtime settings

These values can be changed while the board runs:

- the PID gains,
- the duty cap and ramp,
- the current limit and trip level,
- the stall thresholds,
- the watchdog and telemetry periods,
- the motor and encoder direction flags,
- the CAN node IDs, bit rate and message rates,
- the encoder resolution, gear ratio and encoder location (see below).

**Encoder and gearbox.** Per motor: `ENCx_CPR` is counts per turn of the shaft the encoder is on (4 × the datasheet PPR or lines), `Mx_GEAR` is motor turns per output turn (30 for 30:1), and `ENCx_ON_OUTPUT` is 1 when the encoder sits on the output shaft, so the gear ratio doesn't apply. Together they give counts per output turn: `ENCx_CPR × Mx_GEAR` with the encoder on the motor, `ENCx_CPR` on the output. Control, the UART protocol, `Mx_VEL_MAX`, `Mx_ACCEL`, `Mx_STALL_COUNTS`, the travel and homing settings and the PID gains stay in counts, so changing these never retunes anything. They set the CAN "turns", and the GUI shows and takes output degrees and RPM once counts per output turn is no longer 1.

Their defaults are the `config.h` values marked `[runtime]`. Change them with `CFG` on UART, the CAN vendor commands, or the GUI's Configuration tab. `SAVE` writes them to the last 1 KB flash page (0x0800FC00) and they load at boot; a blank or corrupt page falls back to the defaults. Settings saved by older firmware still load, and settings added since then start at their defaults.

- Saving and restoring defaults are refused while a motor is enabled, because erasing flash stalls the CPU.
- The direction flags can only be changed with both motors OFF.
- The parameter IDs are fixed in `src/control/params.h` and mirrored in `tools/gui/params.py`. Only append new ones.

## Control

Each motor runs a cascade at 1 kHz:

- **POS:** target counts → position PID → velocity set-point (clamped to ±`Mx_VEL_MAX`) → velocity PID → duty
- **VEL:** target counts/s → velocity PID → duty
- **DUTY:** target in [-1, 1] → duty

With `Mx_ACCEL` > 0 the velocity set-point is slewed at that rate (counts/s²) in VEL and POS, and a POS move is also capped at √(2 × ACCEL × distance left), so it decelerates into the target (a trapezoidal move).

The duty is then limited, in this order:

1. `Mx_DUTY_MAX`
2. The current limiter (when `USE_CURRENT_SENSE=1`)
3. The `Mx_DUTY_RAMP` slew window
4. A direction block: a pressed limit switch in block mode, or a soft travel limit, stops motion toward that end at once (no ramp) and still allows the other direction

The duty, speed, current and stall limits are separate settings for each motor (`M1_…`, `M2_…`). `I_TRIP_MS` and `STALL_MS` are shared.

The PIDs use derivative on measurement and conditional integration, so the integrator stops while the output is pinned by any of these limits.

Drive is sign-magnitude. The forward side PWMs while the other side's low switch stays on, so the off-time is a brake (slow decay). Positions are in encoder counts (x4), and velocities in counts/s.

### Faults

**Latched faults:** OVERCURRENT, DRIVER, STALL, LIMIT, and ESTOP (from the CAN Estop command). Each one acts per motor and immediately:
- pulls that motor's INH low,
- zeroes its PWM,
- sets it to OFF,
- lights the ERROR LED,
- prints `EVT FAULT M<n> <names>`.

`MODE` is then refused (`ERR FAULT_LATCHED`) until you send `CLR <n>`.

**Limit switches:** each of LIMIT1/LIMIT2 has four settings:
- `LIMx_MOTOR`: the motor it belongs to (1 or 2, 0 = unused).
- `LIMx_END`: which end it guards, 0 = negative direction, 1 = positive.
- `LIMx_NC`: 1 for a normally-closed switch (a broken wire then reads as pressed).
- `LIMx_FAULT`: 1 (default) latches a LIMIT fault on the press, whichever way the motor was going; 0 only blocks motion toward `LIMx_END`, with no fault, so the motor can drive straight back off it. Set `LIMx_END` correctly before choosing 0.

In fault mode the fault is edge-triggered, so after `CLR` the motor can be driven back off the switch. `CLR` answers `OK LIMIT_STILL_PRESSED` if the switch is still pressed. A switch that is pressed at power-up latches a fault.

**Homing:** `HOME <m>` runs the motor in VEL mode at `Mx_HOME_VEL` toward its switch (the negative-end one if it has two). When the switch is pressed, the position becomes `Mx_HOME_OFFSET`, and the motor moves `Mx_HOME_BACKOFF` counts back off the switch in POS mode and holds there. `GET <m>` shows `HOME=RUNNING`, `DONE` or `NO`. Any `MODE`, `SET` or fault cancels it; `ERR NO_SWITCH` means no switch is assigned to that motor. Homing ignores the soft limits, and the homing switch doesn't fault while homing.

**Soft travel limits:** with `Mx_SOFT_LIM` = 1, POS targets are clamped to `Mx_POS_MIN`…`Mx_POS_MAX`, and every mode is blocked from driving past them. They are in counts from wherever position 0 is, so home or `ZERO` first.

**Command watchdog:** if no valid command arrives on UART, and no frame arrives for either CAN node, for `WDG_MS` (default 500 ms) while a motor is enabled, both motors go to OFF and `EVT WATCHDOG STOP` is printed. This is not latched; send `MODE` again to resume. Set `WDG_MS` to 0 to disable it. The GUI keeps it fed with `PING` while connected.

**IWDG (~100 ms):** refreshed only while both the main loop and the control loop are running. A HardFault drops both INH lines before halting.

## UART3 protocol (115200 8N1)

One command per line. Commands are case-insensitive, motors are `1` or `2`, and every command answers with one `OK …` or `ERR …` line.

| Command | Effect |
|---|---|
| `MODE <m> OFF\|DUTY\|VEL\|POS` | Set mode. Entering POS holds the current position. |
| `SET <m> <x>` | Target: duty -1..1, counts/s, or counts, depending on mode |
| `PID <m> VEL\|POS [kp ki kd]` | Set (or with no gains, read) gains |
| `GET <m>` | `OK M1 MODE= POS= VEL= CUR=(A or NA) DUTY= TGT= FAULT= LIM=` |
| `GET PPM` | Pulse widths in µs (0 = no signal) |
| `GET STAT` | Uptime, control ticks, overruns, UART drops, CAN ESR |
| `CLR <m>\|ALL` | Clear latched faults |
| `ZERO <m>` | Set position to 0 |
| `HOME <m>` | Home onto the motor's limit switch (`ERR NO_SWITCH` if it has none) |
| `ISCAL` | Re-measure the current-sense zero offset (motors must be OFF; enables both bridges at 0 % duty for ~40 ms) |
| `STOP` | Both motors OFF |
| `PING` | Keep the watchdog alive |
| `TELEM <ms>` | Telemetry period (0 = off, default 50) |
| `CFG` | List all settings: `C <id> <name> <value> <min> <max> <default> <flags>` lines, then `OK CFG <n>` |
| `CFG <name\|id> [value]` | Read or set one setting (`ERR RANGE`, `ERR BUSY` if it needs motors OFF) |
| `SAVE` | Write settings to flash (motors must be OFF) |
| `DEFAULTS` | Restore firmware defaults (not saved until `SAVE`) |
| `VER`, `HELP` | Version, build options, parameter count / help |

**Telemetry line:**

```
T,<ms>,<pos1>,<vel1>,<cur1_mA|NA>,<duty1_permille>,<mode1 O/D/V/P>,<faults1 hex>,<pos2>,...,<faults2>,<ppm1_us>,<ppm2_us>,<limit bits>,<home bits>
```

Limit bits: bit 0 motor 1, bit 1 motor 2. Home bits: bit 0/1 homing motor 1/2, bit 2/3 homed motor 1/2.

**Fault bits:** 0x01 OC, 0x02 STALL, 0x04 LIMIT, 0x08 DRIVER, 0x10 ESTOP.

## CAN (ODrive 0.5.x compatible)

The board uses ODrive's "CAN Simple" protocol from firmware 0.5.1 to 0.5.6, so it can share a bus with ODrives and be driven by the same host code. Each motor is one node: the 11-bit ID is `node_id << 5 | cmd`, and payloads are little-endian. The hardware filter only passes this board's two node IDs, so the board never sees or reacts to frames for an ODrive.

**On a bus with ODrives:**
- **Node IDs.** Every node ID must be unique on the bus. This board uses `M1_NODE_ID` = 40 and `M2_NODE_ID` = 41 by default (ODrive's own docs use 0, 1, 3, 0x10 and 0x18 in examples, so 40/41 stay clear of them). Both axes of an ODrive default to node 0, so give each ODrive axis its own ID: `odrv0.axis1.config.can.node_id = 1` on 0.5.6, or `odrv0.axis1.config.can_node_id = 1` on 0.5.1, then save and reboot. Avoid nodes 56–63 if a CANopen device is on the bus, because CANopen heartbeats (0x700 + id) land there.
- **Bit rate.** Everything on the bus must run at one bit rate. ODrive defaults to 250 kbit/s and this board to 125 kbit/s. Either set `CAN_BAUD` = 250 here (save, then reset), or set the ODrive to 125 kbit/s: `odrv0.can.config.baud_rate = 125000` on 0.5.6, or `odrv0.can.set_baud_rate(125000)` on 0.5.1. 500 and 1000 are also accepted, but see the clock note below.
- **Bus load.** At 125 kbit/s, the defaults (heartbeat 100 ms, estimates 20 ms, vendor status 100 ms) use about 15 % of the bus for this board, and a two-axis ODrive with its 0.5.6 defaults uses about 22 %.

**Units.** One "turn" is one output-shaft turn: `ENCx_CPR × Mx_GEAR` counts, or `ENCx_CPR` with `ENCx_ON_OUTPUT` = 1. The defaults make turns equal to counts. "Torque" in voltage mode is duty, from -1 to 1.

**Commands** (both ODrive versions unless noted). A "get" is answered on the same ID when it arrives as an RTR frame, or as a data frame with DLC 0 (as in 0.5.6).

| Cmd | Name | What this board does |
|---|---|---|
| 0x01 | Heartbeat (tx every `CAN_HB_MS`, default 100 ms) | 0.5.6 layout: u32 axis error, u8 state (1 IDLE, 8 CLOSED_LOOP), u8 motor error flag, u8 0, u8 traj_done << 7. 0.5.1 layout when `CAN_ODRV_VER` = 51: u32 axis error, u32 state. |
| 0x02 | Estop | Latches ESTOP on that motor (axis error ESTOP_REQUESTED) until Clear errors. |
| 0x03 | Get motor error | OVERCURRENT → CURRENT_LIMIT_VIOLATION, DRIVER → DRV_FAULT. u64 (0.5.6) or u32 (0.5.1). |
| 0x04, 0x05, 0x1D | Get encoder, sensorless, controller error | Always 0. |
| 0x06 | Set node ID | Takes effect at once. Use vendor SAVE to keep it. |
| 0x07 | Set axis state | 1 IDLE = motor OFF. 8 CLOSED_LOOP_CONTROL = motor on in the selected control mode. Anything else (calibration, homing) sets INVALID_STATE. |
| 0x09 | Encoder estimates (tx every `CAN_ENC_MS`, default 20 ms) | f32 position (turns), f32 velocity (turns/s). |
| 0x0A | Get encoder count | i32 count, i32 count modulo CPR. |
| 0x0B | Set controller mode | Control mode 0 voltage → DUTY, 2 velocity → VEL, 3 position → POS (the default, as on ODrive). Torque control (1) is ignored, because there is no current loop. The input mode is ignored and always works as passthrough. |
| 0x0C, 0x0D, 0x0E | Set input pos, vel, torque | Applied when the motor runs in the matching mode. Feed-forward terms are ignored. |
| 0x0F | Set limits | Velocity limit → `Mx_VEL_MAX`. The 0.5.6 current limit → `Mx_I_LIMIT`. Both apply to the addressed motor only. |
| 0x14 | Get Iq | Setpoint is NaN. Measured is the motor current, or NaN without current sense. |
| 0x15, 0x17, 0x1C | Get sensorless estimates, bus voltage, ADC voltage | NaN. The board doesn't measure these. |
| 0x16 | Reboot | Motors off, then MCU reset. |
| 0x18 | Clear errors | Clears that motor's faults, INVALID_STATE and WATCHDOG_TIMER_EXPIRED. |
| 0x1A | Set pos gain | `Mx_POS_KP`, same units. |
| 0x1B | Set vel gains | `Mx_VEL_KP` / `Mx_VEL_KI` = ODrive gain / CPR. |
| others | NMT, startup config, anticogging, trajectory, linear count | Ignored. |

**Axis error bits:** latched faults are reported as ODrive axis errors. OVERCURRENT and DRIVER map to MOTOR_FAILED (0x40). STALL maps to CONTROLLER_FAILED (0x200). LIMIT maps to MIN_ENDSTOP_PRESSED (0x1000). ESTOP maps to ESTOP_REQUESTED (0x4000). A watchdog stop maps to WATCHDOG_TIMER_EXPIRED (0x800), and a refused state request to INVALID_STATE (0x1).

**Vendor extension** on 0x1E (requests) and 0x1F (replies and status). ODrive 0.5.x doesn't use these two commands. The GUI uses them for settings and native-unit control.

| 0x1E byte 0 | Request | Reply on 0x1F |
|---|---|---|
| 01 | SET_PARAM: id, f32 value | `01`, id, status, 0, f32 value now |
| 02 | GET_PARAM: id | `02`, id, status, 0, f32 value |
| 03 / 04 | SAVE / DEFAULTS (motors must be OFF) | `03`/`04`, 0, status |
| 05 | INFO | `05`, fw major, minor, patch, ENC1_CONFIG, USE_CURRENT_SENSE, param count, motor 1/2 |
| 06 | MODE: 0 OFF, 1 DUTY, 2 VEL, 3 POS | none |
| 07 | TARGET: f32 in native units (duty, counts/s or counts) | none |
| 08 / 09 | ZERO this motor / STOP both motors | none |
| 0A | HOME this motor | none (progress in the status frame) |

The status byte is 0 OK, 1 bad id, 2 out of range, 3 busy (a motor is enabled), 4 flash error. Every `CAN_TELEM_MS` (100 ms), each node also sends a 0x1F status frame: `80`, mode, fault bits, limit/home byte (bit 0 limit pressed, bit 1 homing, bit 2 homed), i16 current mA (`INT16_MIN` = unavailable), i16 duty ‰.

`tools/board_sim/test_can_odrive.py` drives the simulated board with plain ODrive frames while a simulated ODrive shares the bus (see Simulated board).

**Clock note.** Bit timing is 16 time quanta per bit (1 + 11 + 4) at any bit rate, with the prescaler set to 2000 / kbit/s. That gives a 75 % sample point and SJW 4. A lower bit rate does not widen the clock tolerance; the timing segments do. This setting allows about 0.98 % mismatch between nodes. The HSI is only specified to about ±1 % at 25 °C, and ODrives use a crystal, so test the link across your temperature range. 500 and 1000 kbit/s are the most likely to have trouble.

## Bring-up checklist

**1. Before power**

- [ ] Confirm the board matches the config you built (see `ENC1_CONFIG`).
- [ ] Confirm the fitted drivers are BTN8962. A BTN7971 needs a different `IS_KILIS`.
- [ ] Use the GUI (its keep-alive feeds the watchdog), or send `CFG WDG_MS 0` if you will type commands by hand. Otherwise the watchdog stops the motors 0.5 s after each command.

**2. Power, no motors connected**

- [ ] Bench supply at 12 V with a current limit around 100–200 mA.
- [ ] Flash over SWD.
- [ ] On UART3, expect: `BOOT FW=… RESET=POWER`, then `IS offset max N counts` (measured with the bridges enabled at 0 % duty; an enabled BTN8962 adds an offset current, so a few hundred counts is normal), then `CAN OK` (or `CAN FAILED TO START` with no transceiver or bus), then `READY`.
- [ ] STATUS blinks slowly and ERROR is off.
- [ ] `VER`, then `GET STAT`: TICKS increases about 1000/s and OVR stays 0.
- [ ] Scope PA0–PA3: about 20 kHz carrier, 0 % duty. E1/E2 (PC13/PC14) are low.
- [ ] Press each limit switch: `EVT FAULT M1 LIMIT` / `M2`, and ERROR turns on. `CLR ALL` turns it off.
- [ ] Turn each encoder by hand: `GET 1` / `GET 2` POS changes.
- [ ] With an RC receiver connected, `GET PPM` shows about 1000–2000 µs.

**3. One motor at low duty** (supply limit about 1 A)

- [ ] Connect motor 1 to OUT1/OUT2.
- [ ] `MODE 1 DUTY`, then `SET 1 0.1`. E1 goes high and P1 shows about 10 % duty; the motor turns slowly.
- [ ] Check that POS increases for positive duty. If not, set `ENC1_INVERT` (wrong count direction) or `MOTOR1_INVERT` (wrong rotation).
- [ ] `SET 1 -0.1` reverses it and moves the PWM to P2. `MODE 1 OFF` drops E1.
- [ ] Repeat for motor 2 (OUT3/OUT4).

**4. Check IS readings**

- [ ] Run the motor at a steady duty. `GET 1` CUR should roughly match the motor current. Measure it with a clamp meter, or estimate it as supply current ÷ duty.
- [ ] If it reads consistently off, correct `IS_KILIS`.
- [ ] Load the shaft by hand. Current rises, then is held near `CURRENT_LIMIT_A`. Hold it stalled and expect `EVT FAULT … STALL` after 0.5 s.
- [ ] At very low duty, IS readings drop off (see the IS settling-time note below).

**5. Closed loop**

- [ ] `MODE 1 VEL`, small `SET` values. Tune with `PID 1 VEL kp ki kd` (start P-only).
- [ ] `MODE 1 POS`, then tune `PID 1 POS`.
- [ ] Raise the supply current limit only after this works.

**6. Protections**

- [ ] Stop sending commands while a motor runs: `EVT WATCHDOG STOP`.
- [ ] Trip a limit switch while moving.
- [ ] Optionally lower `CURRENT_TRIP_A` temporarily to see an OVERCURRENT trip.

## Assumptions

- **Driver chip.** The fitted drivers are BTN8962TA (all four parts on the schematic are BTN8962TAAUMA1; only the title block says btn7971). `IS_KILIS = 19500` is the typical datasheet ratio and should be calibrated in step 4.
- **Limit switches.** By default LIMIT1 faults motor 1 and LIMIT2 faults motor 2. Each switch closes to GND, or opens with `LIMx_NC` = 1.
- **Encoders.** These are quadrature encoders read at x4. All native units are counts. `ENCx_CPR`, `Mx_GEAR` and `ENCx_ON_OUTPUT` only scale the ODrive "turns" on CAN and the GUI's degrees/RPM.
- **PPM.** PPM inputs are measured and reported only (`GET PPM`, telemetry). Nothing is controlled from them yet.
- **CAN.** The protocol follows ODrive CAN Simple 0.5.x. Default node IDs are 40 and 41, the default bit rate is 125 kbit/s, and the vendor commands use 0x1E/0x1F.
- **Watchdog behaviour.** A watchdog timeout stops the motors without latching a fault. A limit switch pressed at power-up latches a fault.
- **Drive scheme.** Drive is sign-magnitude with slow decay. I did not add a coast option, because INH is static only.
- **Starting gains.** Default PID gains are placeholders. `tools/board_sim` shows that the loops are stable on a generic motor model, not on your motor.

## Not verified from the schematic or on hardware

- **IS overvoltage.** The IS pins feed PA4/PA5/PB0/PB1, which are not 5 V tolerant, through 3.3 kΩ with no series resistor or clamp. In a fault the BTN8962 sources its IS saturation current, which can pull the pin above 3.3 V. The firmware treats full scale as a DRIVER fault, but adding a series resistor (about 1–4.7 kΩ) is advisable.
- **IS settling time.** The current sense needs time to settle after each switch-on. Samples are taken mid-pulse, but at low duty with a 20 kHz carrier the reading will be low. Lower `PWM_FREQ_HZ` if this matters.
- **SR pins.** The SR pins have 3.3 kΩ ∥ 100 nF to GND (R12/C3 etc.). That is unusual, and I couldn't confirm the slew-rate effect of the capacitor.
- **IN thresholds.** I assumed the BTN8962 IN thresholds work with 3.3 V logic through the 1 kΩ series resistors.
- **Not on hardware yet.** The firmware has not run on hardware or in an emulator. All 4 environments, plus the `ENC1_INDEX_ENABLE` variant, compile and link with no warnings using arm-none-eabi-gcc 13.2 and STM32CubeF1 HAL v1.1.10, at about 35 KB flash and 5.7 KB RAM. `pio run` itself was not run, because the PlatformIO registry was unreachable from the build environment. PlatformIO's bundled `framework-stm32cubef1` may be a slightly different HAL version, but it has the same API.

## GUI

```
cd tools/gui
pip install -r requirements.txt     # pyserial, python-can
python motor_gui.py
```

On Windows you can instead double-click `Motor GUI.pyw` (installs the two packages on first run, no console window).

This needs Python 3.9+ with Tkinter. It runs on Windows, macOS and Linux.

**Connection bar**

- **UART:** pick the USB-UART adapter's port at 115200. Wire adapter TX to UART_RX (U4-2), adapter RX to UART_TX (U4-3), and GND to U4-1.
- **CAN:** pick the python-can interface for your adapter. Bitrate is 125000 (or what `CAN_BAUD` is set to), and the node IDs are 40/41. ODrives on the same bus are left alone; the status bar names other nodes it sees.
  - `slcan`: CANable/CANtact-style adapters with slcan firmware, and most cheap USB-CAN dongles. The channel is the serial port.
  - `gs_usb`: candleLight firmware.
  - `pcan`: PEAK, channel `PCAN_USBBUS1`.
  - `socketcan`: Linux, channel `can0`.
  - `kvaser`, `ixxat`, `vector` and others: any python-can backend works. Vendor backends need the vendor's driver.

  The bus needs 120 Ω termination at both ends; R51 is the board's.
- **Keep-alive:** sends `PING` every 200 ms so the command watchdog doesn't stop the motors. If the GUI freezes or the cable comes out, the motors stop.
- **STOP ALL / Esc:** sends STOP at any time.

**Control tab:** live position, velocity, current and duty for each motor, plus faults, the limit switch and homing state. It has mode buttons, a target entry and slider, Zero, Clear fault and Home, and a 10 s plot of either motor's velocity, position, current or duty.

**Configuration tab:** every runtime setting, with the board's value, range, default and a description.
- Double-click a row to edit it, then press **Write changes** to apply.
- **Save to flash** keeps the settings after power-off.
- **Export / Import** saves and loads JSON setting files.
- Settings that don't apply to the build (for example current settings when current sense is off) are greyed out.

**Console tab:** raw traffic. You can send any UART command, or a CAN frame as `id#hexdata` (or `id#R` for an RTR request, e.g. `501#R` asks node 40 for a heartbeat).

## Simulated board

```
sh tools/board_sim/run.sh                        # current sense on
sh tools/board_sim/run.sh -DUSE_CURRENT_SENSE=0
sh tools/board_sim/run.sh --selftest             # scripted check, exits 0 on pass
sh tools/board_sim/run.sh --odrive               # adds a fake ODrive 0.5.6 axis at node 0
python3 tools/board_sim/test_can_odrive.py       # with --odrive running: ODrive coexistence test
```

This compiles the firmware's real `control/` and `comms/` code for a PC, with a DC motor model in place of the hardware. It opens two pseudo-terminals:

- `/tmp/mdsim-uart` speaks the UART protocol.
- `/tmp/mdsim-can` speaks SLCAN, so it looks like a USB-CAN adapter.

Point the GUI at either one to try it without hardware: UART port `/tmp/mdsim-uart`, or CAN with interface `slcan` and channel `/tmp/mdsim-can`. While it runs, type `limit 1` or `load 2` into its terminal to toggle a limit switch or jam a motor shaft. It runs on Linux and macOS only (it needs pseudo-terminals).

<!-- markdownlint-disable MD041 -->
```text
 ___ ___ ___ _______  __     ___         _           _ _
| _ \ _ \_  )__ / __|/  \   / __|___ _ _| |_ _ _ ___| | |___ _ _
|   /  _// / |_ \__ \ () | | (__/ _ \ ' \  _| '_/ _ \ | / -_) '_|
|_|_\_| /___|___/___/\__/   \___\___/_||_\__|_| \___/_|_\___|_|
```

# RP2350 Controller

Firmware for an RP2350-based controller with:

- Core 0 RTIC tasks for USB, encoder interrupts, PWM output, and inter-core messaging
- Core 1 processing for speed control and camera-angle-based steering
- Three HC-SR04 ultrasonic sensors measured in hardware
- Split SIO FIFO signalling with typed SPSC queues for inter-core communication
- Structured USB telemetry for controller, Hall, and ultrasound events

## Hardware Notes

- Target MCU: `RP2350` on a Raspberry Pi Pico 2-class board
- USB: device enumerates as a USB serial interface for command input
- Encoder input: `GPIO 18` rising-edge Hall sensor input
- PWM outputs: `GPIO 16` and `GPIO 17`
- Ultrasonic sensors: trigger on `GPIO 14/12/10`, echo on `GPIO 15/13/11`
- Status LED: `GPIO 25`
- Crystal frequency: `12 MHz`
- Configured system clock: `150 MHz`
- Core split: Core 0 handles I/O and scheduling, Core 1 runs estimation and control

The current firmware assumes the Hall sensor produces one timing event per magnet pass and converts pulse period into longitudinal speed before handing it to Core 1. Steering currently tracks the latest `CameraAlign` angle with a distance-stepped PID; the EKF plumbing remains in place but is not yet the trusted steering-angle source.

## Start Here

- [HOW_IT_WORKS.md](HOW_IT_WORKS.md): runtime walkthrough, RTIC task structure, dual-core architecture, IPC, and command interface
- [CONTROL_THEORY.md](CONTROL_THEORY.md): EKF, bicycle model, timeout behavior, and PID-style control explanation

## Source Layout

- `src/main.rs`: Core 0 application and hardware setup
- `src/ipc.rs`: typed inter-core messages and FIFO channel wrapper
- `src/controller_processor/controller_processor_loop.rs`: Core 1 event loop
- `src/controller_processor/kalman_filter.rs`: estimator math
- `src/controller_processor/controller.rs`: controller output logic
- `src/hc_sr04.rs`: async ultrasonic measurement driver using PWM input mode
- `src/logging.rs`: USB telemetry formatting, including `simple_csv`
- `src/usb_serial.rs`: USB CDC command parsing and control-mode handling

## Runtime Surfaces

| Surface | Default | Purpose |
| ------- | ------- | ------- |
| USB CDC serial | enabled | Host command input and structured telemetry output |
| RTT / defmt | enabled | Embedded logs over the SWD debug probe |
| Control mode | `manual` | Direct PWM commands are allowed until `mode auto` is selected |
| Telemetry format | sparse events | `simple_csv` feature switches to fixed-column CSV rows |

## Build & Run

The crate has a default target configured in `.cargo/config.toml`, so plain `cargo` commands already target `thumbv8m.main-none-eabihf`.

### Common commands

| Command | What it does |
| ------- | ------------ |
| `cargo build` | Build debug firmware for the RP2350 target |
| `cargo build --release` | Build optimized firmware |
| `cargo run` | Flash and run the debug build with `probe-rs` |
| `cargo run --release` | Flash and run the optimized build |
| `DEFMT_LOG=trace cargo run --release` | Run with maximum RTT log verbosity |
| `cargo run --release --features simple_csv` | Emit fixed-column USB telemetry for host parsers |

### Build

```sh
cargo build
```

### Run on RP2350

Flash and run the firmware with:

```sh
cargo run --release
```

By default, debug output goes to the RTT (Real-Time Transfer) probe connected via the SWD debug interface. You can view RTT logs with a tool like `probe-rs` or `JLinkExe`.

The firmware also streams telemetry over USB CDC serial. Control telemetry from Core 1 and ultrasound telemetry from Core 0 are formatted as timestamped event records. In normal mode the stream is sparse `>name:value` output, so fields that do not apply to a given event are omitted. With the `simple_csv` feature enabled, the stream uses a fixed column order and writes `null` for missing fields.

## USB Command Table

Commands are newline-terminated ASCII strings received over the USB CDC serial port.

| Command | Example | Effect |
| ------- | ------- | ------ |
| `pwm-a <microseconds>` | `pwm-a 1600` | Set PWM channel A directly |
| `pwm-b <microseconds>` | `pwm-b 1500` | Set PWM channel B directly |
| `speed <m/s>` | `speed 0.40` | Update the Core 1 speed setpoint |
| `const steering_kp <value>` | `const steering_kp 50` | Update the Core 1 steering PID gain |
| `mode manual` | `mode manual` | Keep direct host PWM commands active |
| `mode auto` | `mode auto` | Allow Core 0 to apply control outputs from Core 1 |

The `align <angle> <confidence>` command is also forwarded to Core 1 and updates the latest camera-derived steering angle used by the steering PID.

The default is `mode manual`, which prevents Core 0 from applying controller outputs received from Core 1 until auto mode is selected.

## Telemetry Streams

| Event family | Produced by | Notes |
| ------------ | ----------- | ----- |
| `controller` | Core 1 | Steer PWM, throttle PWM, setpoint, speed-controller error, and Kalman state |
| `hall_delta_t` | Core 0 | Hall timing events from the encoder path |
| `ultrasound` | Core 0 | Three HC-SR04 distance readings |

These events are what the roof-control hub parses and writes into host-side CSV logs.

### Logging Levels

Control the verbosity of embedded logs via the `DEFMT_LOG` environment variable:

#### Maximum detail (trace level)

```sh
DEFMT_LOG=trace cargo run --release
```

Shows every function entry, loop iteration, and state update. Useful for deep debugging but generates a lot of output.

#### Warnings and errors only (warn level)

```sh
DEFMT_LOG=warn cargo run --release
```

Filters to only critical issues and status warnings. Less spammy for production-like testing.

#### Other levels

- `DEFMT_LOG=debug`: typical debugging output
- `DEFMT_LOG=info`: high-level runtime events (default)
- `DEFMT_LOG=error`: errors only

The `defmt` logs still go over RTT, but the structured telemetry stream is sent over USB serial.

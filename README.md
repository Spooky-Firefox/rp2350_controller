```text
 ___ ___ ___ _______  __     ___         _           _ _
| _ \ _ \_  )__ / __|/  \   / __|___ _ _| |_ _ _ ___| | |___ _ _
|   /  _// / |_ \__ \ () | | (__/ _ \ ' \  _| '_/ _ \ | / -_) '_|
|_|_\_| /___|___/___/\__/   \___\___/_||_\__|_| \___/_|_\___|_|
```

# RP2350 Controller

Firmware for an RP2350-based controller with:

- Core 0 RTIC tasks for USB, encoder interrupts, PWM output, and inter-core messaging
- Core 1 processing for state estimation and speed control
- A typed FIFO IPC layer built on `rp235x-hal`

## Hardware Notes

- Target MCU: `RP2350` on a Raspberry Pi Pico 2-class board
- USB: device enumerates as a USB serial interface for command input
- Encoder input: `GPIO 13` rising-edge Hall sensor input
- PWM outputs: `GPIO 16` and `GPIO 17`
- Status LED: `GPIO 25`
- Crystal frequency: `12 MHz`
- Configured system clock: `150 MHz`
- Core split: Core 0 handles I/O and scheduling, Core 1 runs estimation and control

The current firmware assumes the Hall sensor produces one timing event per magnet pass and converts pulse period into longitudinal speed before handing it to the estimator.

## Start Here

- [HOW_IT_WORKS.md](HOW_IT_WORKS.md): runtime walkthrough, RTIC task structure, dual-core architecture, IPC, and command interface
- [CONTROL_THEORY.md](CONTROL_THEORY.md): EKF, bicycle model, timeout behavior, and PID-style control explanation

## Source Layout

- `src/main.rs`: Core 0 application and hardware setup
- `src/ipc.rs`: typed inter-core messages and FIFO channel wrapper
- `src/controller_processor/controller_processor_loop.rs`: Core 1 event loop
- `src/controller_processor/kalman_filter.rs`: estimator math
- `src/controller_processor/controller.rs`: controller output logic

## Build & Run

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

### Logging Levels

Control the verbosity of embedded logs via the `DEFMT_LOG` environment variable:

#### Maximum detail (trace level):
```sh
DEFMT_LOG=trace cargo run --release
```

Shows every function entry, loop iteration, and state update. Useful for deep debugging but generates a lot of output.

#### Warnings and errors only (warn level):
```sh
DEFMT_LOG=warn cargo run --release
```

Filters to only critical issues and status warnings. Less spammy for production-like testing.

#### Other levels:
- `DEFMT_LOG=debug`: typical debugging output
- `DEFMT_LOG=info`: high-level runtime events (default)
- `DEFMT_LOG=error`: errors only

The logging is done via the `defmt` crate and RTT, not USB serial, so it does not interfere with the USB command interface.

# How the RP2350 Controller Works

## What is this program?

This program runs on an **RP2350 microcontroller** (the chip on a Raspberry Pi Pico 2 board). Unlike a regular computer, a microcontroller has no operating system — our program *is* the only thing running. It starts at power-on and runs forever.

The program does four things:

1. **Controls two servo/motor outputs** via PWM signals (GPIO 16 and 17)
2. **Measures rotation speed** from a Hall-effect encoder (GPIO 13)
3. **Runs a Kalman-filtered speed controller** on a dedicated second CPU core
4. **Receives commands** from a PC over USB serial

---

## Program Structure — RTIC

The program uses a framework called **RTIC** (Real-Time Interrupt-driven Concurrency). Instead of one big loop doing everything in sequence, the program is split into **tasks** that run in response to events (called *interrupts*). Think of it like Excel macros that fire automatically when something changes.

Each task has a **priority level** — higher priority tasks can interrupt lower priority ones if they need to run urgently.

```text
Priority 5 (highest) │  usb_interrupt    — USB data arrived
Priority 4           │  sio_interrupt    — control output from Core 1
Priority 3           │  gpio_interrupt   — encoder edge detected
Priority 2           │  sensor_timeout   — no encoder pulse for 100 ms
Priority 1           │  toggle_led       — periodic heartbeat
                     │  delay_update_setpoint — defer setpoint updates
                     │  log_data         — send logging data to host
                     │  periodic_drain_log_data — trigger log buffer drain
                     │  ultrasound_scan  — poll 3 HC-SR04 sensors in sequence
Priority 0 (lowest)  │  idle             — runs when nothing else does
```

### Tasks

| Task | Trigger | What it does |
| ------ | ------- | ------------ |
| `init` | Power-on, once | Sets up all hardware, spawns Core 1 |
| `idle` | Always (background) | Sleeps (WFE) when nothing is scheduled |
| `toggle_led` | Every 1 second | Blinks the onboard LED — visual "heartbeat" |
| `gpio_interrupt` | Rising edge on GPIO 13 | Measures encoder period, sends `SensorEvent` to Core 1 |
| `sensor_timeout` | Every 100 ms (if no encoder pulse) | Sends a timeout event to Core 1 so the filter can coast |
| `delay_update_setpoint` | Periodic delay (50 ms) | Defers speed setpoint updates to avoid excessive Core 1 wakeups |
| `sio_interrupt` | Core 1 writes to FIFO | Receives `ControlEvent` and applies PWM outputs |
| `usb_interrupt` | USB data received | Reads commands and adjusts speed setpoint or PWM |
| `log_data` | Event from `periodic_drain_log_data` | Drains buffered telemetry data and sends to USB in serial plotter format |
| `periodic_drain_log_data` | Every 50 ms | Signals `log_data` to drain the logging buffer and transmit |
| `ultrasound_scan` | Continuous (60 ms between sensors) | Fires HC-SR04 trigger, awaits echo interrupt, RTT-logs µs + cm |

---

## Dual-Core Architecture

The RP2350 has **two ARM Cortex-M33 cores**. This program uses both:

```text
 ┌──────────────────────────┐         ┌──────────────────────────┐
 │         Core 0           │  FIFO   │         Core 1           │
 │                          │ ──────> │                          │
 │  RTIC tasks:             │ Sensor  │  Blocking event loop:    │
 │  - encoder ISR           │ Events  │  - Kalman filter         │
 │  - USB serial            │         │  - PID speed controller  │
 │  - PWM output            │ <────── │                          │
 │  - sensor timeout        │ Control │                          │
 │                          │ Events  │                          │
 └──────────────────────────┘         └──────────────────────────┘
```

**Core 0** handles all I/O — interrupts, USB, PWM. **Core 1** runs the computationally heavier control loop (EKF + PID). They communicate through the SIO hardware FIFO.

### Inter-Core Communication (IPC)

The two cores talk through a **32-bit hardware FIFO** built into the RP2350's SIO peripheral. Each core has its own TX and RX side — data written by one core appears on the other's read side.

| Direction | Message | Size | Content |
| --------- | ------- | ---- | ------- |
| Core 0 → Core 1 | `SensorEvent` | 6 words | timestamp, speed setpoint, steering, encoder period |
| Core 1 → Core 0 | `ControlEvent` | 1 word | steering PWM + power PWM packed into 16 bits each |

The `FifoChannel` struct owns the `SioFifo` from the HAL, so all FIFO access is safe — no raw pointer manipulation. See [CONTROL_THEORY.md](CONTROL_THEORY.md) for details on what Core 1 does with these events.

---

## PWM — Controlling Servos/Motors

**PWM (Pulse-Width Modulation)** is a way to control motors/servos using a digital pin that rapidly switches on and off. The pin produces a repeating pulse:

```text
  |‾‾‾‾|    |‾‾‾‾‾‾‾‾|    |‾|
  |    |    |        |    | |
──┘    └────┘        └────┘ └──

        ↑ "on-time" controls position/speed
```

- The pulse repeats every **20 ms** (50 Hz — the standard for RC servos)
- The **on-time** (width) controls the servo position or motor speed
- Default on-time: **1500 µs** (center position)
- Range is typically 1000–2000 µs

There are two independent channels: **PWM-A** (GPIO 16) and **PWM-B** (GPIO 17).

---

## HC-SR04 — Ultrasonic Distance Sensor

The HC-SR04 measures distance using ultrasound.  It has two signal pins:

| Pin | Direction | Description |
| --- | --------- | ----------- |
| TRIG | MCU → sensor | Hold high for ≥ 10 µs to fire one measurement |
| ECHO | sensor → MCU | Goes high when the burst is sent; falls when the reflection arrives |

Distance in centimetres ≈ echo pulse width in µs ÷ 58.

### Wiring

| Sensor | TRIG pin | ECHO pin | PWM slice | Notes |
| ------ | -------- | -------- | --------- | ----- |
| 0 | GPIO 14 | GPIO 15 | Pwm7 B | |
| 1 | GPIO 12 | GPIO 13 | Pwm6 B | |
| 2 | GPIO 10 | GPIO 11 | Pwm5 B | |

The TRIG pins are plain digital outputs (SioOutput).  The ECHO pins are connected to the B channel of their respective PWM slices — the B channel is the hardware gate input in `InputHighRunning` mode.

### How the counter works

Instead of using a software timer to measure the echo pulse width, the driver connects the ECHO pin to the **B channel** of a dedicated PWM slice configured in **`InputHighRunning`** mode.  In this mode the hardware counter increments on every system-clock edge **only while pin B is high**.  When the echo falls, the counter value equals the pulse width in clock cycles — no interrupt latency error, no polling.

```text
             trigger
MCU TRIG: ──┐     ┌────────────────── (next measurement)
            └─────┘  ← 20 µs busy-wait

ECHO pin: ─────────────┐              ┌────────────────────
                       └──────────────┘  ← echo pulse

PWM ctr:  0 0 0 0 0 0 0 1 2 3 4 5 … N N N N N N N N N N 0
                         ↑ counting              ↑ read & reset
```

### 🚀 A Word on Over-Engineering

**This PWM approach is completely ridiculous.**

Why? The HC-SR04's own hardware measurement error is ±3 mm — roughly **30 µs of echo pulse width variance** depending on temperature, humidity, and reflectivity of the object. Yet we've dedicated **three PWM slices** to measure with **1 µs resolution** and zero software jitter.

A far simpler approach would be:
```rust
let t_start = MainMono::now();
// (echo fires interrupt)
let t_end = MainMono::now();
let pulse_us = (t_end - t_start).to_micros();
```

This would measure the same thing with **~1 µs ISR-entry jitter** — totally invisible given the sensor's ±3 mm error budget. The "complex" version adds nothing but code complexity and burns hardware resources.

**Why did we do it this way?** Mostly to demonstrate:

- RTIC patterns (split ownership, atomic waker, priority ceiling management)
- Advanced PWM features (`InputHighRunning` counting mode)  
- How to avoid shared resource locks with lock-free atomics

In a real product, you'd use the simple `MainMono` approach. Here, consider this a **pedagogical over-engineered masterpiece** — proof that just because you *can* measure something to ±1 µs doesn't mean you *should*.

**But wait — it gets better.** At the default 150 MHz system clock, each counter tick represents one clock period:

$$\frac{c}{150\,\text{MHz}} = \frac{3 \times 10^8\,\text{m/s}}{150 \times 10^6\,\text{Hz}} \approx 1.9986\,\text{m}$$

That's right: **this counter has a time resolution that corresponds to ~2 metres of light travel distance per tick.** If you somehow connected it to a photon detector, the HC-SR04 PWM measuring infrastructure would be accurate enough to... resolve distances to the nearest two metres. Using light. A sensor that costs €0.40 and has a ±3 mm acoustic error budget.

Overclock the RP2350 to 400 MHz (which it handles) and the resolution improves to ~0.75 m per tick — still completely useless for light, but an impressive spec sheet entry. In practice, parasitic capacitance in PCB traces and pin drivers would make any real photon-timing application impossible, but that is entirely beside the point.

**THE OVER-ENGINEERED SOLUTION IS A MASTERPIECE.**

### `ultrasound_scan` task

The `ultrasound_scan` async task (priority 1) polls all three sensors in a round-robin loop with 60 ms between each measurement start:

```text
t = 0 ms   measure sensor 0  →  delay_until t+60 ms
t = 60 ms  measure sensor 1  →  delay_until t+120 ms
t = 120 ms measure sensor 2  →  delay_until t+180 ms
t = 180 ms measure sensor 0  (next cycle)
```

`delay_until` is used instead of `delay` so the spacing is referenced to an absolute clock — if a measurement takes varying time (e.g. an echo arrives late), the inter-sensor gap stays deterministic.

Each measurement result is logged via RTT (`defmt::info!`) showing both the raw µs value and an approximate centimetre figure.  Timeout (> 50 ms, no echo) is logged as a warning.

### Ownership split

The driver is split into two objects with the same lifetime so neither needs to be an RTIC shared resource (which would raise the priority ceiling):

| Type | Stored in | Role |
| ---- | --------- | ---- |
| `HcSr04Shared` | `init` local (static) | Holds the `AtomicWaker` and done flag |
| `HcSr04Measure` | Measuring async task local | Drives trigger, resets counter, awaits result |
| `HcSr04OnInterrupt` | `gpio_interrupt` task local | Called from ISR; clears interrupt and wakes the future |

```text
  init locals
  ┌──────────────────────────────────┐
  │ HcSr04Shared { waker, done }     │
  │   &'static ──────────────────┐   │
  │   &'static ──────────┐       │   │
  └──────────────────────│───────│───┘
                         │       │
               HcSr04OnInterrupt │     ← gpio_interrupt task
                  .on_interrupt()│
                    sets done    │
                    waker.wake() │
                                 │
                        HcSr04Measure  ← async measuring task
                           .measure().await
                             poll_fn registers waker
                             reads counter on wake
```

### Interrupt flow

1. `HcSr04Measure::measure()` resets `done = false`, sends trigger pulse, resets counter.
2. The future returned by `measure()` registers the task's `Waker` via `AtomicWaker::register()`.
3. When the echo falls, `IO_IRQ_BANK0` fires → `gpio_interrupt` task → `on_interrupt()` is called.
4. `on_interrupt()` checks `echo_pin.interrupt_status(EdgeLow)`, clears it, sets `done = true`, calls `AtomicWaker::wake_by_ref()`.
5. The RTIC executor re-polls `measure()`.  `done` is now `true` → `Poll::Ready` → counter value is returned.
6. If no echo arrives within the caller-supplied timeout, `rtic_time::Monotonic::timeout_after` returns `HcSr04Error::Timeout`.

### Why no RTIC shared resource?

The waker handoff uses an [`atomic_waker::AtomicWaker`](https://docs.rs/atomic-waker) and an `AtomicBool`, both of which are lock-free.  The interrupt handler (`gpio_interrupt`, priority 3) writes to these atomics with `Release` ordering; the async task reads with `Acquire` ordering.  This is sufficient for correct synchronisation on Cortex-M33 without raising any RTIC priority ceiling.

---

## Encoder — Measuring Speed

A Hall-effect encoder detects magnets on a rotating shaft. Each time a magnet passes the sensor, the pin on GPIO 13 goes from low to high (a **rising edge**), triggering `gpio_interrupt`.

The interrupt records the **time between two consecutive rising edges**. Speed is then:

$$v = \frac{d}{t}$$

Where $d$ is the distance one magnet-spacing represents (`13 × π / 300` meters, based on the wheel geometry) and $t$ is the measured time between pulses.

---

## USB Serial — Sending Commands

The microcontroller appears as a **virtual serial port** on the PC. You can use any serial terminal (e.g. PuTTY, screen, tio, minicom) at any baud rate.

### Commands

| Command | Example | Effect |
| ------- | ------- | ------ |
| `pwm-a <microseconds>` | `pwm-a 1700` | Set PWM-A on-time to 1700 µs |
| `speed <m/s>` | `speed 1.2` | Set the speed controller target to 1.2 m/s |
| `pwm-b <microseconds>` | `pwm-b 1200` | Set PWM-B on-time to 1200 µs |

Commands must end with a newline (`\n`). PWM values are capped at the 20 000 µs frame period, and the controller itself later clamps output to the usual servo range of roughly 1000-2000 µs.

`pwm-a` and `pwm-b` directly override the current PWM compare value on Core 0. `speed` updates a shared setpoint that is attached to every `SensorEvent` sent to Core 1.

---

## Shared Data and Why Locking Matters

Some data is **shared** between RTIC tasks. In the current code, examples include:

- `speed_setpoint_mps`: written by `usb_interrupt`, read by `gpio_interrupt` and `sensor_timeout`
- `last_sensor_irq_us`: written by `gpio_interrupt`, read by `sensor_timeout`
- `fifo`: used by both the sensor-producing tasks and the SIO receive task
- `pwm`: updated by both USB commands and control messages from Core 1

Because a higher-priority task can interrupt a lower-priority one mid-read or mid-write, shared state must be protected.

> **Race condition (brief):** If task A is halfway through writing a value and task B reads it, B gets garbage. This is a race condition — the result depends on timing, which is unpredictable.

RTIC prevents this by requiring a **lock** whenever shared data is accessed. While a task holds the lock, no other task can interrupt and touch the same resource. You'll see this in code as:

```rust
ctx.shared.speed_setpoint_mps.lock(|setpoint| *setpoint = new_value);
```

The compiler *enforces* that you lock before accessing — you cannot forget, unlike in most other languages.

For cross-core traffic, the shared RTIC resource is a typed `FifoChannel`, not a raw register block. Core 0 locks it before sending or receiving FIFO messages; Core 1 owns its own `FifoChannel` instance in its blocking loop.

---

## Real-Time Logging — Serial Plotter Integration

The program continuously collects telemetry data and streams it to the host PC in **VS Code Serial Plotter format**. This allows real-time visualization of system state without special software.

### Logging Pipeline

```text
Core 1 (control loop)           Core 0 (I/O)
    │                               │
    └─→ [log_data_producer] ───────→ Channel
                                    │
                                ┌───┴───┐
                                │       ├─→ [log_data_consumer ring buffer]
                                │       │
                                └───┬───┘
                                    │
                 [periodic_drain_log_data] (every 50 ms)
                                    │
                                [log_data task]
                                    │
                              [USB Serial]
                                    │
                              [PC / VS Code]
```

### Format

Each log line is sent in **VS Code Serial Plotter format**, prefixed with `>` and containing comma-separated `name:value` pairs:

```
>time_us:12345678, steer_ms:1500, throttle_ms:1500, speed_mps:0.5234, setpoint_mps:0.5000, error:0.0234, kalman0:0.5200, kalman1:0.0012, kalman2:0.0001, kalman3:-0.0003
```

This data is automatically plotted by VS Code's Serial Plotter extension when it detects a running terminal on the serial port.

### Key Implementation Details

- **Core 1 produces data:** The control loop writes telemetry snapshots to a ring buffer via `log_data_producer`
- **Core 0 consumes data:** The `log_data` task drains the buffer and formats it for USB transmission
- **Throttled transmission:** Data is sent every 50 ms (20 Hz) by `periodic_drain_log_data`, preventing USB from being overwhelmed
- **Async tasks:** Both `log_data` and `periodic_drain_log_data` are async tasks to avoid blocking higher-priority interrupts

---

## Code Organization

The firmware is organized into several Rust modules:

| Module | Purpose |
| ------ | ------- |
| `main.rs` | RTIC task definitions, hardware initialization, interrupt handlers |
| `lib.rs` | Top-level definitions and module exports |
| `constants.rs` | Global configuration constants (clock speeds, PWM settings, encoder calibration) |
| `hc_sr04.rs` | HC-SR04 ultrasonic distance sensor driver (split ownership, async measurement) |
| `logging.rs` | Telemetry logging and VS Code Serial Plotter formatting |
| `utils.rs` | Utility functions (PWM tick conversion, time conversions) |
| `ipc.rs` | Inter-process communication structures (`SensorEvent`, `ControlEvent`, `FifoChannel`) |
| `usb_serial.rs` | USB device initialization and serial port handling |
| `controller_processor/` | Core 1 control loop (Kalman filter, PID controller) |
| `controller_processor/kalman_filter.rs` | Extended Kalman Filter implementation |
| `controller_processor/controller.rs` | PID speed controller logic |

---

## Rust Syntax Cheat-Sheet

You don't need to write Rust, but being able to *read* it helps. Here are the constructs you'll encounter in this codebase.

### Variables

```rust
let x = 5;           // immutable — like a MATLAB constant, cannot be reassigned
let mut y = 5;       // mutable   — like a normal Python variable
y = y + 1;           // OK because y is mut
```

Rust requires you to explicitly opt-in to mutability with `mut`. This prevents accidental modification — a common bug source.

### Types

Rust types are written after a colon. You'll often see type aliases at the top of `main.rs` that give long hardware types a short name:

```rust
type LedPin = gpio::Pin<gpio::bank0::Gpio25, gpio::FunctionSioOutput, gpio::PullDown>;
```

This just means "call this complicated type `LedPin` from now on". It's like a shorthand.

In embedded programming, these detailed types are very important because they encode hardware constraints at compile time. Instead of trusting comments, the type system ensures you cannot accidentally use a pin in the wrong mode.

Short breakdown of the long type:

- `gpio::Pin<...>`: a GPIO pin object
- `gpio::bank0::Gpio25`: the physical pin number (GPIO 25)
- `gpio::FunctionSioOutput`: pin function is digital output mode
- `gpio::PullDown`: internal pull-down resistor configuration

So the type says: "this is exactly GPIO25, configured as a digital output, with pull-down." If code expects an output pin, passing an input pin type will fail at compile time.

Integer sizes are explicit: `u32` = unsigned 32-bit integer, `i32` = signed 32-bit, `f32` = 32-bit float (like `single` in MATLAB), `usize` = pointer-sized unsigned integer (used for array indices).

Why explicit sizes matter on microcontrollers:

- Memory is small, so `u8`/`u16`/`u32` choices affect RAM use
- Registers are fixed-width hardware fields, so matching bit-width avoids bugs
- Timing/math code should use predictable types so behavior is deterministic

### Functions

```rust
fn add(a: f32, b: f32) -> f32 {
    a + b          // no semicolon = this is the return value
}
```

In Python this would be `def add(a, b): return a + b`. The last expression in a block without a `;` is automatically returned.

### Closures (anonymous functions / lambdas)

Closures are the `|variable| expression` syntax you'll see throughout the code:

```rust
ctx.shared.speed_setpoint_mps.lock(|setpoint| *setpoint = new_value);
```

The `|setpoint|` part is like Python's `lambda setpoint:` — it defines an anonymous function that receives `setpoint` as an argument. This particular line means: *"acquire the lock, then run this small function with the protected value"*.

Another example with a longer body:

```rust
(usb_dev, serial).lock(|usb_dev, serial| {
    // code that uses usb_dev and serial safely
});
```

Compare to Python: `lock(lambda usb_dev, serial: ...)` — same idea, different spelling.

### `Option` — values that might not exist

Rust has no `None` confusion like Python. Instead of any variable silently being `None`, you must explicitly handle the possibility:

```rust
// Option<u32> means "either Some(number) or None"
fn parse_on_time_us(arg: &str) -> Option<u32> { ... }

// Using it — must handle both cases:
if let Some(on_time_us) = parse_on_time_us(rest) {
    // on_time_us is a real u32 here
} else {
    // nothing was parsed
}
```

`.unwrap()` means "I'm certain this is `Some`, give me the value — crash if I'm wrong". Used where failure is truly impossible.

### `Result` — operations that can fail

```rust
let clocks = init_clocks_and_plls(...).unwrap();
```

`Result<T, E>` is either `Ok(value)` or `Err(error)` — like a function that either succeeds or returns an error. `.unwrap()` takes the success value or panics. In production code you'd handle the error; here `.unwrap()` is acceptable because a clock initialization failure means the hardware is broken.

### References and `*` (dereferencing)

The `&` prefix means "a reference to" (a pointer, not a copy). The `*` prefix means "follow the reference to get the actual value":

```rust
ctx.shared.speed_setpoint_mps.lock(|setpoint| *setpoint = new_value);
//                                               ^ write through the reference
```

Think of `setpoint` as a pointer in C, or a MATLAB `handle` object — `*setpoint` reaches the value it points to.

### Macros — the `!` suffix

Any call ending in `!` is a macro (code that generates code at compile time), not a regular function:

```rust
info!("encoder speed : {} m/s", value);  // like Python's print(f"encoder speed: {value} m/s")
unwrap!();
```

`info!` and `trace!` are from the `defmt` library — they send formatted strings over the debug probe (not USB serial).

### Imports and `use`

Rust uses `use` to bring names into scope, similar to Python imports.

```rust
use rp235x_hal as hal;
use rp2350_controller::usb_serial::{MyUsbBus, init_usb_serial};
```

- `use rp235x_hal as hal;`: import the crate with a short alias (`hal`)
- `use ...::{A, B};`: import multiple items from one module

Why this helps in embedded code:

- Keeps long hardware paths readable
- Makes intent clearer (`hal::gpio::Pins` is easier to scan)
- Reduces typo risk when repeating deep module paths

### Attributes (`#[...]`)

Attributes are metadata attached to items (functions, modules, constants). They tell the compiler or frameworks how to treat that item.

```rust
#[task(binds = IO_IRQ_BANK0, priority = 3)]
fn gpio_interrupt(...) { ... }
```

In this project, `#[task(...)]` is an RTIC attribute. It registers the function as a real-time task and configures things like:

- which interrupt triggers it (`binds = ...`)
- priority level (`priority = ...`)
- local/shared resources the task is allowed to access

Without this attribute, the function is just a normal Rust function and RTIC will not schedule it.

```rust
#![deny(unsafe_code)]
```

This crate defaults to rejecting `unsafe` code. Small, explicitly marked exceptions exist only where the hardware or boot process requires them, such as startup glue and Core 1's SIO ownership handoff.

Why this is useful:

- keeps most of the code strictly safe
- marks exactly where special low-level operations happen
- makes code review easier because the risky areas are explicit

### `const` — compile-time constants

```rust
const SYS_CLOCK_HZ: u32 = 150_000_000;  // like MATLAB's global constants
```

Underscores in numbers are just visual separators (like a thousands comma): `125_000_000` = 125000000.

### `loop` and `-> !`

```rust
async fn toggle_led(...) -> ! {
    loop {
        // runs forever
    }
}
```

`-> !` means "this function never returns" (the `!` type is called "never"). All tasks run in infinite loops — on a microcontroller, returning from main would be meaningless.

---

## File Overview

| File | Purpose |
| ---- | ------- |
| `src/main.rs` | Core 0 RTIC app: hardware setup, USB commands, PWM output, encoder interrupts |
| `src/ipc.rs` | Typed inter-core messages (`SensorEvent`, `ControlEvent`) and `FifoChannel` |
| `src/controller_processor/controller_processor_loop.rs` | Core 1 event loop: receives sensor events, runs EKF + controller, sends PWM commands back |
| `src/controller_processor/kalman_filter.rs` | Event-driven extended Kalman filter for dead reckoning and speed estimation |
| `src/controller_processor/controller.rs` | Straight-line speed controller (PID-like) producing servo PWM commands |
| `src/usb_serial.rs` | USB serial port initialization (boilerplate, rarely needs changing) |
| `src/entry.rs` | Low-level chip setup (spinlocks, FPU) — do not touch |
| `src/lib.rs` | Declares the modules above |
| `CONTROL_THEORY.md` | Explanation of the estimator and controller math |
| `memory.x` | Tells the linker where Flash and RAM are on the chip |
| `Cargo.toml` | Rust's equivalent of a package list (`requirements.txt` / `pip install`) |
| `build.rs` | Runs at compile time; passes `memory.x` to the linker |

---

## Where To Read Next

- If you want the runtime walkthrough, stay in this file.
- If you want the math and controls background, read `CONTROL_THEORY.md`.
- If you want the exact message format between cores, read `src/ipc.rs`.

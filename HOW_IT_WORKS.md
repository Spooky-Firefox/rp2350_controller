# How the RP2350 Controller Works

## What is this program?

This program runs on an **RP2350 microcontroller** (the chip on a Raspberry Pi Pico 2 board). Unlike a regular computer, a microcontroller has no operating system — our program *is* the only thing running. It starts at power-on and runs forever.

The program does three things:

1. **Controls two servo/motor outputs** via PWM signals (GPIO 14 and 15)
2. **Measures rotation speed** from a Hall-effect encoder (GPIO 13)
3. **Receives commands** from a PC over USB serial

---

## Program Structure — RTIC

The program uses a framework called **RTIC** (Real-Time Interrupt-driven Concurrency). Instead of one big loop doing everything in sequence, the program is split into **tasks** that run in response to events (called *interrupts*). Think of it like Excel macros that fire automatically when something changes.

Each task has a **priority level** — higher priority tasks can interrupt lower priority ones if they need to run urgently.

```text
Priority 3 (highest) │  gpio_interrupt   — encoder edge detected
Priority 2           │  usb_interrupt    — USB data arrived
Priority 1           │  toggle_led       — periodic heartbeat
Priority 0 (lowest)  │  idle             — runs when nothing else does
```

### Tasks

| Task | Trigger | What it does |
|------|---------|--------------|
| `init` | Power-on, once | Sets up all hardware |
| `idle` | Always (background) | Prints current speed to debug log |
| `toggle_led` | Every 1 second | Blinks the onboard LED — visual "heartbeat" |
| `gpio_interrupt` | Rising edge on GPIO 13 | Records the time between encoder pulses |
| `usb_interrupt` | USB data received | Reads commands and adjusts PWM output |

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

There are two independent channels: **PWM-A** (GPIO 14) and **PWM-B** (GPIO 15).

---

## Encoder — Measuring Speed

A Hall-effect encoder detects magnets on a rotating shaft. Each time a magnet passes the sensor, the pin on GPIO 13 goes from low to high (a **rising edge**), triggering `gpio_interrupt`.

The interrupt records the **time between two consecutive rising edges**. Speed is then:

$$v = \frac{d}{t}$$

Where $d$ is the distance one magnet-spacing represents (`13 × π / 300` meters, based on the wheel geometry) and $t$ is the measured time between pulses.

---

## USB Serial — Sending Commands

The microcontroller appears as a **virtual serial port** on the PC. You can use any serial terminal (e.g. PuTTY, screen, minicom) at any baud rate.

### Commands

| Command | Example | Effect |
|---------|---------|--------|
| `pwm-a <microseconds>` | `pwm-a 1700` | Set PWM-A on-time to 1700 µs |
| `pwm-b <microseconds>` | `pwm-b 1200` | Set PWM-B on-time to 1200 µs |

Commands must end with a newline (`\n`). The maximum on-time is capped at 20 000 µs (the full period).

---

## Shared Data and Why Locking Matters

Some data is **shared** between tasks — for example, `encoder_time_diff` is written by `gpio_interrupt` and read by `idle`. Because a higher-priority task can interrupt a lower-priority one *mid-read*, reading a half-updated value is possible without protection.

> **Race condition (brief):** If task A is halfway through writing a value and task B reads it, B gets garbage. This is a race condition — the result depends on timing, which is unpredictable.

RTIC prevents this by requiring a **lock** whenever shared data is accessed. While a task holds the lock, no other task can interrupt and touch the same data. You'll see this in code as:

```rust
ctx.shared.encoder_time_diff.lock(|diff| *diff = time_diff);
```

The compiler *enforces* that you lock before accessing — you cannot forget, unlike in most other languages.

---

## File Overview

| File | Purpose |
|------|---------|
| `src/main.rs` | All application logic — tasks, hardware setup, command parsing |
| `src/usb_serial.rs` | USB serial port initialisation (boilerplate, rarely needs changing) |
| `src/entry.rs` | Low-level chip setup (spinlocks, FPU) — do not touch |
| `src/lib.rs` | Declares the modules above |
| `memory.x` | Tells the linker where Flash and RAM are on the chip |
| `Cargo.toml` | Rust's equivalent of a package list (`requirements.txt` / `pip install`) |
| `build.rs` | Runs at compile time; passes `memory.x` to the linker |

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
ctx.shared.encoder_time_diff.lock(|diff| *diff = time_diff);
```

The `|diff|` part is like Python's `lambda diff:` — it defines an anonymous function that receives `diff` as an argument. This particular line means: *"acquire the lock, then run this small function with the protected value"*.

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

`Result<T, E>` is either `Ok(value)` or `Err(error)` — like a function that either succeeds or returns an error. `.unwrap()` takes the success value or panics. In production code you'd handle the error; here `.unwrap()` is acceptable because a clock initialisation failure means the hardware is broken.

### References and `*` (dereferencing)

The `&` prefix means "a reference to" (a pointer, not a copy). The `*` prefix means "follow the reference to get the actual value":

```rust
ctx.shared.encoder_time_diff.lock(|diff| *diff = time_diff);
//                                         ^ write through the reference
```

Think of `diff` as a pointer in C, or a MATLAB `handle` object — `*diff` reaches the value it points to.

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
#[allow(unsafe_code)]
```

This attribute relaxes a lint rule for the next item (or module/crate scope, depending where it is written). It means: "unsafe code is allowed here." In this codebase, it is used for a few low-level hardware setup parts where `unsafe` is unavoidable.

Why this is useful:

- keeps most of the code strictly safe
- marks exactly where special low-level operations happen
- makes code review easier because the risky areas are explicit

### `const` — compile-time constants

```rust
const SYS_CLOCK_HZ: u32 = 125_000_000;  // like MATLAB's global constants
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
|------|---------|
| `src/main.rs` | All application logic — tasks, hardware setup, command parsing |
| `src/usb_serial.rs` | USB serial port initialisation (boilerplate, rarely needs changing) |
| `src/entry.rs` | Low-level chip setup (spinlocks, FPU) — do not touch |
| `src/lib.rs` | Declares the modules above |
| `memory.x` | Tells the linker where Flash and RAM are on the chip |
| `Cargo.toml` | Rust's equivalent of a package list (`requirements.txt` / `pip install`) |
| `build.rs` | Runs at compile time; passes `memory.x` to the linker |

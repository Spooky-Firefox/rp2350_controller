//! HC-SR04 ultrasonic distance sensor driver.
//!
//! # Sensor overview
//!
//! The HC-SR04 has two signals:
//!  - **Trigger** (output from MCU): held high for ≥10 µs to start a measurement.
//!  - **Echo** (input to MCU): goes high when the ultrasonic burst is sent, falls
//!    low when the reflection is received back.
//!
//! Echo pulse width in microseconds ÷ 58 ≈ distance in centimetres.
//!
//! # Design
//!
//! The module follows the same split-ownership pattern as `heapless::spsc::Queue`:
//!
//! 1. Create an [`HcSr04Shared`] in RTIC `init` locals (gives a `'static` lifetime).
//! 2. Call [`HcSr04Shared::split`] to receive an [`HcSr04Measure`] and an
//!    [`HcSr04OnInterrupt`].
//! 3. Give [`HcSr04Measure`] to the async task that drives distance measurements.
//! 4. Give [`HcSr04OnInterrupt`] to the `gpio_interrupt` task (bound to `IO_IRQ_BANK0`).
//!
//! Internally, the echo pin is connected to the **B channel** of the supplied PWM
//! slice, which is reconfigured to [`InputHighRunning`] mode.  In this mode the
//! hardware counter advances only while the B-channel pin is high, so the counter
//! value at the falling edge equals the echo pulse width in clock cycles.
//!
//! Waker passing between the async measurement future and the interrupt handler is
//! done through an [`AtomicWaker`] and an [`AtomicBool`] — no RTIC shared resource
//! is involved, so no priority ceiling is raised.

use atomic_waker::AtomicWaker;
use core::future::poll_fn;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::Poll;
use embedded_hal::digital::OutputPin;
use rp235x_hal as hal;
use hal::gpio::{self, AnyPin, FunctionPwm, PinId, PullType};
use hal::pwm::{self, FreeRunning, InputHighRunning, SliceId, ValidPwmInputPin};
use rtic_monotonics::Monotonic;

/// Trigger pulse duration in CPU cycles.
///
/// At 150 MHz: 3 000 cycles ≈ 20 µs (HC-SR04 requires ≥ 10 µs).
const TRIGGER_CYCLES: u32 = 3_000;

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

/// Error returned by [`HcSr04Measure::measure`].
#[derive(Debug, defmt::Format)]
pub enum HcSr04Error {
    /// The echo pulse did not arrive within the requested timeout.
    Timeout,
}

// ---------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------

/// Shared state between [`HcSr04Measure`] and [`HcSr04OnInterrupt`].
///
/// Must be stored in RTIC `init` locals so both halves receive a `'static`
/// reference via [`HcSr04Shared::split`].
///
/// ```rust,ignore
/// #[init(local = [hc_sr04_shared: HcSr04Shared = HcSr04Shared::new()])]
/// fn init(ctx: init::Context) -> (Shared, Local) {
///     let (measure, on_interrupt) = ctx.local.hc_sr04_shared.split(
///         trig_pin, echo_pin, pwm_slice,
///     );
///     // ...
/// }
/// ```
pub struct HcSr04Shared {
    waker: AtomicWaker,
    /// Set to `true` by [`HcSr04OnInterrupt::on_interrupt`] when the echo
    /// falls; cleared to `false` at the start of each [`HcSr04Measure::measure`] call.
    done: AtomicBool,
}

impl HcSr04Shared {
    /// Create the shared state.
    ///
    /// This is a `const fn` so it can be used in `static` initialisers or
    /// RTIC `init` local declarations.
    pub const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
            done: AtomicBool::new(false),
        }
    }

    /// Consume the shared state and produce the two driver halves.
    ///
    /// # Arguments
    ///
    /// * `trig_pin` — digital output pin connected to the sensor's TRIG input.
    /// * `echo_pin` — any GPIO pin connected to the sensor's ECHO output.  The
    ///   pin is reconfigured as `FunctionPwm` and connected to the B channel of
    ///   `pwm_slice`.  An edge-low interrupt is enabled on it.
    /// * `pwm_slice` — a free-running PWM slice.  It is reconfigured to
    ///   [`InputHighRunning`] mode (counter advances while the B-channel pin is
    ///   high) and enabled.
    pub fn split<TrigPin, EchoPin, S>(
        &'static mut self,
        trig_pin: TrigPin,
        echo_pin: EchoPin,
        pwm_slice: pwm::Slice<S, FreeRunning>,
    ) -> (
        HcSr04Measure<'static, TrigPin, S>,
        HcSr04OnInterrupt<'static, EchoPin::Id, EchoPin::Pull>,
    )
    where
        TrigPin: OutputPin,
        EchoPin: AnyPin,
        EchoPin::Id: ValidPwmInputPin<S>,
        S: SliceId,
    {
        // Reconfigure the slice so its counter runs only while pin B is high.
        let mut pwm_input = pwm_slice.into_mode::<InputHighRunning>();

        // Connect the echo pin to the B channel as the gate input.
        // input_from() reconfigures the pin to FunctionPwm and returns it.
        let echo_pwm = pwm_input.channel_b.input_from(echo_pin);

        // Fire an interrupt when the echo pin falls (end of echo pulse).
        echo_pwm.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);

        // Enable the slice; it will now count clock cycles while echo is high.
        pwm_input.enable();

        (
            HcSr04Measure {
                trig_pin,
                pwm_slice: pwm_input,
                shared: self,
            },
            HcSr04OnInterrupt {
                echo_pin: echo_pwm,
                shared: self,
            },
        )
    }
}

// ---------------------------------------------------------------------------
// Measurement handle
// ---------------------------------------------------------------------------

/// Measurement handle — give this to the async task that triggers measurements.
///
/// Obtained from [`HcSr04Shared::split`].
pub struct HcSr04Measure<'a, TrigPin, S: SliceId> {
    trig_pin: TrigPin,
    pwm_slice: pwm::Slice<S, InputHighRunning>,
    shared: &'a HcSr04Shared,
}

impl<'a, TrigPin, S> HcSr04Measure<'a, TrigPin, S>
where
    TrigPin: OutputPin,
    S: SliceId,
{
    /// Perform one distance measurement asynchronously.
    ///
    /// 1. Clears the done flag.
    /// 2. Drives the trigger pin high for ~20 µs (busy-wait), then low.
    /// 3. Resets the PWM counter.
    /// 4. Suspends until [`HcSr04OnInterrupt::on_interrupt`] signals completion,
    ///    or until `timeout` elapses.
    ///
    /// # Returns
    ///
    /// `Ok(ticks)` — raw PWM counter value representing echo pulse width in
    /// clock cycles.  Convert to microseconds: `ticks / (sys_clk_hz / 1_000_000)`.
    ///
    /// `Err(HcSr04Error::Timeout)` — no echo arrived within `timeout`.
    pub async fn measure<M: Monotonic>(
        &mut self,
        timeout: M::Duration,
    ) -> Result<u16, HcSr04Error> {
        // Arm: clear any completion signal from a previous measurement.
        self.shared.done.store(false, Ordering::Release);

        // Send trigger pulse (≥ 10 µs required; ~20 µs used for margin).
        let _ = self.trig_pin.set_high();
        cortex_m::asm::delay(TRIGGER_CYCLES);
        let _ = self.trig_pin.set_low();

        // Reset the counter immediately after the trigger so we measure only
        // the current echo pulse.  The echo pin cannot have gone high yet
        // (the HC-SR04 first transmits 8 × 40 kHz cycles ≈ 200 µs).
        self.pwm_slice.set_counter(0);

        // Future that resolves when on_interrupt() sets the done flag.
        let shared = self.shared;
        let interrupt_fut = poll_fn(move |cx| {
            // Fast path: already done (interrupt fired before first poll).
            if shared.done.load(Ordering::Acquire) {
                return Poll::Ready(());
            }
            // Register waker so on_interrupt() can re-schedule this task.
            shared.waker.register(cx.waker());
            // Double-check after registering to close the race window between
            // the load above and register(): if the ISR fires in between, the
            // waker store in AtomicWaker is still visible here.
            if shared.done.load(Ordering::Acquire) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        });

        // Race the interrupt future against the timeout.
        M::timeout_after(timeout, interrupt_fut)
            .await
            .map_err(|_| HcSr04Error::Timeout)?;

        let count = self.pwm_slice.get_counter();
        self.pwm_slice.set_counter(0);
        Ok(count)
    }
}

// ---------------------------------------------------------------------------
// Interrupt handle
// ---------------------------------------------------------------------------

/// Interrupt handle — give this to the `gpio_interrupt` task.
///
/// Obtained from [`HcSr04Shared::split`].
pub struct HcSr04OnInterrupt<'a, I: PinId, P: PullType> {
    echo_pin: gpio::Pin<I, FunctionPwm, P>,
    shared: &'a HcSr04Shared,
}

impl<'a, I: PinId, P: PullType> HcSr04OnInterrupt<'a, I, P> {
    /// Call from the `IO_IRQ_BANK0` interrupt handler (the RTIC `gpio_interrupt` task).
    ///
    /// Checks whether the echo pin fired an edge-low interrupt.
    /// If so: clears the pending interrupt, sets the done flag, and wakes the
    /// measurement future so it can read the PWM counter.
    /// If not (interrupt originated from another pin): does nothing.
    pub fn on_interrupt(&mut self) {
        if self.echo_pin.interrupt_status(gpio::Interrupt::EdgeLow) {
            self.echo_pin.clear_interrupt(gpio::Interrupt::EdgeLow);
            self.shared.done.store(true, Ordering::Release);
            self.shared.waker.wake();
        }
    }
}

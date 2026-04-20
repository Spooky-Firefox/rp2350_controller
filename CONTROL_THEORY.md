# Control Theory Notes

This note explains what the control-side code is doing in plain engineering terms. It is about the logic in:

- `src/controller_processor/kalman_filter.rs`
- `src/controller_processor/controller.rs`
- `src/controller_processor/controller_processor_loop.rs`

## Big Picture

Core 1 does two jobs:

1. Estimate the vehicle state from incomplete sensor data.
2. Turn that estimate into actuator commands.

In this project, the state estimator is an **Extended Kalman Filter (EKF)** and the actuator command logic is a simple **speed controller** with proportional, integral, and derivative terms.

The data flow is:

```text
encoder pulse timing + setpoint
            |
            v
       SensorEvent
            |
            v
   time extension + parsing
            |
            v
   EKF predict/update cycle
            |
            v
 estimated speed v_hat
            |
            v
      PID-like controller
            |
            v
      PWM microseconds
```

## Why Use a Kalman Filter Here?

The encoder does not directly measure the full vehicle state. It gives only intermittent speed-related information: the time between Hall sensor edges.

That means the controller has problems if it uses raw measurements directly:

- measurements arrive only when a magnet passes the sensor
- long gaps mean there is temporarily no new measurement
- noise in pulse timing becomes noise in the inferred speed
- future extensions may want position and heading, not just speed

The EKF provides a consistent state estimate even when measurements are sparse. Between measurements, it predicts forward using a vehicle model. When a new speed sample arrives, it corrects that prediction.

## State Vector

The filter state is:

$$x = [X, Y, \theta, v]$$

where:

- $X, Y$ are planar position in meters
- $\theta$ is heading in radians
- $v$ is longitudinal speed in meters per second

Even though the current controller mainly uses $v$, the filter is structured as a small vehicle model rather than a one-variable smoother.

## Motion Model

The prediction step uses a **kinematic bicycle model**. This is a standard simplified car model where the front wheels are collapsed into one steerable wheel and the rear wheels into one fixed wheel.

The continuous-time equations are:

$$\dot X = v \cos(\theta)$$
$$\dot Y = v \sin(\theta)$$
$$\dot \theta = \frac{v}{L}\tan(\delta)$$
$$\dot v = a_{long}$$

where:

- $L$ is wheelbase
- $\delta$ is steering angle
- $a_{long}$ is longitudinal acceleration input

The implementation advances these equations over a small sample interval $dt$ to get the next predicted state.

## Why It Is an Extended Kalman Filter

A plain Kalman filter assumes everything is linear. This vehicle model is not linear because of terms like:

- $\cos(\theta)$
- $\sin(\theta)$
- $\tan(\delta)$
- the speed measurement model using $|v|$

So the code linearizes the nonlinear model around the current operating point using Jacobians. That is what makes it an **extended** Kalman filter.

## Predict Step

On every event, the filter first computes:

$$dt = t_{now} - t_{last}$$

and clamps it to a maximum configured value so that a long silence does not explode covariance.

Then it performs two related updates:

1. **State prediction** using the vehicle equations.
2. **Covariance prediction** using the linearized Jacobian $F$ and process noise $Q$.

Conceptually:

$$x^-_{k+1} = f(x_k, u_k, dt)$$
$$P^-_{k+1} = F P_k F^T + Q$$

Here:

- $x$ is the state estimate
- $P$ is the covariance matrix
- $u = [\delta, a_{long}]$ is the control input
- $Q$ is process noise, representing model uncertainty

The process noise terms in `EkfConst` let you say how much you trust the model between measurements.

## Measurement Step

When a Hall sensor period arrives, Core 1 converts that period to linear speed:

$$v_{meas} = \frac{d_{pulse}}{t_{pulse}}$$

where $d_{pulse}$ is the arc length per encoder pulse.

The EKF then updates only the speed-related part of the state. The measurement model is:

$$z = |v| + noise$$

The absolute value is used because the encoder period gives speed magnitude, not signed direction.

The code smooths this a bit near zero using:

$$|v| \approx \sqrt{v^2 + \epsilon^2}$$

This avoids a derivative singularity at $v = 0$.

Then the usual Kalman logic happens:

$$y = z - h(x)$$
$$S = H P H^T + R$$
$$K = P H^T S^{-1}$$
$$x \leftarrow x + Ky$$

where:

- $y$ is innovation or residual
- $H$ is the measurement Jacobian
- $R$ is measurement noise variance
- $K$ is Kalman gain

Intuition:

- if the model is uncertain, the filter trusts the measurement more
- if the measurement is noisy, the filter trusts the model more

## Timeout Handling

Not every event contains a fresh encoder speed sample. The timeout task on Core 0 sends a `SensorEvent` with no RPM period when no pulse has arrived for 100 ms.

That causes Core 1 to call `on_timeout()`, which performs only the predict step.

This matters because otherwise the filter would freeze time whenever the encoder is quiet. Instead, the estimate continues to evolve and the covariance grows to reflect reduced certainty.

## Timestamp Extension

The inter-core message packs only the lower 32 bits of the microsecond timestamp. That wraps roughly every 71.6 minutes. `TimeExtender` reconstructs a monotonic 64-bit timeline on Core 1 so the filter can still compute correct $dt$ values across wraparound.

## Controller

After the EKF updates, the controller uses the estimated speed `filt.speed()` rather than the raw measurement.

The error is:

$$e = v_{setpoint} - v_{estimated}$$

The control output is a standard PID-style form:

$$u = u_{neutral} + K_p e + K_i \int e\,dt + K_d \frac{de}{dt}$$

In the implementation:

- `kp`, `ki`, `kd` are gains
- `integral_error` stores the accumulated integral term
- `previous_error` supports the derivative term
- `integral_limit` clamps integral growth to reduce windup
- output is clamped to servo PWM bounds

The controller returns two values:

- steering PWM in microseconds
- power PWM in microseconds

Right now the steering command is effectively fixed and the speed loop mainly drives throttle/power PWM.

## Why Estimate First, Then Control?

This split is usually cleaner than feeding raw sensor values straight into the controller.

Benefits:

- controller sees a smoother, more consistent speed estimate
- control logic is decoupled from raw sensor timing details
- more sensors can be fused later without rewriting the controller
- state variables like heading and position become available for future path tracking

## Tuning Knobs

Important tuning values live in `EkfConst` and `StraightLineSpeedController`.

For the EKF:

- `q_pos`, `q_theta`, `q_v`: process noise terms
- `r_speed`: speed measurement noise
- `eps_v`: smoothing around zero speed
- `dt_max_us`: maximum trusted timestep

For the controller:

- `kp`: immediate response to error
- `ki`: removes steady-state bias
- `kd`: damps fast changes
- `integral_limit`: anti-windup clamp

Practical effect:

- too much `kp` can make throttle oscillate
- too much `ki` can cause slow overshoot and windup
- too much `kd` can amplify estimate noise
- too little `r_speed` makes the EKF chase noisy measurements
- too much `r_speed` makes the EKF sluggish

## Current Limitations

The current code is intentionally simple and has a few placeholders:

- steering is currently stubbed to `0.0` in the sensor events
- longitudinal acceleration input is set to `0.0`
- only speed is directly measured
- the model is dead reckoning only; there is no absolute position sensor

So this is better understood as a clean foundation for controls work, not a finished full-state autonomous vehicle stack.

## Reading the Code

If you want to inspect the implementation with this note open:

- start in `controller_processor_loop.rs` for the runtime flow
- move to `kalman_filter.rs` for the estimator math
- then read `controller.rs` for the PID output logic

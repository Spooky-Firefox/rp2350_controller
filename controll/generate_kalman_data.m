function sim_data = generate_kalman_data(output_file)
% Generate synthetic ground truth and sensor streams for the RC car EKF.
%
% The generated data is shared by both:
%   - kalman.m
%   - kalman_dead_reckoning.m
% It also plots a quick summary so the synthetic dataset is easy to tune.
%
% Usage:
%   sim_data = generate_kalman_data();
%   sim_data = generate_kalman_data('my_file.mat');

    if nargin < 1 || isempty(output_file)
        output_file = fullfile(fileparts(mfilename('fullpath')), 'kalman_sim_data.mat');
    end

    % --- Vehicle / sensor constants used by the simulated plant ---
    PLANT.L   = 0.265;    % wheelbase [m]
    PLANT.r_w = 0.0325;   % wheel radius [m]
    PLANT.W   = 0.200;    % track width [m]
    PLANT.wheel_circumference = 2 * pi * PLANT.r_w;  % full wheel circumference [m]
    PLANT.pulses_per_rotation = 3;                    % encoder pulses per full rotation
    PLANT.pulse_distance = PLANT.wheel_circumference / PLANT.pulses_per_rotation; % [m/pulse]

    % Steering actuator (2nd-order) and PWM mapping configuration.
    PLANT.steer.max_angle_deg = 28;          % +/- max steering angle [deg]
    PLANT.steer.max_rate_deg_s = 360;        % physical steering rate clamp [deg/s]
    PLANT.steer.wn = 18.0;                   % actuator natural frequency [rad/s]
    PLANT.steer.zeta = 0.9;                  % actuator damping ratio [-]

    PLANT.throttle.deadband_us = 25;         % +/- deadband around 1500 us
    PLANT.throttle.max_accel_fwd = 2.2;      % [m/s^2]
    PLANT.throttle.max_accel_rev = 1.8;      % [m/s^2]
    PLANT.throttle.drag_coeff = 0.35;        % simple linear drag [1/s]

    % Timing uncertainty per wheel pulse.
    % On real hardware this is your timer resolution + interrupt latency.
    SENSOR.r_wheel_dt = 0.001;   % timing noise std dev [s] (~1 ms)

    % Timing of the synthetic data sources.
    TIMING.dt_predict = 0.01;          % [s]
    TIMING.axle_dt_range = [];  % legacy field
    TIMING.T_end = 10;                 % [s]

    dt_sim = TIMING.dt_predict;
    t_sim  = 0:dt_sim:TIMING.T_end;
    N      = length(t_sim);

    % PWM command profiles (what your controller sends).
    pwm_throttle = 1500 * ones(size(t_sim));
    pwm_steer    = 1500 * ones(size(t_sim));

    pwm_throttle(t_sim >= 0.8 & t_sim < 3.0) = 1720;
    pwm_throttle(t_sim >= 3.0 & t_sim < 5.3) = 1825;
    pwm_throttle(t_sim >= 5.3 & t_sim < 6.7) = 1500;   % coast to stop
    pwm_throttle(t_sim >= 6.7 & t_sim < 8.5) = 1150;   % strong reverse
    pwm_throttle(t_sim >= 8.5) = 1500;

    pwm_steer(t_sim >= 1.5 & t_sim < 3.4) = 1830;
    pwm_steer(t_sim >= 3.4 & t_sim < 5.1) = 1200;
    pwm_steer(t_sim >= 5.1 & t_sim < 6.2) = 1700;
    pwm_steer(t_sim >= 6.2 & t_sim < 8.6) = 1500;
    pwm_steer(t_sim >= 8.6) = 1320;

    % True steering angle and acceleration are generated from PWM through
    % separate tuneable functions/dynamics.
    delta_true  = zeros(1, N);
    delta_dot   = 0;
    a_long_true = zeros(1, N);

    % Integrate the bicycle model to create the ground-truth trajectory.
    x_true = zeros(4, N);
    x_true(:,1) = [0; 0; 0; 0];
    for k = 1:N-1
        [delta_true(k), delta_dot] = steering_actuator_step( ...
            delta_true(k), delta_dot, pwm_steer(k), dt_sim, PLANT.steer);

        a_long_true(k) = throttle_pwm_to_accel(pwm_throttle(k), ...
            x_true(4, k), PLANT.throttle);

        x_true(:,k+1) = bicycle_step(x_true(:,k), ...
            delta_true(k), a_long_true(k), dt_sim, PLANT);

        delta_true(k+1) = delta_true(k);
    end
    a_long_true(N) = a_long_true(N-1);

    % Wheel-rotation odometry events: one sample per full wheel rotation.
    wheel_rot_count_per_step = zeros(1, N);
    wheel_rot_dir_per_step = zeros(1, N);
    accum_distance = 0;
    for k = 2:N
        ds = x_true(4, k-1) * dt_sim;
        accum_distance = accum_distance + abs(ds);
        while accum_distance >= PLANT.pulse_distance
            wheel_rot_count_per_step(k) = wheel_rot_count_per_step(k) + 1;
            wheel_rot_dir_per_step(k) = sign(x_true(4, k-1));
            accum_distance = accum_distance - PLANT.pulse_distance;
        end
    end
    wheel_rot_idx = find(wheel_rot_count_per_step > 0);
    wheel_rot_count = wheel_rot_count_per_step(wheel_rot_idx);
    wheel_rot_dir = wheel_rot_dir_per_step(wheel_rot_idx);
    wheel_rot_times = t_sim(wheel_rot_idx);

    wheel_rot_dt_true = nan(size(wheel_rot_times));  % true inter-pulse interval [s]
    wheel_rot_dt_meas = nan(size(wheel_rot_times));  % noisy measured interval [s]
    wheel_rot_ds_true = zeros(size(wheel_rot_times));  % signed true travel [m]
    for i = 1:length(wheel_rot_times)
        if i >= 2
            dt_rot = wheel_rot_times(i) - wheel_rot_times(i-1);
            wheel_rot_dt_true(i) = dt_rot;
            % Noise is on the timestamp — distance per pulse is exact.
            wheel_rot_dt_meas(i) = dt_rot + SENSOR.r_wheel_dt * randn();
            wheel_rot_dt_meas(i) = max(wheel_rot_dt_meas(i), 1e-4); % clamp positive
        end
        ds_rot = wheel_rot_dir(i) * wheel_rot_count(i) * PLANT.pulse_distance;
        wheel_rot_ds_true(i) = ds_rot;
    end

    % Legacy compatibility fields (if older scripts still expect them).
    axle_idx = wheel_rot_idx;
    axle_times = wheel_rot_times;
    omega_true = abs(x_true(4, axle_idx)) / max(PLANT.r_w, 1e-6);
    z_axle = omega_true;

    sim_data = struct( ...
        'plant', PLANT, ...
        'sensor', SENSOR, ...
        'timing', TIMING, ...
        'dt_sim', dt_sim, ...
        't_sim', t_sim, ...
        'pwm_throttle', pwm_throttle, ...
        'pwm_steer', pwm_steer, ...
        'delta_true', delta_true, ...
        'a_long_true', a_long_true, ...
        'x_true', x_true, ...
        'wheel_rot_idx', wheel_rot_idx, ...
        'wheel_rot_count', wheel_rot_count, ...
        'wheel_rot_times', wheel_rot_times, ...
        'wheel_rot_dt_true', wheel_rot_dt_true, ...
        'wheel_rot_dt_meas', wheel_rot_dt_meas, ...
        'wheel_rot_ds_true', wheel_rot_ds_true, ...
        'axle_times', axle_times, ...
        'axle_idx', axle_idx, ...
        'omega_true', omega_true, ...
        'z_axle', z_axle);

    save(output_file, 'sim_data');
    fprintf('Saved simulation data to %s\n', output_file);

    plot_generated_data(sim_data);
end

function plot_generated_data(sim_data)
    t_sim = sim_data.t_sim;
    x_true = sim_data.x_true;
    pwm_throttle = sim_data.pwm_throttle;
    pwm_steer = sim_data.pwm_steer;
    delta_true = sim_data.delta_true;
    rot_times = sim_data.wheel_rot_times;

    figure('Name', 'Generated Kalman Dataset', 'Position', [120 120 1300 850]);

    subplot(2,3,1); hold on; grid on;
    plot(x_true(1,:), x_true(2,:), 'k-', 'LineWidth', 1.5);
    xlabel('X [m]'); ylabel('Y [m]'); title('Ground Truth Trajectory');
    legend('True trajectory');
    axis equal;

    subplot(2,3,2); hold on; grid on;
    plot(t_sim, pwm_steer, 'b-', 'LineWidth', 1.2);
    xlabel('t [s]'); ylabel('PWM [us]'); title('Steering PWM Command');

    subplot(2,3,3); hold on; grid on;
    plot(t_sim, pwm_throttle, 'm-', 'LineWidth', 1.2);
    xlabel('t [s]'); ylabel('PWM [us]'); title('Throttle PWM Command');

    subplot(2,3,4); hold on; grid on;
    stem(rot_times(2:end), sim_data.wheel_rot_dt_meas(2:end)*1000, 'r.', 'MarkerSize', 6);
    plot(rot_times(2:end), sim_data.wheel_rot_dt_true(2:end)*1000, 'k-', 'LineWidth', 1.2);
    xlabel('t [s]'); ylabel('\Delta t [ms]'); title('Inter-pulse Interval (true vs measured)');
    legend('Measured \Delta t', 'True \Delta t');

    subplot(2,3,5); hold on; grid on;
    plot(t_sim, rad2deg(delta_true), 'k-', 'LineWidth', 1.2);
    xlabel('t [s]'); ylabel('\delta [deg]'); title('Steering Angle (Actuator Output)');

    subplot(2,3,6); hold on; grid on;
    stairs([0, rot_times], [0, diff([0, rot_times])], 'b-', 'LineWidth', 1.2);
    xlabel('t [s]'); ylabel('\Delta t [s]'); title('Wheel-Rotation Inter-arrival Time');

    sgtitle('Synthetic Dataset Summary for EKF Tuning');
end

function x_next = bicycle_step(x, delta, a_long, dt, C)
    X     = x(1);
    Y     = x(2);
    theta = x(3);
    v     = x(4);

    x_next = [
        X     + v * cos(theta) * dt
        Y     + v * sin(theta) * dt
        wrap_pi(theta + (v / C.L) * tan(delta) * dt)
        v     + a_long * dt
    ];
end

function [delta_next, delta_dot_next] = steering_actuator_step(delta, delta_dot, pwm_us, dt, steer_cfg)
    delta_target = steering_pwm_to_target_angle(pwm_us, steer_cfg.max_angle_deg);

    delta_ddot = steer_cfg.wn^2 * (delta_target - delta) ...
        - 2 * steer_cfg.zeta * steer_cfg.wn * delta_dot;

    delta_dot_next = delta_dot + delta_ddot * dt;
    max_rate = deg2rad(steer_cfg.max_rate_deg_s);
    delta_dot_next = clamp(delta_dot_next, -max_rate, max_rate);

    delta_next = delta + delta_dot_next * dt;
    max_delta = deg2rad(steer_cfg.max_angle_deg);
    delta_next = clamp(delta_next, -max_delta, max_delta);
end

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end

function a = wrap_pi(a)
    a = mod(a + pi, 2*pi) - pi;
end
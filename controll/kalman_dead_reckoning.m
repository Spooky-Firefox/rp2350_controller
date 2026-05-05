%% Extended Kalman Filter for RC Car — Dead Reckoning (No Camera)
% ============================================================
%
% This is the dead-reckoning variant: position and heading are estimated
% purely from the kinematic bicycle model + wheel-rotation odometry.
% There is NO external position/heading correction.
%
% STATE:   x = [X; Y; theta; v]
%   X, Y   : global position [m]
%   theta  : heading angle [rad] (0 = +X axis, CCW positive)
%   v      : longitudinal speed [m/s] (+ forward, - reverse)
%
% INPUTS (control, known at every predict step):
%   pwm_throttle : ESC PWM command [us], 1000..2000
%   pwm_steer    : steering PWM command [us], 1000..2000
%
% MEASUREMENTS (wheel-rotation events, variable rate):
%   ds_wheel : UNSIGNED distance increment per event [m]
%              Direction is inferred from the estimated velocity sign.
%
% WARNING: Without an external position/heading reference, drift in
% X, Y, and theta will grow unboundedly over time. This variant is suitable
% for short-duration runs or as a fallback when the camera is lost.
%
% ============================================================
clear; clc; close all;

%% =================== LOAD GENERATED DATA ===================
script_dir = fileparts(mfilename('fullpath'));
data_file = fullfile(script_dir, 'kalman_sim_data.mat');
if ~isfile(data_file)
    generate_kalman_data(data_file);
end
loaded = load(data_file, 'sim_data');
sim_data = loaded.sim_data;

%% =================== CONFIGURABLE CONSTANTS ===================

% --- Vehicle Geometry ---
CONST.L   = sim_data.plant.L;    % wheelbase [m]
CONST.r_w = sim_data.plant.r_w;  % wheel radius [m]
CONST.W   = sim_data.plant.W;    % track width [m] (info only)
CONST.wheel_circumference = sim_data.plant.wheel_circumference;
CONST.pulse_distance       = sim_data.plant.pulse_distance;

% --- Control-to-dynamics mapping ---
CONST.throttle = sim_data.plant.throttle;
CONST.steer    = sim_data.plant.steer;

% --- Wheel timing noise ---
CONST.r_wheel_dt = sim_data.sensor.r_wheel_dt;  % [s]

% --- Process Noise Std Dev (continuous-time-like; scaled by dt inside) ---
%   For dead reckoning you may want LOWER process noise to trust the
%   model more, since there are no position corrections to pull back drift.
CONST.q_pos   = 0.02;   % position  [m/s]
CONST.q_theta = 0.05;   % heading   [rad/s]
CONST.q_v     = 1.0;    % velocity  [m/s^2]

% --- Timing (for simulation only; real system uses actual dt) ---
CONST.dt_predict = sim_data.timing.dt_predict;       % EKF prediction period  [s]
CONST.axle_dt_range = sim_data.timing.axle_dt_range; % legacy field

%% =================== INITIAL STATE & COVARIANCE ===================
x_est = [0; 0; 0; 0];                          % initial state estimate
P_est = diag([0.01^2, 0.01^2, 0.01^2, 0.5^2]); % start tighter — we trust initial pose

%% =================== SIMULATION: GROUND TRUTH ===================
dt_sim = sim_data.dt_sim;
t_sim  = sim_data.t_sim;
N      = length(t_sim);

delta_true  = sim_data.delta_true;
a_long_true = sim_data.a_long_true;
x_true      = sim_data.x_true;
pwm_throttle = sim_data.pwm_throttle;
pwm_steer    = sim_data.pwm_steer;

%% =================== GENERATE NOISY MEASUREMENTS ===================
wheel_rot_times = sim_data.wheel_rot_times;
wheel_rot_idx   = sim_data.wheel_rot_idx;
wheel_rot_count = sim_data.wheel_rot_count;     % pulses per event
wheel_rot_dt_meas = sim_data.wheel_rot_dt_meas;  % noisy inter-pulse intervals [s]

%% =================== RUN THE EKF (dead reckoning) ===================
x_hist = zeros(4, N);
P_diag = zeros(4, N);
x_hist(:,1) = x_est;
P_diag(:,1) = diag(P_est);

rot_ptr = 1;
delta_est = 0;
delta_dot_est = 0;
wheel_v_meas_log = nan(size(wheel_rot_times));

for k = 2:N
    % ---- 1) PREDICT ----
    [delta_est, delta_dot_est] = steering_actuator_step( ...
        delta_est, delta_dot_est, pwm_steer(k-1), dt_sim, CONST.steer);
    a_cmd = throttle_pwm_to_accel(pwm_throttle(k-1), x_est(4), CONST.throttle);

    u = [delta_est; a_cmd];
    [x_est, P_est] = ekf_predict(x_est, P_est, u, dt_sim, CONST);

    % ---- 2) WHEEL PULSE — speed measurement update ----
    % Distance per pulse is exact (fixed geometry); all noise is on the timestamp.
    % v = pulse_distance / dt_meas
    % By error propagation: sigma_v = (pulse_distance / dt_meas^2) * sigma_dt
    if rot_ptr <= length(wheel_rot_idx) && k == wheel_rot_idx(rot_ptr)
        dt_meas = wheel_rot_dt_meas(rot_ptr);
        if ~isnan(dt_meas) && dt_meas > 1e-4
            ds = wheel_rot_count(rot_ptr) * CONST.pulse_distance;
            v_unsigned = ds / dt_meas;
            v_meas = sign(x_est(4)) * v_unsigned;
            sigma_v = (ds / dt_meas^2) * CONST.r_wheel_dt;
            R_v = sigma_v^2;
            [x_est, P_est] = ekf_update_speed(x_est, P_est, v_meas, R_v);
            wheel_v_meas_log(rot_ptr) = v_meas;
        end
        rot_ptr = rot_ptr + 1;
    end

    % ---- Store ----
    x_hist(:,k) = x_est;
    P_diag(:,k) = diag(P_est);
end

%% =================== PLOTS ===================
figure('Name','EKF Dead Reckoning','Position',[100 100 1200 800]);

% --- Trajectory ---
subplot(2,3,1); hold on; grid on;
plot(x_true(1,:), x_true(2,:), 'k-', 'LineWidth', 1.5);
plot(x_hist(1,:), x_hist(2,:), 'b--', 'LineWidth', 1.2);
legend('True','EKF (DR)');
xlabel('X [m]'); ylabel('Y [m]'); title('Trajectory (Dead Reckoning)');
axis equal;

% --- X position ---
subplot(2,3,2); hold on; grid on;
plot(t_sim, x_true(1,:), 'k-', 'LineWidth', 1.2);
plot(t_sim, x_hist(1,:), 'b--');
fill_sigma(t_sim, x_hist(1,:), sqrt(P_diag(1,:)), [0.5 0.5 1]);
xlabel('t [s]'); ylabel('X [m]'); title('X position');
legend('True','EKF','\pm1\sigma');

% --- Y position ---
subplot(2,3,3); hold on; grid on;
plot(t_sim, x_true(2,:), 'k-', 'LineWidth', 1.2);
plot(t_sim, x_hist(2,:), 'b--');
fill_sigma(t_sim, x_hist(2,:), sqrt(P_diag(2,:)), [0.5 0.5 1]);
xlabel('t [s]'); ylabel('Y [m]'); title('Y position');
legend('True','EKF','\pm1\sigma');

% --- Heading ---
subplot(2,3,4); hold on; grid on;
plot(t_sim, rad2deg(x_true(3,:)), 'k-', 'LineWidth', 1.2);
plot(t_sim, rad2deg(x_hist(3,:)), 'b--');
fill_sigma(t_sim, rad2deg(x_hist(3,:)), rad2deg(sqrt(P_diag(3,:))), [0.5 0.5 1]);
xlabel('t [s]'); ylabel('\theta [deg]'); title('Heading');
legend('True','EKF','\pm1\sigma');

% --- Speed ---
subplot(2,3,5); hold on; grid on;
plot(t_sim, x_true(4,:), 'k-', 'LineWidth', 1.2);
plot(t_sim, x_hist(4,:), 'b--');
plot(wheel_rot_times, wheel_v_meas_log, 'r.', 'MarkerSize', 5);
fill_sigma(t_sim, x_hist(4,:), sqrt(P_diag(4,:)), [0.5 0.5 1]);
xlabel('t [s]'); ylabel('v [m/s]'); title('Speed');
legend('True','EKF','Pulse meas','\pm1\sigma');

% --- Covariance growth (position) ---
subplot(2,3,6); hold on; grid on;
plot(t_sim, sqrt(P_diag(1,:)), 'r-', 'LineWidth', 1.2);
plot(t_sim, sqrt(P_diag(2,:)), 'b-', 'LineWidth', 1.2);
plot(t_sim, rad2deg(sqrt(P_diag(3,:))), 'g-', 'LineWidth', 1.2);
xlabel('t [s]'); ylabel('\sigma'); title('Uncertainty Growth (DR drift)');
legend('\sigma_X [m]','\sigma_Y [m]','\sigma_\theta [deg]');

sgtitle('EKF Dead Reckoning — No Camera (Drift Visible)');

%% =================== RMSE Summary ===================
rmse_x = sqrt(mean((x_hist(1,:) - x_true(1,:)).^2));
rmse_y = sqrt(mean((x_hist(2,:) - x_true(2,:)).^2));
rmse_theta = sqrt(mean(wrap_pi(x_hist(3,:) - x_true(3,:)).^2));
rmse_v = sqrt(mean((x_hist(4,:) - x_true(4,:)).^2));
fprintf('\n--- EKF Dead Reckoning Performance (RMSE) ---\n');
fprintf('  Position X : %.4f m\n', rmse_x);
fprintf('  Position Y : %.4f m\n', rmse_y);
fprintf('  Heading    : %.4f rad (%.2f deg)\n', rmse_theta, rad2deg(rmse_theta));
fprintf('  Speed      : %.4f m/s\n', rmse_v);
fprintf('\nNote: Position/heading RMSE will grow over time (no corrections).\n');
fprintf('Wheel increments directly advance pose by measured arc length.\n');

%% =====================================================================
%%                       LOCAL FUNCTIONS
%% =====================================================================

function [x_p, P_p] = ekf_predict(x, P, u, dt, C)
    theta = x(3);
    v     = x(4);
    delta = u(1);
    a_lon = u(2);
    L     = C.L;

    x_p = [
        x(1) + v * cos(theta) * dt
        x(2) + v * sin(theta) * dt
        wrap_pi(x(3) + (v / L) * tan(delta) * dt)
        x(4) + a_lon * dt
    ];

    F = [ 1, 0, -v*sin(theta)*dt,  cos(theta)*dt;
          0, 1,  v*cos(theta)*dt,  sin(theta)*dt;
          0, 0,  1,                tan(delta)/L*dt;
          0, 0,  0,                1              ];

    Q = diag([ (C.q_pos   * dt)^2, ...
               (C.q_pos   * dt)^2, ...
               (C.q_theta * dt)^2, ...
               (C.q_v     * dt)^2 ]);

    P_p = F * P * F' + Q;
end

function [x_u, P_u] = ekf_update_speed(x, P, v_meas, R_v)
    % Standard Kalman measurement update for the speed state only.
    % Heading is NOT changed — this is the fix for the heading scaling bug.
    H = [0, 0, 0, 1];
    y_inn = v_meas - x(4);
    S = H * P * H' + R_v;
    K = (P * H') / S;
    x_u = x + K * y_inn;
    I4 = eye(4);
    ImKH = I4 - K * H;
    P_u = ImKH * P * ImKH' + K * R_v * K';
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

function a = wrap_pi(a)
    a = mod(a + pi, 2*pi) - pi;
end

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end

function fill_sigma(t, mu, sigma, col)
    fill([t, fliplr(t)], [mu + sigma, fliplr(mu - sigma)], ...
        col, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
end

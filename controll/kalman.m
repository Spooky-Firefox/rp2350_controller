%% Extended Kalman Filter for RC Car — Kinematic Bicycle Model
% ============================================================
%
% STATE:   x = [X; Y; theta; v]
%   X, Y   : global position [m]
%   theta  : heading angle [rad] (0 = +X axis, CCW positive)
%   v      : longitudinal speed [m/s] (+ forward, - reverse)
%
% INPUTS (control, known at every predict step):
%   delta  : front-wheel steering angle [rad]
%   a_long : longitudinal acceleration [m/s^2]
%            - If you have angular accel of drive wheels (alpha):
%              a_long = alpha * r_w
%            - If not available, set to 0 (speed changes handled by
%              process noise)
%
% MEASUREMENTS (asynchronous, each at its own rate):
%   Camera      : [X_meas; Y_meas; theta_meas]
%   Axle speed  : |omega_axle| [rad/s]  (ABSOLUTE — no sign)
%
% PROCESS MODEL (front-wheel-steered bicycle, Euler discretisation):
%   X_{k+1}     = X_k     + v_k * cos(theta_k) * dt
%   Y_{k+1}     = Y_k     + v_k * sin(theta_k) * dt
%   theta_{k+1} = theta_k + (v_k / L) * tan(delta_k) * dt
%   v_{k+1}     = v_k     + a_long_k * dt
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
% All constants collected here — change as needed.

% --- Vehicle Geometry ---
CONST.L   = sim_data.plant.L;      % wheelbase [m]
CONST.r_w = sim_data.plant.r_w;    % wheel radius [m]
CONST.W   = sim_data.plant.W;      % track width [m] (info only; bicycle model ignores it)

% --- Speed Sensor Conversion ---
%   v = omega_sensor * speed_ratio
%   If the sensor spins faster than the wheel, divide by that ratio.
%   Example: axle/motor sensor at 3x wheel speed -> speed_ratio = r_w / 3
CONST.gear_ratio = sim_data.plant.gear_ratio;        % sensor speed / wheel speed [-]
CONST.speed_ratio = sim_data.plant.speed_ratio;      % [m/rad]

% --- Process Noise Std Dev (continuous-time-like; scaled by dt inside) ---
%   Larger values = less trust in the model, more trust in measurements.
CONST.q_pos   = 0.05;   % position  [m/s]      (accounts for unmodelled lateral slip etc.)
CONST.q_theta = 0.1;    % heading   [rad/s]
CONST.q_v     = 1.0;    % velocity  [m/s^2]    (set higher if a_long input is unreliable)

% --- Camera Measurement Noise Std Dev ---
CONST.r_cam_xy    = 0.02;   % x,y position [m]
CONST.r_cam_theta = 0.05;   % heading      [rad]

% --- Axle Speed Sensor Noise Std Dev ---
CONST.r_omega = 0.5;        % angular speed [rad/s]

% --- Absolute-speed Jacobian smoothing ---
%   Avoids singularity at v = 0.  Keep small (1e-3 to 1e-2 m/s).
CONST.eps_v = 1e-3;  % [m/s]

% --- Timing (for simulation only; real system uses actual dt) ---
CONST.dt_predict = sim_data.timing.dt_predict;       % EKF prediction period  [s]
CONST.dt_camera  = sim_data.timing.dt_camera;        % camera update period   [s]
CONST.axle_dt_range = sim_data.timing.axle_dt_range; % axle sensor inter-arrival [s]

%% =================== INITIAL STATE & COVARIANCE ===================
x_est = [0; 0; 0; 0];                          % initial state estimate
P_est = diag([0.1^2, 0.1^2, 0.1^2, 0.5^2]);   % initial covariance

%% =================== SIMULATION: GROUND TRUTH ===================
dt_sim = sim_data.dt_sim;
t_sim  = sim_data.t_sim;
N      = length(t_sim);

delta_true  = sim_data.delta_true;
a_long_true = sim_data.a_long_true;
x_true      = sim_data.x_true;

%% =================== GENERATE NOISY MEASUREMENTS ===================
cam_idx    = sim_data.cam_idx;
z_cam      = sim_data.z_cam;
axle_times = sim_data.axle_times;
axle_idx   = sim_data.axle_idx;
z_axle     = sim_data.z_axle;

%% =================== RUN THE EKF ===================
x_hist = zeros(4, N);       % state estimates
P_diag = zeros(4, N);       % diagonal of covariance (for plotting)
x_hist(:,1) = x_est;
P_diag(:,1) = diag(P_est);

cam_ptr  = 1;   % pointer into camera measurement array
axle_ptr = 1;   % pointer into axle measurement array

for k = 2:N
    % ---- 1) PREDICT ----
    u = [delta_true(k-1); a_long_true(k-1)];   % use known inputs
    [x_est, P_est] = ekf_predict(x_est, P_est, u, dt_sim, CONST);

    % ---- 2) CAMERA UPDATE (if sample available) ----
    if cam_ptr <= length(cam_idx) && k == cam_idx(cam_ptr)
        [x_est, P_est] = ekf_update_camera(x_est, P_est, ...
            z_cam(:, cam_ptr), CONST);
        cam_ptr = cam_ptr + 1;
    end

    % ---- 3) AXLE SPEED UPDATE (if sample available) ----
    if axle_ptr <= length(axle_idx) && k == axle_idx(axle_ptr)
        [x_est, P_est] = ekf_update_axle_speed(x_est, P_est, ...
            z_axle(axle_ptr), CONST);
        axle_ptr = axle_ptr + 1;
    end

    % ---- Store ----
    x_hist(:,k) = x_est;
    P_diag(:,k) = diag(P_est);
end

%% =================== PLOTS ===================
figure('Name','EKF Results','Position',[100 100 1200 800]);

% --- Trajectory ---
subplot(2,3,1); hold on; grid on;
plot(x_true(1,:), x_true(2,:), 'k-', 'LineWidth', 1.5);
plot(x_hist(1,:), x_hist(2,:), 'b--', 'LineWidth', 1.2);
plot(z_cam(1,:), z_cam(2,:), 'r.', 'MarkerSize', 4);
legend('True','EKF','Camera');
xlabel('X [m]'); ylabel('Y [m]'); title('Trajectory');
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
xlabel('t [s]'); ylabel('\theta [deg]'); title('Heading');
legend('True','EKF');

% --- Speed ---
subplot(2,3,5); hold on; grid on;
plot(t_sim, x_true(4,:), 'k-', 'LineWidth', 1.2);
plot(t_sim, x_hist(4,:), 'b--');
v_meas_pts = z_axle .* CONST.speed_ratio;  % convert back to linear speed
plot(axle_times, v_meas_pts, 'r.', 'MarkerSize', 4);
fill_sigma(t_sim, x_hist(4,:), sqrt(P_diag(4,:)), [0.5 0.5 1]);
xlabel('t [s]'); ylabel('v [m/s]'); title('Speed');
legend('True','EKF','|v| meas','\pm1\sigma');

% --- Speed estimation error ---
subplot(2,3,6); hold on; grid on;
err_v = x_hist(4,:) - x_true(4,:);
plot(t_sim, err_v, 'b-');
plot(t_sim, 2*sqrt(P_diag(4,:)), 'r--');
plot(t_sim, -2*sqrt(P_diag(4,:)), 'r--');
xlabel('t [s]'); ylabel('v error [m/s]'); title('Speed Error & 2\sigma bounds');
legend('Error','\pm2\sigma');

sgtitle('Extended Kalman Filter — Kinematic Bicycle Model RC Car');

%% =================== RMSE Summary ===================
rmse_x = sqrt(mean((x_hist(1,:) - x_true(1,:)).^2));
rmse_y = sqrt(mean((x_hist(2,:) - x_true(2,:)).^2));
rmse_theta = sqrt(mean(wrap_pi(x_hist(3,:) - x_true(3,:)).^2));
rmse_v = sqrt(mean((x_hist(4,:) - x_true(4,:)).^2));
fprintf('\n--- EKF Performance (RMSE) ---\n');
fprintf('  Position X : %.4f m\n', rmse_x);
fprintf('  Position Y : %.4f m\n', rmse_y);
fprintf('  Heading    : %.4f rad (%.2f deg)\n', rmse_theta, rad2deg(rmse_theta));
fprintf('  Speed      : %.4f m/s\n', rmse_v);

%% =====================================================================
%%                       LOCAL FUNCTIONS
%% =====================================================================

function x_next = bicycle_step(x, delta, a_long, dt, C)
    % One Euler step of the kinematic bicycle model.
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

function [x_p, P_p] = ekf_predict(x, P, u, dt, C)
    % EKF PREDICTION STEP
    %
    %   x  : state [X; Y; theta; v]       (4×1)
    %   P  : covariance                    (4×4)
    %   u  : [delta; a_long]               (2×1)
    %   dt : time step [s]                 (scalar)
    %   C  : CONST struct
    %
    % delta  = front steering angle [rad]
    % a_long = longitudinal acceleration [m/s^2]
    %          (= alpha_drive_wheels * r_w, or 0 if unavailable)

    theta = x(3);
    v     = x(4);
    delta = u(1);
    a_lon = u(2);
    L     = C.L;

    % --- Nonlinear state prediction ---
    x_p = [
        x(1) + v * cos(theta) * dt
        x(2) + v * sin(theta) * dt
        wrap_pi(x(3) + (v / L) * tan(delta) * dt)
        x(4) + a_lon * dt
    ];

    % --- Jacobian  F = df/dx ---
    F = [ 1, 0, -v*sin(theta)*dt,  cos(theta)*dt;
          0, 1,  v*cos(theta)*dt,  sin(theta)*dt;
          0, 0,  1,                tan(delta)/L*dt;
          0, 0,  0,                1              ];

    % --- Process noise covariance Q ---
    %   Scaled by dt so noise is consistent across different time steps.
    Q = diag([ (C.q_pos   * dt)^2, ...
               (C.q_pos   * dt)^2, ...
               (C.q_theta * dt)^2, ...
               (C.q_v     * dt)^2 ]);

    % --- Covariance prediction ---
    P_p = F * P * F' + Q;
end

function [x_u, P_u] = ekf_update_camera(x, P, z, C)
    % EKF MEASUREMENT UPDATE — CAMERA
    %
    %   z : [X_meas; Y_meas; theta_meas]  (3×1)
    %
    % Linear measurement model: h(x) = H * x

    H = [1 0 0 0;
         0 1 0 0;
         0 0 1 0];

    R = diag([C.r_cam_xy^2, C.r_cam_xy^2, C.r_cam_theta^2]);

    % Innovation
    y_inn = z - H * x;
    y_inn(3) = wrap_pi(y_inn(3));   % angle wrapping

    % Kalman gain
    S = H * P * H' + R;
    K = (P * H') / S;

    % State update
    x_u = x + K * y_inn;
    x_u(3) = wrap_pi(x_u(3));

    % Covariance update (Joseph form — numerically stable)
    I4 = eye(4);
    ImKH = I4 - K * H;
    P_u = ImKH * P * ImKH' + K * R * K';
end

function [x_u, P_u] = ekf_update_axle_speed(x, P, omega_meas, C)
    % EKF MEASUREMENT UPDATE — AXLE SPEED (absolute value)
    %
    %   omega_meas : |angular speed| of axle sensor  [rad/s]  (scalar >= 0)
    %
    % Measurement model:  h(x) = |v| / speed_ratio
    %   Smoothed version:  h(x) = sqrt(v^2 + eps^2) / speed_ratio
    %   This avoids the non-differentiable |·| at v = 0.
    %
    % NOTE: Because the sensor is UNSIGNED, the filter uses the current
    % velocity sign from the state to resolve direction.  If speed is
    % near zero the measurement has little influence (by design).

    v   = x(4);
    sr  = C.speed_ratio;
    eps = C.eps_v;

    % Predicted measurement (smoothed absolute speed)
    v_abs  = sqrt(v^2 + eps^2);
    h_pred = v_abs / sr;

    % Jacobian  dh/dx = [0, 0, 0, dh/dv]
    dh_dv = v / (v_abs * sr);
    H = [0, 0, 0, dh_dv];

    % Measurement noise (in omega domain)
    R = C.r_omega^2;

    % Innovation
    y_inn = omega_meas - h_pred;

    % Kalman gain
    S = H * P * H' + R;
    K = (P * H') / S;

    % State update
    x_u = x + K * y_inn;

    % Covariance update (Joseph form)
    I4 = eye(4);
    ImKH = I4 - K * H;
    P_u = ImKH * P * ImKH' + K * R * K';
end

function a = wrap_pi(a)
    % Wrap angle(s) to [-pi, pi].
    a = mod(a + pi, 2*pi) - pi;
end

function fill_sigma(t, mu, sigma, col)
    % Shade ±1σ band.
    fill([t, fliplr(t)], [mu + sigma, fliplr(mu - sigma)], ...
        col, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
end

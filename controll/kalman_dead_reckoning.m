%% Extended Kalman Filter for RC Car — Dead Reckoning (No Camera)
% ============================================================
%
% This is the dead-reckoning variant: position and heading are estimated
% purely from the kinematic bicycle model + axle speed sensor.
% There is NO external position/heading correction.
%
% STATE:   x = [X; Y; theta; v]
%   X, Y   : global position [m]
%   theta  : heading angle [rad] (0 = +X axis, CCW positive)
%   v      : longitudinal speed [m/s] (+ forward, - reverse)
%
% INPUTS (control, known at every predict step):
%   delta  : front-wheel steering angle [rad]
%   a_long : longitudinal acceleration [m/s^2]
%
% MEASUREMENTS (only axle speed, variable rate):
%   Axle speed  : |omega_axle| [rad/s]  (ABSOLUTE — no sign)
%
% WARNING: Without an external position/heading reference, drift in
% X, Y, and theta will grow unboundedly over time.  The axle speed
% sensor only constrains the speed state.  This variant is suitable
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

% --- Speed Sensor Conversion ---
%   Sensor spins 3x faster than the wheel.
CONST.gear_ratio = sim_data.plant.gear_ratio;        % sensor speed / wheel speed [-]
CONST.speed_ratio = sim_data.plant.speed_ratio;      % [m/rad]

% --- Process Noise Std Dev (continuous-time-like; scaled by dt inside) ---
%   For dead reckoning you may want LOWER process noise to trust the
%   model more, since there are no position corrections to pull back drift.
CONST.q_pos   = 0.02;   % position  [m/s]
CONST.q_theta = 0.05;   % heading   [rad/s]
CONST.q_v     = 1.0;    % velocity  [m/s^2]

% --- Axle Speed Sensor Noise Std Dev ---
CONST.r_omega = 0.5;    % angular speed [rad/s]

% --- Absolute-speed Jacobian smoothing ---
CONST.eps_v = 1e-3;     % [m/s]

% --- Timing (for simulation only; real system uses actual dt) ---
CONST.dt_predict = sim_data.timing.dt_predict;       % EKF prediction period  [s]
CONST.axle_dt_range = sim_data.timing.axle_dt_range; % axle sensor inter-arrival [s]

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

%% =================== GENERATE NOISY MEASUREMENTS ===================
axle_times = sim_data.axle_times;
axle_idx   = sim_data.axle_idx;
z_axle     = sim_data.z_axle;

%% =================== RUN THE EKF (dead reckoning) ===================
x_hist = zeros(4, N);
P_diag = zeros(4, N);
x_hist(:,1) = x_est;
P_diag(:,1) = diag(P_est);

axle_ptr = 1;

for k = 2:N
    % ---- 1) PREDICT ----
    u = [delta_true(k-1); a_long_true(k-1)];
    [x_est, P_est] = ekf_predict(x_est, P_est, u, dt_sim, CONST);

    % ---- 2) AXLE SPEED UPDATE (if sample available) ----
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
figure('Name','EKF Dead Reckoning (No Camera)','Position',[100 100 1200 800]);

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
v_meas_pts = z_axle .* CONST.speed_ratio;
plot(axle_times, v_meas_pts, 'r.', 'MarkerSize', 4);
fill_sigma(t_sim, x_hist(4,:), sqrt(P_diag(4,:)), [0.5 0.5 1]);
xlabel('t [s]'); ylabel('v [m/s]'); title('Speed');
legend('True','EKF','|v| meas','\pm1\sigma');

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
fprintf('Speed RMSE should remain bounded (axle sensor constrains it).\n');

%% =====================================================================
%%                       LOCAL FUNCTIONS
%% =====================================================================

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

function [x_u, P_u] = ekf_update_axle_speed(x, P, omega_meas, C)
    v   = x(4);
    sr  = C.speed_ratio;
    eps = C.eps_v;

    v_abs  = sqrt(v^2 + eps^2);
    h_pred = v_abs / sr;

    dh_dv = v / (v_abs * sr);
    H = [0, 0, 0, dh_dv];

    R = C.r_omega^2;

    y_inn = omega_meas - h_pred;

    S = H * P * H' + R;
    K = (P * H') / S;

    x_u = x + K * y_inn;

    I4 = eye(4);
    ImKH = I4 - K * H;
    P_u = ImKH * P * ImKH' + K * R * K';
end

function a = wrap_pi(a)
    a = mod(a + pi, 2*pi) - pi;
end

function fill_sigma(t, mu, sigma, col)
    fill([t, fliplr(t)], [mu + sigma, fliplr(mu - sigma)], ...
        col, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
end

function kalman_dead_reckoning(log_file, length_per_data_point_cm, remove_segments_s, plot_slice_s)
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
clc; close all;

if nargin < 1 || isempty(log_file)
    log_file = "log_20260509_133305.csv";
end
if nargin < 2 || isempty(length_per_data_point_cm)
    length_per_data_point_cm = 3.03;
end
if nargin < 3 || isempty(remove_segments_s)
    remove_segments_s = zeros(0, 2);
end
if nargin < 4 || isempty(plot_slice_s)
    plot_slice_s = [];
end

length_per_data_point_m = length_per_data_point_cm * 1e-2;
if ~isfinite(length_per_data_point_m) || length_per_data_point_m <= 0
    error('length_per_data_point_cm must be a positive finite scalar.');
end

%% =================== RUN MODE ===================
% 'sim'  -> original synthetic data pipeline
% 'log'  -> run EKF directly on recorded CSV and compare to measured reality
script_dir = fileparts(mfilename('fullpath'));
RUN_MODE = 'log';
if isfile(log_file)
    LOG_FILE = log_file;
else
    LOG_FILE = fullfile(script_dir, 'actual_on_ground_data', char(log_file));
end

if strcmpi(RUN_MODE, 'log')
    run_dead_reckoning_on_log(LOG_FILE, script_dir, length_per_data_point_m, remove_segments_s, plot_slice_s);
    return;
end

%% =================== LOAD GENERATED DATA ===================
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
CONST.q_v     = 5.0;    % velocity  [m/s^2]

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
throttle_cfg = CONST.throttle;
throttle_cfg.dt = dt_sim;
throttle_cfg.reset = true;
throttle_pwm_to_accel(1500, 0, throttle_cfg);
throttle_cfg.reset = false;

for k = 2:N
    % ---- 1) PREDICT ----
    [delta_est, delta_dot_est] = steering_actuator_step( ...
        delta_est, delta_dot_est, pwm_steer(k-1), dt_sim, CONST.steer);
    a_cmd = throttle_pwm_to_accel(pwm_throttle(k-1), x_est(4), throttle_cfg);

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

end

%% =====================================================================
%%                       LOCAL FUNCTIONS
%% =====================================================================

function run_dead_reckoning_on_log(log_file, script_dir, length_per_data_point_m, remove_segments_s, plot_slice_s)
    if ~isfile(log_file)
        error('Log file not found: %s', log_file);
    end

    data_file = fullfile(script_dir, 'kalman_sim_data.mat');
    if ~isfile(data_file)
        generate_kalman_data(data_file);
    end
    loaded = load(data_file, 'sim_data');
    sim_data = loaded.sim_data;

    T = readtable(log_file);
    req = {'timestamp_us', 'steer_us', 'throttle_us', 'speed_mps'};
    has_all = all(ismember(req, T.Properties.VariableNames));
    if ~has_all
        error('CSV must contain columns: %s', strjoin(req, ', '));
    end

    t_evt = (double(T.timestamp_us) - double(T.timestamp_us(1))) * 1e-6;
    keep = [true; diff(t_evt) > 0];
    t_evt = t_evt(keep);
    pwm_steer_evt = double(T.steer_us(keep));
    pwm_throttle_evt = double(T.throttle_us(keep));
    speed_mps_evt = double(T.speed_mps(keep));

    if ~ismatrix(remove_segments_s) || size(remove_segments_s, 2) ~= 2
        error('remove_segments_s must be an Nx2 matrix: [t_start t_end; ...]');
    end
    if any(any(~isfinite(remove_segments_s)))
        error('remove_segments_s must contain only finite values.');
    end

    if ~isempty(plot_slice_s)
        if ~isvector(plot_slice_s) || numel(plot_slice_s) ~= 2 || any(~isfinite(plot_slice_s))
            error('plot_slice_s must be [t_start t_end].');
        end
        plot_slice_s = sort(plot_slice_s(:)).';
    end

    if ~isempty(remove_segments_s)
        remove_segments_s = sort(remove_segments_s, 2);
        keep_mask = true(size(t_evt));
        for i = 1:size(remove_segments_s, 1)
            t0 = remove_segments_s(i, 1);
            t1 = remove_segments_s(i, 2);
            keep_mask = keep_mask & ~(t_evt >= t0 & t_evt <= t1);
        end
        t_evt = t_evt(keep_mask);
        pwm_steer_evt = pwm_steer_evt(keep_mask);
        pwm_throttle_evt = pwm_throttle_evt(keep_mask);
        speed_mps_evt = speed_mps_evt(keep_mask);
    end

    if ~isempty(plot_slice_s)
        keep_slice = t_evt >= plot_slice_s(1) & t_evt <= plot_slice_s(2);
        t_evt = t_evt(keep_slice);
        pwm_steer_evt = pwm_steer_evt(keep_slice);
        pwm_throttle_evt = pwm_throttle_evt(keep_slice);
        speed_mps_evt = speed_mps_evt(keep_slice);
    end

    if numel(t_evt) < 3
        error('Not enough valid samples in log for EKF run.');
    end

    % Build a uniform time grid and derive speed from delta-position (ds/dt).
    dt_nom = sim_data.timing.dt_predict;
    if ~isfinite(dt_nom) || dt_nom <= 1e-5
        dt_nom = median(diff(t_evt), 'omitnan');
    end
    dt_nom = min(max(dt_nom, 0.005), 0.05);
    
    fprintf('dt_nom selected: %.6f s (%.1f Hz)\n', dt_nom, 1/dt_nom);

    t_s = (t_evt(1):dt_nom:t_evt(end)).';
    if numel(t_s) < 3
        error('Not enough timeline samples after applying segment filters.');
    end

    % Hold last commanded PWM over the uniform timeline.
    pwm_steer = interp1(t_evt, pwm_steer_evt, t_s, 'previous', 'extrap');
    pwm_throttle = interp1(t_evt, pwm_throttle_evt, t_s, 'previous', 'extrap');

    % Motion events: only rows where firmware reported nonzero speed.
    pulse_evt = speed_mps_evt > 1e-6;

    % Infer pulse-event speed from inter-pulse time: v = pulse_distance / dt_pulse.
    speed_inferred_evt = zeros(size(t_evt));
    dt_pulse_evt = nan(size(t_evt));
    pulse_idx = find(pulse_evt);
    if numel(pulse_idx) >= 2
        for j = 2:numel(pulse_idx)
            i_prev = pulse_idx(j-1);
            i_now = pulse_idx(j);
            dt_pulse = t_evt(i_now) - t_evt(i_prev);
            if dt_pulse > 1e-4
                speed_inferred_evt(i_now) = length_per_data_point_m / dt_pulse;
                dt_pulse_evt(i_now) = dt_pulse;
            end
        end
        speed_inferred_evt(pulse_idx(1)) = speed_inferred_evt(pulse_idx(2));
        dt_pulse_evt(pulse_idx(1)) = dt_pulse_evt(pulse_idx(2));
    end

    % Continuous reference line for plotting.
    speed_inferred = interp1(t_evt, speed_inferred_evt, t_s, 'linear', 0);

    % Map pulse events to uniform timeline.
    N = numel(t_s);
    event_step_idx = 1 + floor((t_evt - t_s(1)) / dt_nom);
    event_step_idx = min(max(event_step_idx, 1), N);

    pulse_count_step = accumarray(event_step_idx, double(pulse_evt), [N, 1], @sum, 0);

    % Build one speed measurement per uniform step from pulse-event inferred speeds.
    pulse_valid_evt = pulse_evt & (speed_inferred_evt > 0);
    speed_meas_step = nan(N, 1);
    dt_pulse_step = nan(N, 1);
    if any(pulse_valid_evt)
        pulse_step_idx = event_step_idx(pulse_valid_evt);
        pulse_speed_vals = speed_inferred_evt(pulse_valid_evt);
        pulse_dt_vals = dt_pulse_evt(pulse_valid_evt);
        speed_meas_step = accumarray(pulse_step_idx, pulse_speed_vals, [N, 1], @mean, NaN);
        dt_pulse_step = accumarray(pulse_step_idx, pulse_dt_vals, [N, 1], @mean, NaN);
    end

    % Count only events that also have a valid inferred speed sample.
    pulse_count_valid_step = accumarray(event_step_idx(pulse_valid_evt), 1, [N, 1], @sum, 0);

    % Reject outliers where measured acceleration jumps too much.
    accel_jump_limit_mps2 = 2.0;
    meas_idx = find(~isnan(speed_meas_step));
    if numel(meas_idx) >= 3
        prev_idx = meas_idx(1);
        prev_acc = NaN;
        for ii = 2:numel(meas_idx)
            k_now = meas_idx(ii);
            dt_meas = t_s(k_now) - t_s(prev_idx);
            if dt_meas <= 1e-4
                continue;
            end
            acc_now = (speed_meas_step(k_now) - speed_meas_step(prev_idx)) / dt_meas;
            if ~isnan(prev_acc) && abs(acc_now - prev_acc) > accel_jump_limit_mps2
                speed_meas_step(k_now) = NaN;
                dt_pulse_step(k_now) = NaN;
                pulse_count_valid_step(k_now) = 0;
                continue;
            end
            prev_acc = acc_now;
            prev_idx = k_now;
        end
    end

    % Shared acceptance mask: a measurement is used by both EKF and distance
    % only if it survived all validity and outlier filters.
    accepted_meas_step = ~isnan(speed_meas_step);

    CONST = struct();
    CONST.L = sim_data.plant.L;
    CONST.r_w = sim_data.plant.r_w;
    CONST.W = sim_data.plant.W;
    CONST.wheel_circumference = sim_data.plant.wheel_circumference;
    CONST.pulse_distance = length_per_data_point_m;
    CONST.throttle = sim_data.plant.throttle;
    CONST.steer = sim_data.plant.steer;

    % Slightly conservative process noise for real-log run.
    CONST.q_pos = 0.03;
    CONST.q_theta = 0.06;
    CONST.q_v = 1.2;

    x_est = [0; 0; 0; 0];
    P_est = diag([0.05^2, 0.05^2, deg2rad(3)^2, 0.6^2]);

    x_hist = zeros(4, N);
    P_diag = zeros(4, N);
    delta_hist = zeros(1, N);
    a_cmd_hist = zeros(1, N);
    speed_from_dt = nan(N, 1);
    v_innov = nan(1, N);

    x_hist(:,1) = x_est;
    P_diag(:,1) = diag(P_est);
    delta_est = 0;
    delta_dot_est = 0;

    throttle_cfg = CONST.throttle;
    throttle_cfg.dt = dt_nom;
    throttle_cfg.reset = true;
    throttle_pwm_to_accel(1500, 0, throttle_cfg);
    throttle_cfg.reset = false;

    for k = 2:N
        dt = dt_nom;

        [delta_est, delta_dot_est] = steering_actuator_step( ...
            delta_est, delta_dot_est, pwm_steer(k-1), dt, CONST.steer);
        throttle_cfg.dt = dt;
        a_cmd = throttle_pwm_to_accel(pwm_throttle(k-1), x_est(4), throttle_cfg);

        u = [delta_est; a_cmd];
        [x_est, P_est] = ekf_predict(x_est, P_est, u, dt, CONST);

        % Measurement update from inferred pulse speed in this step.
        if accepted_meas_step(k)
            v_m = speed_meas_step(k);
            speed_from_dt(k) = v_m;
            % Timestamp-jitter propagation for pulse-timed speed.
            sigma_dt = max(sim_data.sensor.r_wheel_dt, 1e-4);
            dt_meas = dt_pulse_step(k);
            if isnan(dt_meas) || dt_meas <= 1e-4
                dt_meas = dt;
            end
            sigma_v = (CONST.pulse_distance / (dt_meas^2)) * sigma_dt;
            sigma_v = max(sigma_v, 0.01);
            R_v = sigma_v^2;

            v_innov(k) = v_m - x_est(4);
            [x_est, P_est] = ekf_update_speed(x_est, P_est, v_m, R_v);
        else
            % No pulse means no direct speed sample in this step.
            % Do not plot or measure speed here.
            speed_from_dt(k) = NaN;
            v_innov(k) = NaN;
        end

        x_hist(:,k) = x_est;
        P_diag(:,k) = diag(P_est);
        delta_hist(k) = delta_est;
        a_cmd_hist(k) = a_cmd;
    end

    speed_ekf = x_hist(4, :)';

    % Reference speed is directly from logged speed_mps data.

    dist_meas = cumsum(pulse_count_valid_step .* double(accepted_meas_step)) * CONST.pulse_distance;
    dist_ekf_cont = cumtrapz(t_s, speed_ekf);  % Continuous model-integrated distance
    dist_from_dt = cumsum(pulse_count_step) * CONST.pulse_distance;  % Direct from pulses

    % Event-driven EKF distance: only advance when a valid speed measurement exists.
    dist_ekf = zeros(N, 1);
    for k = 2:N
        dist_ekf(k) = dist_ekf(k-1);
        if accepted_meas_step(k)
            dt_evt_k = dt_pulse_step(k);
            if isnan(dt_evt_k) || dt_evt_k <= 1e-4
                dt_evt_k = dt_nom;
            end
            dist_ekf(k) = dist_ekf(k) + max(speed_ekf(k), 0) * dt_evt_k;
        end
    end

    rmse_speed = sqrt(mean((speed_ekf - speed_inferred).^2, 'omitnan'));
    rmse_dist = sqrt(mean((dist_ekf - dist_meas).^2, 'omitnan'));

    figure('Name', 'EKF Dead Reckoning on Real Log', 'Position', [120 100 1300 820]);

    subplot(2,3,1); hold on; grid on;
    plot(t_s, speed_inferred, 'k-', 'LineWidth', 1.3);
    plot(t_s, speed_ekf, 'b--', 'LineWidth', 1.2);
    fill_sigma(t_s', speed_ekf', sqrt(P_diag(4, :)), [0.6 0.8 1.0]);
    xlabel('t [s]'); ylabel('v [m/s]');
    title(sprintf('Speed Comparison (Data vs EKF)'));
    legend('Speed from data points (3.03cm/dt)', 'EKF speed', '\pm1\sigma', 'Location', 'best');

    subplot(2,3,2); hold on; grid on;
    plot(t_s, dist_meas, 'k-', 'LineWidth', 1.3);
    plot(t_s, dist_ekf, 'b--', 'LineWidth', 1.2);
    plot(t_s, dist_ekf_cont, 'c-.', 'LineWidth', 1.0);
    plot(t_s, dist_from_dt, 'm:', 'LineWidth', 1.1);
    xlabel('t [s]'); ylabel('distance [m]');
    title(sprintf('Distance Comparison (RMSE = %.3f m)', rmse_dist));
    legend('Distance from pulse count', 'EKF distance (event-driven)', ...
        'EKF distance (continuous \int v_{ekf}dt)', ...
        'Distance from \int v_{\Delta t}dt', 'Location', 'best');

    subplot(2,3,3); hold on; grid on;
    plot(t_s, pwm_throttle, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.2);
    plot(t_s, pwm_steer, 'Color', [0.00 0.45 0.74], 'LineWidth', 1.1);
    yline(1500, '--', 'Neutral');
    xlabel('t [s]'); ylabel('PWM [us]');
    title('Control Inputs');
    legend('Throttle', 'Steer', 'Location', 'best');

    subplot(2,3,4); hold on; grid on;
    plot(t_s, v_innov, 'r-', 'LineWidth', 1.0);
    yline(0, ':k');
    xlabel('t [s]'); ylabel('innovation [m/s]');
    title('Speed Innovation: v_{\Delta s/\Delta t} - v_{pred}');

    subplot(2,3,5); hold on; grid on;
    plot(t_s, rad2deg(delta_hist), 'b-', 'LineWidth', 1.2);
    xlabel('t [s]'); ylabel('\delta [deg]');
    title('Estimated Steering Angle');

    subplot(2,3,6); hold on; grid on;
    plot(t_s, x_hist(1, :), 'LineWidth', 1.2);
    plot(t_s, x_hist(2, :), 'LineWidth', 1.2);
    xlabel('t [s]'); ylabel('position [m]');
    title('EKF Pose States (no absolute ground truth)');
    legend('X_{ekf}', 'Y_{ekf}', 'Location', 'best');

    sgtitle(sprintf('EKF Dead Reckoning on %s', log_file), 'Interpreter', 'none');

    fprintf('\n--- EKF on Real Log (Speed inferred from data points) ---\n');
    fprintf('Log file                  : %s\n', log_file);
    fprintf('Samples used              : %d\n', N);
    fprintf('EKF speed updates used    : %d\n', nnz(accepted_meas_step));
    fprintf('Speed RMSE (EKF vs data)  : %.4f m/s\n', rmse_speed);
    fprintf('Dist RMSE  (EKF vs pulses): %.4f m\n', rmse_dist);
    fprintf('Final distance (pulses)   : %.3f m\n', dist_meas(end));
    fprintf('Final distance (EKF,event): %.3f m\n', dist_ekf(end));
    fprintf('Final distance (EKF,cont) : %.3f m\n', dist_ekf_cont(end));
    fprintf('\nNote: Data speed inferred as 3.03cm / dt for each data point (when original speed_mps ~= 0).\n');
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

function fitted_params = fit_throttle_model(csv_file, length_per_data_point_m, remove_segments_s, varargin)
% Fit throttle-to-acceleration model parameters from drive log CSV.
%
% Usage:
%   fitted_params = fit_throttle_model("log_20260509_133305.csv")
%   fitted_params = fit_throttle_model(csv_file, 0.0303, [3.0 4.2; 8.5 9.0])
%
% Outputs fitted parameters: A, throttle_tau_s, back_emf_gain, deadband_us, throttle_exponent
%
% Fixed (measured) parameters: drag_k0, drag_k1
%
% The model is:
%   u = (pwm_us - 1500) / 500, clipped to [0, 1] (forward-only)
%   u_eff = deadband-adjusted throttle command
%   u_filt = first-order lag of u_eff with time constant tau
%   a = A * u_filt^n / (1 + k_e*v) - (k0 + k1*v)

    if nargin < 1 || strlength(string(csv_file)) == 0
        csv_file = fullfile(fileparts(mfilename("fullpath")), ...
            "log_20260509_133305.csv");
    end

    if nargin < 2 || isempty(length_per_data_point_m)
        length_per_data_point_m = 0.0303;
    end

    if nargin < 3 || isempty(remove_segments_s)
        remove_segments_s = zeros(0, 2);
    end
    if nargin < 4 || isempty(varargin)
        plot_slice_s = [];
    else
        plot_slice_s = varargin{1};
    end

    % Load CSV
    T = readtable(csv_file);
    required = ["timestamp_us", "steer_us", "throttle_us", "speed_mps"];
    missing = required(~ismember(required, string(T.Properties.VariableNames)));
    if ~isempty(missing)
        error("Missing CSV columns: %s", strjoin(missing, ", "));
    end

    t_s = (double(T.timestamp_us) - double(T.timestamp_us(1))) * 1e-6;
    throttle_us = double(T.throttle_us);
    speed_mps_logged = double(T.speed_mps);

    % Infer speed from timing (like plot_drive_log does)
    dt_for_speed = [NaN; diff(t_s)];
    if numel(dt_for_speed) >= 2 && ~isfinite(dt_for_speed(1))
        dt_for_speed(1) = dt_for_speed(2);
    end
    moving_mask = isfinite(speed_mps_logged) & (speed_mps_logged ~= 0);
    valid_dt_mask = isfinite(dt_for_speed) & dt_for_speed > 0;
    speed_mps = zeros(size(speed_mps_logged));
    infer_mask = moving_mask & valid_dt_mask;
    speed_mps(infer_mask) = length_per_data_point_m ./ dt_for_speed(infer_mask);

    % Filter speed spikes
    max_speed_step_mps = 1.0;
    speed_step_mps = [0; abs(diff(speed_mps))];
    speed_mps(speed_step_mps > max_speed_step_mps) = NaN;

    % Remove segments
    if ~isempty(remove_segments_s)
        remove_segments_s = sort(remove_segments_s, 2);
        keep_mask = true(size(t_s));
        for i = 1:size(remove_segments_s, 1)
            t0 = remove_segments_s(i, 1);
            t1 = remove_segments_s(i, 2);
            keep_mask = keep_mask & ~(t_s >= t0 & t_s <= t1);
        end
        t_s = t_s(keep_mask);
        throttle_us = throttle_us(keep_mask);
        speed_mps = speed_mps(keep_mask);
    end

    % Slice to time window
    if ~isempty(plot_slice_s)
        if ~isvector(plot_slice_s) || numel(plot_slice_s) ~= 2 || any(~isfinite(plot_slice_s))
            error("plot_slice_s must be [t_start t_end].");
        end
        plot_slice_s = sort(plot_slice_s(:))';
        keep_slice = t_s >= plot_slice_s(1) & t_s <= plot_slice_s(2);
        t_s = t_s(keep_slice);
        throttle_us = throttle_us(keep_slice);
        speed_mps = speed_mps(keep_slice);
    end

    % Compute measured acceleration
    dt = [NaN; diff(t_s)];
    accel_raw = [NaN; diff(speed_mps) ./ dt(2:end)];
    
    % Smooth acceleration over ~200 ms window
    median_dt = median(dt(2:end), "omitnan");
    if ~isfinite(median_dt) || median_dt <= 0
        span_samples = 7;
    else
        span_samples = max(5, round(0.20 / median_dt));
    end
    accel_mps2 = movmean(accel_raw, span_samples, "omitnan");

    % Build fit dataset: exclude early transients, bad samples
    min_time_s = 0.5;  % skip very start
    skip_accel_spikes = true;
    transient_skip_s = 0.050;  % skip 50ms after throttle changes
    
    % Remove any samples where measured speed was reported as zero.
    nonzero_speed_idx = speed_mps > 0;

    % Reject points where acceleration jumps by more than 2 m/s^2 from the
    % previous accepted acceleration sample.
    accel_jump_limit_mps2 = 2.0;
    accel_jump_ok = true(size(accel_mps2));
    first_valid_acc = find(isfinite(accel_mps2), 1, "first");
    if ~isempty(first_valid_acc)
        last_acc = accel_mps2(first_valid_acc);
        for i = (first_valid_acc + 1):numel(accel_mps2)
            if ~isfinite(accel_mps2(i))
                continue;
            end
            if abs(accel_mps2(i) - last_acc) > accel_jump_limit_mps2
                accel_jump_ok(i) = false;
            else
                last_acc = accel_mps2(i);
            end
        end
    end

    valid_idx = t_s > min_time_s & isfinite(accel_mps2) & isfinite(speed_mps) & ...
        isfinite(throttle_us) & nonzero_speed_idx & accel_jump_ok;
    
    if skip_accel_spikes
        % Remove points where throttle just changed (within transient_skip_s)
        throttle_change_idx = [true; abs(diff(throttle_us)) > 5];
        time_since_change = NaN(size(t_s));
        last_change_time = 0;
        for i = 1:numel(t_s)
            if throttle_change_idx(i)
                last_change_time = t_s(i);
            end
            time_since_change(i) = t_s(i) - last_change_time;
        end
        valid_idx = valid_idx & (time_since_change > transient_skip_s);
    end

    t_fit = t_s(valid_idx);
    throt_fit = throttle_us(valid_idx);
    speed_fit = speed_mps(valid_idx);
    accel_fit = accel_mps2(valid_idx);

    if numel(t_fit) < 20
        error("Not enough valid samples for fitting. Got %d, need at least 20.", numel(t_fit));
    end

    fprintf("\n=== Throttle Model Fitting ===\n");
    fprintf("CSV file:         %s\n", csv_file);
    fprintf("Fit samples:      %d\n", numel(t_fit));
    fprintf("Speed range:      %.3f to %.3f m/s\n", min(speed_fit), max(speed_fit));
    fprintf("Throttle range:   %.0f to %.0f us\n", min(throt_fit), max(throt_fit));
    fprintf("Accel range:      %.3f to %.3f m/s^2\n", min(accel_fit), max(accel_fit));

    % Fixed drag parameters (measured from coast analysis)
    drag_k0 = 1.97678;
    drag_k1 = 0.1724;

    % Initial guess
    x0 = [
        2.2      % A (max_accel_fwd) [m/s^2]
        0.12     % tau (throttle_tau_s) [s]
        0.25     % k_e (back_emf_gain) [s/m]
        25       % deadband_us [us]
        2.0      % n (throttle exponent)
    ];

    % Bounds
    lower = [1.0,  0.02, 0.0,  0.0, 1.0];
    upper = [8.0,  0.50, 1.0, 80.0, 3.5];

    % Optimizer options
    opts = optimoptions('lsqnonlin', ...
        'MaxIterations', 1000, ...
        'MaxFunctionEvaluations', 5000, ...
        'Display', 'iter', ...
        'TolFun', 1e-6, ...
        'TolX', 1e-7);

    % Run optimization
    fprintf("\nStarting optimization...\n");
    [x_opt, resnorm, residual, exitflag, output] = lsqnonlin(...
        @(x) residual_fn(x, t_fit, throt_fit, speed_fit, accel_fit, drag_k0, drag_k1, dt(valid_idx)), ...
        x0, lower, upper, opts);

    % Extract results
    fitted_params = struct();
    fitted_params.A = x_opt(1);
    fitted_params.throttle_tau_s = x_opt(2);
    fitted_params.back_emf_gain = x_opt(3);
    fitted_params.deadband_us = x_opt(4);
    fitted_params.n = x_opt(5);
    fitted_params.drag_k0 = drag_k0;
    fitted_params.drag_k1 = drag_k1;
    fitted_params.resnorm = resnorm;
    fitted_params.rmse = sqrt(resnorm / numel(residual));

    % Print results
    fprintf("\n=== FITTED PARAMETERS ===\n");
    fprintf("A (max_accel_fwd)       = %.4f m/s^2\n", fitted_params.A);
    fprintf("tau (throttle_tau_s)    = %.4f s\n", fitted_params.throttle_tau_s);
    fprintf("k_e (back_emf_gain)     = %.4f s/m\n", fitted_params.back_emf_gain);
    fprintf("deadband                = %.1f us\n", fitted_params.deadband_us);
    fprintf("n (throttle exponent)   = %.4f\n", fitted_params.n);
    fprintf("RMSE (fitted model)     = %.4f m/s^2\n", fitted_params.rmse);
    fprintf("\nFitted drag (fixed):\n");
    fprintf("  k0 = %.4f m/s^2\n", drag_k0);
    fprintf("  k1 = %.4f 1/s\n", drag_k1);

    fprintf("\n=== CODE TO UPDATE throttle_pwm_to_accel.m ===\n");
    fprintf("Replace the defaults in throttle_pwm_to_accel.m function header:\n\n");
    fprintf("    if ~isfield(cfg, ""max_accel_fwd"")\n");
    fprintf("        cfg.max_accel_fwd = %.6f;  %% fitted\n", fitted_params.A);
    fprintf("    end\n");
    fprintf("    if ~isfield(cfg, ""throttle_tau_s"")\n");
    fprintf("        cfg.throttle_tau_s = %.6f;  %% fitted\n", fitted_params.throttle_tau_s);
    fprintf("    end\n");
    fprintf("    if ~isfield(cfg, ""back_emf_gain"")\n");
    fprintf("        cfg.back_emf_gain = %.6f;   %% fitted\n", fitted_params.back_emf_gain);
    fprintf("    end\n");
    fprintf("    if ~isfield(cfg, ""deadband_us"")\n");
    fprintf("        cfg.deadband_us = %.1f;        %% fitted\n", fitted_params.deadband_us);
    fprintf("    end\n");
    fprintf("    if ~isfield(cfg, ""n_throttle"")\n");
    fprintf("        cfg.n_throttle = %.6f;     %% fitted (optional: add if not present)\n", fitted_params.n);
    fprintf("    end\n\n");

    % Plot comparison
    figure("Name", "Throttle Model Fit Validation", "Color", "w");
    
    % Predict acceleration for all fit samples
    accel_pred = predict_accel(x_opt, throt_fit, speed_fit, dt(valid_idx), drag_k0, drag_k1);

    subplot(2, 2, 1);
    plot(t_fit, accel_fit, 'b-', 'LineWidth', 1.2); hold on;
    plot(t_fit, accel_pred, 'r--', 'LineWidth', 1.0);
    grid on; xlabel('Time [s]'); ylabel('Accel [m/s^2]');
    title('Measured vs Predicted Acceleration');
    legend('Measured', 'Fitted model');

    subplot(2, 2, 2);
    scatter(accel_fit, accel_pred, 16, 'filled', 'MarkerFaceAlpha', 0.5); hold on;
    lims = [min([accel_fit; accel_pred]), max([accel_fit; accel_pred])];
    plot(lims, lims, 'k--', 'LineWidth', 1);
    grid on; xlabel('Measured [m/s^2]'); ylabel('Predicted [m/s^2]');
    title('Predicted vs Measured (Scatter)');
    axis equal;

    subplot(2, 2, 3);
    err = accel_fit - accel_pred;
    plot(t_fit, err, 'g.', 'MarkerSize', 4); hold on;
    yline(0, 'k--');
    grid on; xlabel('Time [s]'); ylabel('Error [m/s^2]');
    title(sprintf('Residual (RMSE = %.3f m/s^2)', fitted_params.rmse));

    subplot(2, 2, 4);
    scatter(throt_fit, accel_fit, 16, t_fit, 'filled', 'MarkerFaceAlpha', 0.6);
    colormap("turbo"); colorbar; caxis([t_fit(1) t_fit(end)]);
    grid on; xlabel('Throttle [us]'); ylabel('Accel [m/s^2]');
    title('Accel vs Throttle (colored by time)');

    sgtitle(sprintf('Throttle Model Fit: %s', csv_file), 'Interpreter', 'none');

end

function residuals = residual_fn(x, t_fit, throt_fit, speed_fit, accel_fit, k0, k1, dt_fit)
    % Compute model prediction and return residuals.
    accel_pred = predict_accel(x, throt_fit, speed_fit, dt_fit, k0, k1);
    residuals = accel_fit - accel_pred;
end

function accel = predict_accel(x, throttle_us, speed_mps, dt, drag_k0, drag_k1)
    % Forward model: predict acceleration given throttle and speed.
    % x = [A, tau, k_e, deadband_us, n]
    
    A = x(1);
    tau = x(2);
    k_e = x(3);
    deadband_us = x(4);
    n = x(5);

    N = numel(throttle_us);
    accel = zeros(N, 1);
    u_filt = 0;

    for k = 1:N
        % Throttle command
        u = (throttle_us(k) - 1500) / 500;
        u = max(u, 0);  % forward-only

        % Deadband
        db = deadband_us / 500;
        if abs(u) <= db
            u_eff = 0;
        else
            u_eff = sign(u) * ((abs(u) - db) / (1 - db));
        end
        u_eff = clamp(u_eff, 0, 1);

        % First-order lag
        if tau > 1e-6 && k > 1
            dt_k = dt(k);
            if ~isfinite(dt_k) || dt_k <= 0
                dt_k = 0.01;
            end
            alpha = exp(-dt_k / tau);
            u_filt = alpha * u_filt + (1 - alpha) * u_eff;
        else
            u_filt = u_eff;
        end
        u_filt = clamp(u_filt, 0, 1);

        % Drive acceleration with back-EMF
        v_fwd = max(speed_mps(k), 0);
        emf_scale = 1 / (1 + k_e * v_fwd);
        a_drive = A * (u_filt^n) * emf_scale;

        % Drag
        drag = drag_k0 + drag_k1 * v_fwd;

        % Net acceleration (clamp to prevent negative velocity slip)
        v_epsilon = 0.03;
        if v_fwd > v_epsilon
            accel(k) = a_drive - drag;
        else
            accel(k) = max(a_drive - drag, 0);
        end
    end
end

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end

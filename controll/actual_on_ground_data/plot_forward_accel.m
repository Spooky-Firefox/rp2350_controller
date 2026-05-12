function data = plot_forward_accel(csv_file, length_per_rotation_m, remove_segments_s, plot_slice_s)
% Plot speed, derived position, acceleration, and forward-throttle segments from a drive log CSV.
%
% Usage:
%   plot_forward_accel("log_20260509_111211.csv")
%   plot_forward_accel(csv_file, 0.0303)
%   plot_forward_accel(csv_file, 0.0303, [3.0 4.2; 8.5 9.0])
%   plot_forward_accel(csv_file, 0.0303, [3.0 4.2], [2.0 12.0])
%
% Inputs:
%   csv_file               Path to CSV with columns:
%                          timestamp_us, steer_us, throttle_us, speed_mps
%   length_per_rotation_m  Distance traveled per logged wheel rotation [m]
%                          (default matches measured constant: 0.0303 m)
%   remove_segments_s      Optional Nx2 matrix of time segments [t_start t_end]
%                          in seconds (relative to log start) to remove.
%   plot_slice_s           Optional [t_start t_end] in seconds to keep only
%                          one time window for plotting and analysis.
%
% Output:
%   data struct with processed vectors for further analysis.

    if nargin < 1 || strlength(string(csv_file)) == 0
        csv_file = fullfile(fileparts(mfilename("fullpath")), ...
            "log_20260509_133305.csv");
    end

    if nargin < 2 || isempty(length_per_rotation_m)
        % Measured travel per data point.
        length_per_rotation_m = 0.0303;
    end

    if nargin < 3 || isempty(remove_segments_s)
        remove_segments_s = zeros(0, 2);
    end

    if nargin < 4 || isempty(plot_slice_s)
        plot_slice_s = [];
    end

    T = readtable(csv_file);

    required = ["timestamp_us", "steer_us", "throttle_us", "speed_mps"];
    missing = required(~ismember(required, string(T.Properties.VariableNames)));
    if ~isempty(missing)
        error("Missing CSV columns: %s", strjoin(missing, ", "));
    end

    t_s = (double(T.timestamp_us) - double(T.timestamp_us(1))) * 1e-6;
    speed_mps = double(T.speed_mps);
    throttle_us = double(T.throttle_us);
    steer_us = double(T.steer_us);

    if ~ismatrix(remove_segments_s) || size(remove_segments_s, 2) ~= 2
        error("remove_segments_s must be an Nx2 matrix: [t_start t_end; ...]");
    end
    if any(any(~isfinite(remove_segments_s)))
        error("remove_segments_s must contain only finite values.");
    end

    if ~isempty(plot_slice_s)
        if ~isvector(plot_slice_s) || numel(plot_slice_s) ~= 2 || any(~isfinite(plot_slice_s))
            error("plot_slice_s must be [t_start t_end].");
        end
        plot_slice_s = sort(plot_slice_s(:)).';
    end

    if ~isempty(remove_segments_s)
        remove_segments_s = sort(remove_segments_s, 2);
        keep_mask = true(size(t_s));
        for i = 1:size(remove_segments_s, 1)
            t0 = remove_segments_s(i, 1);
            t1 = remove_segments_s(i, 2);
            keep_mask = keep_mask & ~(t_s >= t0 & t_s <= t1);
        end

        t_s = t_s(keep_mask);
        speed_mps = speed_mps(keep_mask);
        throttle_us = throttle_us(keep_mask);
        steer_us = steer_us(keep_mask);
    end

    if ~isempty(plot_slice_s)
        keep_slice = t_s >= plot_slice_s(1) & t_s <= plot_slice_s(2);
        t_s = t_s(keep_slice);
        speed_mps = speed_mps(keep_slice);
        throttle_us = throttle_us(keep_slice);
        steer_us = steer_us(keep_slice);
    end

    n = numel(t_s);
    if n < 3
        error("Not enough samples left after removing segments.");
    end

    position_m = (0:n-1)' * length_per_rotation_m;

    dt = [NaN; diff(t_s)];
    accel_raw_mps2 = [NaN; diff(speed_mps) ./ dt(2:end)];

    median_dt = median(dt(2:end), "omitnan");
    if ~isfinite(median_dt) || median_dt <= 0
        span_samples = 7;
    else
        span_samples = max(5, round(0.20 / median_dt));
    end
    accel_mps2 = movmean(accel_raw_mps2, span_samples, "omitnan");

    speed_from_rot_mps = [NaN; length_per_rotation_m ./ dt(2:end)];
    throttle_neutral_us = 20;
    min_forward_fit_speed_mps = 1.0;
    forward_mask = throttle_us >= 1500 + throttle_neutral_us;
    speed_forward_mps = speed_mps;
    speed_forward_mps(~forward_mask) = NaN;
    forward_segments = mask_to_segments(t_s, forward_mask);

    gap_threshold_s = max(3 * median_dt, 0.20);
    if ~isfinite(gap_threshold_s) || gap_threshold_s <= 0
        gap_threshold_s = 0.20;
    end

    [t_plot, speed_plot] = insert_nans_at_gaps(t_s, speed_mps, gap_threshold_s);
    [~, speed_rot_plot] = insert_nans_at_gaps(t_s, speed_from_rot_mps, gap_threshold_s);
    [~, position_plot] = insert_nans_at_gaps(t_s, position_m, gap_threshold_s);
    [~, steer_plot] = insert_nans_at_gaps(t_s, steer_us, gap_threshold_s);
    [~, throttle_plot] = insert_nans_at_gaps(t_s, throttle_us, gap_threshold_s);
    [~, accel_plot] = insert_nans_at_gaps(t_s, accel_mps2, gap_threshold_s);
    [~, speed_forward_plot] = insert_nans_at_gaps(t_s, speed_forward_mps, gap_threshold_s);
    forward_skip_s = 0.130;
    forward_fits = fit_forward_segments(t_s, speed_mps, throttle_us, forward_segments, min_forward_fit_speed_mps, forward_skip_s);

    figure("Name", "Drive Log Analysis", "Color", "w");
    tiledlayout(3, 1, "TileSpacing", "compact", "Padding", "compact");

    nexttile;
    plot(t_plot, speed_plot, "LineWidth", 1.3);
    hold on;
    plot(t_plot, speed_rot_plot, "--", "LineWidth", 1.0);
    grid on;
    ylabel("Speed [m/s]");
    legend("Logged speed", "Speed from rotation timing", "Location", "best");
    title(sprintf("%s", csv_file), "Interpreter", "none", "FontSize", 10);

    nexttile;
    plot(t_plot, position_plot, "LineWidth", 1.3);
    grid on;
    ylabel("Position [m]");
    title("Position (integrated as one rotation per sample)");

    nexttile;
    plot(t_plot, steer_plot, "LineWidth", 1.0);
    hold on;
    plot(t_plot, throttle_plot, "LineWidth", 1.0);
    grid on;
    ylabel("PWM [us]");
    xlabel("Time [s]");
    legend("Steering", "Throttle", "Location", "best");
    title("Control commands");

    figure("Name", "Acceleration and Throttle vs Time", "Color", "w");
    yyaxis left;
    h_acc = plot(t_plot, accel_plot, "LineWidth", 1.4, "Color", [0.00 0.45 0.74]);
    ylabel("Acceleration [m/s^2]");
    yline(0, ":", "Zero accel", "LabelVerticalAlignment", "bottom");

    yyaxis right;
    hold on;
    h_thr = plot(t_plot, throttle_plot, "LineWidth", 1.2, "Color", [0.85 0.33 0.10]);
    ylabel("Throttle [us]");
    yline(1500, "--", "Throttle neutral", "LabelVerticalAlignment", "bottom");
    grid on;
    xlabel("Time [s]");
    title("Acceleration and Throttle Over Time");
    legend([h_acc h_thr], {"Acceleration", "Throttle"}, "Location", "best");

    figure("Name", "Forward Speed and Throttle vs Time", "Color", "w");
    yyaxis left;
    h_spd = plot(t_plot, speed_plot, "LineWidth", 1.4, "Color", [0.13 0.55 0.13]);
    ylabel("Speed [m/s]");
    yline(0, ":", "Zero speed", "LabelVerticalAlignment", "bottom");
    hold on;
    forward_handles = gobjects(0);
    forward_labels = strings(0);
    h_forward = plot(t_plot, speed_forward_plot, "-", "LineWidth", 1.6, "Color", [0.60 0.60 0.60]);
    for i = 1:numel(forward_fits)
        if forward_fits(i).ok
            forward_handles(end+1) = plot(forward_fits(i).t_line, forward_fits(i).y_line, "b-", "LineWidth", 2.2); %#ok<AGROW>
            forward_labels(end+1) = sprintf("Forward fit #%d (v >= %.1f m/s, +%.0fms skip, throttle fit ~ %.0f us)", ...
                i, min_forward_fit_speed_mps, forward_skip_s*1000, forward_fits(i).fit_throttle_us); %#ok<AGROW>
        end
    end

    yyaxis right;
    hold on;
    h_thr2 = plot(t_plot, throttle_plot, "LineWidth", 1.2, "Color", [0.85 0.33 0.10]);
    ylabel("Throttle [us]");
    yline(1500, "--", "Throttle neutral", "LabelVerticalAlignment", "bottom");
    grid on;
    xlabel("Time [s]");
    title("Forward Speed and Throttle Over Time");
    if isempty(forward_handles)
        legend([h_spd h_forward h_thr2], {"Speed", "Forward speed (throttle high)", "Throttle"}, "Location", "best");
    else
        legend([h_spd h_forward forward_handles(:).' h_thr2], ["Speed", "Forward speed (throttle high)", forward_labels(:).', "Throttle"], "Location", "best");
    end

    forward_valid = [forward_fits.ok] & isfinite([forward_fits.fit_throttle_us]) & isfinite([forward_fits.slope]);
    forward_throttle_us = [forward_fits.fit_throttle_us];
    forward_speed_slopes = [forward_fits.slope];
    forward_throttle_us = forward_throttle_us(forward_valid);
    forward_speed_slopes = forward_speed_slopes(forward_valid);

    figure("Name", "Forward Slope vs Fit Throttle", "Color", "w");
    hold on;
    grid on;
    scatter(forward_throttle_us, forward_speed_slopes, 42, "filled", "MarkerFaceColor", [0.20 0.20 0.20]);
    xlabel("Throttle during fit window [us]");
    ylabel("Speed slope [m/s^2]");
    title("Forward Acceleration vs Fit-Window Throttle");

    if numel(forward_throttle_us) >= 2
        p_forward = polyfit(forward_throttle_us, forward_speed_slopes, 1);
        x_line = linspace(min(forward_throttle_us), max(forward_throttle_us), 100).';
        y_line = polyval(p_forward, x_line);
        plot(x_line, y_line, "r-", "LineWidth", 1.5);
        legend("Forward fits", sprintf("Linear trend: %.4g x %+.4g", p_forward(1), p_forward(2)), "Location", "best");
    else
        legend("Forward fits", "Location", "best");
    end

    data = struct();
    data.time_s = t_s;
    data.speed_mps = speed_mps;
    data.speed_from_rotation_mps = speed_from_rot_mps;
    data.speed_forward_mps = speed_forward_mps;
    data.position_m = position_m;
    data.accel_mps2 = accel_mps2;
    data.throttle_us = throttle_us;
    data.steer_us = steer_us;
    data.length_per_rotation_m = length_per_rotation_m;
    data.remove_segments_s = remove_segments_s;
    data.plot_slice_s = plot_slice_s;
    data.gap_threshold_s = gap_threshold_s;
    data.throttle_neutral_us = throttle_neutral_us;
    data.min_forward_fit_speed_mps = min_forward_fit_speed_mps;
    data.forward_segments = forward_segments;
    data.forward_fits = forward_fits;
    data.forward_throttle_us = forward_throttle_us;
    data.forward_speed_slopes = forward_speed_slopes;
    data.csv_file = csv_file;

    fprintf("\n--- Forward Fits ---\n");
    if isempty(forward_fits)
        fprintf("No forward-throttle segments found.\n");
    end
    for i = 1:numel(forward_fits)
        if forward_fits(i).ok
            fprintf("Forward fit #%d [%.3f, %.3f] s (v >= %.1f m/s, throttle_fit = %.1f us [%.0f..%.0f]): speed slope = %.6f m/s^2, R^2 = %.4f\n", ...
                i, forward_fits(i).t0, forward_fits(i).t1, min_forward_fit_speed_mps, ...
                forward_fits(i).fit_throttle_us, forward_fits(i).fit_throttle_min_us, forward_fits(i).fit_throttle_max_us, ...
                forward_fits(i).slope, forward_fits(i).r2);
        end
    end
    if numel(forward_throttle_us) >= 2
        p_forward = polyfit(forward_throttle_us, forward_speed_slopes, 1);
        fprintf("Forward slope vs fit-window throttle trend: slope = %.6g (m/s^2)/us, intercept = %.6g m/s^2\n", p_forward(1), p_forward(2));
    end

end

function [t_out, y_out] = insert_nans_at_gaps(t_in, y_in, gap_threshold_s)
    if numel(t_in) ~= numel(y_in)
        error("insert_nans_at_gaps requires vectors with equal length.");
    end

    t_out = t_in(1);
    y_out = y_in(1);

    for k = 2:numel(t_in)
        if (t_in(k) - t_in(k-1)) > gap_threshold_s
            t_out(end+1,1) = NaN; %#ok<AGROW>
            y_out(end+1,1) = NaN; %#ok<AGROW>
        end
        t_out(end+1,1) = t_in(k); %#ok<AGROW>
        y_out(end+1,1) = y_in(k); %#ok<AGROW>
    end
end

function segments = mask_to_segments(t_s, mask)
    if numel(t_s) ~= numel(mask)
        error("mask_to_segments requires vectors with equal length.");
    end

    mask = mask(:) ~= 0;
    t_s = t_s(:);
    edges = diff([false; mask; false]);
    start_idx = find(edges == 1);
    end_idx = find(edges == -1) - 1;

    segments = zeros(numel(start_idx), 2);
    for i = 1:numel(start_idx)
        segments(i, :) = [t_s(start_idx(i)), t_s(end_idx(i))];
    end
end

function fits = fit_forward_segments(t_s, speed_mps, throttle_us, segments, min_speed_mps, skip_s)
    if nargin < 6 || isempty(skip_s)
        skip_s = 0;
    end
    fits = repmat(struct( ...
        "ok", false, "t0", NaN, "t1", NaN, "slope", NaN, "intercept", NaN, ...
        "r2", NaN, "t_line", [], "y_line", [], "fit_t0", NaN, "fit_t1", NaN, ...
        "pre_throttle_us", NaN, "fit_throttle_us", NaN, ...
        "fit_throttle_min_us", NaN, "fit_throttle_max_us", NaN), size(segments, 1), 1);

    for i = 1:size(segments, 1)
        t0 = segments(i, 1);
        t1 = segments(i, 2);
        fit_start = t0 + skip_s;
        in_seg = t_s >= fit_start & t_s <= t1 & isfinite(speed_mps) & speed_mps >= min_speed_mps & isfinite(throttle_us);
        t = t_s(in_seg);
        y = speed_mps(in_seg);
        thr = throttle_us(in_seg);

        pre_idx = find(t_s < t0 & isfinite(throttle_us), 1, "last");

        if numel(t) < 3
            continue;
        end

        p = polyfit(t, y, 1);
        y_hat = polyval(p, t);
        ss_res = sum((y - y_hat).^2);
        ss_tot = sum((y - mean(y)).^2);
        if ss_tot <= 0
            r2 = NaN;
        else
            r2 = 1 - ss_res / ss_tot;
        end

        fits(i).ok = true;
        fits(i).t0 = t0;
        fits(i).t1 = t1;
        fits(i).slope = p(1);
        fits(i).intercept = p(2);
        fits(i).r2 = r2;
        fits(i).fit_t0 = t(1);
        fits(i).fit_t1 = t(end);
        fits(i).t_line = [t(1); t(end)];
        fits(i).y_line = polyval(p, fits(i).t_line);
        if isempty(pre_idx)
            fits(i).pre_throttle_us = NaN;
        else
            fits(i).pre_throttle_us = throttle_us(pre_idx);
        end
        fits(i).fit_throttle_us = mean(thr, "omitnan");
        fits(i).fit_throttle_min_us = min(thr);
        fits(i).fit_throttle_max_us = max(thr);
    end
end

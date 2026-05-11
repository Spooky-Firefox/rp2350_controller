function data = plot_drive_log(csv_file, length_per_rotation_m, remove_segments_s, plot_slice_s)
% Plot speed, derived position, acceleration, and control commands from a drive log CSV.
%
% Usage:
%   plot_drive_log("log_20260509_111211.csv")
%   plot_drive_log(csv_file, 13*pi/600)
%   plot_drive_log(csv_file, 13*pi/600, [3.0 4.2; 8.5 9.0])
%   plot_drive_log(csv_file, 13*pi/600, [3.0 4.2], [2.0 12.0])
%
% Inputs:
%   csv_file               Path to CSV with columns:
%                          timestamp_us, steer_us, throttle_us, speed_mps
%   length_per_rotation_m  Distance traveled per logged wheel rotation [m]
%                          (default matches firmware constant: 13*pi/600)
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
        % Firmware default: LENGTH_PER_ENCODER_PULSE_METERS = 13*pi/600
        length_per_rotation_m = 13 * pi / 600;
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

    % Position derived from assumption: one log sample per wheel rotation.
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
    min_coast_fit_speed_mps = 1.0;
    coast_mask = abs(throttle_us - 1500) <= throttle_neutral_us;
    speed_coast_mps = speed_mps;
    speed_coast_mps(~coast_mask) = NaN;
    coast_segments = mask_to_segments(t_s, coast_mask);

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
    [~, speed_coast_plot] = insert_nans_at_gaps(t_s, speed_coast_mps, gap_threshold_s);
    coast_skip_s = 0.130;  % skip this many seconds after throttle drop (transient)
    coast_fits = fit_coast_segments(t_s, speed_mps, throttle_us, coast_segments, min_coast_fit_speed_mps, coast_skip_s);

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

    % figure("Name", "Acceleration vs Throttle", "Color", "w");
    % valid = isfinite(accel_mps2) & isfinite(throttle_us);
    % scatter(throttle_us(valid), accel_mps2(valid), 16, t_s(valid), "filled", ...
    %     "MarkerFaceAlpha", 0.70);
    % colormap("turbo");
    % cb = colorbar;
    % cb.Label.String = "Time [s]";
    % grid on;
    % xlabel("Throttle [us]");
    % ylabel("Acceleration [m/s^2]");
    % title("Direct Acceleration vs Throttle (No Time Axis)");

    figure("Name", "Speed and Throttle vs Time", "Color", "w");
    yyaxis left;
    h_spd = plot(t_plot, speed_plot, "LineWidth", 1.4, "Color", [0.13 0.55 0.13]);
    ylabel("Speed [m/s]");
    yline(0, ":", "Zero speed", "LabelVerticalAlignment", "bottom");
    hold on;
    coast_handles = gobjects(0);
    coast_labels = strings(0);
    h_coast = plot(t_plot, speed_coast_plot, "-", "LineWidth", 1.6, "Color", [0.60 0.60 0.60]);
    for i = 1:numel(coast_fits)
        if coast_fits(i).ok
            coast_handles(end+1) = plot(coast_fits(i).t_line, coast_fits(i).y_line, "b-", "LineWidth", 2.2); %#ok<AGROW>
            coast_labels(end+1) = sprintf("Coast fit #%d (v >= %.1f m/s, +%.0fms skip, throttle ~ %.0f us)", ...
                i, min_coast_fit_speed_mps, coast_skip_s*1000, coast_fits(i).pre_throttle_us); %#ok<AGROW>
        end
    end

    yyaxis right;
    hold on;
    h_thr2 = plot(t_plot, throttle_plot, "LineWidth", 1.2, "Color", [0.85 0.33 0.10]);
    ylabel("Throttle [us]");
    yline(1500, "--", "Throttle neutral", "LabelVerticalAlignment", "bottom");
    grid on;
    xlabel("Time [s]");
    title("Speed and Throttle Over Time");
    if isempty(coast_handles)
        legend([h_spd h_coast h_thr2], {"Speed", "Coast speed (throttle neutral)", "Throttle"}, "Location", "best");
    else
        legend([h_spd h_coast coast_handles(:).' h_thr2], ["Speed", "Coast speed (throttle neutral)", coast_labels(:).', "Throttle"], "Location", "best");
    end

    % figure("Name", "Speed vs Throttle", "Color", "w");
    % valid_spd = isfinite(speed_mps) & isfinite(throttle_us);
    % scatter(throttle_us(valid_spd), speed_mps(valid_spd), 16, t_s(valid_spd), "filled", ...
    %     "MarkerFaceAlpha", 0.70);
    % colormap("turbo");
    % cb2 = colorbar;
    % cb2.Label.String = "Time [s]";
    % grid on;
    % xlabel("Throttle [us]");
    % ylabel("Speed [m/s]");
    % title("Direct Speed vs Throttle (No Time Axis)");

    coast_valid = [coast_fits.ok] & isfinite([coast_fits.pre_throttle_us]) & isfinite([coast_fits.slope]);
    coast_throttle_us = [coast_fits.pre_throttle_us];
    coast_speed_slopes = [coast_fits.slope];
    coast_throttle_us = coast_throttle_us(coast_valid);
    coast_speed_slopes = coast_speed_slopes(coast_valid);

    figure("Name", "Coast Slope vs Pre-Coast Throttle", "Color", "w");
    hold on;
    grid on;
    scatter(coast_throttle_us, coast_speed_slopes, 42, "filled", "MarkerFaceColor", [0.20 0.20 0.20]);
    xlabel("Pre-coast throttle [us]");
    ylabel("Speed slope [m/s^2]");
    title("Coast Deceleration vs Pre-Coast Throttle");

    if numel(coast_throttle_us) >= 2
        p_coast = polyfit(coast_throttle_us, coast_speed_slopes, 1);
        x_line = linspace(min(coast_throttle_us), max(coast_throttle_us), 100).';
        y_line = polyval(p_coast, x_line);
        plot(x_line, y_line, "r-", "LineWidth", 1.5);
        legend("Coast fits", sprintf("Linear trend: %.4g x %+.4g", p_coast(1), p_coast(2)), "Location", "best");
    else
        legend("Coast fits", "Location", "best");
    end

    % --- Drag model: a = -k0 - k1*v ---
    % Method 1: instantaneous (v, a) pairs from all coast samples (noisy)
    coast_drag_mask = coast_mask & isfinite(accel_mps2) & isfinite(speed_mps) & speed_mps >= min_coast_fit_speed_mps;
    coast_v_all = speed_mps(coast_drag_mask);
    coast_a_all = accel_mps2(coast_drag_mask);
    coast_seg_id_all = zeros(size(coast_v_all));  % which segment each sample belongs to
    raw_idx = find(coast_drag_mask);
    for i = 1:size(coast_segments, 1)
        seg_mask_i = t_s(raw_idx) >= coast_segments(i,1) & t_s(raw_idx) <= coast_segments(i,2);
        coast_seg_id_all(seg_mask_i) = i;
    end

    drag_k0 = NaN; drag_k1 = NaN; drag_r2 = NaN;
    if numel(coast_v_all) >= 4
        X_drag = [ones(numel(coast_v_all), 1), coast_v_all(:)];
        neg_a = -coast_a_all(:);
        p_drag = X_drag \ neg_a;
        drag_k0 = p_drag(1);
        drag_k1 = p_drag(2);
        neg_a_hat = X_drag * p_drag;
        ss_res = sum((neg_a - neg_a_hat).^2);
        ss_tot = sum((neg_a - mean(neg_a)).^2);
        if ss_tot > 0
            drag_r2 = 1 - ss_res / ss_tot;
        end
    end

    % Method 2: segment-wise (mean_v, -slope) pairs — more robust
    seg_mean_v = NaN(numel(coast_fits), 1);
    seg_decel  = NaN(numel(coast_fits), 1);
    for i = 1:numel(coast_fits)
        if coast_fits(i).ok
            in_seg = t_s >= coast_fits(i).t0 & t_s <= coast_fits(i).t1 & ...
                     isfinite(speed_mps) & speed_mps >= min_coast_fit_speed_mps;
            seg_mean_v(i) = mean(speed_mps(in_seg));
            seg_decel(i)  = -coast_fits(i).slope;
        end
    end
    valid_seg = isfinite(seg_mean_v) & isfinite(seg_decel);
    drag_seg_k0 = NaN; drag_seg_k1 = NaN; drag_seg_r2 = NaN;
    if sum(valid_seg) >= 2
        X_seg = [ones(sum(valid_seg),1), seg_mean_v(valid_seg)];
        p_seg = X_seg \ seg_decel(valid_seg);
        drag_seg_k0 = p_seg(1);
        drag_seg_k1 = p_seg(2);
        y_seg_hat = X_seg * p_seg;
        ss_res_s = sum((seg_decel(valid_seg) - y_seg_hat).^2);
        ss_tot_s = sum((seg_decel(valid_seg) - mean(seg_decel(valid_seg))).^2);
        if ss_tot_s > 0
            drag_seg_r2 = 1 - ss_res_s / ss_tot_s;
        end
    end

    figure("Name", "Drag Model: Decel vs Speed", "Color", "w");
    hold on; grid on;
    % Scatter instantaneous samples colored by segment
    seg_colors = lines(max(coast_seg_id_all));
    unique_segs = unique(coast_seg_id_all(coast_seg_id_all > 0));
    h_inst = gobjects(numel(unique_segs), 1);
    for si = 1:numel(unique_segs)
        idx_si = coast_seg_id_all == unique_segs(si);
        h_inst(si) = scatter(coast_v_all(idx_si), -coast_a_all(idx_si), 14, ...
            seg_colors(unique_segs(si),:), "filled", "MarkerFaceAlpha", 0.4);
    end
    % Per-segment mean points
    h_seg_pts = scatter(seg_mean_v(valid_seg), seg_decel(valid_seg), 80, "k", "filled", "Marker", "^");
    xlabel("Speed [m/s]");
    ylabel("Deceleration [m/s^2]");
    title("Coast Deceleration vs Speed — Drag Model Fit  (a = -k_0 - k_1 v)");
    v_range = linspace(min(coast_v_all), max(coast_v_all), 200).';
    leg_handles = [h_inst(:); h_seg_pts];
    leg_labels  = arrayfun(@(s) sprintf("Seg %d samples", s), unique_segs, "UniformOutput", false);
    leg_labels{end+1} = "Segment mean v";
    if isfinite(drag_k0)
        h_fit1 = plot(v_range, drag_k0 + drag_k1*v_range, "b--", "LineWidth", 1.5);
        leg_handles(end+1) = h_fit1;
        leg_labels{end+1} = sprintf("Instantaneous fit: k_0=%.3g, k_1=%.3g, R²=%.3f", drag_k0, drag_k1, drag_r2);
    end
    if isfinite(drag_seg_k0)
        h_fit2 = plot(v_range, drag_seg_k0 + drag_seg_k1*v_range, "r-", "LineWidth", 2.0);
        leg_handles(end+1) = h_fit2;
        leg_labels{end+1} = sprintf("Segment fit:       k_0=%.3g, k_1=%.3g, R²=%.3f", drag_seg_k0, drag_seg_k1, drag_seg_r2);
    end
    legend(leg_handles, leg_labels, "Location", "best");

    data = struct();
    data.time_s = t_s;
    data.speed_mps = speed_mps;
    data.speed_from_rotation_mps = speed_from_rot_mps;
    data.speed_coast_mps = speed_coast_mps;
    data.position_m = position_m;
    data.accel_mps2 = accel_mps2;
    data.throttle_us = throttle_us;
    data.steer_us = steer_us;
    data.length_per_rotation_m = length_per_rotation_m;
    data.remove_segments_s = remove_segments_s;
    data.plot_slice_s = plot_slice_s;
    data.gap_threshold_s = gap_threshold_s;
    data.throttle_neutral_us = throttle_neutral_us;
    data.min_coast_fit_speed_mps = min_coast_fit_speed_mps;
    data.coast_segments = coast_segments;
    data.coast_fits = coast_fits;
    data.coast_throttle_us = coast_throttle_us;
    data.coast_speed_slopes = coast_speed_slopes;
    data.coast_slope_throttle_trend = [];
    data.drag_k0 = drag_k0;
    data.drag_k1 = drag_k1;
    data.drag_r2 = drag_r2;
    data.drag_seg_k0 = drag_seg_k0;
    data.drag_seg_k1 = drag_seg_k1;
    data.drag_seg_r2 = drag_seg_r2;
    data.csv_file = csv_file;

    fprintf("\n--- Coast Fits ---\n");
    if isempty(coast_fits)
        fprintf("No neutral-throttle coast segments found.\n");
    end
    for i = 1:numel(coast_fits)
        if coast_fits(i).ok
            fprintf("Coast fit #%d [%.3f, %.3f] s (v >= %.1f m/s, pre-coast throttle = %.0f us): speed slope = %.6f m/s^2, decel = %.6f m/s^2, R^2 = %.4f\n", ...
                i, coast_fits(i).t0, coast_fits(i).t1, min_coast_fit_speed_mps, coast_fits(i).pre_throttle_us, coast_fits(i).slope, -coast_fits(i).slope, coast_fits(i).r2);
        end
    end
    if numel(coast_throttle_us) >= 2
        p_coast = polyfit(coast_throttle_us, coast_speed_slopes, 1);
        data.coast_slope_throttle_trend = p_coast;
        fprintf("Coast slope vs pre-coast throttle trend: slope = %.6g (m/s^2)/us, intercept = %.6g m/s^2\n", p_coast(1), p_coast(2));
    end

    fprintf("\n--- Drag Model (a = -k0 - k1*v) ---\n");
    fprintf("  Method 1 — instantaneous samples (%d points, noisy):\n", numel(coast_v_all));
    if isfinite(drag_k0)
        fprintf("    k0 (rolling resistance) = %.6g m/s^2\n", drag_k0);
        fprintf("    k1 (viscous drag)       = %.6g s^-1\n", drag_k1);
        fprintf("    R^2                     = %.4f  (low R^2 expected if noise >> k1*dv)\n", drag_r2);
    else
        fprintf("    Not enough samples.\n");
    end
    fprintf("  Method 2 — segment mean speed vs segment slope (%d segments):\n", sum(valid_seg));
    if isfinite(drag_seg_k0)
        fprintf("    k0 (rolling resistance) = %.6g m/s^2\n", drag_seg_k0);
        fprintf("    k1 (viscous drag)       = %.6g s^-1\n", drag_seg_k1);
        fprintf("    R^2                     = %.4f\n", drag_seg_r2);
    else
        fprintf("    Not enough segments.\n");
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

function fits = fit_coast_segments(t_s, speed_mps, throttle_us, segments, min_speed_mps, skip_s)
    if nargin < 6 || isempty(skip_s)
        skip_s = 0;
    end
    fits = repmat(struct( ...
        "ok", false, "t0", NaN, "t1", NaN, "slope", NaN, "intercept", NaN, ...
        "r2", NaN, "t_line", [], "y_line", [], "fit_t0", NaN, "fit_t1", NaN, ...
        "pre_throttle_us", NaN), size(segments, 1), 1);

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
    end
end

function data = plot_camera_degrees(csv_file, time_window_s, opts)
% Plot camera angle (degrees) over time and its distribution.
%
% Usage:
%   plot_camera_degrees
%   plot_camera_degrees("data/camera_alignment_1778588231549(1).csv")
%   plot_camera_degrees("data/camera_alignment_1778588231549(1).csv", [2.0 34.0])
%   plot_camera_degrees("data/camera_alignment_1778588231549(1).csv", [2.0 34.0], ...
%       struct("min_confidence", 0.35, "max_outlier_lines", 0))
%
% Inputs:
%   csv_file      Path to CSV file.
%   time_window_s Optional [t_start t_end] in seconds relative to log start.
%   opts          Optional struct with filters:
%                 - axis               (default: "vertical")
%                 - min_confidence     (default: 0.25)
%                 - max_outlier_lines  (default: 0)
%                 - max_abs_angle_deg  (default: 30)
%                 - smooth_window_s    (default: 0.25)
%                 - histogram_bins     (default: 40)
%
% Output:
%   data struct with raw/filtered vectors and applied settings.

    if nargin < 1 || strlength(string(csv_file)) == 0
        csv_file = fullfile(fileparts(mfilename("fullpath")), ...
            "data", "camera_alignment_1778588231549(1).csv");
    end

    if nargin < 2 || isempty(time_window_s)
        time_window_s = [];
    end

    if nargin < 3 || isempty(opts)
        opts = struct();
    end

    opts = apply_defaults(opts, struct( ...
        "axis", "vertical", ...
        "min_confidence", 0.25, ...
        "max_outlier_lines", 0, ...
        "max_abs_angle_deg", 30, ...
        "smooth_window_s", 0.25, ...
        "histogram_bins", 40));

    T = readtable(csv_file);

    required = [ ...
        "timestamp_us", ...
        "axis", ...
        "angle_from_vertical_deg", ...
        "confidence", ...
        "outlier_lines"];

    missing = required(~ismember(required, string(T.Properties.VariableNames)));
    if ~isempty(missing)
        error("Missing CSV columns: %s", strjoin(missing, ", "));
    end

    t_s = (double(T.timestamp_us) - double(T.timestamp_us(1))) * 1e-6;
    axis_col = string(T.axis);
    angle_deg = double(T.angle_from_vertical_deg);
    confidence = double(T.confidence);
    outlier_lines = double(T.outlier_lines);

    if ~isempty(time_window_s)
        if ~isvector(time_window_s) || numel(time_window_s) ~= 2 || any(~isfinite(time_window_s))
            error("time_window_s must be [t_start t_end].");
        end
        time_window_s = sort(time_window_s(:)).';
    end

    valid = isfinite(t_s) & isfinite(angle_deg) & isfinite(confidence) & isfinite(outlier_lines);

    if strlength(string(opts.axis)) > 0
        valid = valid & strcmpi(axis_col, string(opts.axis));
    end

    if ~isempty(opts.min_confidence)
        valid = valid & (confidence >= opts.min_confidence);
    end

    if ~isempty(opts.max_outlier_lines)
        valid = valid & (outlier_lines <= opts.max_outlier_lines);
    end

    if ~isempty(opts.max_abs_angle_deg)
        valid = valid & (abs(angle_deg) <= opts.max_abs_angle_deg);
    end

    if ~isempty(time_window_s)
        valid = valid & (t_s >= time_window_s(1)) & (t_s <= time_window_s(2));
    end

    t_keep = t_s(valid);
    angle_keep = angle_deg(valid);
    conf_keep = confidence(valid);

    if numel(t_keep) < 3
        error("Not enough samples left after filtering. Loosen filters in opts.");
    end

    dt_med = median(diff(t_keep), "omitnan");
    if ~isfinite(dt_med) || dt_med <= 0
        smooth_n = 7;
    else
        smooth_n = max(3, round(opts.smooth_window_s / dt_med));
    end
    smooth_angle = movmedian(angle_keep, smooth_n, "omitnan");

    removed = numel(t_s) - numel(t_keep);
    fprintf("Rows total: %d\n", numel(t_s));
    fprintf("Rows kept:  %d\n", numel(t_keep));
    fprintf("Rows removed by filters/time window: %d\n", removed);
    fprintf("Angle mean/std after filtering: %.3f / %.3f deg\n", ...
        mean(angle_keep, "omitnan"), std(angle_keep, "omitnan"));

    figure("Name", "Camera Angle Analysis", "Color", "w");
    tiledlayout(2, 1, "TileSpacing", "compact", "Padding", "compact");

    nexttile;
    plot(t_keep, angle_keep, ".", "MarkerSize", 7, "Color", [0.25 0.45 0.80]);
    hold on;
    plot(t_keep, smooth_angle, "-", "LineWidth", 1.6, "Color", [0.85 0.20 0.15]);
    yline(0, ":", "0 deg", "LabelVerticalAlignment", "bottom");
    grid on;
    xlabel("Time [s]");
    ylabel("Angle [deg]");
    title(sprintf("Angle vs Time (axis=%s)", string(opts.axis)), "Interpreter", "none");
    legend("Filtered samples", sprintf("Median smooth (%d samples)", smooth_n), "Location", "best");

    nexttile;
    histogram(angle_keep, opts.histogram_bins, "Normalization", "pdf", ...
        "FaceColor", [0.20 0.70 0.45], "FaceAlpha", 0.8);
    grid on;
    xlabel("Angle [deg]");
    ylabel("Probability density");
    title("Distribution of Filtered Angle Measurements");

    data = struct();
    data.csv_file = csv_file;
    data.time_window_s = time_window_s;
    data.options = opts;
    data.t_raw_s = t_s;
    data.angle_raw_deg = angle_deg;
    data.valid_mask = valid;
    data.t_filtered_s = t_keep;
    data.angle_filtered_deg = angle_keep;
    data.confidence_filtered = conf_keep;
    data.angle_smooth_deg = smooth_angle;
end

function out = apply_defaults(in, defaults)
    out = defaults;
    fn = fieldnames(in);
    for i = 1:numel(fn)
        out.(fn{i}) = in.(fn{i});
    end
end
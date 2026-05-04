function results = ultrasound_step_variance(csv_file, step_threshold_cm, settle_samples, outlier_threshold_cm)
% Compute per-step variance of ultrasound distance measurements.
%
% The input signal is assumed to be a staircase in distance, where each new
% plateau is at least ~10 cm away from the previous one. This function:
%   1) removes large outlier spikes,
%   2) detects step transitions,
%   3) discards settling samples after each transition,
%   4) computes variance for each stable plateau,
%   5) plots variance vs distance.
%
% Usage:
%   results = ultrasound_step_variance();
%   results = ultrasound_step_variance('data/data_ultraljud.csv');
%
% Inputs:
%   csv_file            - path to CSV with one column named "cm"
%   step_threshold_cm   - minimum step change [cm] used for transition detection
%   settle_samples      - samples ignored after each transition
%   outlier_threshold_cm- outlier threshold [cm] relative to local trend
%
% Output struct:
%   results.distance_cm   - mean distance for each stable step [cm]
%   results.variance_cm2  - variance for each stable step [cm^2]
%   results.segment_range - [start_idx, end_idx] for each retained stable segment

    if nargin < 1 || isempty(csv_file)
        csv_file = fullfile(fileparts(mfilename('fullpath')), 'data', 'data_ultraljud.csv');
    end
    if nargin < 2 || isempty(step_threshold_cm)
        step_threshold_cm = 10;
    end
    if nargin < 3 || isempty(settle_samples)
        settle_samples = 55;
    end
    if nargin < 4 || isempty(outlier_threshold_cm)
        outlier_threshold_cm = 10;
    end

    raw = readmatrix(csv_file);
    raw = raw(:);
    raw = raw(~isnan(raw));

    if isempty(raw)
        error('No numeric data found in %s', csv_file);
    end

    % Hard rule requested: remove early error samples above 100 cm.
    force_remove_mask = false(size(raw));
    force_remove_mask(1:min(1110, numel(raw))) = raw(1:min(1110, numel(raw))) > 100;

    % Remove spikes that are far from their local neighborhood.
    local_trend = movmedian(raw, 11);
    is_outlier = abs(raw - local_trend) > outlier_threshold_cm;
    is_outlier = is_outlier | force_remove_mask;
    clean = raw;
    clean(is_outlier) = NaN;

    % Use a gap-filled version only for transition detection.
    clean_for_detection = fillmissing(clean, 'linear', 'EndValues', 'nearest');

    % Smooth for robust step-transition detection.
    smooth_signal = movmedian(clean_for_detection, 21);

    % Compare samples separated in time so gradual transitions are still detected.
    delta_window = max(10, round(settle_samples / 2));
    delta = smooth_signal((1 + delta_window):end) - smooth_signal(1:(end - delta_window));
    transition_candidates = find(abs(delta) > step_threshold_cm) + floor(delta_window / 2);

    min_transition_gap = max(20, round(settle_samples / 2));
    transitions = cluster_transition_indices(transition_candidates, min_transition_gap);

    boundaries = [1; transitions(:) + 1; numel(clean) + 1];

    distance_cm = [];
    variance_cm2 = [];
    segment_range = [];

    for i = 1:(numel(boundaries) - 1)
        seg_start = boundaries(i);
        seg_end = boundaries(i + 1) - 1;

        % Ignore initial settling after each detected transition.
        if i == 1
            stable_start = seg_start;
        else
            stable_start = seg_start + settle_samples;
        end

        if stable_start > seg_end
            continue;
        end

        segment = clean(stable_start:seg_end);
        segment = segment(~isnan(segment));
        if numel(segment) < 10
            continue;
        end

        distance_cm(end + 1, 1) = mean(segment); %#ok<AGROW>
        variance_cm2(end + 1, 1) = var(segment, 0); %#ok<AGROW>
        segment_range(end + 1, :) = [stable_start, seg_end]; %#ok<AGROW>
    end

    if isempty(distance_cm)
        error('No stable segments detected. Try lowering step_threshold_cm or settle_samples.');
    end

    % Sort by distance so the variance curve is easier to interpret.
    [distance_cm, order] = sort(distance_cm);
    variance_cm2 = variance_cm2(order);
    segment_range = segment_range(order, :);

    figure('Name', 'Ultrasound Step Variance', 'Position', [150 150 1200 700]);

    subplot(2, 1, 1); hold on; grid on;
    plot(clean, 'b', 'LineWidth', 1.2, 'DisplayName', 'Signal used for variance');
    yl = ylim;
    y_low = yl(1);
    y_high = yl(2);
    for k = 1:numel(transitions)
        t0 = transitions(k);
        t1 = min(numel(clean), t0 + settle_samples);
        if k == 1
            patch([t0 t1 t1 t0], [y_low y_low y_high y_high], [1 0.4 0.4], ...
                'FaceAlpha', 0.18, 'EdgeColor', 'none', ...
                'DisplayName', 'Settling window after transition');
        else
            patch([t0 t1 t1 t0], [y_low y_low y_high y_high], [1 0.4 0.4], ...
                'FaceAlpha', 0.18, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        end
    end
    for k = 1:numel(transitions)
        xline(transitions(k), '--r', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    end
    ylim(yl);
    xlabel('Sample index');
    ylabel('Distance [cm]');
    title('Cleaned Signal with Transition and Settling-Time Bars');
    legend('Location', 'best');

    subplot(2, 1, 2); hold on; grid on;
    plot(distance_cm, variance_cm2, '-o', 'LineWidth', 1.4, ...
        'MarkerFaceColor', [0.2 0.5 0.9], 'MarkerSize', 6, ...
        'DisplayName', 'Measured variance');

    % Exponential regression: variance = a * exp(b * distance)
    fit_mask = isfinite(distance_cm) & isfinite(variance_cm2) & (variance_cm2 > 0);
    exclude_idx = [3, 10];
    exclude_idx = exclude_idx(exclude_idx >= 1 & exclude_idx <= numel(fit_mask));
    fit_mask(exclude_idx) = false;
    exp_model = struct('a', NaN, 'b', NaN, 'r2', NaN, 'n_points', 0, 'excluded_indices', exclude_idx);
    if nnz(fit_mask) >= 2
        x_fit = distance_cm(fit_mask);
        y_fit = variance_cm2(fit_mask);

        p = polyfit(x_fit, log(y_fit), 1);
        b = p(1);
        a = exp(p(2));

        y_hat_fit = a * exp(b * x_fit);
        ss_res = sum((y_fit - y_hat_fit).^2);
        ss_tot = sum((y_fit - mean(y_fit)).^2);
        if ss_tot > 0
            r2 = 1 - ss_res / ss_tot;
        else
            r2 = NaN;
        end

        x_line = linspace(min(distance_cm), max(distance_cm), 300);
        y_line = a * exp(b * x_line);
        plot(x_line, y_line, '--', 'Color', [0.9 0.2 0.2], 'LineWidth', 1.6, ...
            'DisplayName', sprintf('Exp fit: y = %.3g e^{%.3g x}, R^2 = %.3f', a, b, r2));

        exp_model.a = a;
        exp_model.b = b;
        exp_model.r2 = r2;
        exp_model.n_points = nnz(fit_mask);
    end

    xlabel('Distance [cm]');
    ylabel('Variance [cm^2]');
    title('Variance per Stable Distance Step');
    legend('Location', 'best');

    results = struct();
    results.distance_cm = distance_cm;
    results.variance_cm2 = variance_cm2;
    results.segment_range = segment_range;
    results.transitions = transitions;
    results.exp_model = exp_model;
end

function transitions = cluster_transition_indices(candidates, min_gap)
    if isempty(candidates)
        transitions = [];
        return;
    end

    candidates = candidates(:);
    keep = [true; diff(candidates) > min_gap];
    transitions = candidates(keep);
end

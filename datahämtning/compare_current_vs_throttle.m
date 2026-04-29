
%% Compare current to throttle with automatic time offset/scale alignment
% This script aligns two independently timestamped logs and then compares
% Current_mA against throttle_us around the 1500 us neutral point.

clear; clc; close all;

% Input files (adjust if needed)
scriptDir = fileparts(mfilename("fullpath"));
throttleFile = fullfile(scriptDir, "log_4.0.csv");
currentFile  = fullfile(scriptDir, "power4.0.csv");

% Read data tables
thrTbl = readtable(throttleFile, "VariableNamingRule", "preserve");
curTbl = readtable(currentFile, "VariableNamingRule", "preserve");

% Validate expected columns
requiredThrCols = ["timestamp_us", "throttle_us"];
requiredCurCols = ["Timestamp_ms", "Current_mA"];
assert(all(ismember(requiredThrCols, string(thrTbl.Properties.VariableNames))), ...
    "Throttle CSV must include columns: timestamp_us, throttle_us");
assert(all(ismember(requiredCurCols, string(curTbl.Properties.VariableNames))), ...
    "Current CSV must include columns: Timestamp_ms, Current_mA");

% Extract and normalize time to seconds from each file start
tThr = (double(thrTbl.timestamp_us) - double(thrTbl.timestamp_us(1))) / 1e6;
thr  = double(thrTbl.throttle_us);

tCur = (double(curTbl.Timestamp_ms) - double(curTbl.Timestamp_ms(1))) / 1e3;
cur  = double(curTbl.Current_mA * 8.0);  % mA * V => mW (power)

% Remove NaNs
validThr = isfinite(tThr) & isfinite(thr);
validCur = isfinite(tCur) & isfinite(cur);
tThr = tThr(validThr);
thr = thr(validThr);
tCur = tCur(validCur);
cur = cur(validCur);

thrCentered = thr - 1500;
thrMag = abs(thrCentered);
curMag = abs(cur);

% Alignment mode:
%  - true  => drift-aware piecewise warp (recommended when alignment drifts over time)
%  - false => single global affine map
usePiecewiseWarp = true;

% Initial time-scale guess from total durations
a0 = range(tThr) / max(range(tCur), eps);

% Coarse search for offset b (seconds) that maximizes correlation
bCandidates = linspace(-20, 20, 801);
score = nan(size(bCandidates));
for k = 1:numel(bCandidates)
    score(k) = alignmentCorr(tThr, thrMag, tCur, curMag, a0, bCandidates(k));
end
[~, bestIdx] = max(score);
b0 = bCandidates(bestIdx);

% Refine both scale (a) and offset (b)
x0 = [a0, b0];
obj = @(x) -alignmentCorr(tThr, thrMag, tCur, curMag, x(1), x(2));
opts = optimset("Display", "off", "TolX", 1e-8, "TolFun", 1e-8, "MaxIter", 2000, "MaxFunEvals", 4000);
[xOpt, fval] = fminsearch(obj, x0, opts);
a = xOpt(1);
b = xOpt(2);
maxCorr = -fval;

% Map current timestamps onto throttle timeline
if usePiecewiseWarp
    warpCfg.windowSec = 8.0;
    warpCfg.stepSec = 2.0;
    warpCfg.maxLagSec = 2.5;
    warpCfg.evalDtSec = 0.05;
    [mappedTCur, warpCentersCur, warpCentersThr] = piecewiseTimeWarp(tThr, thrMag, tCur, curMag, a, b, warpCfg);
else
    mappedTCur = a * tCur + b;
    warpCentersCur = [];
    warpCentersThr = [];
end

thrAtCur = interp1(tThr, thr, mappedTCur, "linear", NaN);
idx = isfinite(thrAtCur) & isfinite(cur);

% =========================================================================
% DATA SLICING — remove bad time windows (aligned / throttle timebase, seconds)
%
% Add one row per window you want to CUT OUT, format: [start_sec, end_sec]
% Leave the matrix empty  (sliceWindows = [])  to keep everything.
%
% Example:
%   sliceWindows = [10, 15; 40, 42];   % cuts 10-15 s and 40-42 s
% =========================================================================
sliceWindows = [107, 123];   % <-- edit here

% Preview plot so you can verify the windows before they take effect.
if ~isempty(sliceWindows)
    figure("Name", "Slice preview — before removal", "Color", "w");
    yyaxis left;
    plot(mappedTCur, movmean(cur, 5), "LineWidth", 1.0);
    ylabel("Current (mA)");
    yyaxis right;
    plot(tThr, thr, "LineWidth", 1.0);
    ylabel("Throttle (us)");
    yline(1500, "--");
    for si = 1:size(sliceWindows, 1)
        xregion(sliceWindows(si, 1), sliceWindows(si, 2), ...
            "FaceColor", [1 0.2 0.2], "FaceAlpha", 0.25);
    end
    xlabel("Aligned time (s)");
    title("Red shaded = data that will be removed");
    grid on;
    drawnow;
end

% Apply slices: overwrite bad windows with neutral values IN-PLACE so the
% timeline stays contiguous (no slope artifact from interpolating across a gap).
%   throttle => 1500 us (neutral)
%   current  => 0 mA
%   curMag   => 0
for si = 1:size(sliceWindows, 1)
    t0 = sliceWindows(si, 1);
    t1 = sliceWindows(si, 2);
    maskCur = mappedTCur >= t0 & mappedTCur <= t1;
    maskThr = tThr        >= t0 & tThr        <= t1;
    cur(maskCur)    = 0;
    curMag(maskCur) = 0;
    thr(maskThr)    = 1500;
    thrMag(maskThr) = 0;
end

% Recompute derived quantities after neutralising.
thrAtCur = interp1(tThr, thr, mappedTCur, "linear", NaN);
idx = isfinite(thrAtCur) & isfinite(cur);

if ~isempty(sliceWindows)
    fprintf("\nSlicing applied: %d window(s) set to neutral.\n", size(sliceWindows, 1));
end

% Optional denoising for display only
curDisp = movmean(cur, 5);

% Report alignment and polarity clues
fprintf("\nAlignment results:\n");
fprintf("  t_throttle ~= a * t_current + b\n");
fprintf("  a (scale)  = %.8f\n", a);
fprintf("  b (offset) = %.4f s\n", b);
fprintf("  max corr(|throttle-1500|, |current|) = %.4f\n", maxCorr);
if usePiecewiseWarp
    driftSec = warpCentersThr - (a * warpCentersCur + b);
    fprintf("  piecewise warp enabled: local lag range = [%.3f, %.3f] s\n", min(driftSec), max(driftSec));
end

x = thrAtCur(idx) - 1500;
y = cur(idx);

corrSigned = corrSafe(x, y);
corrMag = corrSafe(abs(x), abs(y));

neutralIdx = idx & abs(thrAtCur - 1500) <= 10;
forwardIdx = idx & (thrAtCur > 1550);
reverseIdx = idx & (thrAtCur < 1450);

neutralMean = mean(cur(neutralIdx), "omitnan");
forwardMean = mean(cur(forwardIdx), "omitnan");
reverseMean = mean(cur(reverseIdx), "omitnan");

fprintf("\nBehavior around neutral throttle:\n");
fprintf("  Mean power at |throttle-1500| <= 10 us: %.2f mW\n", neutralMean);
    fprintf("  Mean power for throttle > 1550 us:      %.2f mW\n", forwardMean);
    fprintf("  Mean power for throttle < 1450 us:      %.2f mW\n", reverseMean);
fprintf("  corr(throttle-1500, current):            %.4f\n", corrSigned);
fprintf("  corr(|throttle-1500|, |current|):        %.4f\n", corrMag);

if abs(corrSigned) < 0.20 && corrMag > abs(corrSigned) + 0.10
    fprintf("  Interpretation: current likely behaves mostly as magnitude (absolute value).\n");
elseif corrSigned < -0.20
    fprintf("  Interpretation: signed current is present and appears inverted (higher throttle => more negative current).\n");
elseif corrSigned > 0.20
    fprintf("  Interpretation: signed current is present with expected polarity (higher throttle => more positive current).\n");
else
    fprintf("  Interpretation: inconclusive sign behavior (weak signed and magnitude correlation).\n");
end

% Plot aligned timeseries
figure("Name", "Aligned current vs throttle", "Color", "w");
tiledlayout(2,1, "Padding", "compact", "TileSpacing", "compact");

nexttile;
yyaxis left;
plot(mappedTCur, curDisp, "LineWidth", 1.0);
ylabel("Current (mA)");

yyaxis right;
plot(tThr, thr, "LineWidth", 1.0);
ylabel("Throttle (us)");
yline(1500, "--", "Neutral 1500 us", "LabelHorizontalAlignment", "left");

xlabel("Aligned time (s)");
title("Time-aligned signals (current mapped to throttle timebase)");
grid on;
legend("Current (smoothed)", "Throttle", "Location", "best");

if usePiecewiseWarp && ~isempty(warpCentersCur)
    figure("Name", "Estimated clock drift", "Color", "w");
    driftSec = warpCentersThr - (a * warpCentersCur + b);
    plot(warpCentersCur, driftSec, "o-", "LineWidth", 1.2, "MarkerSize", 4);
    yline(0, "--", "No local drift");
    xlabel("Current-log time (s)");
    ylabel("Extra local lag (s)");
    title("Local drift relative to global affine alignment");
    grid on;
end

% Plot scatter in control space
nexttile;
scatter(thrAtCur(idx), cur(idx), 10, mappedTCur(idx), "filled", "MarkerFaceAlpha", 0.45);
xline(1500, "--", "Neutral 1500 us");
xlabel("Throttle (us)");
ylabel("Current (mA)");
title("Current vs throttle after alignment");
cb = colorbar;
cb.Label.String = "Aligned time (s)";
grid on;

% Add binned median trend to make sign easier to see
hold on;
nbins = 60;
edges = linspace(min(thrAtCur(idx)), max(thrAtCur(idx)), nbins + 1);
centers = (edges(1:end-1) + edges(2:end)) / 2;
medianCur = nan(1, nbins);
for i = 1:nbins
    m = idx & thrAtCur >= edges(i) & thrAtCur < edges(i+1);
    if nnz(m) > 5
        medianCur(i) = median(cur(m), "omitnan");
    end
end
plot(centers, medianCur, "k", "LineWidth", 2.0);
legend("Samples", "Neutral 1500 us", "Binned median", "Location", "best");

% Dedicated relationship figure (signed and magnitude views)
relThr = thrAtCur(idx);
relCur = cur(idx);
relX = relThr - 1500;

% Use points far enough from neutral for stable linear fits
deadband = 30;
fwdMask = relX > deadband;
revMask = relX < -deadband;
absMask = abs(relX) > deadband;

pFwd = [NaN NaN];
pRev = [NaN NaN];
pAbs = [NaN NaN];
if nnz(fwdMask) > 20
    pFwd = polyfit(relX(fwdMask), relCur(fwdMask), 1);
end
if nnz(revMask) > 20
    pRev = polyfit(relX(revMask), relCur(revMask), 1);
end
if nnz(absMask) > 40
    pAbs = polyfit(abs(relX(absMask)), abs(relCur(absMask)), 1);
end

fprintf("\nLinear relationship estimates:\n");
if all(isfinite(pFwd))
    fprintf("  Forward branch: power ~= %.4f*(throttle-1500) + %.2f mW\n", pFwd(1), pFwd(2));
else
    fprintf("  Forward branch: not enough points for reliable fit.\n");
end
if all(isfinite(pRev))
    fprintf("  Reverse branch: power ~= %.4f*(throttle-1500) + %.2f mW\n", pRev(1), pRev(2));
else
    fprintf("  Reverse branch: not enough points for reliable fit.\n");
end
if all(isfinite(pAbs))
    fprintf("  Magnitude model: |power| ~= %.4f*|throttle-1500| + %.2f mW\n", pAbs(1), pAbs(2));
else
    fprintf("  Magnitude model: not enough points for reliable fit.\n");
end

% Convert throttle to predicted current using the fitted branch models.
% If the relationship is perfect, predicted current overlaps measured current.
predCurFromThr = nan(size(relCur));
if all(isfinite(pFwd))
    predCurFromThr(relX >= 0) = polyval(pFwd, relX(relX >= 0));
end
if all(isfinite(pRev))
    predCurFromThr(relX < 0) = polyval(pRev, relX(relX < 0));
end

% Fallback for any unfitted region: use one global linear model.
missingMask = ~isfinite(predCurFromThr);
if any(missingMask)
    pGlobal = polyfit(relX, relCur, 1);
    predCurFromThr(missingMask) = polyval(pGlobal, relX(missingMask));
else
    pGlobal = [NaN NaN];
end

rmseOverlap = sqrt(mean((relCur - predCurFromThr).^2, "omitnan"));
corrOverlap = corrSafe(relCur, predCurFromThr);
fprintf("  Overlap quality: RMSE(measured vs predicted) = %.2f mW, corr = %.4f\n", rmseOverlap, corrOverlap);

figure("Name", "Current-throttle relationship", "Color", "w");
tiledlayout(2,1, "Padding", "compact", "TileSpacing", "compact");

nexttile;
scatter(relThr, relCur, 10, mappedTCur(idx), "filled", "MarkerFaceAlpha", 0.35);
hold on;
xline(1500, "--", "Neutral 1500 us");
if all(isfinite(pFwd))
    xxF = linspace(max(1500 + deadband, min(relThr)), max(relThr), 100);
    yyF = polyval(pFwd, xxF - 1500);
    plot(xxF, yyF, "r", "LineWidth", 2.0);
end
if all(isfinite(pRev))
    xxR = linspace(min(relThr), min(1500 - deadband, max(relThr)), 100);
    yyR = polyval(pRev, xxR - 1500);
    plot(xxR, yyR, "b", "LineWidth", 2.0);
end
xlabel("Throttle (us)");
ylabel("Power (mW)");
title("Signed relationship: power vs throttle");
grid on;
cb2 = colorbar;
cb2.Label.String = "Aligned time (s)";
legend("Samples", "Neutral", "Forward fit", "Reverse fit", "Location", "best");

nexttile;
scatter(abs(relX), abs(relCur), 10, mappedTCur(idx), "filled", "MarkerFaceAlpha", 0.35);
hold on;
if all(isfinite(pAbs))
    xxA = linspace(0, max(abs(relX)), 100);
    yyA = polyval(pAbs, xxA);
    plot(xxA, yyA, "k", "LineWidth", 2.0);
end
xlabel("|Throttle - 1500| (us)");
ylabel("|Power| (mW)");
title("Magnitude relationship: |power| vs |throttle-1500|");
grid on;
legend("Samples", "Magnitude fit", "Location", "best");

figure("Name", "Perfect-overlap check", "Color", "w");
tiledlayout(2,1, "Padding", "compact", "TileSpacing", "compact");

nexttile;
% Scale throttle to sit within the current axis range without affecting it.
curRange = max(relCur) - min(relCur);
thrAtCurIdx = thrAtCur(idx);
thrScaled = (thrAtCurIdx - min(thrAtCurIdx)) / max(range(thrAtCurIdx), eps) ...
            * curRange + min(relCur);
plot(mappedTCur(idx), thrScaled, "Color", [0.75 0.75 0.75], "LineWidth", 0.8);
hold on;
plot(mappedTCur(idx), relCur, "LineWidth", 1.2);
plot(mappedTCur(idx), predCurFromThr, "LineWidth", 1.2);
xlabel("Aligned time (s)");
ylabel("Power (mW)");
title("Measured power vs power predicted from throttle");
grid on;
legend("Throttle (scaled)", "Measured power", "Predicted from throttle model", "Location", "best");

nexttile;
sampleErr = relCur - predCurFromThr;
plot(mappedTCur(idx), sampleErr, "LineWidth", 1.0);
yline(0, "--", "Zero error");
xlabel("Aligned time (s)");
ylabel("Prediction error (mW)");
title(sprintf("Prediction error (RMSE = %.2f mW, corr = %.3f)", rmseOverlap, corrOverlap));
grid on;

%% Spike identification after throttle steps
% Tuned for log_4.0/power4.0: real commanded steps are typically >=100 us
% and a few rapid transitions inside slice windows should be ignored.
spikeCfg.stepThresholdUs = 60;          % minimum throttle jump to consider as a true step
spikeCfg.groupingWindowSec = 0.15;      % group candidates within this time of each other
spikeCfg.minStepSeparationSec = 2.00;   % one detected event per commanded transition
spikeCfg.preBaselineSec = 0.70;         % baseline window before each step (increased for sparse data)
spikeCfg.postSearchSec = 1.30;          % search window for spike after step
spikeCfg.postSteadySec = 0.50;          % steady-state window at end of postSearch window
spikeCfg.smoothSamples = 7;             % smoothing for robust step detection
spikeCfg.minPreSamples = 2;             % minimum samples in baseline (reduced for ~10 Hz data)
spikeCfg.minPostSamples = 2;            % minimum samples in post-window (reduced for ~10 Hz data)
spikeCfg.excludeSliceWindows = true;    % ignore events inside neutralised/sliced time windows
spikeCfg.sliceEdgeGuardSec = 0.40;      % ignore events close to slice edges

tRel = mappedTCur(idx);
uRel = relThr(:);
iRel = relCur(:);

uDet = movmedian(uRel, spikeCfg.smoothSamples);

dU = [0; diff(uDet)];
candidateSteps = find(abs(dU) >= spikeCfg.stepThresholdUs);

% Group nearby candidates to avoid counting the same transition multiple times.
stepIdx = [];
if ~isempty(candidateSteps)
    groupStart = 1;
    for k = 2:numel(candidateSteps)
        prevIdx = candidateSteps(k-1);
        currIdx = candidateSteps(k);
        isSeparated = (currIdx - prevIdx > 1) || ((tRel(currIdx) - tRel(prevIdx)) > spikeCfg.groupingWindowSec);
        if isSeparated
            groupIdx = candidateSteps(groupStart:k-1);
            [~, localBest] = max(abs(dU(groupIdx)));
            stepIdx(end+1, 1) = groupIdx(localBest); %#ok<AGROW>
            groupStart = k;
        end
    end
    groupIdx = candidateSteps(groupStart:end);
    [~, localBest] = max(abs(dU(groupIdx)));
    stepIdx(end+1, 1) = groupIdx(localBest); %#ok<AGROW>
end

% Final debounce in time domain.
if ~isempty(stepIdx)
    keep = true(size(stepIdx));
    lastT = -inf;
    for k = 1:numel(stepIdx)
        tk = tRel(stepIdx(k));
        if tk - lastT < spikeCfg.minStepSeparationSec
            keep(k) = false;
        else
            lastT = tk;
        end
    end
    stepIdx = stepIdx(keep);
end

% Exclude events close to slice windows if enabled
if spikeCfg.excludeSliceWindows && ~isempty(sliceWindows)
    keep = true(size(stepIdx));
    for k = 1:numel(stepIdx)
        tk = tRel(stepIdx(k));
        for sw_idx = 1:size(sliceWindows, 1)
            t_start = sliceWindows(sw_idx, 1);
            t_end = sliceWindows(sw_idx, 2);
            if tk >= (t_start - spikeCfg.sliceEdgeGuardSec) && tk <= (t_end + spikeCfg.sliceEdgeGuardSec)
                keep(k) = false;
                break;
            end
        end
    end
    stepIdx = stepIdx(keep);
end

spikeRows = [];
for k = 1:numel(stepIdx)
    ii = stepIdx(k);
    t0 = tRel(ii);

    preMask = tRel >= (t0 - spikeCfg.preBaselineSec) & tRel < t0;
    postMask = tRel >= t0 & tRel <= (t0 + spikeCfg.postSearchSec);
    steadyMask = tRel >= (t0 + spikeCfg.postSearchSec - spikeCfg.postSteadySec) & tRel <= (t0 + spikeCfg.postSearchSec);
    
    nPre = nnz(preMask);
    nPost = nnz(postMask);
    
    if nPre < spikeCfg.minPreSamples || nPost < spikeCfg.minPostSamples
        continue;
    end

    uPre = median(uDet(preMask), "omitnan");
    if nnz(steadyMask) >= 4
        uPost = median(uDet(steadyMask), "omitnan");
    else
        uPost = uDet(ii);
    end
    dUStep = uPost - uPre;
    if abs(dUStep) < spikeCfg.stepThresholdUs
        continue;
    end

    iBase = median(iRel(preMask), "omitnan");
    iPost = iRel(postMask);
    tPost = tRel(postMask);
    devPost = iPost - iBase;

    if nnz(steadyMask) >= 4
        iSteady = median(iRel(steadyMask), "omitnan");
        expectedSign = sign(iSteady - iBase);
    else
        expectedSign = sign(dUStep);
    end

    [posDev, posIdx] = max(devPost);
    [negDev, negIdx] = min(devPost);
    if expectedSign > 0
        pkLocal = posIdx;
    elseif expectedSign < 0
        pkLocal = negIdx;
    else
        [~, pkLocal] = max(abs(devPost));
    end

    % If expected-sign peak is tiny, fall back to absolute largest excursion.
    if abs(devPost(pkLocal)) < 0.5 * max(abs(devPost))
        [~, pkLocal] = max(abs(devPost));
    end

    iPeak = iPost(pkLocal);
    dIPeak = iPeak - iBase;
    dtPeak = tPost(pkLocal) - t0;

    spikeRows(end+1, :) = [t0, uDet(ii), dUStep, iBase, iPeak, dIPeak, dtPeak, posDev, negDev]; %#ok<AGROW>
end

if isempty(spikeRows)
    fprintf("\nSpike detection: no spikes detected with current thresholds.\n");
else
    spikeTbl = array2table(spikeRows, "VariableNames", ...
        {'tStep_s', 'throttle_us', 'dThrottle_us', 'baselineCurrent_mA', 'peakCurrent_mA', 'spike_mA', 'timeToPeak_s', 'posSpike_mA', 'negSpike_mA'});

    fprintf("\nSpike detection summary:\n");
    fprintf("  Steps analyzed: %d\n", height(spikeTbl));
    fprintf("  Positive spikes: %d\n", nnz(spikeTbl.spike_mA > 0));
    fprintf("  Negative spikes: %d\n", nnz(spikeTbl.spike_mA < 0));
    fprintf("  Mean |step|:   %.2f us\n", mean(abs(spikeTbl.dThrottle_us), "omitnan"));
    fprintf("  Mean |spike|:  %.2f mW\n", mean(abs(spikeTbl.spike_mA), "omitnan"));
    fprintf("  Max |spike|:   %.2f mW\n", max(abs(spikeTbl.spike_mA), [], "omitnan"));
    fprintf("  Mean t_peak:   %.3f s\n", mean(spikeTbl.timeToPeak_s, "omitnan"));

    [~, ord] = sort(abs(spikeTbl.spike_mA), "descend");
    nShow = min(8, numel(ord));
    fprintf("\nTop %d spikes (largest |spike|):\n", nShow);
    % disp(spikeTbl(ord(1:nShow), :));  % Skip table display due to memory issue
    for ii = 1:nShow
        idx_spk = ord(ii);
        fprintf("  %.3f s: dThrottle=%7.1f us, spike=%8.1f mA, peak_t=%6.3f s\n", ...
            spikeTbl.tStep_s(idx_spk), spikeTbl.dThrottle_us(idx_spk), ...
            spikeTbl.spike_mA(idx_spk), spikeTbl.timeToPeak_s(idx_spk));
    end

    figure("Name", "Detected current spikes", "Color", "w");
    tiledlayout(2,1, "Padding", "compact", "TileSpacing", "compact");

    nexttile;
    plot(tRel, iRel, "LineWidth", 1.0);
    hold on;
    plot(spikeTbl.tStep_s, spikeTbl.baselineCurrent_mA, "ko", "MarkerSize", 4, "LineWidth", 1.0);
    plot(spikeTbl.tStep_s + spikeTbl.timeToPeak_s, spikeTbl.peakCurrent_mA, "rx", "MarkerSize", 7, "LineWidth", 1.4);
    xlabel("Aligned time (s)");
    ylabel("Power (mW)");
    title("Detected spikes after throttle steps");
    grid on;
    legend("Power", "Baseline at step", "Detected peak", "Location", "best");

    nexttile;
    scatter(abs(spikeTbl.dThrottle_us), abs(spikeTbl.spike_mA), 30, spikeTbl.timeToPeak_s, "filled");
    hold on;
    if height(spikeTbl) >= 2
        pSpike = polyfit(abs(spikeTbl.dThrottle_us), abs(spikeTbl.spike_mA), 1);
        xxS = linspace(min(abs(spikeTbl.dThrottle_us)), max(abs(spikeTbl.dThrottle_us)), 100);
        yyS = polyval(pSpike, xxS);
        plot(xxS, yyS, "k", "LineWidth", 1.8);
        fitSlopeTxt = sprintf("%.3f mA/us", pSpike(1));
    else
        fitSlopeTxt = "n/a";
    end
    xlabel("|Throttle step| (us)");
    ylabel("|Spike amplitude| (mW)");
    title(sprintf("Spike size vs step size (fit slope = %s)", fitSlopeTxt));
    cb3 = colorbar;
    cb3.Label.String = "Time to peak (s)";
    grid on;
    legend("Detected spikes", "Linear fit", "Location", "best");

    %% Post-spike de-spiked overlap (re-fitted model on cleaned data)
    % Build a mask that excludes the transient window after every detected step.
    spikeExcludeSec = spikeCfg.postSearchSec;  % same window as spike search
    cleanMask = true(size(tRel));
    for si2 = 1:height(spikeTbl)
        t0s = spikeTbl.tStep_s(si2);
        cleanMask = cleanMask & ~(tRel >= t0s & tRel <= t0s + spikeExcludeSec);
    end

    relX_c  = relX(cleanMask);
    relCur_c = relCur(cleanMask);
    tRel_c  = tRel(cleanMask);
    thrAtCur_c = relThr(cleanMask);

    % Re-fit linear model on cleaned data.
    fwdC = relX_c > deadband;
    revC = relX_c < -deadband;
    absC = abs(relX_c) > deadband;
    pFwdC = [NaN NaN]; pRevC = [NaN NaN]; pAbsC = [NaN NaN];
    if nnz(fwdC) > 20, pFwdC = polyfit(relX_c(fwdC), relCur_c(fwdC), 1); end
    if nnz(revC) > 20, pRevC = polyfit(relX_c(revC), relCur_c(revC), 1); end
    if nnz(absC) > 40, pAbsC = polyfit(abs(relX_c(absC)), abs(relCur_c(absC)), 1); end

    % Predicted power on all points using the cleaned model.
    predC = nan(size(relCur));
    if all(isfinite(pFwdC)), predC(relX >= 0) = polyval(pFwdC, relX(relX >= 0)); end
    if all(isfinite(pRevC)), predC(relX < 0)  = polyval(pRevC, relX(relX < 0));  end
    misC = ~isfinite(predC);
    if any(misC)
        pGlobalC = polyfit(relX_c, relCur_c, 1);
        predC(misC) = polyval(pGlobalC, relX(misC));
    end

    rmseClean = sqrt(mean((relCur_c - predC(cleanMask)).^2, "omitnan"));
    corrClean = corrSafe(relCur_c, predC(cleanMask));

    fprintf("\nPost-spike model (transients excluded from fit):\n");
    if all(isfinite(pFwdC))
        fprintf("  Forward: P ~= %.4f*(u-1500) + %.2f mW\n", pFwdC(1), pFwdC(2));
    end
    if all(isfinite(pRevC))
        fprintf("  Reverse: P ~= %.4f*(u-1500) + %.2f mW\n", pRevC(1), pRevC(2));
    end
    if all(isfinite(pAbsC))
        fprintf("  Magnitude: |P| ~= %.4f*|u-1500| + %.2f mW\n", pAbsC(1), pAbsC(2));
    end
    fprintf("  RMSE on cleaned data:   %.2f mW  (all-data RMSE was %.2f mW)\n", rmseClean, rmseOverlap);
    fprintf("  corr  on cleaned data:  %.4f    (all-data corr was %.4f)\n", corrClean, corrOverlap);

    figure("Name", "Post-spike overlap (de-spiked model)", "Color", "w");
    tiledlayout(3,1, "Padding", "compact", "TileSpacing", "compact");

    nexttile;
    % Throttle background (scaled).
    pwr_range_c = max(relCur) - min(relCur);
    thrSc_c = (relThr - min(relThr)) / max(range(relThr), eps) * pwr_range_c + min(relCur);
    plot(tRel, thrSc_c, "Color", [0.75 0.75 0.75], "LineWidth", 0.8);
    hold on;
    plot(tRel, relCur,  "LineWidth", 1.2);
    plot(tRel, predC,   "LineWidth", 1.2);
    % Mark excluded spike windows.
    for si2 = 1:height(spikeTbl)
        xregion(spikeTbl.tStep_s(si2), spikeTbl.tStep_s(si2) + spikeExcludeSec, ...
            "FaceColor", [1 0.6 0.6], "FaceAlpha", 0.18);
    end
    ylabel("Power (mW)");
    title("De-spiked model: measured vs predicted power");
    grid on;
    legend("Throttle (scaled)", "Measured power", "Predicted (de-spiked model)", ...
           "Spike window", "Location", "best");

    nexttile;
    errAll   = relCur   - predC;
    errClean = relCur_c - predC(cleanMask);
    plot(tRel,   errAll,  "Color", [0.7 0.7 0.7], "LineWidth", 0.8);
    hold on;
    plot(tRel_c, errClean, "LineWidth", 1.0);
    yline(0, "--");
    ylabel("Error (mW)");
    title(sprintf("Prediction error — cleaned RMSE = %.2f mW vs %.2f mW all-data", rmseClean, rmseOverlap));
    grid on;
    legend("All-data error", "Cleaned-data error", "Location", "best");

    nexttile;
    % Throttle -> Power function: show both fitted models on one plot.
    uGrid = linspace(min(relThr), max(relThr), 300);
    uGridX = uGrid - 1500;
    % All-data model.
    predAll_fwd = nan(size(uGrid)); predAll_rev = nan(size(uGrid));
    if all(isfinite(pFwd)),  predAll_fwd(uGridX >= 0) = polyval(pFwd, uGridX(uGridX >= 0)); end
    if all(isfinite(pRev)),  predAll_rev(uGridX < 0)  = polyval(pRev, uGridX(uGridX < 0));  end
    predAll_grid = nansum(cat(1, predAll_fwd, predAll_rev), 1);
    predAll_grid(isnan(predAll_fwd) & isnan(predAll_rev)) = NaN;
    % De-spiked model.
    predDS_fwd = nan(size(uGrid)); predDS_rev = nan(size(uGrid));
    if all(isfinite(pFwdC)), predDS_fwd(uGridX >= 0) = polyval(pFwdC, uGridX(uGridX >= 0)); end
    if all(isfinite(pRevC)), predDS_rev(uGridX < 0)  = polyval(pRevC, uGridX(uGridX < 0));  end
    predDS_grid = nansum(cat(1, predDS_fwd, predDS_rev), 1);
    predDS_grid(isnan(predDS_fwd) & isnan(predDS_rev)) = NaN;

    scatter(relThr, relCur, 6, [0.80 0.80 0.80], "filled", "MarkerFaceAlpha", 0.30);
    hold on;
    scatter(relThr(cleanMask), relCur_c, 6, [0.50 0.72 0.92], "filled", "MarkerFaceAlpha", 0.45);
    plot(uGrid, predAll_grid, "r--",  "LineWidth", 1.8);
    plot(uGrid, predDS_grid,  "b",    "LineWidth", 2.0);
    xline(1500, "--", "Neutral");
    xlabel("Throttle (us)");
    ylabel("Power (mW)");
    title("P(u): throttle \\rightarrow power — all-data vs de-spiked model");
    grid on;
    legend("All samples", "Cleaned samples", ...
           "All-data fit", "De-spiked fit", "Location", "best");
end

%% Local helper functions
function r = alignmentCorr(tThr, thrMag, tCur, curMag, a, b)
    mapped = a * tCur + b;
    thrMapped = interp1(tThr, thrMag, mapped, "linear", NaN);
    m = isfinite(thrMapped) & isfinite(curMag);
    if nnz(m) < 50
        r = -inf;
        return;
    end
    x = thrMapped(m);
    y = curMag(m);
    r = corrSafe(x, y);
end

function c = corrSafe(x, y)
    x = x(:);
    y = y(:);
    m = isfinite(x) & isfinite(y);
    if nnz(m) < 3
        c = NaN;
        return;
    end
    x = x(m);
    y = y(m);
    x = x - mean(x);
    y = y - mean(y);
    denom = sqrt(sum(x.^2) * sum(y.^2));
    if denom <= eps
        c = NaN;
    else
        c = sum(x .* y) / denom;
    end
end

function [mappedTCur, centersCur, centersThrWarp] = piecewiseTimeWarp(tThr, thrMag, tCur, curMag, a, b, cfg)
    mappedTCurGlobal = a * tCur + b;
    tCurStart = max(tCur(1), tCur(1) + cfg.windowSec / (2 * max(a, eps)));
    tCurStop = min(tCur(end), tCur(end) - cfg.windowSec / (2 * max(a, eps)));

    if tCurStop <= tCurStart
        mappedTCur = mappedTCurGlobal;
        centersCur = [];
        centersThrWarp = [];
        return;
    end

    centersCur = (tCurStart:cfg.stepSec:tCurStop).';
    if isempty(centersCur)
        mappedTCur = mappedTCurGlobal;
        centersCur = [];
        centersThrWarp = [];
        return;
    end

    centersThrWarp = nan(size(centersCur));
    halfW = cfg.windowSec / 2;
    lagGrid = (-cfg.maxLagSec:cfg.evalDtSec:cfg.maxLagSec).';
    localGridThr = (-halfW:cfg.evalDtSec:halfW).';

    for i = 1:numel(centersCur)
        tc = centersCur(i);
        t0Global = a * tc + b;

        % Sample current in a window mapped into throttle-time units.
        tCurSample = tc + localGridThr / max(a, eps);
        curSeg = interp1(tCur, curMag, tCurSample, "linear", NaN);
        if nnz(isfinite(curSeg)) < 0.8 * numel(curSeg)
            continue;
        end

        bestR = -inf;
        bestLag = 0;
        for j = 1:numel(lagGrid)
            lag = lagGrid(j);
            thrSeg = interp1(tThr, thrMag, t0Global + localGridThr + lag, "linear", NaN);
            r = corrSafe(curSeg, thrSeg);
            if isfinite(r) && r > bestR
                bestR = r;
                bestLag = lag;
            end
        end

        if isfinite(bestR)
            centersThrWarp(i) = t0Global + bestLag;
        end
    end

    ok = isfinite(centersThrWarp);
    centersCur = centersCur(ok);
    centersThrWarp = centersThrWarp(ok);

    if numel(centersCur) < 4
        mappedTCur = mappedTCurGlobal;
        centersCur = [];
        centersThrWarp = [];
        return;
    end

    % Enforce monotonic mapping (time should not go backward).
    for i = 2:numel(centersThrWarp)
        centersThrWarp(i) = max(centersThrWarp(i), centersThrWarp(i-1) + 1e-6);
    end

    mappedTCur = interp1(centersCur, centersThrWarp, tCur, "pchip", "extrap");
end

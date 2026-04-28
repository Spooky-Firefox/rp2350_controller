%% Compare fitted P(u) to wheel speed (car in air)
% Uses only log_4.0 (throttle + speed) and a pre-fitted P(u) model.
% No power log is loaded and no time alignment is performed.
% NOTE: data is with car in air, so this captures rotational behavior only.

clear; clc; close all;

%% Inputs
scriptDir = fileparts(mfilename("fullpath"));
throttleFile = fullfile(scriptDir, "log_4.0.csv");

% Choose which fitted model parameters to use.
% Example below from your post-spike model:
%   Forward: P ~= 51.7922*(u-1500) + 4777.71 mW
%   Reverse: P ~= 68.3985*(u-1500) - 728.23 mW
%   Magnitude: |P| ~= 58.4852*|u-1500| + 3064.74 mW
model.forward = [51.7922, 4777.71];    % [slope, intercept] in mW/us and mW
model.reverse = [68.3985, -728.23];    % [slope, intercept] in mW/us and mW
model.magnitude = [58.4852, 3064.74];  % [slope, intercept] for |P|

deadbandUs = 30;      % around neutral where branch model is less certain
useMagnitudeNearZero = true;
speedIsAbsolute = true;  % speed_mps is measured as magnitude only

%% Read and validate
tbl = readtable(throttleFile, "VariableNamingRule", "preserve");
required = ["timestamp_us", "throttle_us", "speed_mps"];
assert(all(ismember(required, string(tbl.Properties.VariableNames))), ...
    "Log file must include: timestamp_us, throttle_us, speed_mps");

%% Extract and clean
t = (double(tbl.timestamp_us) - double(tbl.timestamp_us(1))) / 1e6;
u = double(tbl.throttle_us);
v = double(tbl.speed_mps);

m = isfinite(t) & isfinite(u) & isfinite(v);
t = t(m);
u = u(m);
v = v(m);

%% Compute fitted power from throttle only: P_fit(u)
x = u - 1500;
pFit = nan(size(u));

fwd = x > deadbandUs;
rev = x < -deadbandUs;
near = ~(fwd | rev);

% Piecewise forward/reverse model
pFit(fwd) = polyval(model.forward, x(fwd));
pFit(rev) = polyval(model.reverse, x(rev));

% Near neutral: either mirror-magnitude model or midpoint blend
if useMagnitudeNearZero
    pFit(near) = polyval(model.magnitude, abs(x(near)));
else
    pMid = 0.5 * (polyval(model.forward, x(near)) + polyval(model.reverse, x(near)));
    pFit(near) = pMid;
end

% If speed is unsigned, compare it against absolute fitted power.
if speedIsAbsolute
    v = abs(v);
    pForSpeed = abs(pFit);
    pSymbol = "|P|";
    pAxisLabel = "|Fitted power| (mW)";
else
    pForSpeed = pFit;
    pSymbol = "P";
    pAxisLabel = "Fitted power P(u) (mW)";
end

%% Fit speed as a function of fitted power
valid = isfinite(v) & isfinite(pForSpeed);
pUse = pForSpeed(valid);
vUse = v(valid);

linV = polyfit(pUse, vUse, 1);
quadV = polyfit(pUse, vUse, 2);

vHatLin = polyval(linV, pUse);
vHatQuad = polyval(quadV, pUse);

r2Lin = r2score(vUse, vHatLin);
r2Quad = r2score(vUse, vHatQuad);
rmseLin = sqrt(mean((vUse - vHatLin).^2, "omitnan"));
rmseQuad = sqrt(mean((vUse - vHatQuad).^2, "omitnan"));

%% Report
fprintf("\nUsing fitted power model P(u):\n");
fprintf("  Forward:   P ~= %.4f*(u-1500) + %.2f mW\n", model.forward(1), model.forward(2));
fprintf("  Reverse:   P ~= %.4f*(u-1500) + %.2f mW\n", model.reverse(1), model.reverse(2));
fprintf("  Magnitude: |P| ~= %.4f*|u-1500| + %.2f mW\n", model.magnitude(1), model.magnitude(2));

fprintf("\nSpeed vs fitted power (no power-log alignment used):\n");
fprintf("  Linear:    v ~= %.6f*%s + %.4f, R^2=%.4f, RMSE=%.4f m/s\n", ...
    linV(1), pSymbol, linV(2), r2Lin, rmseLin);
fprintf("  Quadratic: v ~= %.6e*%s^2 + %.6f*%s + %.4f, R^2=%.4f, RMSE=%.4f m/s\n", ...
    quadV(1), pSymbol, quadV(2), pSymbol, quadV(3), r2Quad, rmseQuad);
if speedIsAbsolute
    fprintf("  Note: speed is unsigned, so regression uses |P_fit|.\n");
end

%% Plots
figure("Name", "Fitted P(u) vs speed (single-log)", "Color", "w");
tiledlayout(3,1, "Padding", "compact", "TileSpacing", "compact");

nexttile;
yyaxis left;
plot(t, pForSpeed, "LineWidth", 1.0);
hold on;
if speedIsAbsolute
    plot(t, pFit, "--", "Color", [0.6 0.6 0.6], "LineWidth", 0.8);
end
ylabel(pAxisLabel);
yyaxis right;
plot(t, v, "Color", [0.2 0.2 0.2], "LineWidth", 0.9);
ylabel("Speed (m/s)");
xlabel("Time (s)");
if speedIsAbsolute
    title("Time series from same log: |P_fit|, signed P_fit (dashed), and |speed|");
else
    title("Time series from same log: fitted power and speed");
end
grid on;
if speedIsAbsolute
    legend("|Fitted P(u)|", "Signed P(u)", "Speed", "Location", "best");
else
    legend("Fitted P(u)", "Speed", "Location", "best");
end

nexttile;
scatter(u, pFit, 10, v, "filled", "MarkerFaceAlpha", 0.35);
xline(1500, "--", "Neutral");
xlabel("Throttle (us)");
ylabel("Signed fitted power (mW)");
title("P(u) generated from fitted model (signed branches)");
cb = colorbar; cb.Label.String = "Speed (m/s)";
grid on;

nexttile;
scatter(pUse, vUse, 10, u(valid), "filled", "MarkerFaceAlpha", 0.35);
hold on;
xx = linspace(min(pUse), max(pUse), 250);
plot(xx, polyval(linV, xx), "k--", "LineWidth", 1.4);
plot(xx, polyval(quadV, xx), "k", "LineWidth", 1.8);
xlabel("Fitted power P(u) (mW)");
ylabel("Speed (m/s)");
xlabel(pAxisLabel);
if speedIsAbsolute
    title(sprintf("|speed| vs |P_fit| (R^2 lin=%.3f, quad=%.3f)", r2Lin, r2Quad));
else
    title(sprintf("Speed vs fitted power (R^2 lin=%.3f, quad=%.3f)", r2Lin, r2Quad));
end
cb2 = colorbar; cb2.Label.String = "Throttle (us)";
grid on;
legend("Samples", "Linear fit", "Quadratic fit", "Location", "best");

%% Local helper
function r2 = r2score(yTrue, yHat)
    yTrue = yTrue(:); yHat = yHat(:);
    m = isfinite(yTrue) & isfinite(yHat);
    if nnz(m) < 3
        r2 = NaN;
        return;
    end
    yt = yTrue(m);
    yh = yHat(m);
    ssRes = sum((yt - yh).^2);
    ssTot = sum((yt - mean(yt)).^2);
    if ssTot <= eps
        r2 = NaN;
    else
        r2 = 1 - ssRes / ssTot;
    end
end

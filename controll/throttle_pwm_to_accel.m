function a_long = throttle_pwm_to_accel(pwm_us, v_current, cfg)
% Map throttle PWM to longitudinal acceleration.
% Update this function as your real ESC/motor characterization improves.
%
% Inputs:
%   pwm_us    : throttle command in microseconds [1000..2000]
%   v_current : current vehicle speed [m/s]
%   cfg       : optional struct with fields:
%                 deadband_us, max_accel_fwd, max_accel_rev, drag_coeff

    if nargin < 3 || isempty(cfg)
        cfg.deadband_us = 25;
        cfg.max_accel_fwd = 2.2;
        cfg.max_accel_rev = 1.8;
        cfg.drag_coeff = 0.35;
    end

    u = (pwm_us - 1500) / 500;
    u = clamp(u, -1.0, 1.0);

    db = cfg.deadband_us / 500;
    if abs(u) <= db
        u_eff = 0;
    else
        u_eff = sign(u) * ((abs(u) - db) / (1 - db));
    end

    if u_eff >= 0
        drive_accel = cfg.max_accel_fwd * (u_eff^2);
    else
        drive_accel = -cfg.max_accel_rev * (abs(u_eff)^2);
    end

    % Simple rolling/air drag term to keep the simulation realistic.
    a_long = drive_accel - cfg.drag_coeff * v_current;
end

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end

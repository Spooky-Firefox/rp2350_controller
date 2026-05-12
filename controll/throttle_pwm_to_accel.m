function a_long = throttle_pwm_to_accel(pwm_us, v_current, cfg)
% Map throttle PWM to longitudinal acceleration.
% Update this function as your real ESC/motor characterization improves.
%
% Inputs:
%   pwm_us    : throttle command in microseconds [1000..2000]
%   v_current : current vehicle speed [m/s]
%   cfg       : optional struct with fields:
%                 deadband_us, max_accel_fwd,
%                 drag_k0 (rolling resistance [m/s^2]),
%                 drag_k1 (viscous drag [1/s]),
%                 throttle_tau_s (1st-order throttle lag [s]),
%                 back_emf_gain (torque reduction vs speed [s/m]),
%                 dt (step time [s]), reset (bool), forward_only (bool)

    persistent u_filt initialized;

    if nargin < 3 || isempty(cfg)
        cfg = struct();
    end
    if ~isfield(cfg, "max_accel_fwd")
        cfg.max_accel_fwd = 8.000000;  % fitted
    end
    if ~isfield(cfg, "throttle_tau_s")
        cfg.throttle_tau_s = 0.025184;  % fitted
    end
    if ~isfield(cfg, "back_emf_gain")
        cfg.back_emf_gain = 0.116924;   % fitted
    end
    if ~isfield(cfg, "deadband_us")
        cfg.deadband_us = 0.0;        % fitted
    end
    if ~isfield(cfg, "n_throttle")
        cfg.n_throttle = 1.000000;     % fitted (optional: add if not present)
    end
    if ~isfield(cfg, "drag_k0")
        cfg.drag_k0 = 1.97678;   % rolling resistance [m/s^2]  — measured coast fit
    end
    if ~isfield(cfg, "drag_k1")
        cfg.drag_k1 = 0.1724;   % viscous drag [1/s]           — measured coast fit
    end
    if ~isfield(cfg, "dt")
        cfg.dt = 0.01;
    end
    if ~isfield(cfg, "reset")
        cfg.reset = false;
    end
    if ~isfield(cfg, "forward_only")
        cfg.forward_only = true;
    end
    if ~isfield(cfg, "v_epsilon")
        cfg.v_epsilon = 0.03;
    end

    if cfg.reset || isempty(initialized) || ~initialized
        u_filt = 0;
        initialized = true;
    end

    u = (pwm_us - 1500) / 500;
    u = clamp(u, -1.0, 1.0);

    if cfg.forward_only
        u = max(u, 0.0);
    end

    db = cfg.deadband_us / 500;
    if abs(u) <= db
        u_eff = 0;
    else
        u_eff = sign(u) * ((abs(u) - db) / (1 - db));
    end

    if cfg.throttle_tau_s > 1e-6
        alpha = exp(-max(cfg.dt, 1e-4) / cfg.throttle_tau_s);
        u_filt = alpha * u_filt + (1 - alpha) * u_eff;
    else
        u_filt = u_eff;
    end

    u_filt = clamp(u_filt, 0.0, 1.0);

    % Drive force with explicit back-EMF reduction versus forward speed.
    v_fwd = max(v_current, 0.0);
    emf_scale = 1.0 / (1.0 + cfg.back_emf_gain * v_fwd);
    drive_accel = cfg.max_accel_fwd * (u_filt^cfg.n_throttle) * emf_scale;

    % Forward-only drag model.
    drag = cfg.drag_k0 + cfg.drag_k1 * v_fwd;

    if v_fwd > cfg.v_epsilon
        a_long = drive_accel - drag;
    else
        % Near standstill, prevent drag from pulling velocity negative.
        a_long = max(drive_accel - drag, 0.0);
    end
end

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end

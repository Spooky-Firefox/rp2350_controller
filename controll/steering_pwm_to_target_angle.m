function delta_target = steering_pwm_to_target_angle(pwm_us, max_angle_deg)
% Map steering PWM [1000..2000] to a target steering angle.
% 1500 us -> 0 deg, 1000 us -> -max_angle_deg, 2000 us -> +max_angle_deg.

    if nargin < 2 || isempty(max_angle_deg)
        max_angle_deg = 28;
    end

    u = (pwm_us - 1500) / 500;
    u = clamp(u, -1.0, 1.0);

    delta_target = deg2rad(max_angle_deg) * u;
end

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end

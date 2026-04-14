function sim_data = generate_kalman_data(output_file)
% Generate synthetic ground truth and sensor streams for the RC car EKF.
%
% The generated data is shared by both:
%   - kalman.m
%   - kalman_dead_reckoning.m
% It also plots a quick summary so the synthetic dataset is easy to tune.
%
% Usage:
%   sim_data = generate_kalman_data();
%   sim_data = generate_kalman_data('my_file.mat');

    if nargin < 1 || isempty(output_file)
        output_file = fullfile(fileparts(mfilename('fullpath')), 'kalman_sim_data.mat');
    end

    % --- Vehicle / sensor constants used by the simulated plant ---
    PLANT.L   = 0.265;    % wheelbase [m]
    PLANT.r_w = 0.0325;   % wheel radius [m]
    PLANT.W   = 0.200;    % track width [m]

    % Sensor shaft spins 3x faster than the wheel.
    PLANT.gear_ratio = 3.0;                      % sensor speed / wheel speed [-]
    PLANT.speed_ratio = PLANT.r_w / PLANT.gear_ratio;   % [m/rad]

    % Measurement noise used to synthesize sensor data.
    SENSOR.r_cam_xy    = 0.02;   % [m]
    SENSOR.r_cam_theta = 0.05;   % [rad]
    SENSOR.r_omega     = 0.5;    % [rad/s]

    % Timing of the synthetic data sources.
    TIMING.dt_predict = 0.01;          % [s]
    TIMING.dt_camera  = 0.033;         % [s]
    TIMING.axle_dt_range = [];  % legacy field, not used in rotation-triggered mode
    TIMING.T_end = 10;                 % [s]

    dt_sim = TIMING.dt_predict;
    t_sim  = 0:dt_sim:TIMING.T_end;
    N      = length(t_sim);

    % Steering and acceleration profiles define the simulated path.
    %delta_true = 0.3 * sin(2*pi*0.2*t_sim);
    delta_true = 0*ones(size(t_sim));
    
    a_long_true = 0.8 * ones(size(t_sim));
    a_long_true(t_sim > 5 & t_sim < 7) = -2.5;

    % Integrate the bicycle model to create the ground-truth trajectory.
    x_true = zeros(4, N);
    x_true(:,1) = [0; 0; 0; 0];
    for k = 1:N-1
        x_true(:,k+1) = bicycle_step(x_true(:,k), ...
            delta_true(k), a_long_true(k), dt_sim, PLANT);
    end

    % Camera measurements at a fixed rate.
    cam_period = max(1, round(TIMING.dt_camera / dt_sim));
    cam_idx  = 1:cam_period:N;
    z_cam    = x_true(1:3, cam_idx) + ...
        diag([SENSOR.r_cam_xy; SENSOR.r_cam_xy; SENSOR.r_cam_theta]) * randn(3, length(cam_idx));

    % Axle speed measurements are event-driven: one sample per sensor revolution.
    % This makes sample interval inversely proportional to speed.
    axle_idx = [];
    theta_sensor = 0;  % integrated sensor angle [rad]
    for k = 2:N
        omega_sensor = abs(x_true(4, k-1)) / PLANT.speed_ratio;  % [rad/s]
        theta_sensor = theta_sensor + omega_sensor * dt_sim;
        while theta_sensor >= 2*pi
            axle_idx(end+1) = k; %#ok<SAGROW>
            theta_sensor = theta_sensor - 2*pi;
        end
    end
    axle_idx = unique(axle_idx, 'stable');
    axle_times = t_sim(axle_idx);

    omega_true = abs(x_true(4, axle_idx)) / PLANT.speed_ratio;
    z_axle     = abs(omega_true + SENSOR.r_omega * randn(size(omega_true)));

    sim_data = struct( ...
        'plant', PLANT, ...
        'sensor', SENSOR, ...
        'timing', TIMING, ...
        'dt_sim', dt_sim, ...
        't_sim', t_sim, ...
        'delta_true', delta_true, ...
        'a_long_true', a_long_true, ...
        'x_true', x_true, ...
        'cam_idx', cam_idx, ...
        'z_cam', z_cam, ...
        'axle_times', axle_times, ...
        'axle_idx', axle_idx, ...
        'omega_true', omega_true, ...
        'z_axle', z_axle);

    save(output_file, 'sim_data');
    fprintf('Saved simulation data to %s\n', output_file);

    plot_generated_data(sim_data);
end

function plot_generated_data(sim_data)
    t_sim = sim_data.t_sim;
    x_true = sim_data.x_true;
    delta_true = sim_data.delta_true;
    a_long_true = sim_data.a_long_true;
    cam_idx = sim_data.cam_idx;
    z_cam = sim_data.z_cam;
    axle_times = sim_data.axle_times;
    z_axle = sim_data.z_axle;
    v_axle = z_axle * sim_data.plant.speed_ratio;

    figure('Name', 'Generated Kalman Dataset', 'Position', [120 120 1300 850]);

    subplot(2,3,1); hold on; grid on;
    plot(x_true(1,:), x_true(2,:), 'k-', 'LineWidth', 1.5);
    plot(z_cam(1,:), z_cam(2,:), 'r.', 'MarkerSize', 5);
    xlabel('X [m]'); ylabel('Y [m]'); title('Ground Truth and Camera Samples');
    legend('True trajectory', 'Camera samples');
    axis equal;

    subplot(2,3,2); hold on; grid on;
    plot(t_sim, rad2deg(delta_true), 'b-', 'LineWidth', 1.2);
    xlabel('t [s]'); ylabel('\delta [deg]'); title('Steering Input');

    subplot(2,3,3); hold on; grid on;
    plot(t_sim, a_long_true, 'm-', 'LineWidth', 1.2);
    xlabel('t [s]'); ylabel('a_{long} [m/s^2]'); title('Acceleration Input');

    subplot(2,3,4); hold on; grid on;
    plot(t_sim, x_true(4,:), 'k-', 'LineWidth', 1.2);
    plot(axle_times, v_axle, 'r.', 'MarkerSize', 6);
    xlabel('t [s]'); ylabel('v [m/s]'); title('Speed and Axle-Speed Samples');
    legend('True speed', 'Measured |v|');

    subplot(2,3,5); hold on; grid on;
    plot(t_sim, rad2deg(x_true(3,:)), 'k-', 'LineWidth', 1.2);
    plot(t_sim(cam_idx), rad2deg(z_cam(3,:)), 'r.', 'MarkerSize', 5);
    xlabel('t [s]'); ylabel('\theta [deg]'); title('Heading and Camera Heading');
    legend('True heading', 'Measured heading');

    subplot(2,3,6); hold on; grid on;
    stairs([0, axle_times], [0, diff([0, axle_times])], 'b-', 'LineWidth', 1.2);
    xlabel('t [s]'); ylabel('\Delta t [s]'); title('Axle Sensor Inter-arrival Time');

    sgtitle('Synthetic Dataset Summary for EKF Tuning');
end

function x_next = bicycle_step(x, delta, a_long, dt, C)
    X     = x(1);
    Y     = x(2);
    theta = x(3);
    v     = x(4);

    x_next = [
        X     + v * cos(theta) * dt
        Y     + v * sin(theta) * dt
        wrap_pi(theta + (v / C.L) * tan(delta) * dt)
        v     + a_long * dt
    ];
end

function a = wrap_pi(a)
    a = mod(a + pi, 2*pi) - pi;
end
% =========================================================================
% STEWART PLATFORM 360° AZIMUTH SWEEP & XY TRANSLATION - DIGITAL TWIN SIMULATION
% =========================================================================
clear; clc; close all;

%% 1. PHYSICAL DIMENSIONS & LIMIT CONSTRAINTS
L_short = 5.1;          % Servo horn length (cm)
L_long = 17.1;          % Connecting rod length (cm)
r_base = 8.36;          % Base motor radius (cm)
r_plat = 8.13;          % Platform anchor radius (cm)
Z_home = 12.13;         % Equilibrium baseline height (cm)

MIN_ARM_ANGLE_VERT_DEG = 18.0;  % Limit switch floor
MAX_ARM_ANGLE_VERT_DEG = 136.0; 

%% 2. DEFINE SWEEP & TRANSLATION PARAMETERS
tilt_deg = 10.0;        % Constant tilt angle across sweep
xy_translate_mag = 3; % Translational shift magnitude in the XY plane (cm)
azimuth_step = 5.0;     % Step size in degrees
azimuth_angles = 0 : azimuth_step : 360;

theta_tilt = deg2rad(tilt_deg);

%% 3. GEOMETRY SETUP
face_centers_deg = [0, 120, 240];
base_offsets_deg  = [-14.37, 14.37]; % Base motor pairing offsets
plat_offsets_deg  = [-2.33,   2.33];  % Platform joint pairing offsets

Base_Points = zeros(3, 6);
Platform_Local = zeros(3, 6);
base_angles_deg = zeros(1, 6);

idx = 1;
for f = 1:3
    center = face_centers_deg(f);
    for p = 1:2
        b_ang = center + base_offsets_deg(p);
        base_angles_deg(idx) = b_ang;
        theta_b = deg2rad(b_ang);
        Base_Points(:, idx) = [r_base * cos(theta_b); r_base * sin(theta_b); 0.0];
        
        p_ang = center + plat_offsets_deg(p);
        theta_p = deg2rad(p_ang);
        Platform_Local(:, idx) = [r_plat * cos(theta_p); r_plat * sin(theta_p); 0.0];
        idx = idx + 1;
    end
end

%% 4. EXECUTE 360° SIMULATION SWEEP WITH XY TRANSLATION & VISUALIZE
disp('=== STARTING 360° THEORETICAL SWEEP + XY TRANSLATION SIMULATION ===');
figure('Name', 'Stewart Platform - Sweep with XY Translation', 'Color', 'w');

% Pre-allocate storage for recorded trajectory
trajectory_angles = zeros(length(azimuth_angles), 6);

for k = 1:length(azimuth_angles)
    az = azimuth_angles(k);
    phi_azimuth = deg2rad(az);
    
    % --- NEW: Dynamic XY Translational Shift towards Azimuth ---
    X_offset = xy_translate_mag * cos(phi_azimuth);
    Y_offset = xy_translate_mag * sin(phi_azimuth);
    
    % Rotation Matrix (Tilt Axis perpendicular to azimuth direction)
    axis_x = sin(phi_azimuth);
    axis_y = -cos(phi_azimuth);
    rot_axis = [axis_x; axis_y; 0];
    K = [0, -rot_axis(3), rot_axis(2); rot_axis(3), 0, -rot_axis(1); -rot_axis(2), rot_axis(1), 0];
    R_tilt = eye(3) + sin(theta_tilt)*K + (1 - cos(theta_tilt))*(K^2);
    
    % Auto-Lift Z Optimization (Evaluates valid workspace at current X, Y, Tilt)
    Z_optimized = Z_home;
    found_valid = false;
    for z_test = Z_home : 0.05 : (Z_home + 10.0)
        T_test = [X_offset; Y_offset; z_test];
        [~, valid_test] = evaluate_ik(R_tilt, T_test, Base_Points, Platform_Local, base_angles_deg, L_short, L_long, MIN_ARM_ANGLE_VERT_DEG, MAX_ARM_ANGLE_VERT_DEG);
        if valid_test
            Z_optimized = z_test;
            found_valid = true;
            break;
        end
    end
    
    if ~found_valid
        fprintf('Azimuth %3.1f°: Out of workspace bounds! Skipping.\n', az);
        continue;
    end
    
    % Solve IK angles at optimized Z and dynamic XY offset
    T_tilt = [X_offset; Y_offset; Z_optimized];
    [arm_angles_vert_deg, ~] = evaluate_ik(R_tilt, T_tilt, Base_Points, Platform_Local, base_angles_deg, L_short, L_long, MIN_ARM_ANGLE_VERT_DEG, MAX_ARM_ANGLE_VERT_DEG);
    trajectory_angles(k, :) = arm_angles_vert_deg;
    
    % Live Plotting Update
    clf; hold on; grid on; axis equal; view(3);
    
    % Draw Base
    scatter3(Base_Points(1,:), Base_Points(2,:), Base_Points(3,:), 80, 'r', 'filled');
    
    % Draw Platform (Translated & Rotated)
    Platform_Global = R_tilt * Platform_Local + T_tilt;
    scatter3(Platform_Global(1,:), Platform_Global(2,:), Platform_Global(3,:), 80, 'b', 'filled');
    
    % Draw Links (Servos and Rods)
    for i = 1:6
        face_idx = floor((i-1)/2) + 1;
        face_theta = deg2rad(face_centers_deg(face_idx));
        tx = -sin(face_theta); ty = cos(face_theta);
        side_sign = (mod(i, 2) == 0)*2 - 1; 
        theta_vert_rad = deg2rad(arm_angles_vert_deg(i));
        B = Base_Points(:, i); P = Platform_Global(:, i);
        E = B + L_short * sin(theta_vert_rad) * (side_sign * [tx; ty; 0]) + L_short * cos(theta_vert_rad) * [0; 0; 1];
        plot3([B(1), E(1)], [B(2), E(2)], [B(3), E(3)], 'Color', [1, 0.5, 0], 'LineWidth', 3);
        plot3([E(1), P(1)], [E(2), P(2)], [E(3), P(3)], 'g-', 'LineWidth', 2);
    end
    
    % Draw Platform Disk Boundary
    theta_circle = linspace(0, 2*pi, 100);
    plate_local = [r_plat * cos(theta_circle); r_plat * sin(theta_circle); zeros(1, length(theta_circle))];
    plate_global = R_tilt * plate_local + T_tilt;
    fill3(plate_global(1,:), plate_global(2,:), plate_global(3,:), 'c', 'FaceAlpha', 0.25, 'EdgeColor', 'b', 'LineWidth', 1.5);
    
    % Peak Azimuth & Translation Vector Arrow
    peak_local_dir = [r_plat * cos(phi_azimuth); r_plat * sin(phi_azimuth); 0];
    peak_global = R_tilt * peak_local_dir + T_tilt;
    quiver3(0, 0, 0, T_tilt(1), T_tilt(2), T_tilt(3), 0, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5); % XY Shift Vector
    quiver3(T_tilt(1), T_tilt(2), T_tilt(3), peak_global(1)-T_tilt(1), peak_global(2)-T_tilt(2), peak_global(3)-T_tilt(3), 0, 'm', 'LineWidth', 3, 'MaxHeadSize', 0.5);
    
    title(sprintf('Sweep + XY Translation | Az: %.1f° | Tilt: %.1f° | XY Shift: %.2f cm | Z-Lift: +%.2f cm', ...
        az, tilt_deg, xy_translate_mag, Z_optimized - Z_home));
    xlim([-18 18]); ylim([-18 18]); zlim([-5 25]);
    drawnow;
end

disp('=== SIMULATION COMPLETE ===');

%% --- IK EVALUATION FUNCTION ---
function [angles_vert_deg, valid] = evaluate_ik(R, T, Base_Points, Platform_Local, base_angles_deg, L_s, L_l, min_limit, max_limit)
    angles_vert_deg = zeros(1, 6);
    valid = true;
    Platform_Global = R * Platform_Local + T;
    
    for i = 1:6
        P = Platform_Global(:, i);
        B = Base_Points(:, i);
        face_idx = floor((i-1)/2) + 1;
        face_theta = deg2rad(base_angles_deg(2*face_idx - 1));
        side_sign = (mod(i, 2) == 0)*2 - 1;
        tx = -sin(face_theta); ty = cos(face_theta);
        
        BP = P - B;
        z_rel = BP(3);
        xy_proj = dot(BP, side_sign * [tx; ty; 0]);
        r_rel = sqrt(xy_proj^2 + z_rel^2);
        
        if r_rel > (L_s + L_l) || r_rel < abs(L_s - L_l)
            valid = false; return;
        end
        
        cos_beta = (L_s^2 + r_rel^2 - L_l^2) / (2 * L_s * r_rel);
        cos_beta = max(-1, min(1, cos_beta));
        beta = acos(cos_beta);
        gamma = atan2(z_rel, xy_proj);
        
        theta_swing = gamma - beta;
        theta_vert_deg = 90.0 - rad2deg(theta_swing);
        
        if theta_vert_deg < min_limit || theta_vert_deg > max_limit
            valid = false; return;
        end
        angles_vert_deg(i) = theta_vert_deg;
    end
end

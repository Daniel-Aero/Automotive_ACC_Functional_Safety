%% Automotive ACC with Functional Safety (Emergency Braking + Safe State)
% Author: Daniel-Aero
%
% Description:
%   - Adaptive Cruise Control (ACC) with:
%       1) Lead vehicle emergency braking scenario
%       2) Radar sensor failure → safe-state fallback
%   - Demonstrates safety-aware longitudinal control inspired by ISO 26262.
%
%   Timeline:
%     *  t < 10 s   : Normal driving, lead vehicle at constant speed
%     * 10–12 s     : Lead vehicle performs emergency braking
%     * 12–18 s     : New lower lead speed, ACC follows
%     *  t >= 18 s  : Radar sensor failure → ego enters low-speed safe state

clear; close all; clc;


%% ========== 1. Simulation Settings ==========
dt = 0.01;          % [s] simulation time step
T  = 30;            % [s] total simulation duration
N  = T/dt;          % number of simulation steps


%% ========== 2. Initial States ==========
% Ego vehicle (our car)
v_ego = 0;          % [m/s] initial ego speed
x_ego = 0;          % [m]   initial ego position

% Lead vehicle (front car)
v_lead = 20;        % [m/s] initial lead speed (~72 km/h)
x_lead = 40;        % [m]   lead starts 40 m ahead of ego
a_lead = 0;         % [m/s^2] lead acceleration (will be used for braking)


%% ========== 3. Control Targets & Distance Policy ==========
v_ref_nominal = 25; % [m/s] nominal desired cruise speed (~90 km/h)
v_ref = v_ref_nominal; % this may change in safe-state mode

d0 = 5;             % [m] minimum standstill distance
Th = 1.5;           % [s] time headway (safe following distance)
% Desired distance:
%   d_des = d0 + Th * v_ego


%% ========== 4. Controller Gains ==========
Kp_v = 0.5;         % speed control gain
Kp_d = 0.3;         % distance control gain


%% ========== 5. Logging Buffers ==========
t_hist    = zeros(1, N);
v_ego_h   = zeros(1, N);
v_lead_h  = zeros(1, N);
d_hist    = zeros(1, N);
d_des_h   = zeros(1, N);
a_cmd_h   = zeros(1, N);
a_lead_h  = zeros(1, N);
v_ref_h   = zeros(1, N);   % log reference speed (for safe-state change)


%% ========== 6. Main Simulation Loop ==========
for k = 1:N
    t = (k-1)*dt;      % current time [s]
    
    
    %% --- 6.1 Lead vehicle emergency braking profile ---
    % Scenario:
    %   - Before 10 s: a_lead = 0        (no braking)
    %   - 10–12 s    : a_lead = -4 m/s^2 (hard braking)
    %   - After 12 s : a_lead = 0        (keep new lower speed)
    if t >= 10 && t < 12
        a_lead = -4.0;      % emergency braking
    else
        a_lead = 0.0;       % no additional braking
    end
    
    % Update lead speed, ensuring it does not go below zero
    v_lead = max(v_lead + a_lead*dt, 0);
    
    % Update lead position
    x_lead = x_lead + v_lead*dt;
    
    
    %% --- 6.2 Sensor fault & safe-state transition ---
    % Assume radar sensor fails after 18 seconds.
    % When the sensor is not OK, the ego vehicle enters a low-speed mode
    % to reduce collision risk (simple safe-state concept).
    sensor_ok = (t < 18);   % true until 18 s, then false
    
    if sensor_ok
        v_ref = v_ref_nominal;  % normal cruise speed
    else
        v_ref = 5;              % [m/s] fallback safe-state speed (~18 km/h)
    end
    
    
    %% --- 6.3 Gap distance and desired distance ---
    d    = x_lead - x_ego;        % [m] actual gap
    d_des = d0 + Th * v_ego;      % [m] desired safe gap
    
    
    %% --- 6.4 Speed control (cruise mode) ---
    e_v = v_ref - v_ego;          % speed tracking error
    a_v = Kp_v * e_v;             % speed-control acceleration command
    
    
    %% --- 6.5 Distance control (following mode) ---
    e_d = d - d_des;              % distance error
    a_d = Kp_d * e_d;             % distance-control acceleration command
    
    
    %% --- 6.6 Mode switching (ACC logic) ---
    if d < d_des
        % Too close to the lead vehicle → prioritize distance safety.
        a_cmd = min(a_v, a_d);
    else
        % Gap is safe → follow nominal speed command.
        a_cmd = a_v;
    end
    
    
    %% --- 6.7 Ego actuator saturation ---
    a_max = 2.0;   % [m/s^2] max acceleration
    a_min = -3.0;  % [m/s^2] max braking
    
    if a_cmd > a_max
        a_cmd = a_max;
    elseif a_cmd < a_min
        a_cmd = a_min;
    end
    
    
    %% --- 6.8 Update ego vehicle dynamics ---
    v_ego = v_ego + a_cmd*dt;
    x_ego = x_ego + v_ego*dt;
    
    
    %% --- 6.9 Store histories for plotting ---
    t_hist(k)    = t;
    v_ego_h(k)   = v_ego;
    v_lead_h(k)  = v_lead;
    d_hist(k)    = d;
    d_des_h(k)   = d_des;
    a_cmd_h(k)   = a_cmd;
    a_lead_h(k)  = a_lead;
    v_ref_h(k)   = v_ref;
end


%% ========== 7. Plot Results ==========

% 7.1 Ego & lead speeds + reference speed
figure;
plot(t_hist, v_ego_h, 'LineWidth', 2); hold on;
plot(t_hist, v_lead_h, '--', 'LineWidth', 2);
plot(t_hist, v_ref_h, ':k', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Speed [m/s]');
title('ACC with Emergency Braking and Safe-State Speed Reference');
legend('Ego speed','Lead speed','Reference/Safe-state speed','Location','southeast');

% 7.2 Distance vs desired safe distance
figure;
plot(t_hist, d_hist, 'LineWidth', 2); hold on;
plot(t_hist, d_des_h, '--', 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Gap Distance [m]');
title('Gap Distance vs Desired Safe Distance');
legend('Actual distance','Desired distance','Location','best');

% 7.3 Ego command acceleration
figure;
plot(t_hist, a_cmd_h, 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Ego Command Acceleration [m/s^2]');
title('Ego Control Command under Emergency Braking + Safe-State');

% 7.4 Lead braking profile
figure;
plot(t_hist, a_lead_h, 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Lead Acceleration [m/s^2]');
title('Lead Vehicle Emergency Braking Profile');

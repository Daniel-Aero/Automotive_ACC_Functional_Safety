%% Adaptive Cruise Control (ACC) with Lead Vehicle
% Author: Daniel-Aero
%
% Description:
%   - This script simulates a simple Adaptive Cruise Control (ACC) system.
%   - The ego vehicle (our car) tries to:
%       1) Reach and maintain a desired cruise speed in free-flow traffic.
%       2) Follow a lead vehicle while keeping a safe distance.
%   - Two control objectives are combined:
%       - Speed control (cruise control mode)
%       - Distance control (car-following mode)
%   - A simple mode-switching logic is used:
%       - If gap distance is large enough → speed control dominates.
%       - If gap distance becomes too small → distance control dominates.
%
%   This model is intentionally simple but captures the essential behavior
%   of a longitudinal ACC system and can be extended with safety / fault
%   scenarios (ISO 26262-inspired).

clear;          % Clear all variables from workspace
close all;      % Close figures
clc;            % Clear command window


%% ================= 1. Simulation Settings =================
dt = 0.01;      % [s] simulation time step
T  = 30;        % [s] total simulation duration
N  = T/dt;      % number of simulation steps

% Example:
%   T = 30 s, dt = 0.01 s → N = 3000 loop iterations.


%% ================= 2. Vehicle Initial States =================
% Ego vehicle (our car)
v_ego = 0;      % [m/s] initial ego speed
x_ego = 0;      % [m]   initial ego position

% Lead vehicle (vehicle in front)
v_lead = 20;    % [m/s] constant speed of lead vehicle (~72 km/h)
x_lead = 40;    % [m]   initial position of lead, 40 m ahead of ego

% Notes:
%   - At t = 0, ego is standing still, lead is already moving.
%   - Ego will accelerate and try to catch up while being safe.


%% ================= 3. Control Targets & Safety Policy =================
% Desired cruise speed for ego (free-driving condition)
v_ref = 25;     % [m/s] (~90 km/h)

% Time-gap policy parameters for safe distance:
d0 = 5;         % [m] standstill (minimum) gap
Th = 1.5;       % [s] time headway
% Desired distance:
%   d_des = d0 + Th * v_ego
%   - As speed increases, required safe gap increases.
%   - Widely used concept in real ACC systems.


%% ================= 4. Controller Gains =================
% Speed controller (cruise control part)
Kp_v = 0.5;     % proportional gain for speed error

% Distance controller (car-following part)
Kp_d = 0.3;     % proportional gain for distance error

% These are tuning parameters:
%   - Larger Kp → faster response but risk of overshoot/oscillation.
%   - Smaller Kp → slower but smoother response.


%% ================= 5. Logging Buffers =================
t_hist   = zeros(1, N);   % time history
v_ego_h  = zeros(1, N);   % ego speed history
v_lead_h = zeros(1, N);   % lead speed history
d_hist   = zeros(1, N);   % actual distance gap history
d_des_h  = zeros(1, N);   % desired distance gap history
a_cmd_h  = zeros(1, N);   % command acceleration history


%% ================= 6. Main Simulation Loop =================
for k = 1:N
    t = (k-1)*dt;     % current simulation time
    
    %% --- 6.1 Update lead vehicle state ---
    % In this basic model, the lead vehicle drives at a constant speed.
    % Later, you can modify v_lead over time to simulate braking events.
    x_lead = x_lead + v_lead*dt;
    
    
    %% --- 6.2 Compute gap distance and desired distance ---
    d = x_lead - x_ego;           % [m] actual gap (lead - ego)
    d_des = d0 + Th * v_ego;      % [m] desired safe gap based on time headway
    
    
    %% --- 6.3 Speed control (cruise mode) ---
    % Speed tracking error:
    %   e_v > 0 → ego is slower than desired speed → need to accelerate.
    %   e_v < 0 → ego is faster than desired speed → need to decelerate.
    e_v = v_ref - v_ego;
    
    % P-controller for speed:
    a_v = Kp_v * e_v;             % [m/s^2] speed-control based acceleration
    
    
    %% --- 6.4 Distance control (following mode) ---
    % Distance error:
    %   e_d > 0 → ego is farther than desired → can accelerate.
    %   e_d < 0 → ego is too close → must decelerate.
    e_d = d - d_des;
    
    % P-controller for distance:
    a_d = Kp_d * e_d;             % [m/s^2] distance-control acceleration
    
    
    %% --- 6.5 Mode-switching logic for ACC ---
    % Idea:
    %   - If actual gap is smaller than desired → prioritize distance safety.
    %   - If gap is larger than desired → follow the speed target.
    %
    % Note:
    %   - min(a_v, a_d) tends to choose the more conservative (usually smaller)
    %     acceleration when too close to the lead vehicle.
    
    if d < d_des
        % Too close to lead vehicle:
        % Distance control should dominate (usually braking).
        a_cmd = min(a_v, a_d);
    else
        % Gap is safe:
        % Use normal speed control (cruise control behavior).
        a_cmd = a_v;
    end
    
    
    %% --- 6.6 Actuator saturation (physical limits) ---
    a_max = 2.0;      % [m/s^2] maximum comfortable acceleration
    a_min = -3.0;     % [m/s^2] maximum comfortable braking
    
    if a_cmd > a_max
        a_cmd = a_max;
    elseif a_cmd < a_min
        a_cmd = a_min;
    end
    
    
    %% --- 6.7 Update ego vehicle dynamics ---
    % Simple longitudinal point-mass model:
    %   dv/dt = a_cmd  →  v(k+1) = v(k) + a_cmd * dt
    %   dx/dt = v      →  x(k+1) = x(k) + v * dt
    v_ego = v_ego + a_cmd * dt;
    x_ego = x_ego + v_ego * dt;
    
    
    %% --- 6.8 Store history for plotting ---
    t_hist(k)   = t;
    v_ego_h(k)  = v_ego;
    v_lead_h(k) = v_lead;
    d_hist(k)   = d;
    d_des_h(k)  = d_des;
    a_cmd_h(k)  = a_cmd;
end


%% ================= 7. Plot Results =================

% ---------- 7.1 Speed Tracking ----------
figure;
plot(t_hist, v_ego_h, 'LineWidth', 2); hold on;
plot(t_hist, v_lead_h, '--', 'LineWidth', 2);
yline(v_ref, ':k', 'LineWidth', 1.5);  % Optional: show desired speed
grid on;
xlabel('Time [s]');
ylabel('Speed [m/s]');
title('ACC - Speed Tracking Behavior');
legend('Ego speed','Lead speed','Reference speed','Location','southeast');

% Interpretation:
%   - When the road is free, ego speed tends toward v_ref.
%   - When approaching the lead vehicle, ego speed should not exceed
%     the lead speed by too much and will eventually settle near it.


% ---------- 7.2 Distance Control ----------
figure;
plot(t_hist, d_hist, 'LineWidth', 2); hold on;
plot(t_hist, d_des_h, '--', 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Gap Distance [m]');
title('ACC - Distance vs. Desired Safe Distance');
legend('Actual distance','Desired distance','Location','best');

% Interpretation:
%   - If the controller works well, the actual distance will stay
%     above or around the desired safe distance curve.


% ---------- 7.3 Command Acceleration ----------
figure;
plot(t_hist, a_cmd_h, 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Command Acceleration [m/s^2]');
title('ACC - Control Command (a_{cmd})');

% Interpretation:
%   - Shows how aggressively the controller accelerates or brakes.
%   - Peaks should stay within the saturation limits [a_min, a_max].

%% Basic Cruise Control Simulation (with detailed English descriptions)
% Author: Daniel-Aero
% Description:
%   - This script simulates a basic Cruise Control (CC) system.
%   - The ego vehicle attempts to reach and maintain a target speed.
%   - Controller used: Proportional (P) control.
%   - This is the foundational step before Adaptive Cruise Control (ACC).

clear;          % Clear all variables from workspace
close all;      % Close any open figure windows
clc;            % Clear command window text


%% ================= Simulation Settings =================
dt = 0.01;      % Simulation time step [sec]
T  = 20;        % Total simulation duration [sec]
N  = T/dt;      % Number of simulation steps (loop iterations)

% Example:
%   T = 20 sec, dt = 0.01 sec  →  N = 2000 simulation updates


%% ================= Vehicle States =================
v = 0;          % Ego vehicle initial speed [m/s]
x = 0;          % Ego vehicle initial position [m]

% State Definitions:
%   v : longitudinal speed
%   x : longitudinal displacement


%% ================= Control Target =================
v_ref = 25;     % Target cruise speed [m/s] (~90 km/h)


%% ================= Controller Gain =================
Kp = 0.3;       % Proportional gain (tuning parameter)

% Notes:
%   - If Kp is too large → unstable (oscillations)
%   - If Kp is too small → slow response


%% ================= Logging Buffers =================
t_hist = zeros(1, N);   % Store time values
v_hist = zeros(1, N);   % Store speed values
x_hist = zeros(1, N);   % Store position values
a_hist = zeros(1, N);   % Store acceleration commands


%% ================= Main Simulation Loop =================
for k = 1:N
    t = (k-1)*dt;   % Current simulation time [sec]
    
    % --- Speed Control Error ---
    %   Difference between target speed and actual speed
    e_v = v_ref - v;
    
    % --- P-Control Output (Acceleration Command) ---
    a_cmd = Kp * e_v;
    
    % --- Actuator Saturation ---
    %   Limit realistic acceleration & braking capability
    a_max = 2.0;    % Maximum acceleration [m/s^2]
    a_min = -3.0;   % Maximum braking [m/s^2]
    
    if a_cmd > a_max
        a_cmd = a_max;
    elseif a_cmd < a_min
        a_cmd = a_min;
    end
    
    % --- Vehicle Dynamics Update ---
    % Speed update using discrete-time integration
    v = v + a_cmd * dt;
    
    % Position update using new speed
    x = x + v * dt;
    
    % --- Save Simulation Data for Plotting ---
    t_hist(k) = t;
    v_hist(k) = v;
    x_hist(k) = x;
    a_hist(k) = a_cmd;
end


%% ================= Plot Results =================

% --- Speed Response Plot ---
figure;
plot(t_hist, v_hist, 'LineWidth', 2);
hold on;
yline(v_ref, '--r', 'LineWidth', 1.5); % Target speed reference line
grid on;
xlabel('Time [s]');
ylabel('Speed [m/s]');
title('Basic Cruise Control - Speed Response');
legend('Vehicle Speed','Reference Speed','Location','southeast');

% --- Acceleration Command Plot ---
figure;
plot(t_hist, a_hist, 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Acceleration Command [m/s^2]');
title('Control Command Output (P-Control)');

% --- Position Plot ---
figure;
plot(t_hist, x_hist, 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Position [m]');
title('Vehicle Position Over Time');


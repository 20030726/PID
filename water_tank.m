clc;
clear;
close all;
%% Water tank system

% Parameters
rho = 1000; % kg/m³
full_tank = 100; % m³
tank_length = 10; % m
tank_width = 1; % m
tank_height = 10; % m

% Initial condition
water_height = 0; % m
v_0 = water_height * tank_length * tank_width; % m³
m_dot_0 = 0; % kg/s

% Time span
Hz = 50; % times/s
delta_t = 1/Hz;
t = 0:delta_t:100;

% Dynamics model
%f = @(m_dot) (1/rho) * m_dot;

%% Controller

% PID Parameters
Kp = 1000;
Ki = 30;
Kd = 300;

% Goal
v_desired = 80; % m³

%% Loop
v = zeros(length(t), 1);
m_dot = zeros(length(t), 1);
e = zeros(length(t), 1);
v(1) = v_0;
m_dot(1) = m_dot_0;
Integral_e = 0;

for i = 2:length(t)
    % Desired volume
    if t(i) < 8
        v_desired = 80; % m³
    elseif t(i) < 15
        v_desired = 50; % m³
    else
        v_desired = 20; % m³
    end
    % Error
    e(i-1) = v_desired - v(i-1);
    % Break condition
    if abs(e(i-1)) < 1e-2
        % Truncate data
        v = v(1:i-1);
        m_dot = m_dot(1:i-1);
        e = e(1:i-1);
        t = t(1:i-1);
        break;
    end
    % Control input
    P_controller = Kp * e(i-1);
    %Integral_e = Integral_e + e(i-1) * delta_t;
    %I_controller = Ki * Integral_e;
    % if i > 2
    %     D_controller = Kd * (e(i-1) - e(i-2)) / delta_t;
    % else
    %     D_controller = 0;
    % end

    % System update
    m_dot(i-1) = P_controller;
    %m_dot(i-1) = P_controller + I_controller + D_controller;
    v(i) = v(i-1) + (1/rho) * (m_dot(i-1) + m_dot(i)) / 2 * delta_t;

    % Compute the water height for the current volume
    water_height = v(i) / (tank_length * tank_width);
    desired_height = v_desired / (tank_length * tank_width);
    
    %% Plot the tank and water as animation
    figure(1);
    clf;
    % Plot the tank occupying the first four positions
    subplot(2, 3, [1 2 4 5]); % Occupy the left four positions
    hold on;

    % Draw tank
    rectangle('Position', [0, 0, tank_length, tank_height], 'EdgeColor', 'black', 'LineWidth', 2);

    % Draw water
    rectangle('Position', [0, 0, tank_length, water_height], 'FaceColor', 'b');

    % Draw the desired water level line (red)
    line([0, tank_length], [desired_height, desired_height], 'Color', 'r', 'LineWidth', 2); % Red target line

    xlabel('Length (m)');
    ylabel('Height (m)');
    title(sprintf('Water Tank Volume: %.2f m³', v(i)));
    axis([0 tank_length 0 tank_height]);

    grid on;

    % Plot error in the 3rd position
    subplot(2, 3, 3);
    plot(t(1:i), e(1:i)); % Assuming an error array e
    xlabel('Time (s)');
    ylabel('Error');
    title('Error over Time');
    grid on;

    % Plot mass flow rate in the 6th position
    subplot(2, 3, 6);
    plot(t(1:i), m_dot(1:i)); % Assuming a mass flow rate array m_dot
    xlabel('Time (s)');
    ylabel('Mass Flow Rate (kg/s)');
    title('Mass Flow Rate over Time');
    grid on;

    drawnow;

    % Ensure volume stays within physical limits before calculating control input
    if v(i-1) >= full_tank
        v(i-1) = full_tank; % Cap the volume to the full tank capacity
        m_dot(i-1) = 0; % Stop inflow if the tank is full
        disp('Tank is full');
        continue; % Skip the rest of the iteration to avoid updating the volume
    elseif v(i-1) <= 0
        v(i-1) = 0; % Prevent negative volume
        m_dot(i-1) = 0; % Stop outflow if the tank is empty
        disp('Tank is empty');
        disp(['i=', num2str(i)]);
        continue; % Skip the rest of the iteration
    end
end

%% Plot results
figure;
plot(t, v, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Water Volume (m³)');
title('Water Tank Volume Control using PID');
grid on;
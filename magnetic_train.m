clc;
clear;
close all;
%% Environmnet Parameters
g = 9.81; % m/s^2
%% Slide rail
x_boundary = [0 120];% m
y_boundary = [0 50];% m
theta = 0*pi/180; % rad
%% Magnetic_train system
% Parameters
M = 1000; % kg
cart_width = 10; % m
cart_height = 5; % m

% Initial condition
F_a_0 = 0; % kg/s
x_0 = 100; % m
v_I_0 = 0; % m/s
I_0 = x_0/cos(theta); % m
% Time span
Hz = 50; % times/s
delta_t = 1/Hz;
t = 0:delta_t:100;

% Dynamics model
%dvadt = -g*sin(theta) - 1/M * F_a;

%% Drop box system
% Parameters
box_width = 5; % m
box_height = 5; % m

% Initial condition
x_desired = x_boundary(1)+(x_boundary(2)-x_boundary(1))*rand;% uniform distrbution
y_desired_0 = 50;% m
vy_desired_0 = 0; % m/s
%% Controller

% PID Parameters
Kp = 1000;
Ki = 30;
Kd = 30;

% Goal

%% Loop
y_desired = zeros(length(t), 1);
F_a = zeros(length(t), 1);
v_I = zeros(length(t), 1);
I = zeros(length(t), 1);
e = zeros(length(t), 1);
y_desired(1) = y_desired_0;
F_a(1) = F_a_0;
v_I(1) = v_I_0;
I(1) = I_0;
Integral_e = 0;

for i = 2:length(t)
    % Error
    e(i-1) = sqrt((x_desired - I(i-1)*cos(theta))^2 + (y_desired(i-1) - I(i-1)*sin(theta))^2);
    % Break condition
    if abs(e(i-1)) < 1e-2
        % Truncate data
        I = I(1:i-1);
        F_a = F_a(1:i-1);
        e = e(1:i-1);
        t = t(1:i-1);
        break;
    end
    % Control input
    P_controller = Kp * e(i-1);
    Integral_e = Integral_e + e(i-1) * delta_t;
    I_controller = Ki * Integral_e;
    if i > 2
        D_controller = Kd * (e(i-1) - e(i-2)) / delta_t;
    else
        D_controller = 0;
    end

    % System update
    
    % cart
    F_a(i-1) = P_controller + I_controller + D_controller;

    Integral_F_a(i) = (F_a(i-1) + F_a(i)) / 2 * delta_t;
    vI(i) = v_I_0 - g*sin(theta)*t(i) - 1/M * Integral_F_a(i);

    Integral_Integral_F_a(i) = (Integral_F_a(i-1) + Integral_F_a(i)) / 2 * delta_t;
    I(i) = I_0 + v_I_0 - 1/2*g*sin(theta)*t(i)^2 - 1/M * Integral_Integral_F_a(i);
    
    % drop box
    y_desired(i) = y_desired_0 + vy_desired_0*t(i) -1/2*g*t(i)^2;

   
    
 %% Plot the cart and the falling box
    figure(1);
    clf;
    hold on;
    
    % Plot the slope
    plot([x_boundary(1), x_boundary(2)], [0, x_boundary(2) * tan(theta)], 'k-', 'LineWidth', 2);
    
    % Plot the cart as a rectangle on the slope
    cart_x = I(i)*cos(theta) - 0.5*cart_width; % Position of the cart along the slope
    cart_y = I(i)*sin(theta) - 0.5*cart_height; % Vertical position of the cart
    rectangle('Position', [cart_x, cart_y, cart_width, cart_height], ...
              'FaceColor', 'b', 'EdgeColor', 'b');%[x, y, width, height]
    
    % Plot the falling box as a rectangle
    rectangle('Position', [x_desired - box_width / 2, y_desired(i) - box_height / 2, box_width, box_height], ...
              'FaceColor', 'r', 'EdgeColor', 'r');
    
    % Plot the force vectors (e.g., gravitational force on the cart)
    quiver(cart_x + cart_width / 2, cart_y + cart_height / 2, ...
           0, -M * g / 1000, 'r', 'LineWidth', 2); % Gravitational force (scaled down for visualization)
    
    % Set the axis limits and labels
    axis equal;
    xlim([x_boundary(1) - 10, x_boundary(2) + 10]); % Add some padding to x-axis
    ylim(y_boundary); % Add some padding to y-axis
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title(sprintf('Time: %.2f s', t(i)));
    
    grid on;
    drawnow;

end
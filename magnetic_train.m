clc
clear
close all

%% Simulation Parameters
num_simulations = 10; % Number of simulations to run
sim_time = 100; % Total simulation time
Hz = 50;
delta_t = 1/Hz;
t = 0:delta_t:sim_time;

%% Environment Parameters
g = 9.81; % m/s^2
x_boundary = [0 120]; % m [left right]
y_boundary = [0 50]; % m [bottom top]
theta = 15*pi/180; % rad
M = 1000; % kg

% Controller Parameters
Kp = 300;
Ki = 30;
Kd = 300;

%% Initialize storage for results
errors = zeros(length(t), num_simulations);
results = strings(1, num_simulations); % To store success/fail results
success_time = 0;
fail_time = 0;

figure;

for sim_num = 1:num_simulations
    %% Initial Condition for Each Simulation
    % Box parameters
    x_box = x_boundary(1) + (x_boundary(2) - x_boundary(1)) * rand; % Random initial position
    y_box_0 = y_boundary(2); % m drop height Initial
    vy_box_0 = 0; % m/s
    box_width = 5; % m
    box_height = 5; % m
    
    % Magnetic Train system
    x_train_0 = x_boundary(1) + (x_boundary(2) - x_boundary(1)) * rand; % Random initial position
    y_train_0 = x_train_0 * tan(theta); % m
    train_width = 10; % m
    train_height = 5; % m
    
    % Slide rail
    I_0 = x_train_0 / cos(theta); % m
    vI_0 = 0; % m/s
    
    %% Matrix Setup for this Simulation
    e = zeros(length(t), 1);
    I = zeros(length(t), 1);
    vI = zeros(length(t), 1);
    x_train = zeros(length(t), 1);
    y_train = zeros(length(t), 1);
    y_box = zeros(length(t), 1);
    Integral_e = zeros(length(t), 1);
    Differential_e = zeros(length(t), 1);
    F_a = zeros(length(t), 1);
    Integral_F_a = zeros(length(t), 1);
    IIntegral_F_a = zeros(length(t), 1);
    
    % Initialize Matrix
    I(1) = I_0;
    vI(1) = vI_0;
    x_train(1) = x_train_0;
    y_train(1) = y_train_0;
    y_box(1) = y_box_0;
    %Integral_e(1) = 0;
    %Differential_e(1) = 0;
    %% Initialize Plot
    figure(1);
    hold on;
    plot([x_boundary(1), x_boundary(2)], [0, x_boundary(2) * tan(theta)], 'k-', 'LineWidth', 2);
    train_rect = rectangle('Position', [x_train_0-train_width/2, y_train_0-train_height/2, train_width, train_height], ...
        'FaceColor', 'b', 'EdgeColor', 'b');
    box_rect = rectangle('Position', [x_box-box_width/2, y_box_0-box_height/2, box_width, box_height], ...
        'FaceColor', 'r', 'EdgeColor', 'r');
    axis equal;
    xlim([x_boundary(1) - 10, x_boundary(2) + 10]);
    ylim([y_boundary(1) - 10, y_boundary(2) + 10]);
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('Magnetic Train and Falling Box Simulation');
    grid on;

    for i = 2:length(t)
        % Error
        e(i-1) = x_box - x_train(i-1);
        
        %% Controller
        P_controller = Kp * e(i-1);
        if i > 2
            Integral_e(i-1) = Integral_e(i-1) + (e(i-1) + e(i-2)) / 2 * delta_t;
        end
        I_controller = Ki * Integral_e(i-1);
        
        if i > 2
            Differential_e(i-1) = (e(i-1) - e(i-2)) / delta_t;
        end
        D_controller = Kd * Differential_e(i-1);
        
        %% System
        % box
        y_box(i-1) = y_box_0 + vy_box_0 * t(i-1) - 0.5 * g * t(i-1)^2;
        
        box_left = x_box - 0.5 * box_width;
        box_right = x_box + 0.5 * box_width;
        box_bottom = y_box(i-1) - 0.5 * box_height;
        
        % train
        F_a(i-1) = P_controller + I_controller + D_controller;
        
        if i > 2
            Integral_F_a(i-1) = Integral_F_a(i-2) + (F_a(i-1) + F_a(i-2)) / 2 * delta_t;
            IIntegral_F_a(i-1) = IIntegral_F_a(i-2) + (Integral_F_a(i-1) + Integral_F_a(i-2)) / 2 * delta_t;
            vI(i-1) = vI_0 - g * sin(theta) * t(i-1) + 1 / M * Integral_F_a(i-2);
            I(i-1) = I_0 + vI_0 * t(i-1) - 0.5 * g * sin(theta) * t(i-1)^2 + 1 / M * IIntegral_F_a(i-2);
        end
        x_train(i-1) = I(i-1) * cos(theta);
        y_train(i-1) = I(i-1) * sin(theta);
        
        train_left = x_train(i-1) - train_width / 2;
        train_right = x_train(i-1) + train_width / 2;
        train_top = y_train(i-1) + train_height / 2;
        
        % Check collision with train
        [result, x_train(i)] = Environment(x_train(i), x_boundary, box_bottom, train_top, train_right, box_left, box_right, train_left);
        y_train(i) = x_train(i) * tan(theta);
        if result ~= 0
            break;
        end
        % Update Plot
        set(train_rect, 'Position', [train_left, y_train(i-1) - train_height/2, train_width, train_height]);
        set(box_rect, 'Position', [box_left, box_bottom, box_width, box_height]);
        title(sprintf('Time: %.2f s', t(i-1)));
        drawnow;
    end
    
    % Truncate data at break point
    e = e(1:i-1);
    t = t(1:i-1);
    
    % Store the error and result for this simulation
    errors(1:length(e), sim_num) = e;
    if result == 2
        results(sim_num) = "Success";
        success_time = success_time + 1;
    else
        results(sim_num) = "Fail";
        fail_time = fail_time + 1;
    end
    
    % Plot the error in a subplot
    subplot(ceil(sqrt(num_simulations)), ceil(sqrt(num_simulations)), sim_num);
    plot(t, e);
    xlabel('Time (s)');
    ylabel('Error (m)');
    title(['Simulation ', num2str(sim_num), ' - ', results(sim_num)]);
    grid on;
end
% sgtitle('Tracking Error Across Multiple Simulations');
disp(["Success",num2str(success_time)])
disp(["Fail",num2str(fail_time)])

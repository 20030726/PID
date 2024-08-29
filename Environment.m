function [result,x_train] = Environment(x_train, x_boundary, box_bottom, train_top, train_right, box_left, box_right, train_left)
    x_train = min(max(x_train, x_boundary(1)), x_boundary(2));
    result = 0; % Initialize result to 0, indicating to continue the loop
    % Check if the box collides with the train
    if box_bottom <= train_top 
        if box_left > train_right || box_right < train_left 
            result = 1; % Indicates collision failed, exit the loop
        else
            result = 2; % Indicates collision occurred, exit the loop
        end
        return;
    end
end

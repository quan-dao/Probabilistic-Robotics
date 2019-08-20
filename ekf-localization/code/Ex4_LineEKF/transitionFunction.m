function [f, F_x, F_u] = transitionFunction(x,u, b)
% [f, F_x, F_u] = transitionFunction(x,u,b) predicts the state x at time t given
% the state at time t-1 and the input u at time t. F_x denotes the Jacobian
% of the state transition function with respect to the state evaluated at
% the state and input provided. F_u denotes the Jacobian of the state
% transition function with respect to the input evaluated at the state and
% input provided.
% State and input are defined according to "Introduction to Autonomous Mobile Robots", pp. 337
 
sum_delta = sum(u);
diff_delta = u(2) - u(1);

f = x + [0.5 * sum_delta * cos(x(3) + 0.5 * diff_delta / b);
         0.5 * sum_delta * sin(x(3) + 0.5 * diff_delta / b);
         diff_delta / b];
F_x = eye(3) + [0, 0, -0.5 * sum_delta * sin(x(3) + 0.5 * diff_delta / b);
                0, 0, 0.5 * sum_delta * cos(x(3) + 0.5 * diff_delta / b);
                0, 0, 0];
F_u = zeros(3, 2);

F_u(1, 1) = 0.5 * cos(x(3) + diff_delta / (2 * b)) + ...
    (sum_delta / (4 * b)) * sin(x(3) + diff_delta / (2 * b));

F_u(1, 2) = 0.5 * cos(x(3) + diff_delta / (2 * b)) - ...
    (sum_delta / (4 * b)) * sin(x(3) + diff_delta / (2 * b));

F_u(2, 1) = 0.5 * sin(x(3) + diff_delta / (2 * b)) - ...
    (sum_delta / (4 * b)) * cos(x(3) + diff_delta / (2 * b));


F_u(2, 2) = 0.5 * sin(x(3) + diff_delta / (2 * b)) + ...
    (sum_delta / (4 * b)) * cos(x(3) + diff_delta / (2 * b));

F_u(3, :) = [-1/b, 1/b];
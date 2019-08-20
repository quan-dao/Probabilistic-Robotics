function [h, H_x] = measurementFunction(x, m)
% [h, H_x] = measurementFunction(x, m) returns the predicted measurement
% given a state x and a single map entry m. H_x denotes the Jacobian of the
% measurement function with respect to the state evaluated at the state
% provided.
% Map entry and state are defined according to "Introduction to Autonomous Mobile Robots" pp. 337

% extract alpha & r from m
alpha = m(1); r = m(2);

h = [alpha - x(3);
     r - (x(1) * cos(alpha) + x(2) * sin(alpha))];
 
H_x = [0,           0,          -1;
       -cos(alpha), -sin(alpha), 0];

[h(1), h(2), isRNegated] = normalizeLineParameters(h(1), h(2));

if isRNegated 
    H_x(2, :) = - H_x(2, :);
end


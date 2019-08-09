function [x_posterori, P_posterori] = incrementalLocalization(x, P, u, S, M, params, k, g, b)
% [x_posterori, P_posterori] = incrementalLocalization(x, P, u, S, R, M,
% k, b, g) returns the a posterori estimate of the state and its covariance,
% given the previous state estimate, control inputs, laser measurements and
% the map

% C_TR = diag([repmat(0.1^2, 1, size(S, 2)) repmat(0.1^2, 1, size(S, 2))]);

% convert polar coord to XY
XY = zeros(2, size(S, 2));
XY(1, :) = S(2, :) .* cos(S(1, :));
XY(2, :) = S(2, :) .* sin(S(1, :));

% extract lines & compute their covariance 
[~, ~, ~, ~, pointIdx] = extractLines(XY, params);
num_lines = size(pointIdx, 1);
Z = zeros(2, num_lines);
R = [];
for i = 1 : num_lines
    theta = S(1, pointIdx(i, :));
    rho = S(2, pointIdx(i, :));
    C_TR = diag(zeros(1, 4) + 0.1^2);
    [alpha, r, C_AR] = fitLinePolar(theta, rho, C_TR);
    % store alpha & r into Z
    Z(:, i) = [alpha; r];
    % store C_AR into R
    if i == 1
        R = reshape(C_AR, [2, 2, 1]);
    else
        R = cat(3, R, reshape(C_AR, [2, 2, 1]));
    end
end

[x_posterori, P_posterori] = filterStep(x, P, u, Z, R, M, k, g, b);

%[f, ~, ~] = transitionFunction(x, u, b);
%x_posterori = f;

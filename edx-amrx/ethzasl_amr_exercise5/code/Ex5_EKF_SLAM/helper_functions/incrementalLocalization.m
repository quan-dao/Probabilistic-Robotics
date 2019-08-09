function [x_posterori, P_posterori] = incrementalLocalization(x, P, u, Q, S, M, params, b, g)
% [x_posterori, P_posterori] = incrementalLocalization(x, P, u, Q, S, R, M,
% b, g) returns the a posterori estimate of the state and its covariance,
% given the previous state estimate, control inputs, laser measurements and
% the map

C_TR = diag([ones(size(S(1,:))), ones(size(S(2,:)))]) * 0.0001;

[z, R, ans] = extractLinesPolar(S(1,:), S(2,:), C_TR, params);

z_prior = zeros(size(M));
for k = 1:size(M,2)
    [z_prior(:,k), ~] = measurementFunction(x, k);
end

R = repmat(.01*eye(2), [1, 1, size(z,2)]);

% estimate robot pose
[x_posterori, P_posterori] = filterStep(x, P, u, Q, z, R, g, b);

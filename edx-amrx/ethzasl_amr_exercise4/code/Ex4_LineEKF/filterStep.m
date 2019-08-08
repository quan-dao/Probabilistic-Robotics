function [x_posteriori, P_posteriori] = filterStep(x, P, u, Z, R, M, k, g, b)
% [x_posteriori, P_posteriori] = filterStep(x, P, u, z, R, M, k, g, b)
% returns an a posteriori estimate of the state and its covariance

% additional bells and whistles in case no line was detected, please
% incorporate this at a sensical position in your code
if size(Z,2) == 0
    x_posteriori = x;
    P_posteriori = P;
    return;
end

% Prediction update
[x_hat, F_x, F_u] = transitionFunction(x, u, b);
Q = [k * abs(u(1)), 0;
     0,             k * abs(u(2))];
P_hat = F_x * P * F_x' + F_u * Q * F_u';

% Matching observation against prediction measurement
[v, H, R] = associateMeasurements(x_hat, P_hat, Z, R, M, g);

% Estimation
K = size(H, 3);  % number of matched observation

% construct composite H_t, Sigma_IN_t, R_t, v_t
H_t = permute(H,[1 3 2]);  % this & below to concatenate a 3D tensor vertically
H_t = reshape(H_t, [], size(H, 2), 1);

R_t = blockDiagonal(R);

Sigma_IN_t = H_t * P_hat * H_t' + R_t;

v_t = reshape(v, [2 * K, 1]);

% Kalman gain
K = P_hat * H_t' * inv(Sigma_IN_t);

x_posteriori = x_hat + K * v_t;
P_posteriori = P_hat - P_hat * H_t' * K';


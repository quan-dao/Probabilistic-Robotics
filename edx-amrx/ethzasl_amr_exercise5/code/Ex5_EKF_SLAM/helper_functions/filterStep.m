function [x_posteriori, P_posteriori] = filterStep(x, P, u, Q, z, R, g, b)

% propagate the state (p. 337)
[x_priori, F_x, F_u] = transitionFunction(x, u, b);
P_priori = F_x * P * F_x' + F_u * Q * F_u';

[v, H, R] = associateMeasurements(x_priori, P_priori, z, R, g);

y = reshape(v, [], 1);
H = reshape(permute(H, [1,3,2]), [], length(x));
R = blockDiagonal(R);

% update state estimates (pp. 335)
S = H * P_priori * H' + R;
K = P_priori * H' * inv(S);

P_posteriori = P_priori - K * S * K';
x_posteriori = x_priori + K * y;
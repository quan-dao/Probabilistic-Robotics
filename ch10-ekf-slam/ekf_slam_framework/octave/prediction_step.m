function [mu, sigma] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model,
% mu: (2N + 3) x 1 vector representing the state mean
% sigma: (2N + 3) x (2N + 3) covariance matrix
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% TODO: Compute the new mu based on the noise-free (odometry-based) motion model
% Remember to normalize theta after the update (hint: use the function normalize_angle available in tools)
x = mu(1); y = mu(2); theta = mu(3);
mu(1) = x + u.t * cos(theta + u.r1);
mu(2) = y + u.t * cos(theta + u.r1);
mu(3) = normalize_angle(theta + u.r1 + u.r2);

% TODO: Compute the 3x3 Jacobian Gx of the motion model
Gx = [1, 0, -u.t * sin(theta + u.r1);
      0, 1, u.t * cos(theta + u.r1);
      0, 0, 1];

% TODO: Construct the full Jacobian G
N = (size(sigma,1) - 3)/2;  % number of landmarks
G = [Gx, zeros(3, 2*N);
     zeros(2*N, 3), zeros(2*N, 2*N)];

% Motion noise
motionNoise = 0.1;
R3 = [motionNoise, 0, 0; 
     0, motionNoise, 0; 
     0, 0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

% TODO: Compute the predicted sigma after incorporating the motion
sigma = G * sigma * G' + R;

end

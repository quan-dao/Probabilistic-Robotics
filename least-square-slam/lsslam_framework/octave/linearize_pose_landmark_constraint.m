% Compute the error of a pose-landmark constraint
% x 3x1 vector (x,y,theta) of the robot pose
% l 2x1 vector (x,y) of the landmark
% z 2x1 vector (x,y) of the measurement, the position of the landmark in
%   the coordinate frame of the robot given by the vector x
%
% Output
% e 2x1 error of the constraint
% A 2x3 Jacobian wrt x
% B 2x2 Jacobian wrt l
function [e, A, B] = linearize_pose_landmark_constraint(x, l, z)

  % TODO compute the error and the Jacobians of the error
  
  % Get pose from vector
  X_i = v2t(x);
  
  % extract rotation & translation matrix
  R_i = X_i(1:2, 1:2);  t_i = X_i(1:2, 3);
  
  % compute error
  e = R_i' * (l - t_i) - z;
  
  % Jacobian w.r.t x
  de_dxi = R_i' * [-1; 0];
  de_dyi = R_i' * [0; -1];
  theta_i = x(3);
  de_dthetai = [-sin(theta_i), -cos(theta_i); 
                cos(theta_i), -sin(theta_i)]' * (l - t_i);
  A = [de_dxi, de_dyi, de_dthetai];
  
  % Jacobian w.r.t l
  B = R_i';


end

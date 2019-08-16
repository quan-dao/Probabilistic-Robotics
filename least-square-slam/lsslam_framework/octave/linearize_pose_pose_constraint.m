% Compute the error of a pose-pose constraint
% x1 3x1 vector (x,y,theta) of the first robot pose
% x2 3x1 vector (x,y,theta) of the second robot pose
% z 3x1 vector (x,y,theta) of the measurement
%
% You may use the functions v2t() and t2v() to compute
% a Homogeneous matrix out of a (x, y, theta) vector
% for computing the error.
%
% Output
% e 3x1 error of the constraint
% A 3x3 Jacobian wrt x1
% B 3x3 Jacobian wrt x2
function [e, A, B] = linearize_pose_pose_constraint(x1, x2, z)

  % TODO compute the error and the Jacobians of the error
  
  % get pose from vector
  X_i = v2t(x1);
  X_j = v2t(x2);
  Z_ij = v2t(z);
  
  % extract rotation & translation vector
  R_ij = Z_ij(1:2, 1:2);    t_ij = Z_ij(1:2, 3);
  R_i = X_i(1:2, 1:2);      t_i = X_i(1:2, 3);
                            t_j = X_j(1:2, 3);
  % compute error
  e = [R_ij' * (R_i' * (t_j - t_i) - t_ij);
      normalize_angle(x2(3) - x1(3) - z(3))];
  
  % Jacobian w.r.t x1
  de_dxi = [R_ij' * R_i' * [-1; 0];
            0];
        
  de_dyi = [R_ij' * R_i' * [0; -1];
            0];
  
  theta_i = x1(3);
  de_dthetai = [R_ij' * [-sin(theta_i), -cos(theta_i); cos(theta_i), -sin(theta_i)]' * (t_j - t_i);
                -1];
            
  A = [de_dxi, de_dyi, de_dthetai];
  
  % Jacobian w.r.t x2
  de_dxj = [R_ij' * R_i' * [1; 0];
            0];
  de_dyj = [R_ij' * R_i' * [0; 1];
            0];
  de_dthetaj = [zeros(2, 1);
                1];
            
  B = [de_dxj, de_dyj, de_dthetaj];


end

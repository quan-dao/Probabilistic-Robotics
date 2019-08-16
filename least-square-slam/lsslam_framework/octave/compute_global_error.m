% Computes the total error of the graph
function Fx = compute_global_error(g)

Fx = 0;

% Loop over all edges
for eid = 1:length(g.edges)
  edge = g.edges(eid);

  % pose-pose constraint
  if (strcmp(edge.type, 'P') ~= 0)

    x1 = v2t(g.x(edge.fromIdx:edge.fromIdx+2));  % the first robot pose
    x2 = v2t(g.x(edge.toIdx:edge.toIdx+2));      % the second robot pose

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    
    % get transformation represents measurement
    z = v2t(edge.measurement);
    
    % extract rotation matrix & translation vector 
    R_ij = z(1:2, 1:2);
    t_ij = z(1:2, 3);
    theta_ij = atan2(R_ij(2, 1), R_ij(2, 2));
    
    R_i = x1(1:2, 1:2);
    t_i = x1(1:2, 3);
    theta_i = atan2(R_i(2, 1), R_i(2, 2));
    
    t_j = x2(1:2, 3);
    theta_j = atan2(x2(2, 1), x2(2, 2));
    
    % compute error
    e = [R_ij' * (R_i' * (t_j - t_i) - t_ij);
        normalize_angle(theta_j - theta_i - theta_ij)];
    
  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') ~= 0)
    x = g.x(edge.fromIdx:edge.fromIdx+2);  % the robot pose
    l = g.x(edge.toIdx:edge.toIdx+1);      % the landmark position

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    
    % compute error
    theta = x(3);
    e = [cos(theta), -sin(theta);
        sin(theta), cos(theta)]' * (l - x(1:2)) - edge.measurement;
  end
  
  % add error to Fx
  Fx = Fx + e' * edge.information * e; 

end

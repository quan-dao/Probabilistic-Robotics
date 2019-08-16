% performs one iteration of the Gauss-Newton algorithm
% each constraint is linearized and added to the Hessian

function dx = linearize_and_solve(g)

nnz = nnz_of_graph(g);

% allocate the sparse H and the vector b
H = spalloc(length(g.x), length(g.x), nnz);
b = zeros(length(g.x), 1);

needToAddPrior = true;

% compute the addend term to H and b for each of our constraints
disp('linearize and build system');
for eid = 1:length(g.edges)
  edge = g.edges(eid);

  % pose-pose constraint
  if (strcmp(edge.type, 'P') ~= 0)
    % edge.fromIdx and edge.toIdx describe the location of
    % the first element of the pose in the state vector
    % You should use also this index when updating the elements
    % of the H matrix and the vector b.
    % edge.measurement is the measurement
    % edge.information is the information matrix
    x1 = g.x(edge.fromIdx : edge.fromIdx + 2);  % the first robot pose
    x2 = g.x(edge.toIdx : edge.toIdx + 2);      % the second robot pose

    % Computing the error and the Jacobians
    % e the error vector
    % A Jacobian wrt x1
    % B Jacobian wrt x2
    [e, A, B] = linearize_pose_pose_constraint(x1, x2, edge.measurement);


    % TODO: compute and add the term to H and b
    
    % get index in state vector of 2 nodes in this edge
    i = edge.fromIdx; j = edge.toIdx;
    
    % update H_ii
    H(i : i + 2, i : i + 2) = H(i : i + 2, i : i + 2) + A' * edge.information * A;
    % update H_ij
    H(i : i + 2, j : j + 2) = H(i : i + 2, j : j + 2) + A' * edge.information * B;
    % update H_ji
    H(j : j + 2, i : i + 2) = H(i : i + 2, j : j + 2)';
    % update H_jj
    H(j : j + 2, j : j + 2) = H(j : j + 2, j : j + 2) + B' * edge.information * B;
    
    % update b_i
    b(i : i + 2) = b(i : i + 2) + (e' * edge.information * A)';
    % update b_j
    b(j : j + 2) = b(j : j + 2) + (e' * edge.information * B)';
    
    if (needToAddPrior)
      % TODO: add the prior for one pose of this edge
      % This fixes one node to remain at its current location
      H(i, i) = H(i, i) + 1;
      needToAddPrior = false;
    end

  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') ~= 0)
    % edge.fromIdx and edge.toIdx describe the location of
    % the first element of the pose and the landmark in the state vector
    % You should use also this index when updating the elements
    % of the H matrix and the vector b.
    % edge.measurement is the measurement
    % edge.information is the information matrix
    x1 = g.x(edge.fromIdx : edge.fromIdx+2);  % the robot pose
    x2 = g.x(edge.toIdx : edge.toIdx+1);      % the landmark

    % Computing the error and the Jacobians
    % e the error vector
    % A Jacobian wrt x1
    % B Jacobian wrt x2
    [e, A, B] = linearize_pose_landmark_constraint(x1, x2, edge.measurement);


    % TODO: compute and add the term to H and b
    
    % get index in state vector of 2 nodes in this edge
    i = edge.fromIdx; j = edge.toIdx;
    
    % update H_ii
    H(i : i + 2, i : i + 2) = H(i : i + 2, i : i + 2) + A' * edge.information * A;
    % update H_ij
    H(i : i + 2, j : j + 1) = H(i : i + 2, j : j + 1) + A' * edge.information * B;
    % update H_ji
    H(j : j + 1, i : i + 2) = H(i : i + 2, j : j + 1)';
    % update H_jj
    H(j : j + 1, j : j + 1) = H(j : j + 1, j : j + 1) + B' * edge.information * B;
    
    % update b_i
    b(i : i + 2) = b(i : i + 2) + (e' * edge.information * A)';
    % update b_j
    b(j : j + 1) = b(j : j + 1) + (e' * edge.information * B)';
    
  end
end

disp('solving system');

% TODO: solve the linear system, whereas the solution should be stored in dx
% Remember to use the backslash operator instead of inverting H
dx = -H \ b;

end

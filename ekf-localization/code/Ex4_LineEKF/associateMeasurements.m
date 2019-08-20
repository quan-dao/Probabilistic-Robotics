function [v, H, R] = associateMeasurements(x, P, Z, R, M, g)
% [v, H, R] = associateMeasurements(x, P, Z, R, M, g) returns a set of
% innovation vectors and associated jacobians and measurement covariances
% by matching line features by Mahalanobis distance.


v = [];
H = [];
R_idx = [];  %

L = size(M, 2);  % total number of features in the map M
N = size(Z, 2);  % total number of measurements

for j = 1 : N  % j iterate observation
    % get observation 
    z_obs = Z(:, j);
    % initialize min of Mahalanobis dist & associate H_i
    min_d = Inf;
    chosen_H_i = zeros(2, 3);
    chosen_v_ij = zeros(2, 1);
    
    for i = 1 : L  % i iterate features in the map M 
        % get the expected measurement for feature m_i
        [z_pred, H_hat_i] = measurementFunction(x, M(:, i));
        % innovation
        v_ij = z_obs - z_pred;
        % innovation covariance 
        sigma_ij = H_hat_i * P * H_hat_i' + R(:, :, j);
        % Mahalanobis distance 
        d = v_ij' * inv(sigma_ij) * v_ij;
        % check with validation gate
        if d <= g^2 && d < min_d
            min_d = d;
            chosen_v_ij = v_ij;
            chosen_H_i = H_hat_i;
        end
    end
    
    % check if any successful matches
    if min_d < Inf
        % concatenate v, H, R
        v = [v, chosen_v_ij];

        if isempty(H)
            H = reshape(chosen_H_i, [2, 3, 1]);
        else
            H = cat(3, H, reshape(chosen_H_i, [2, 3, 1]));
        end

        R_idx = [R_idx, j];
    end
    
end
% extract R_t associated with actual observations have matched map entry 
R = R(:, :, R_idx);



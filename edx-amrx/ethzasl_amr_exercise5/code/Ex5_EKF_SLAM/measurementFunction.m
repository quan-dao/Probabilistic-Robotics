function [h, H_x] = measurementFunction(x, idxLandmark)
% [h, H_x] = measurementFunction(x, idxLandmark) returns the predicted measurement
% given a state x and a single map entry with index idxLandmark. H_x denotes the Jacobian of the
% measurement function with respect to the state evaluated at the state
% provided.
% Map entry and state are defined according to the book pp. 337-338

%Extract individual landmark from the state vector
m = x(3+2*(idxLandmark-1)+1:3+2*(idxLandmark));


h  = [m(1) - x(3);
      m(2) - (x(1) * cos(m(1)) + x(2) * sin(m(1)))];

H_x = zeros(2,length(x));

% Derivative of h w.r.t robot position [x, y, theta]
H_x(1:2,1:3) = [0, 0, -1;
                -cos(m(1)), -sin(m(1)), 0];

%Do not correct first two landmarks as they remain fixed
if (idxLandmark>2)
    % Derivative of h w.r.t landmark represented by idxLandmark
   H_x(1,3 + (idxLandmark-1)*2+1) = 1;
   H_x(2,3 + (idxLandmark-1)*2+1) = 0;
   H_x(1,3 + (idxLandmark)*2) = 0;
   H_x(2,3 + (idxLandmark)*2) = 1;
end

[h(1), h(2), isRNegated] = normalizeLineParameters(h(1), h(2));

if isRNegated 
    H_x(2, :) = - H_x(2, :);
end

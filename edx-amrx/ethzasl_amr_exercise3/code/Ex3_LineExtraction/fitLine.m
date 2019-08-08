%---------------------------------------------------------------------
% This function computes the parameters (r, alpha) of a line passing
% through input points that minimize the total-least-square error.
%
% Input:   XY - [2,N] : Input points
%
% Output:  alpha, r: paramters of the fitted line

function [alpha, r] = fitLine(XY)
% Compute the centroid of the point set (xmw, ymw) considering that
% the centroid of a finite set of points can be computed as
% the arithmetic mean of each coordinate of the points.

% XY(1,:) contains x position of the points
% XY(2,:) contains y position of the points


    xc = mean(XY(1, :));
    yc = mean(XY(2, :));

    % compute parameter alpha (see exercise pages)
    num   = -2 * sum((XY(1, :) - xc) .* (XY(2, :) - yc));
    denom = sum((XY(2, :) - yc).^2 - (XY(1, :) - xc).^2); 
    alpha = 0.5 * atan2(num, denom);

    % compute parameter r (see exercise pages)
    rho = sqrt(XY(1, :).^2 + XY(2, :).^2);
    theta = atan2(XY(2, :), XY(1, :));
    r = sum(rho .* cos(theta - alpha)) / size(XY, 2);


% Eliminate negative radii
if r < 0
    alpha = alpha + pi;
    if alpha > pi, alpha = alpha - 2 * pi; end
    r = -r;
end

end

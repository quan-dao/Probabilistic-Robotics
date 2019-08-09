function [ output_args ] = plotLandmarks( x, x_gt )
%PLOTLANDMARKS Summary of this function goes here
%   Detailed explanation goes here

M = x(4:end);
M_gt = x_gt(4:end);

polarLine = reshape(M, 2,size(M,1)/2);
polarLine_gt = reshape(M_gt, 2,size(M_gt,1)/2);

plot(polarLine(1,:), polarLine(2,:),'bo')
hold on
plot(polarLine_gt(1,:), polarLine_gt(2,:),'ro')
xlabel('angle');
ylabel('distance');
legend('Estimate','Ground Truth')
end


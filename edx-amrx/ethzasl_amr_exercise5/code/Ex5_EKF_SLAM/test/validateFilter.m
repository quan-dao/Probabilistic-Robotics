function validateFilter()

base = fileparts(mfilename('fullpath'));
cd(base)

load data/validationData.mat

if ~validateTransitionFunction()
    fprintf('Execution halted. Test of individual building blocks was unsuccessful. Please test them individually and re-run this script.\n');
    close all;
    return;
end
close all


x_bl = x;       %save baseline results
clear x

x_fw = zeros(size(x_gt));
x_fw(:,1) = x_gt(:,1);

for i = 2:size(u,2)
    x_fw(:,i) = transitionFunction(x_fw(:,i-1), u(:,i));
end

x = zeros(size(x_gt));
x(:,1) = x_bl(:,1);
for i = 2:size(u,2)
    [x(:,i), P(:,:,i)] = filterStep(x(:,i-1), P(:,:,i-1), u(:,i), Q, z(:,:,i), R, g, b);
end

figure(1); cla, hold on, axis equal;
plot(x_gt(1,:), x_gt(2,:), 'k-');
plot(x_bl(1,:), x_bl(2,:), 'b:')
plot(x(1,:), x(2,:), 'r-');
plot(x_fw(1,:), x_fw(2,:), 'g-');
legend({'ground truth', 'baseline implementation', 'your implementation', 'forward integration'});

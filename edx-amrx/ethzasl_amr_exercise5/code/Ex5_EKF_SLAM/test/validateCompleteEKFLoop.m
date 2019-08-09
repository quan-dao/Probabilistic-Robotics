% test EKF localization
clear all
close all

%tuning parameters
noLines = 5; % number of lines

g = sqrt(10.0); % validation gate

sigma_pos = .15; % uncertainty of initial position
sigma_theta = 0.2; % uncertainty of initial heading
sigma_alpha = 0.08; % uncertainty of initial landmark angle
sigma_r = 0.1; % uncertainty of initial landmark distance

sigma_u = 1.5e-2; % perturbation of control inputs
sigma_z_alpha = 2.0e-3; % perturbarion of landmark angle measurements
sigma_z_r = 1.0e-2; % perturbarion of landmark distance measurements

b = .1; % inter wheel distance

%addpath('solution/')

t = 0:.1:6;

%Generate landmarks
M = [-pi + 2*pi*rand(1, noLines)
    2.5*rand(1, noLines)];


%Uncertainty of landmarks
R = repmat(diag([sigma_z_alpha^2, sigma_z_r^2 ]), [1, 1, size(M,2)]);

%Generate control inputs
u_gt = .3 * [sin(t).^2
    sin(t+.4).^2];
Q = sigma_u^2 * eye(size(u_gt,1));

%generate trajectory & observations from control inputs
x_gt = zeros(3,length(t));
z_gt = zeros(size(M,1), size(M,2), length(t-1));

x_gt = [x_gt; repmat(reshape(M, 1, numel(M))',1,size(x_gt,2))];
for i = 2:length(t)
    [x_gt(:,i), Fx_gt(:,:,i), Fu_gt(:,:,i)] = transitionFunction(x_gt(:,i-1), u_gt(:,i));
    for j = 1:size(M,2)
        [z_gt(:,j,i), Hx_gt(:,:,i)] = measurementFunction(x_gt(:,i), j); %ACHTUNG: this does not save all Hx!
    end
end
figure(1); cla;
plot(x_gt(1,:), x_gt(2,:), 'b-'); hold on;
xlabel('x'), ylabel('y'); axis equal;

% generate initial pose, corrupted control inputs and measurements
x = zeros(size(x_gt));
x_fw = zeros(size(x_gt));

%Add noise on initial position and heading
x(1:2,1) = x_gt(1:2,1) + sigma_pos*randn(size(x_gt(1:2,1)));
x(3,1) = x_gt(3,1) + sigma_theta*randn(size(x_gt(3,1)));


%Add noise on landmarks in state
x(8:end,:) = x_gt(8:end,:) + repmat([sigma_z_alpha*randn(1), sigma_z_r*randn(1)]', (size(x,1)-7)/2, size(x,2));

%Initializing first two landmarks as fixed (to fix unobservable yaw and
%initial position)
x(4:7) = x_gt(4:7);

% Generate initial state covariance matrix
% Landmark covariance
P_lm = repmat([sigma_alpha^2 sigma_r^2], 1, (size(x,1)-7)/2);
% Position & heading covariance. Fixed landmarks get very low variance
P(:,:,1) = diag([sigma_pos^2 sigma_pos^2 sigma_theta^2 2e-8*ones(1,4) P_lm]);

%Perturb control inputs and observations
u = u_gt + sigma_u*randn(size(u_gt));
z = z_gt + sigma_z_r*randn(size(z_gt));

% run forward integration of the state
for i = 2:size(u,2)
    x_fw(:,i) = transitionFunction(x_fw(:,i-1), u(:,i));
end

% close estimation loop
for i = 2:size(u,2)
    [x(:,i), P(:,:,i)] = filterStep(x(:,i-1), P(:,:,i-1), u(:,i), Q, z(:,:,i), R, g, b);
end

figure(1);
plot(x(1,1:end), x(2,1:end), 'r-');
plot(x_fw(1,1:end), x_fw(2,1:end), 'g-');
legend({'ground truth', 'kalman filter localization', 'forward integration'});

%% V-REP Simulation Exercise 5: EKF-based Simultaneous Localization and Mapping
% In order to run the simulation:
%   - Start V-Rep
%   - Load the scene matlab/common/vrep/mooc_exercise.ttt
%   - Hit the run button in V-REP
%   - Start this script

%% Parameters setup

% try

%% Initialize connection with V-Rep
startup;

connection = simulation_setup();
connection = simulation_openConnection(connection, 0);
simulation_start(connection);
[bodyDiameter wheelDiameter interWheelDist scannerPose] = bob_init(connection);
bob_setGhostVisible(connection, 1);

params.MIN_SEG_LENGTH = 0.01;
params.LINE_POINT_DIST_THRESHOLD = 0.005;
params.MIN_POINTS_PER_SEGMENT = 20;

img = bob_getMap(connection);
M = generateMap(img);
g = sqrt(10);
b = bob_getInterWheelDistance(connection);
d = bob_getWheelDiameter(connection);

P_lm = repmat([0.2^2 0.35^2], 1, (size(M,2)-2));
P = diag([ 0.1; 0.1; 0.01; 2e-8*ones(4,1) ; P_lm']);

Q = diag([ 1e-1; 1e-1]);
x = zeros(3,1);
x = [x; reshape(M,1,size(M,2)*2)'];
sigma_z_alpha = 2.0e-3; % perturbarion of landmark angle measurements
sigma_z_r = 1.0e-2; % perturbarion of landmark distance measurements
x(8:end,:) = x(8:end,:) + repmat([sigma_z_alpha*randn(1), sigma_z_r*randn(1)]', (size(x,1)-7)/2, size(x,2));

[x(1), x(2), x(3)] = bob_getPose(connection);

simulation_setStepped(connection, true);
simulation_triggerStep(connection);

u = [0;0];
bob_setWheelSpeeds(connection, 0.25, 0.35);
simStep = 50e-3;    % simulation step duration in seconds
laserRate = .5;     % laser rate in Hz, no intermediate propagations with the motion model will be performed

t = cputime;
for i = 1:100
    
    for l = 1:round(laserRate/simStep)
        simulation_triggerStep(connection);
    end

    
    [v(1), v(2)] = bob_getWheelSpeeds(connection);
    dt = cputime - t;
    t = cputime;
    u = v * dt * d/2;
    
    % extract lines
    [laserX, laserY] = bob_getLaserData(connection);
    theta = atan2(laserY, laserX);
    rho = laserX./cos(theta);
    inRangeIdx = find(rho < 4.9);
    theta  = theta(inRangeIdx);
    rho  = rho(inRangeIdx);
    
    [x, P] = incrementalLocalization(x, P, u, Q, [theta; rho], M, params, b, g);
    
    % plot pose estimate in vrep
    bob_setGhostPose(connection, x(1), x(2), x(3));
    %pause(.1)
    
end
simulation_stop(connection);
simulation_closeConnection(connection);

% catch exception
%     simulation_closeConnection(connection);
%     rethrow(exception);
% end

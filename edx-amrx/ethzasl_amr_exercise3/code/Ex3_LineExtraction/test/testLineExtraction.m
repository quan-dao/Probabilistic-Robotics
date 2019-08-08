% % % % % % % % % % % % % % % % % % % % % % 
% Line extraction test script
% % % % % % % % % % % % % % % % % % % % % % 

startup;
close all;
clear all;

% params for line extraction
params.MIN_SEG_LENGTH = 0.01;
params.LINE_POINT_DIST_THRESHOLD = 0.005;
params.MIN_POINTS_PER_SEGMENT = 20;

fig = figure;
imgPerCol = 2;
imgPerRow = 3;
cmpCovariance = logical([0, 0, 0, 0, 1, 1]);

for testIndex=1:6
    fprintf('Testing laser scan %i...', testIndex);
    
    % Load z, R, theta, rho
    file = sprintf('testLineExtraction%d.mat', testIndex);
    if ~exist(file, 'file')        
        file = sprintf('../test/data/testLineExtraction%d.mat', testIndex);
    end
    load(file);
    
    C_TR = diag([ones(size(theta)), ones(size(rho))]);    
    
    [zT, RT, segendsT] = extractLinesPolar(theta, rho, C_TR, params);
    
    N = size(z, 2);
    Ps = perms(1:N);
    green = false;
    for i = 1:size(Ps, 1)
        P = Ps(i,:);
        if any(size(z) ~= size(zT))
            continue;
        end
        zError = max(calcZDiff(z(:, P), zT));
        if zError > 2E-2
            continue;
        end
        
        if cmpCovariance(testIndex)
            rIsGood = true;
            for j = 1:size(RT, 3)
                rError = norm(RT(:, :, j) - R(:, :, P(j)));
                if (rError > 1E-1)
                    rIsGood = false;
                    break;
                end
            end
            if not(rIsGood)
                continue;
            end        
        end
        
        green = true;
        break;
    end
        
    subplot(imgPerCol, imgPerRow, testIndex);
    plotLinesAndMeasurementsPolar(theta', rho', zT, segendsT, z);    
    
    if green
        visGreen = 'Ok';        
    else
        visGreen = 'Failed';        
    end 
    fprintf('%s\n', visGreen);
    title(sprintf('Test %d (%s)', testIndex, visGreen)); 
end

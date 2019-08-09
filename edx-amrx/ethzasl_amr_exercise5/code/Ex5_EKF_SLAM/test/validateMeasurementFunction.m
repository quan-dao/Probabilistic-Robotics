function [varargout] = validateMeasurementFunction()

tolerance = 1e-5;
success = 1;


base = fileparts(mfilename('fullpath'));
cd(base)

load data/validationData.mat

clear z Hx

try
    
    for i = 1:size(x_gt,2)
        for j = 1:size(M,2)
            [z(:,j,i), Hx(:,:,i, j)] = measurementFunction(x_gt(:,i), j);
        end
    end
    
    z_error = z - z_gt;
    if any(z_error > tolerance)
        fprintf('Error in z!\n');
        success = 0;
    end
    
    Hx_error = Hx - Hx_gt;
    if any(Hx_error > tolerance)
        fprintf('Error in Hx!\n');
        success = 0;
    end
    
    if success == 1
        fprintf('measurement function appears to be correct!\n');
    end
    
catch
    fprintf('Error in the user supplied function. Please make sure that your function is syntactically correct and adheres to the conventions described in the exercise!\n')
    success = 0;
end

if nargout == 1
    varargout{1} = success;
end

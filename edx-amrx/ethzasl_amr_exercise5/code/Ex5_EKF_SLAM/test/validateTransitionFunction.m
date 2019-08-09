function [varargout] = validateTransitionFunction()
tolerance = 1e-5;
success = 1;

load data/validationData.mat
clear x Fx Fu

x = zeros(size(x_gt));
x(:,1) = x_gt(:,1);

try
    for i = 2:size(u_gt,2)
        [x(:,i), Fx(:,:,i), Fu(:,:,i)] = transitionFunction(x(:,i-1), u(:,i), b);
    end
    
    figure(1); hold on, axis equal;
    plot(x_gt(1,1:end-1), x_gt(2,1:end-1), 'g-');
    plot(x(1,1:end-1), x(2,1:end-1), 'r:');
    legend({'ground truth', 'forward integration'});
    axis equal

    x_error = x - x_gt;
    if any(x_error > tolerance)
        fprintf('Error in x!\n');
        success = 0;
    end

    Fx_error = Fx - Fx_gt;
    if any(Fx_error > tolerance)
        fprintf('Error in Fx!\n');
        success = 0;
    end

    Fu_error = Fu - Fu_gt;
    if any(Fu_error > tolerance)
        fprintf('Error in Fu!\n');
        success = 0;
    end
    
    if success == 1
        fprintf('State transition function appears to be correct!\n');
    end
    
catch e
    fprintf(1,'The identifier was:%s\n',e.identifier);
    fprintf(1,'There was an error! The message was:%s\n',e.message);
    fprintf('Error in the user supplied function. Please make sure that your function is syntactically correct and adheres to the conventions described in the exercise!\n')
    success = 0;
end

if nargout == 1
    varargout{1} = success;
end

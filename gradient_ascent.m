function gradient_ascent(use_sim)
     if nargin < 1
        use_sim = true;
    end

    % Starting positions and directions
    position = [0.2; -0.3]; 
    heading = [0; 1]; 

    % Connect to Neato robot or use sim
    if use_sim
        neatov2.connect();
    else
        neatov2.connect('YOUR_NEATO_IP_HERE'); % For real Neato, specify IP
    end

    % Define landscape
    [xvals, yvals] = meshgrid(linspace(-1.25, 1.25, 100), linspace(-0.5, 0.5, 100));
    zvals = 8*exp(-1.5*(xvals + yvals - 0.75).^2 - 0.5*(xvals - yvals - 0.75).^2) + 6*exp(-(xvals + 0.75).^2 - (yvals + 0.25).^2);

    % Set initial position and plot contours for simulated Neato
    if use_sim
        neatov2.setPositionAndOrientation(position(1), position(2), atan2(heading(2), heading(1)));
        neatov2.setFlatlandContours(xvals, yvals, zvals);
        f = figure;
        neatov2.plotSim();
    end

    % Gradient ascent implementation
    function grad = gradient_f(x, y)
        dx = -8 * 1.5 * (x + y - 0.75) .* exp(-1.5*(x + y - 0.75).^2 - 0.5*(x - y - 0.75).^2) ...
             - 6 * (x + 0.75) .* exp(-(x + 0.75).^2 - (y + 0.25).^2);
        dy = -8 * (1.5 * (x + y - 0.75) + 0.5 * (x - y - 0.75)) .* exp(-1.5*(x + y - 0.75).^2 - 0.5*(x - y - 0.75).^2) ...
             - 6 * (y + 0.25) .* exp(-(x + 0.75).^2 - (y + 0.25).^2);
        grad = [dx; dy];
    end

    step_size = 0.01; % Step size, smaller step size would lead to more precise ascent
    threshold = 0.1; % Gradient magnitude threshold for stopping 

    % Steepest ascent loop
    while true
        grad = gradient_f(position(1), position(2));
        if norm(grad) < threshold
            break; % Stop if gradient is small, indicating a peak, based on the threshold
        end
        direction = grad / norm(grad); % Direction of steepest ascentasz
        position = position + step_size * direction; % Move in that direction

        if use_sim
            neatov2.setPositionAndOrientation(position(1), position(2), atan2(direction(2), direction(1)));
            neatov2.plotSim(); % Update simulation plot
        end
        pause(0.1); % Slow down the loop for visualization
    end
end

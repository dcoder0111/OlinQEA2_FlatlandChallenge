function gradient_ascent(use_sim, position, heading)

    % Starting positions and directions
    % position = [0.2; -0.3]; 
    % heading = [0; 1]; 


    % Connect to Neato robot or use sim
    if use_sim
        neatov2.connect();
    else
        neatov2.connect('YOUR_NEATO_IP_HERE'); % For real Neato, specify IP
    end

    % Define the landscape grid
    [xvals, yvals] = meshgrid(linspace(-1.25, 1.25, 100), linspace(-0.5, 0.5, 100));
    
    % Define landscape symbolically
    syms x y
    f_sym = 8*exp(-1.5*(x + y - 0.75).^2 - 0.5*(x - y - 0.75).^2) + 6*exp(-(x + 0.75).^2 - (y + 0.25).^2);

    % Compute gradient symbolically
    grad_f = gradient(f_sym, [x, y]);

    % Convert symbolic gradient to function handles
    grad_f_x = matlabFunction(grad_f(1), 'Vars', [x y]);
    grad_f_y = matlabFunction(grad_f(2), 'Vars', [x y]);

    step_size = 0.01; % Step size, smaller step size leads to more precise ascent
    threshold = 0.1; % Gradient magnitude threshold for stopping  

    if use_sim
        zvals = double(subs(f_sym, {x, y}, {xvals, yvals}));
        neatov2.setPositionAndOrientation(position(1), position(2), atan2(heading(2), heading(1)));
        neatov2.setFlatlandContours(xvals, yvals, zvals);
        f = figure;
        neatov2.plotSim();
    end
    
    points = [];
    % Steepest ascent loop
    while true
        grad_x = grad_f_x(position(1), position(2));
        grad_y = grad_f_y(position(1), position(2));
        grad = [grad_x; grad_y];

        if norm(grad) < threshold
            break; % Stop if gradient is small, indicating a peak
        end

        direction = grad / norm(grad); % Direction of steepest ascent
        position = position + step_size * direction; % Move in that direction
        points(size(points, 1) + 1, :) = position;

        if use_sim
            neatov2.setPositionAndOrientation(position(1), position(2), atan2(direction(2), direction(1)));
            neatov2.plotSim(); % Update simulation plot
        end
        pause(0.05); % Slow down the loop for visualization
    end
    [xvals, yvals] = meshgrid(linspace(-1.25, 1.25, 100), linspace(-0.5, 0.5, 100));
    syms x y
    f_sym = 8*exp(-1.5*(x + y - 0.75).^2 - 0.5*(x - y - 0.75).^2) + 6*exp(-(x + 0.75).^2 - (y + 0.25).^2);
    zvals = double(subs(f_sym, {x, y}, {xvals, yvals}));
    %theoretical trajectory plot
    f = figure;
    contour(xvals, yvals, zvals, 20);
    hold on
    plot(points(:, 1), points(:, 2))
    text(points(1, 1), points(1, 2), "\leftarrow Start")
    text(points(end, 1), points(end, 2), "\leftarrow End")
end


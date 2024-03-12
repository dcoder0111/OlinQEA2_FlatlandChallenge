function gradient_ascent(use_sim, position, heading)

    % Starting positions and directions
    % position = [0.2; -0.3]; 
    % heading = [0; 1]; 


    % Connect to Neato robot or use sim
    if use_sim
        neatov2.connect();
    else
        neatov2.connect('192.168.16.58'); % For real Neato, specify IP
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
    decay = 1;
    threshold = 0.1; % Gradient magnitude threshold for stopping  
    zvals = double(subs(f_sym, {x, y}, {xvals, yvals}));

    if use_sim
        neatov2.setPositionAndOrientation(position(1), position(2), atan2(heading(2), heading(1)));
        neatov2.setFlatlandContours(xvals, yvals, zvals);
        f = figure;
        neatov2.plotSim();
    end
    
    points = [];
    heading = [];
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
        heading(size(heading, 1) + 1, :) = atan2(direction(2), direction(1));

        if use_sim
            neatov2.setPositionAndOrientation(position(1), position(2), atan2(direction(2), direction(1)));
            neatov2.plotSim(); % Update simulation plot
        else

        end
        %update step size
        step_size = step_size * decay;
        pause(0.05); % Slow down the loop for visualization
    end
    %Linear velocity is constant due to step size
    linear_velocity = diff(points);
    linear_velocity = sqrt(linear_velocity(:,1).^2 + linear_velocity(:,2).^2);
    angular_velocity = diff(heading); %find omega from the changing heading
    beta = 3; %velocity scale factor b/c it's slow
    v_l = (linear_velocity - angular_velocity * 0.248/2) .* beta;
    v_r = (linear_velocity + angular_velocity * 0.248/2) .* beta;
    if ~use_sim
        for i=1:size(v_l)
            neatov2.setVelocities(v_l(i), v_r(i))
            pause(0.4) %b/c of the lack of sample sizes run each one for longer
        end
    end
    %theoretical trajectory plot
    f = figure;
    contour(xvals, yvals, zvals, 20);
    hold on
    plot(points(:, 1), points(:, 2))
    text(points(1, 1), points(1, 2), "\leftarrow Start")
    text(points(end, 1), points(end, 2), "\leftarrow End")
end


function [out] = ekf_sim() 

    n = 250; % Number of samples
    dt = 0.01; % Time step amount (s)
    
    v = 4; % launch velocity (ms^-1)
    alpha = deg2rad(45); % launch angle (radians)
    m = 0.3; % projectile mass (kg)
    g = -9.81; % g (ms^-2)
    
    rho = 1.184; % Density of air at 25 degC and 1 atm
    Cd = 0.47; % Coefficient of Friction of a sphere
    r = (3/(4*55.85) * m/pi)^(1/3); % Radius bassed on density of iron and m
    Ad = pi*r^2; % Cross sectional area of object in airstream
    
    dr = (rho*Cd*Ad)/2;
    
    % Initial Conditions
    x_0 = [0; v*sin(alpha); 0; v*cos(alpha)];
    
    % Output matrix mapping state vector to output vector
    C = eye(size(x_0), size(x_0));
    
    # Input force due to gravity
    f_g = m*g;
    
    % State vector and Model output vector
    x = x_0;
    x_out = ones(size(x_0,1), size(x_0,2), n);
    
    y = C*x_0;
    y_out = ones(size(y,1), size(y,2), n);
    
    x_out(:,:,1) = x;
    y_out(:,:,1) = y;
    
    for i=2:n
        
        % Run model
        x_dot = [x(2); dr/m*x(2)^2 + f_g/m; x(4); dr/m*x(4)^2];
        x_next = x + dt*x_dot;
        y_next = C*x_next;
    
        x_out(:,:,i) = x_next;
        y_out(:,:,i) = y_next;
        x = x_next;
        y = y_next;
    
    end
    
end

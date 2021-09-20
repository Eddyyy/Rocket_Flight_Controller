
n = 250; % Number of samples
dt = 0.01; % Time step amount (s)

v = 4; % launch velocity (ms^-1)
alpha = deg2rad(45); % launch angle (radians)
m = 0.3; % projectile mass (kg)
g = -9.81; % g (ms^-2)

% Initial Conditions
x_0 = [0; v*sin(alpha); 0; v*cos(alpha)];

% State Space Model
A = [0,1,0,0;
    0,0,0,0;
    0,0,0,1;
    0,0,0,0];

b = [0; 1/m; 0; 0];

C = eye(size(x_0), size(x_0));

F_g = m*g;
u = F_g;

% State vector and Model output vector
x = x_0;
x_out = ones(size(x_0,1), size(x_0,2), n);

y = C*x_0;
y_out = ones(size(y,1), size(y,2), n);

x_out(:,:,1) = x;
y_out(:,:,1) = y;

for i=2:n
    
    % Run model
    x_dot = A*x + b*u;
    x_next = x + dt*x_dot;
    y_next = C*x_next;

    x_out(:,:,i) = x_next;
    y_out(:,:,i) = y_next;
    x = x_next;
    y = y_next;

end

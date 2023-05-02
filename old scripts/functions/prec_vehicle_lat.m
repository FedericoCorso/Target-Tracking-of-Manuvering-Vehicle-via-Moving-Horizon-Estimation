function zdot = prec_vehicle_lat(t,z,th,vx)   
% aggiungere model noise
% Function to describe longitudinal dynamics 
% inputs:
% output:

% parameters 6x1 vector
m       =      th(1,1);     % vehicle mass (kg)
Iz      =      th(2,1);     % vehicle moment of inertia (kg*m^2)
Lf      =      th(3,1);     % distance between center of gravity and front axle (m)
Lr      =      th(4,1);     % distance between center of gravity and rear axle (m)
Cf      =      th(5,1);     % front axle cornering stiffness (N/rad)
Cr      =      th(6,1);     % rear axle cornering stiffness (N/rad)
mu      =      th(7,1);     % road - tire coefficient


% states
vy      = z(1,1);   % lateral speed
gamma   = z(2,1);   % yaw rate
delta_f = z(3,1);   % front steering angle
e       = z(4,1);   % trajectory tracking error
psi     = z(5,1);   % heading angle
c       = z(6,1);   % road curvature


% fy front and rear definition
Beta = atan(vy/vx);
Fz_front =  m*Lr*9.81/(Lf+Lr);
Fz_rear =  m*Lf*9.81/(Lf+Lr);

alpha_front = Beta + (Lf/vx)*gamma - delta_f;
alpha_rear = Beta - (Lr/vx)*gamma;

Fy_front = lat_tire_force(mu, Fz_front, Cf, alpha_front);
Fy_rear = lat_tire_force(mu, Fz_rear, Cr, alpha_rear);


% equations

zdot(1,1)  =   (Fy_front + Fy_rear)/m - gamma*vx;       % lateral acceletarion vdot_y
zdot(2,1)  =   (Lf*Fy_front - Lr*Fy_rear)/Iz;           % yaw rate acceleration
zdot(3,1)  =   0 ;                                      % steering angle acceleration - constant
zdot(4,1)  =   vx*sin(psi) + vy*cos(psi);               % distance between c.o.m. traj. and nominal traj. 
zdot(5,1)  =   gamma - c*(vx*cos(psi) - vy*sin(psi));   % psi 
zdot(6,1)  =   0;                                       % road curvature - constant

end



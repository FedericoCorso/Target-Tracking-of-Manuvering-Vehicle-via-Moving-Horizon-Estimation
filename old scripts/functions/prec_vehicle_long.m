function [F_bar,gamma_bar] = prec_vehicle_long(dk,th)

% PREC_VEHICLE_LONG Summary of this function goes here
% input: 
% state at instant k (z_k)
% noise value at instant k (w_k)
% sampling time (dk)
% vector of parameters (th)


% parameters
C1 = th(1,1);
C2 = th(2,1);
C3 = th(3,1);

% states
% vp       =       z_k(1,1);    % preceding vehicle speed (m/s)
% d        =       z_k(2,1);    % relative distance host to prec. (m)
% vh       =       z_k(3,1);    % host speed (m/s)
% C4       =       z_k(5,1);    % aggressiveness parameter

% matrices 
gamma = [   dk             0;
        (1/2)*dk^2   -(1/2)*dk^2; 
            0              dk     ]; % 3x2 matrix

gamma_a = [ 0  0      0;
           C1  C2   C3-C1]; % 2x3 matrix

% C = [0; z_k(5,1)]; % 2x1

F = [1  0   0;
     dk 1 -dk;
     0  0   1]; % 3x3

A = F + gamma*gamma_a; % 3x3

% matrices augmented state
F_bar = [A           gamma; 
         zeros(2,3)  eye(2)]; % 5x5

gamma_bar = [gamma; 
             zeros(2,2)];

% acceleration vector 
% a = gamma_a*z_k(1:3,1) + C; 

% dynamic model D.T.
% z_kplus1 = F_bar*z_k + gamma_bar*w_k;

end


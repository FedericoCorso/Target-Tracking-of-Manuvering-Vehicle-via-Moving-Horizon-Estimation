%% MHE LONGITUDINAL ESTIMATION PROBLEM
% cost function: one norm of output prediction error
% additional terms: 1-norm of disturbance, 1-norm of terminal cost
clear all
clc
close all
%% initial parameters for longitudinal dynamics
th_long = zeros(3,1);

th_long(1,1) = 0.5;     % C1 
th_long(2,1) = 0.125;   % C2 
th_long(3,1) = -0.125;  % C3 

dk = longitudinal_speed_ego.Time(1,1);

%% Build data vector
% NaN in some rows

for ii = 1:length(distance_meas.Time) 

    Yvec(2*ii-1,1) = distance_meas.measDistance(ii); % Measured yaw rate
    if isnan(Yvec(2*ii-1,1))
        Yvec(2*ii-1,1) = Yvec(2*ii-3,1); % NaN replaced with previus value of sensor
    end
    Yvec(2*ii,1) = longitudinal_speed_ego.LongitudinalVel(find(distance_meas.Time(ii)==longitudinal_speed_ego.Time));

end

N = length(Yvec);  % Total number of data points

%% system matrices 
[F_bar,Gamma_bar] = prec_vehicle_long(dk,th_long);                         % Model A matrix, discrete time (finite forward differences)
D = [zeros(2,1),eye(2),zeros(2,2)];

%% Define dimension of variables
nz      =   size(F_bar,1); % number of state variables
nw      =   size(Gamma_bar,2); % number of inputs
ny      =   size(D,1); % number of outputs

%% Algorithm parameters
Msec            =   0.1;             % Duration of Moving Horizon Estimator window - user chosen
M               =   round(Msec/dk);  % Steps of Moving Horizon Estimator window
MHE_interval    =   5;               % user chosen - Interval (in steps) between two subsequent MHE estimates' computation
% to check whether to overlap the windows ???

% parameter to tune smoothness of the estimate
gamma_obj       =   1e-2;            % Weight on the error between previous and new state estimate - !user chosen!

[Lambda_y,Gamma_y,Lambda_z,Gamma_z]  =   Traj_matrices(M,F_bar,Gamma_bar,D,0);   % Build recursion matrices for LTI system trajectories
nC              =   (M+1)*ny+(M+1)*nw+nz;                                        % Number of slack variables to convert l1-norm problem in LP


%% siamo arrivati qui!!!!
c               =   [zeros(nz,1);ones(nC-nz,1);gamma_obj*ones(nz,1)];   % Cost function vector for LP reformulation
Aconstr         =   [-[Lambda_y;eye(nz)] -eye(nC);
                    [Lambda_y;eye(nz)] -eye(nC)];                       % Matrix of LP constraints (l1-norm)
options         =   optimset('linprog');                                % linprog optimization options
options.Display =   'off';
tvec_MHE        =   Data_1ms.time(M+1:MHE_interval:end);                % Time vector for MHE estimates
% the number of estimations depend on the number of times the algorithm is
% run
zhat           =   zeros(nz,length(tvec_MHE));                          % Initialize empty matrix for MHE estimates
zhat_m1        =   zeros(nz,1);                                         % Initialize previous MHE estimate

% simulation of the moving horizon estimator
tic

% we start from M+1 since we need first to fill in the buffer of the moving
% horizon estimator

for ind = M+1:MHE_interval:N % iterate through the number of output measurements N
    ind;                                                                     % Just to see where the simulation is
    Ytilde          =   Yvec(ind-M:ind,1);                                  % Sequence of past outputs
    Utilde          =   Uvec(ind-M:ind-1,1);                                % Sequence of past inputs
    bconstr         =   [-[Ytilde-Gamma_y*Utilde;zhat_m1];         
                        [Ytilde-Gamma_y*Utilde;zhat_m1]];                   % b-matrix for LP constraints
    Xsol            =   linprog(c,Aconstr,bconstr,[],[],[],[],[],options);  % Solve MHE problem
    z_hat_sim       =   Lambda_z*Xsol(1:nz,1)+Gamma_z*Utilde;               % Simulate trajectory with optimal solution
    zhat(:,ceil((ind-M)/MHE_interval)) = z_hat_sim(end-nz+1:end,1);         % Update estimate
    zhat_m1        =   ...
        z_hat_sim(MHE_interval*nz+1:MHE_interval*nz+nz,1);                  % Update previous MHE estimate
end
toc


% function to compute the cost function to be optimized.
%
% INPUTS:
%   x   =   optimization variables specified as column vector
%           [state1; state2; state3; state4; state5; disturbance 1st component; disturbance 2nd component]
%
%   nz  =   number of state variables
%
%   M   =   number of steps MHE woindow
%
%   Ts  =   sensor sampling time, needed to compute error at measured time
%
%   Ts_sim = simulation sampling time, for single shooting
%
%   ymeas_camera   =   measurement vector, specified as row vector
%               [row with x coord.;
%                row with y coord.;
%                row with v_x     ;
%                row with v_y      ]
%
%   ymeas_radar   =   measurement vector, specified as row vector
%               [row with x coord.;
%                row with y coord.;
%                row with v_x     ;
%                row with v_y      ]
%
%   z_minus = [state1; state2; state3; state4; state5], state z(-M+1|t-1) 
%
%   R_camera = 3x3 camera covariance matrix
%
%   R_radar = 3x3 radar covariance matrix
%
%   Q = 2x2 process covariance matrix
%
%   A_terminal = 5x5 state covariance matrix
%
%   ego_pos = [X,Y] position of ego vehicle on the world frame, as a row vector
% 
%   ego_or = ego yaw angle wit respect to the world frame, as a row vector
%   
% OUTPUTS:
%   F       =   vector valued function F(x) such that f = F(x)'F(x)
%   h       =   value of non linear inequality constraints
function out = target_cost_GN_con(x,nz,M,Ts,Ts_sim,ymeas_camera,ymeas_radar,z_minus,R_camera,R_radar,Q,A_terminal,ego_pos,ego_or)

%% initialization - extract components in the optimization variables vector
z0 = x(1:nz,1); % extract states components from 'x' (everything apart from disturbances)
d_in = [x(6:(6+M-1),1)';x(6+M:end,1)']; % extract disturbances vectors from 'x' (2*M+2 elements since the disturbance has 2 components)

%% Run simulation with DT model
% Simulation parameters 
time_FFD    =   0:Ts_sim:M*Ts;    
Nsim_FFD    =   length(time_FFD);   % total number of simulation steps
z_sim      =   zeros(nz,Nsim_FFD);  % to store simulation results
z_sim(:,1) =   z0;                  % set initial state for simulation

% Simulation loop
for ind = 2:Nsim_FFD
    d                  =   d_in(:,floor(time_FFD(ind-1)/Ts)+1); % disturbances kept constant for time interval = Ts (sensor sampling)
    if abs(z_sim(5,ind-1)) > eps(ones(1,'like',z_sim))
        z_sim(:,ind)      = CTRV_exact_DT_cartesian(z_sim(:,ind-1),d,Ts_sim); % Cartesian Exact model
    else
        z_sim(5,ind-1) = eps(ones(1,'like',z_sim));
        z_sim(:,ind)      = CTRV_exact_DT_cartesian(z_sim(:,ind-1),d,Ts_sim); % Cartesian Exact model
    end
end

%% Extract simulation results and respective measured data 
ysim = [z_sim(1,:);z_sim(2,:);z_sim(3,:);z_sim(4,:)]; % row vector 3*Nsim

if Ts == Ts_sim
    ysim_resampled = ysim; % downsampling
else
    ysim_resampled = ysim(:,1:1/Ts:end); % downsampling
end


%% compute the error vectors
err_camera = inv(sqrt(R_camera))*(ymeas_camera-ysim_resampled); % row vector
err_radar = inv(sqrt(R_radar))*(ymeas_radar-ysim_resampled); % row vector
err_dist = inv(sqrt(Q))*d_in;
A_terminal = diag(diag(A_terminal)); % take only diagonal terms, corresponding to state variance, avoid taking the square of correlation terms which might be begative
err_terminal = inv(sqrt(A_terminal))*(z_sim(:,1)-z_minus);
    
err_vec = [err_camera(1,:) err_camera(2,:) err_camera(3,:) err_camera(4,:)...
    err_radar(1,:) err_radar(2,:) err_radar(3,:) err_radar(4,:)...
    err_dist(1,:) err_dist(2,:) ...
    err_terminal']'; % column vector F(x)

%% no disturbance quadratic term
% err_vec = [err_camera(1,:) err_camera(2,:) err_camera(3,:) err_camera(4,:)...
%     err_radar(1,:) err_radar(2,:) err_radar(3,:) err_radar(4,:)...
%     err_terminal']'; % column vector F(x)

%% no terminal cost
% err_vec = [err_camera(1,:) err_camera(2,:) err_camera(3,:) err_camera(4,:)...
%     err_radar(1,:) err_radar(2,:) err_radar(3,:) err_radar(4,:) err_dist(2,:)]'; % column vector F(x)

%% Compute sum of squared errors
% cost    =   sum(err_vec.*err_vec); % scalar product of error vectors: f(x) = F'(x)F(x)
F = err_vec;

%% Compute non-linear inequality constraints - from known road geometry 
% need to provide road geometry to the function - values of the constraints stacked as a column vector
% assume ego-position and orientation are provided by onboard state
% estimation and localization modules

h = zeros(2*(M+1),1); % column vector containing non linear inequality constraints

% as the road is a circular segment, we bound the world coordinate of the
% target within road boundaries: outer and inner radius.

% constraint number depends on horizon length !!!

for iind = 1:2:2*(M+1) % build each constraint pair
    % rotation matrix from ego-local to world frame
    R = [cos(ego_or(1,(iind+1)/2)) -sin(ego_or(1,(iind+1)/2));
         sin(ego_or(1,(iind+1)/2))  cos(ego_or(1,(iind+1)/2))];

    % target position on world frame:
    % [X;Y] = [X_ego,Y_ego]+R*[x,y]
    target_in_world = [ego_pos(1,(iind+1)/2);ego_pos(2,(iind+1)/2)]+R*[z_sim(1,(iind+1)/2);z_sim(2,(iind+1)/2)];
    % compute target distance from origin
    target_radius = norm(target_in_world); % euclidean norm-distance from world fram origin
    
    % constraint values
    h(iind,1) = -target_radius + (760 + 7.2/2); % target within outer radius of road r_target < r_out: distance of target right border
    h(iind+1,1) =  target_radius - (760-7.2/2);   % target within inner radius of road r_target > r_in: distance of target from left border
end


out = [F;h];
end
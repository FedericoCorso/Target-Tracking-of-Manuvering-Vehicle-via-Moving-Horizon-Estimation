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
%   cost    =   value of the cost function f(x)
%   p       =   value of non linear equality constraints
%   h       =   value of non linear inequality constraints
function out = target_cost_con_multipleshooting(x,nz,M,Ts,Ts_sim,ymeas_camera,ymeas_radar,z_minus,R_camera,R_radar,Q,A_terminal,ego_pos,ego_or)

% Course of the state through the window
z_course = x(1:nz*(M+1),1);

% Course of disturbance through the window, row vect
d_in = [x(nz*(M+1) + 1:nz*(M+1)+M,1)';x(nz*(M+1)+M+1:end,1)'];

% "simulated" states over the window length
z_sim = [z_course(1:nz:end)';z_course(2:nz:end)';z_course(3:nz:end)';z_course(4:nz:end)';z_course(5:nz:end)']; % row vector 5*Nsim
ysim = [z_course(1:nz:end)';z_course(2:nz:end)';z_course(3:nz:end)';z_course(4:nz:end)']; % row vector 4*Nsim

if Ts == Ts_sim
    ysim_resampled = ysim; % downsampling
else
    ysim_resampled = ysim(:,1:1/Ts:end); % downsampling
end

%% Compute sum of squared errors
cost   =   sum((ymeas_camera-ysim_resampled).*(inv(R_camera)*(ymeas_camera-ysim_resampled)),'all') + sum((ymeas_radar-ysim_resampled).*(inv(R_radar)*(ymeas_radar-ysim_resampled)),'all')+...
            sum(d_in.*(inv(Q)*d_in),'all') + sum((z_sim(:,1)-z_minus).*(inv(A_terminal)*(z_sim(:,1)-z_minus)),'all');

%% non-linear equality constraints
p = zeros(nz*M,1);

for ind = 1:nz:nz*M
    d = [d_in(1,floor(ind/nz + 1));d_in(2,floor(ind/nz + 1))];
    p(ind:ind+nz-1,1) = z_sim(:,floor(ind/nz+1)+1)-CTRV_exact_DT_cartesian(z_sim(:,floor(ind/nz +1)),d,Ts); % z(-M+1|t) - fz(z(-M|t),d(-M|t)) = 0
    % z(-M+2|t) - fz(z(-M+1|t),d(-M+1|t)) = 0
    % z(-M+3|t) - fz(z(-M+2|t),d(-M+2|t)) = 0
    % z(-M+2|t) - fz(z(-M+1|t),d(-M+1|t)) = 0
    % .
    % .
    % .
    % z(0|t) - fz(z(-1|t),d(-1|t)) = 0
end

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

out = [cost;p;h]; % stack cost and constraints

end
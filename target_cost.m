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
% OUTPUTS:
%   cost   = value of the cost function f(x)

function cost = target_cost(x,nz,M,Ts,Ts_sim,ymeas_camera,ymeas_radar,z_minus,R_camera,R_radar,Q,A_terminal)

%% initialization - extract components in the optimization variables vector
z0 = x(1:nz,1); % extract states components from 'x' (everything apart from disturbances)
d_in = [x(6:(6+M-1),1)';x(6+M:end,1)']; % extract disturbances vectors from 'x' (2*M+2 elements since the disturbance has 2 components)

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

%% cost = sum of squared errors
% cost    =   sum((err_vec).*(err_vec)); % scalar product of error vectors: f(x) = F'(x)F(x)

%% Compute sum of squared errors - avoid imaginary numbers problems, same cost
cost   =   sum((ymeas_camera-ysim_resampled).*(inv(R_camera)*(ymeas_camera-ysim_resampled)),'all') + sum((ymeas_radar-ysim_resampled).*(inv(R_radar)*(ymeas_radar-ysim_resampled)),'all')+...
            sum(d_in.*(inv(Q)*d_in),'all') + sum((z_sim(:,1)-z_minus).*(inv(A_terminal)*(z_sim(:,1)-z_minus)),'all');

end
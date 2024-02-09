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
%   F    = vector valued function F(x) such that f = F(x)'F(x)
% gradF  = gradient of the function F, computed through analytical expression 

function [F,gradF] = target_cost_GN_grad(x,nz,M,Ts,Ts_sim,ymeas_camera,ymeas_radar,z_minus,R_camera,R_radar,Q,A_terminal)

%% initialization - extract components in the optimization variables vector
z0 = x(1:nz,1);                             % extract states components from 'x' (everything apart from disturbances)
d_in = [x(6:(6+M-1),1)';x(6+M:end,1)'];     % extract disturbances vectors from 'x' (2*M elements since the disturbance has 2 components)

%% Run simulation with DT model
time_FFD    =   0:Ts_sim:M*Ts;    
Nsim_FFD    =   length(time_FFD);   % total number of simulation steps

z_sim      =   zeros(nz,Nsim_FFD);  % to store simulation results
z_sim(:,1) =   z0;                  % set initial state for simulation
j_z = zeros(nz,(nz+size(d_in,1))*Nsim_FFD);

% Simulation loop
for ind = 2:Nsim_FFD + 1
    if ind == Nsim_FFD+1
        d = [0;0];
        j_z(:,(ind-2)*7+1:(ind-1)*7) = analytical_grad(Ts,z_sim(:,ind-1),d);
    else
        d                  =   d_in(:,floor(time_FFD(ind-1)/Ts)+1); % disturbances kept constant for time interval = Ts (sensor sampling)
        if abs(z_sim(5,ind-1)) > eps(ones(1,'like',z_sim))
            z_sim(:,ind)      = CTRV_exact_DT_cartesian(z_sim(:,ind-1),d,Ts_sim); % Cartesian Exact model
            j_z(:,(ind-2)*7+1:(ind-1)*7) = analytical_grad(Ts,z_sim(:,ind-1),d);
        else
            z_sim(5,ind-1) = eps(ones(1,'like',z_sim));
            z_sim(:,ind)      = CTRV_exact_DT_cartesian(z_sim(:,ind-1),d,Ts_sim); % Cartesian Exact model
            j_z(:,(ind-2)*7+1:(ind-1)*7) = analytical_grad(Ts,z_sim(:,ind-1),d);
        end
    end
end

% disp('jacobian contains real' + string(isreal(j_z)))

%% Extract simulation results and respective measured data 
ysim = [z_sim(1,:);z_sim(2,:);z_sim(3,:);z_sim(4,:)]; % row vector 3*Nsim

if Ts == Ts_sim
    ysim_resampled = ysim; % downsampling
else
    ysim_resampled = ysim(:,1:1/Ts:end); % downsampling
end

%% Error Vector Computation

err_camera = inv(sqrt(R_camera))*(ymeas_camera-ysim_resampled); % row vector
err_radar = inv(sqrt(R_radar))*(ymeas_radar-ysim_resampled); % row vector
err_dist = inv(sqrt(Q))*d_in;
A_terminal = diag(diag(A_terminal)); % take only diagonal terms, corresponding to state variance, avoid taking the square of correlation terms which might be begative
err_terminal = inv(sqrt(A_terminal))*(z_sim(:,1)-z_minus);
err_vec = [err_camera(1,:) err_camera(2,:) err_camera(3,:) err_camera(4,:)...
    err_radar(1,:) err_radar(2,:) err_radar(3,:) err_radar(4,:)...
    err_dist(1,:) err_dist(2,:) ...
    err_terminal']'; % column vector F(x)

% compute analytical gradient iteratively
jacobF = zeros(length(err_vec),length(x));
jacob_z = zeros(nz,length(x)*Nsim_FFD); % matrix of size 5x(M+1*(nz+2*M));
jacob_z(:,1:length(x)) = [eye(nz) zeros(nz,length(x)-nz)]; % grad_x(z(-M|t))

% build delta_x_f_z
for ind = 2:Nsim_FFD % through all simulation steps
    jacob_z(:,((ind-1)*length(x))+1:ind*length(x)) = j_z(:,((ind-2)*(nz+size(d_in,1)))+1:(ind-1)*(nz+size(d_in,1))-2)*jacob_z(:,((ind-2)*length(x))+1:(ind-1)*length(x));
    if ind == 2
        jacob_x_f_z = [j_z(:,1:nz+1) zeros(nz,M-1) j_z(:,nz+2) zeros(nz,M-1)];
    else
        jacob_x_f_z = zeros(nz,length(x));
        jacob_x_f_z(:,nz+ind-1) = j_z(:,(ind-1)*(nz+1));
        jacob_x_f_z(:,nz+M+ind-1) = j_z(:,(ind-1)*(nz+2));
    end
    jacob_z(:,((ind-1)*length(x))+1:ind*length(x)) = jacob_z(:,((ind-1)*length(x))+1:ind*length(x))+jacob_x_f_z;
end

% filling jacobF - radar
for ind = 1:4
    for iind = 1:(M+1)
        jacobF((ind-1)*(M+1)+iind,:) = -inv(sqrt(R_radar(ind,ind)))*jacob_z(ind,(iind-1)*length(x)+1:(iind)*length(x));
    end
end

% filling jacobF - camera
for ind = 1:4
    for iind = 1:(M+1)
        jacobF((ind-1)*(M+1)+iind+4*(M+1),:) = -inv(sqrt(R_camera(ind,ind)))*jacob_z(ind,(iind-1)*length(x)+1:(iind)*length(x));
    end
end

% filling jacobF - disturbance
dist_jacob = zeros(2*M,length(x));

%first component of disturbance
dist_jacob(1:M,:) = [zeros(M,nz) eye(M,M) zeros(M,M)];
dist_jacob(M+1:end,:) = [zeros(M,nz) zeros(M,M) eye(M,M)];

jacobF(end-nz-2*M+1:end-nz-M,:) = inv(sqrt(Q(1,1)))*dist_jacob(1:M,:);
jacobF(end-nz-M+1:end-nz,:) = inv(sqrt(Q(2,2)))*dist_jacob(M+1:end,:);

% filling jacobF - terminal
jacobF(end-nz+1:end,:) = [inv(sqrt(A_terminal)) zeros(nz,2*M)];

% disp('jacobian contains real' + string(isreal(jacobF)))

%% no disturbance quadratic term
% err_vec = [err_camera(1,:) err_camera(2,:) err_camera(3,:) err_camera(4,:)...
%     err_radar(1,:) err_radar(2,:) err_radar(3,:) err_radar(4,:)...
%     err_terminal']'; % column vector F(x)

%% no terminal cost
% err_vec = [err_camera(1,:) err_camera(2,:) err_camera(3,:) err_camera(4,:)...
%     err_radar(1,:) err_radar(2,:) err_radar(3,:) err_radar(4,:) err_dist(2,:)]'; % column vector F(x)


F = err_vec;
gradF = jacobF'; 
end
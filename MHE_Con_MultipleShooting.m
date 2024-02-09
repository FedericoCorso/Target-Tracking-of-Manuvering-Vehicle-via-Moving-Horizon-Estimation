% Function executing MHE algorithm - with EKF terminal cost update
% INPUT:    Ts              filter/measurement sampling time
%           Ts_sim          filter/measurement sampling time
%           N               legth of measurement vector
%           nz              state vector length
%           ny              lenght of measurement vector
%           nu              length of input vector
%           Msec            seconds of Moving Horizon Estimator window
%           MHE_interval    number of steps after which optimization routine is repeated
%           R_camera        Camera sensor covariance matrix
%           R_radar         Radar sensor covariance matrix
%           Q               Process Noise Covariance
%           A_terminal      State/terminal Cost Covariance matrix
%           z_minus         State z(-M+1|t-1)
%           myoptions       options for optimization routine
%           ymeas_camera    Data from camera
%           ymeas_radar     Data from radar
%           a_max           Maximum longitudinal acceleration
%           a_min           Minimum longitudinal acceleration
%           alpha_max       Maximum angular acceleration
%           alpha_min       Minimum angular acceleration
%           ego_pos         [X,Y] ego vehicle global coordinates, as a row vector       
%           ego_or          ego yaw angle in global frame, row vector
%
% OUTPUT:   z_est          State estimate
%           d_est          Input estimate
%           comp_time      vector containing the computational time for
%                          each iteration of the filter. Column vector 
function [z_est,d_est,comp_time] = MHE_Con_MultipleShooting(x0,Ts,Ts_sim,N,nz,ny,nu,Msec,MHE_interval,R_camera,R_radar,Q,A_terminal,z_minus,myoptions,ymeas_camera,ymeas_radar,a_max,a_min,alpha_max,alpha_min,ego_pos,ego_or)
    
    M               =   round(Msec/Ts); % number of steps in the window
    
    z_est = zeros(nz,N); % array to store estimated state
    d_est = zeros(nu,N); % array to store estimated disturbances

% multiple shooting routine
for ind = M+1:MHE_interval:N
    tic

    % Defining Constraints - Linear Inequality on d - to redefine the
    % matrices
    C = [ zeros(2*M,nz*(M+1))   -eye(2*M);
          zeros(2*M,nz*(M+1))   eye(2*M)];

    d = [-a_max*ones(M,1);-alpha_max*ones(M,1);a_min*ones(M,1);alpha_min*ones(M,1)];


    q = 2*(M+1); % number of non linear inequality constraints
    
    p = nz*M; % number of non-linear equality constraints
    
    myoptions.GN_funF       = @(x)target_cost_con_GN_multipleshooting(x,nz,M,Ts,Ts_sim,ymeas_camera(:,ind-M:ind),ymeas_radar(:,ind-M:ind),z_minus,R_camera,R_radar,Q,A_terminal,ego_pos(:,ind-M:ind),ego_or(:,ind-M:ind));
    
    [xstar,~,~,~,~] = myfmincon(@(x)target_cost_con_multipleshooting(x,nz,M,Ts,Ts_sim,ymeas_camera(:,ind-M:ind),ymeas_radar(:,ind-M:ind),z_minus,R_camera,R_radar,Q,A_terminal,ego_pos(:,ind-M:ind),ego_or(:,ind-M:ind)),x0,[],[],C,d,p,q,myoptions);
    
    % new initial guess
    x0 = [xstar(nz+1:nz*(M+1)); zeros(nz,1); xstar(nz*(M+1)+2:nz*(M+1)+M);0.5;xstar(nz*(M+1)+M+2:end);0.5]; % initialize the next estimation problem with the results of the last opt routine
    
    % course of the state over the estimation window
    z_course = xstar(1:nz*(M+1),1); % column vector containing all the state values over the window
    
    z_sim = [z_course(1:nz:end)';z_course(2:nz:end)';z_course(3:nz:end)';z_course(4:nz:end)';z_course(5:nz:end)']; % row vector 5*Nsim
    
    % course of the disturbance over the estimation window
    d_course = [xstar(nz*(M+1) + 1:nz*(M+1)+M,1)';xstar(nz*(M+1)+M+1:end,1)'];
    
    z_est(:,ind-M) = [z_course(end-nz+1,1);z_course(end-nz+2,1);z_course(end-nz+3,1);z_course(end-nz+4,1);z_course(end,1);]; % z(0|t)
    d_est(:,ind-M) = [d_course(1,1);d_course(2,1)]; % d(-M|t)
    
    % z_minus = z_sim(:,Ts/Ts_sim+1); % z(-M+1|t) = z_minus(-M|t) new initial guess, taken out of the simulation.

    [z_minus,A_terminal] = EKF_terminal_cost_update(ymeas_radar(:,ind-M),ymeas_camera(:,ind-M),z_est(:,ind-M),z_minus,Ts,Q,R_radar,R_camera,A_terminal);% compute the z_minus term with the EKF update
    Computational_time(ind,1) = toc;
end

z_est = [zeros(nz,M) z_est(:,1:end-M)];
d_est = [zeros(nu,M) d_est(:,1:end-M)];
comp_time = Computational_time;
end
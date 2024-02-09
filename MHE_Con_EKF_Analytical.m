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

function [z_est,d_est,comp_time] = MHE_Con_EKF_Analytical(Ts,Ts_sim,N,nz,ny,nu,Msec,MHE_interval,R_camera,R_radar,Q,A_terminal,z_minus,myoptions,ymeas_camera,ymeas_radar,a_max,a_min,alpha_max,alpha_min,ego_pos,ego_or)
    M               =   round(Msec/Ts); % number of steps in the window
    
    % slect initial guess
    x0 = [z_minus;0*ones(2*M,1)];
    
    z_est = zeros(nz,N); % array to store estimated state
    d_est = zeros(nu,N); % array to store estimated disturbances
    Computational_time = zeros(N,1); % array storing computational time for each estimation

for ind = M+1:MHE_interval:N
    tic
    myoptions.GN_funF       =  @(x)target_cost_GN_con_grad(x,nz,M,Ts,Ts_sim,ymeas_camera(:,ind-M:ind),ymeas_radar(:,ind-M:ind),z_minus,R_camera,R_radar,Q,A_terminal,ego_pos(:,ind-M:ind),ego_or(:,ind-M:ind));

    q = 2*(M+1); % number of non linear inequality constraints

    % Run solver on window from 'ind-M' up to 'ind' step
    [xstar,~,~,~,~] = myfmincon(@(x)target_cost_con(x,nz,M,Ts,Ts_sim,ymeas_camera(:,ind-M:ind),ymeas_radar(:,ind-M:ind),z_minus,R_camera,R_radar,Q,A_terminal,ego_pos(:,ind-M:ind),ego_or(:,ind-M:ind)),x0,[],[],[],[],0,q,myoptions);
    
    % simulation of the system with optimal solution
    z_minus_M_t = xstar(1:nz,1); % z(-M|t) of current iteration, aka estimated initial state of the system
    d_course = [xstar(6:6+M-1,1)';xstar(6+M:end,1)']; % course of disturbance
    
    %% Run simulation with FFD
    time_FFD    =   0:Ts_sim:M*Ts;
    Nsim_FFD    =   length(time_FFD);
    
    z_sim      =   zeros(nz,Nsim_FFD);
    z_sim(:,1) =   z_minus_M_t;
    
    % system simulation - Choose the proper model
    for iind=2:Nsim_FFD
        
        d                  =   d_course(:,1+floor(time_FFD(iind-1)/Ts));
        
        if abs(z_sim(5,iind-1)) > eps(ones(1,'like',z_sim))
            z_sim(:,iind)      = CTRV_exact_DT_cartesian(z_sim(:,iind-1),d,Ts_sim); % Cartesian Exact model
        else
            z_sim(5,iind-1) = eps(ones(1,'like',z_sim));
            z_sim(:,iind)      = CTRV_exact_DT_cartesian(z_sim(:,iind-1),d,Ts_sim); % Cartesian Exact model
        end
       
    end

    x0 = [z_sim(1,Ts/Ts_sim + 1); z_sim(2,Ts/Ts_sim + 1); z_sim(3,Ts/Ts_sim + 1);z_sim(4,Ts/Ts_sim + 1);z_sim(5,Ts/Ts_sim + 1);[xstar(7:6+M,1);0];[xstar(6+M+2:end,1)];0];
    z_est(:,ind-M) = [z_sim(1,end); z_sim(2,end); z_sim(3,end);z_sim(4,end);z_sim(5,end)]; % z(0|t)
    d_est(:,ind-M) = [d_course(1,1);d_course(2,1)]; % d(-M|t)
    [z_minus,A_terminal] = EKF_terminal_cost_update(ymeas_radar(:,ind-M),ymeas_camera(:,ind-M),z_est(:,ind-M),z_minus,Ts,Q,R_radar,R_camera,A_terminal);% compute the z_minus term with the EKF update
    Computational_time(ind,1) = toc;
end

% check results against ground truth
z_est = [zeros(nz,M) z_est(:,1:end-M)];
d_est = [zeros(nu,M) d_est(:,1:end-M)];
comp_time = Computational_time;
end
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
%
% OUTPUT:   z_est          State estimate
%           d_est          Input estimate
%           comp_time      vector containing the computational time for
%                          each iteration of the filter. Column vector 
function [z_est,d_est,comp_time] = MHE_Unc_EKF(Ts,Ts_sim,N,nz,ny,nu,Msec,MHE_interval,R_camera,R_radar,Q,A_terminal,z_minus,myoptions,ymeas_camera,ymeas_radar)
    
    M               =   round(Msec/Ts); % number of steps in the window
    x0 = [z_minus;0*ones(2*M,1)]; % slect initial guess for the optimization routine

    z_est = zeros(nz,N); % store estimated state
    d_est = zeros(nu,N); % store estimated disturbances
    Computational_time = zeros(N,1); % array storing computational time for each estimation

for ind = M+1:MHE_interval:N
    tic
    % when using GN method
    myoptions.GN_funF       =  @(x)target_cost_GN(x,nz,M,Ts,Ts_sim,ymeas_camera(:,ind-M:ind),...
                                                  ymeas_radar(:,ind-M:ind),z_minus,...
                                                  R_camera,R_radar,Q,A_terminal);
    
    % Run solver on window from 'ind-M' up to 'ind' step
    [xstar,~,~,~,~] = myfminunc(@(x)target_cost(x,nz,M,Ts,Ts_sim,ymeas_camera(:,ind-M:ind),ymeas_radar(:,ind-M:ind),z_minus,R_camera,R_radar,Q,A_terminal),x0,myoptions);

    % optimal solution
    z_minus_M_t = xstar(1:nz,1); % z(-M|t) of current iteration, aka estimated initial state of the system
    d_course = [xstar(6:6+M-1,1)';xstar(6+M:end,1)']; % course of disturbance, size 2xM
    
    time_FFD    =   0:Ts_sim:M*Ts; % simulation time
    Nsim_FFD    =   length(time_FFD); % # of simulation steps
    
    z_sim      =   zeros(nz,Nsim_FFD); % store simulation results
    z_sim(:,1) =   z_minus_M_t; % output of the optimization 
    
    % system simulation - Choose the proper model
    for iind=2:Nsim_FFD

        d                  =   d_course(:,1+floor(time_FFD(iind-1)/Ts)); 
        
        % avoid division by 0
        if abs(z_sim(5,iind-1)) > eps(ones(1,'like',z_sim))
            z_sim(:,iind)      = CTRV_exact_DT_cartesian(z_sim(:,iind-1),d,Ts_sim); % Cartesian Exact model
        else
            z_sim(5,iind-1) = eps(ones(1,'like',z_sim));
            z_sim(:,iind)      = CTRV_exact_DT_cartesian(z_sim(:,iind-1),d,Ts_sim); % Cartesian Exact model
        end
       
    end
    
    % new initial point for next iteration optimization problem
    x0 = [z_sim(1,Ts/Ts_sim + 1); z_sim(2,Ts/Ts_sim + 1); z_sim(3,Ts/Ts_sim + 1);z_sim(4,Ts/Ts_sim + 1);z_sim(5,Ts/Ts_sim + 1);[xstar(7:6+M,1);0];[xstar(6+M+2:end,1)];0];

    z_est(:,ind-M) = [z_sim(1,end); z_sim(2,end); z_sim(3,end);z_sim(4,end);z_sim(5,end)]; % z(0|t).
    d_est(:,ind-M) = [d_course(1,1);d_course(2,1)]; % d(-M|t), value of the disturbance which will not be changed in next iteration steps.
    
    [z_minus,A_terminal] = EKF_terminal_cost_update(ymeas_radar(:,ind-M),ymeas_camera(:,ind-M),z_est(:,ind-M),z_minus,Ts,Q,R_radar,R_camera,A_terminal);% compute the z_minus term with the EKF update
    Computational_time(ind,1) = toc;
end

z_est = [zeros(nz,M) z_est(:,1:end-M)];
d_est = [zeros(nu,M) d_est(:,1:end-M)];
comp_time = Computational_time;
end
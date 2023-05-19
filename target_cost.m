function cost = target_cost(x,M,Ts,ymeas)
% function to compute the cost function to be optimized
z0 = x(1:5,1); % 5 states components
d_in = [x(6:6+M,1)';x(6+M+1:end,1)']; % 2*M+2 elements since the disturbance has 2 components

%% Run simulation with FFD
time_FFD    =   [0:0.01:M*Ts];
Nblock      =   Ts/0.01;
Nsim_FFD    =   length(time_FFD);

z_sim      =   zeros(5,Nsim_FFD);
z_sim(:,1) =   z0;

% system simulation to compute stage costs at ith time instant
for ind=2:Nsim_FFD
    d                  =   d_in(:,1+floor(time_FFD(ind)/Ts));
    zdot               =   CTCV_model(0,z_sim(:,ind-1),0,d,0);
    z_sim(:,ind)       =   z_sim(:,ind-1)+Ts/Nblock*zdot;
end

% collect the simulated output, and downsample it so it has the same
% dimension as measured signal
ysim = [z_sim(1,:);z_sim(2,:);z_sim(3,:)]; % row vector 3*Nsim
ysim_resampled = ysim(:,1:1/Ts:end);
ymeas_resampled = ymeas(:,1:length(ysim_resampled));

% compute the error
err = (ymeas_resampled-ysim_resampled);

% stack errors in a vector
err_vec = [err(1,:) err(2,:) err(3,:)]'; % F(x)

%% Compute sum of squared errors
cost    =   sum(err_vec.*err_vec); % f(x) = F'(x)F(x)
end
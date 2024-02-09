% CTRV_model_DT Discrete Time implementation of the Coordinated Turn Model
% Inputs:   z_k         state at k-th time step
%           d_k         input disturbance/noise at k-th time step (acc_std_dev*Ts)
%           Ts          discretization step/sampling time
%
% Outputs:  z_k_plus_1  state value at k+1 time step   
function z_k_plus_1 = CTRV_model_DT(z_k,d_k,Ts)
z_k_plus_1(1,1) = z_k(1,1) + Ts*z_k(3,1)*cos(z_k(4,1));
z_k_plus_1(2,1) = z_k(2,1) + Ts*z_k(3,1)*sin(z_k(4,1));
z_k_plus_1(3,1) = z_k(3,1) + Ts*d_k(1,1);
z_k_plus_1(4,1) = z_k(4,1) + Ts*z_k(5,1);
z_k_plus_1(5,1) = z_k(5,1) + Ts*d_k(2,1); 
end
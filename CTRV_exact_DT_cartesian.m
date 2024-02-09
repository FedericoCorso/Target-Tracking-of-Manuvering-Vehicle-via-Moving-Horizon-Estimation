% function simulating one step ahead the CTRV model.
%
% INPUTS:
%   z_k   =  current state variables
%
%   d_k  =   current value of input disturbances
%
%   Ts   =   sampling time
%   
% OUTPUTS:
%   z_k_plus_1   = value of state variables at next time step
function z_k_plus_1 = CTRV_exact_DT_cartesian(z_k,d_k,Ts)

    h = atan2(z_k(4,1),z_k(3,1));

    z_k_plus_1(1,1) = z_k(1,1) + (z_k(3,1)/z_k(5,1))*sin(Ts*z_k(5,1)) - (z_k(4,1)/z_k(5,1))*(1-cos(Ts*z_k(5,1))) + (Ts^2/2)*cos(h)*d_k(1,1);
    z_k_plus_1(2,1) = z_k(2,1) + (z_k(4,1)/z_k(5,1))*sin(Ts*z_k(5,1)) + (z_k(3,1)/z_k(5,1))*(1-cos(Ts*z_k(5,1))) + (Ts^2/2)*sin(h)*d_k(1,1);
    z_k_plus_1(3,1) = z_k(3,1)*cos(z_k(5,1)*Ts)-z_k(4,1)*sin(z_k(5,1)*Ts) + Ts*cos(h)*d_k(1,1);
    z_k_plus_1(4,1) = z_k(3,1)*sin(z_k(5,1)*Ts)+z_k(4,1)*cos(z_k(5,1)*Ts) + (Ts)*sin(h)*d_k(1,1);
    z_k_plus_1(5,1) = z_k(5,1) + Ts*d_k(2,1);
end
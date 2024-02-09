% function executing the EKF terminal cost update
% INPUT:    radar_meas      = radar measurements
%           camera_meas     = camera measurements
%           z_est           = current estimate z(-M|t)
%           z_minus         = previous estimate z(-M|t)
%           Ts              = sampling time
%           Q               = process covariance
%           R_radar         = radar covariance matrix
%           R_camera        = camera covariance matrix
%           P_old           = previous terminal cost covariance matrix
% 
% OUTPUT:   z_minus_new     = estimate of z(-M+1|t-1)
%           P_new           = new terminal cost covariance matrix

function [z_minus_new,P_new] = EKF_terminal_cost_update(radar_meas,camera_meas,z_est,z_minus,Ts,Q,R_radar,R_camera,P_old)

    % select measurement gathered at -M|t, at the beginning of the horizon
    y = [radar_meas;camera_meas];

    % last result from state estimation
    yK = [z_minus(1,1);z_minus(2,1);z_minus(3,1);z_minus(4,1);z_minus(1,1);...
          z_minus(2,1);z_minus(3,1);z_minus(4,1)];
    e =  y - yK; % difference between measurements and estimated z_est(-M|t)

    F_k = computeFJacobian(Ts,z_est); % f(z) jacobian at the last optimal solution
    F_k_t = F_k'; % jacobian transpose
    
    % 8x5 measurement matrix
    H = [eye(4,5); % radar
         eye(4,5)];% camera
    I = eye(5);

    Ht = H'; % H transposed, compute just once and use 2 times
    
    % measurement covariance matrix
    R = [R_radar zeros(4,4);
         zeros(4,4) R_camera];

    G = P_old*Ht/(H*P_old*Ht + R);

    z_temp = z_minus + G*e;

    % P_temp = (I-G*H)*P_old; % quicker and easier to implement but do not
    % guarantee SPD of covariance matrix P_temp
    
    % Joseph Stabilized Update
    P_temp = (I-G*H)*P_old*(I-G*H)' + G*R*G';

    % computing process covariance
    h = atan2(z_est(4,1),z_est(3,1));
    G_k = [(Ts^2/2)*cos(h) 0;
           (Ts^2/2)*sin(h) 0;
                 Ts*cos(h) 0;
                 Ts*sin(h) 0;
                         0 Ts];
    E = G_k*Q*G_k'; % at the last estimated output
    
    %% PREDICTION
    z_est_plus_1_hat(1,1) = z_est(1,1) + (z_est(3,1)/z_est(5,1))*sin(Ts*z_est(5,1)) - (z_est(4,1)/z_est(5,1))*(1-cos(Ts*z_est(5,1)));
    z_est_plus_1_hat(2,1) = z_est(2,1) + (z_est(4,1)/z_est(5,1))*sin(Ts*z_est(5,1)) + (z_est(3,1)/z_est(5,1))*(1-cos(Ts*z_est(5,1)));
    z_est_plus_1_hat(3,1) = z_est(3,1)*cos(z_est(5,1)*Ts)-z_est(4,1)*sin(z_est(5,1)*Ts);
    z_est_plus_1_hat(4,1) = z_est(3,1)*sin(z_est(5,1)*Ts)+z_est(4,1)*cos(z_est(5,1)*Ts);
    z_est_plus_1_hat(5,1) = z_est(5,1);
    
    z_minus_new = z_est_plus_1_hat - F_k*z_est + F_k*z_temp;
    P_new = F_k*P_temp*F_k_t + E;

end
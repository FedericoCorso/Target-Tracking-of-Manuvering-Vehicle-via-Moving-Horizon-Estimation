% Function to execute EKF algorithm
% Inputs:   radar_meas        radar measurement at k-th time step,
%                             [x;y;vx;vy] as column vector
%           camera_meas       camera measurement at k-th time step,
%                             [x;y;vx;vy] as column vector
%           z_k               State at previous time step
%           Ts                Filter time step
%           Q                 Process Covariance Matrix
%           R_radar           Radar measurement covariance matrix
%           R_camera          Camera measurement covariance matrix
%           P_old             A priori state covariance matrix
%
% Outputs:  z_k_plus_1        estimated state 
%           P_k_plus_1        estimated state covariance matrix

function [z_k_plus_1,P_k_plus_1] = EKF(radar_meas,camera_meas,z_k,Ts,Q,R_radar,R_camera,P_old)
    %% %{=============== PREDICTION =============}%
    z_k_plus_1_hat(1,1) = z_k(1,1) + (z_k(3,1)/z_k(5,1))*sin(Ts*z_k(5,1)) - (z_k(4,1)/z_k(5,1))*(1-cos(Ts*z_k(5,1)));
    z_k_plus_1_hat(2,1) = z_k(2,1) + (z_k(4,1)/z_k(5,1))*sin(Ts*z_k(5,1)) + (z_k(3,1)/z_k(5,1))*(1-cos(Ts*z_k(5,1)));
    z_k_plus_1_hat(3,1) = z_k(3,1)*cos(z_k(5,1)*Ts)-z_k(4,1)*sin(z_k(5,1)*Ts);
    z_k_plus_1_hat(4,1) = z_k(3,1)*sin(z_k(5,1)*Ts)+z_k(4,1)*cos(z_k(5,1)*Ts);
    z_k_plus_1_hat(5,1) = z_k(5,1);

    F_k = computeFJacobian(Ts,z_k);
    
    %% covariance and output
    h = atan2(z_k(4,1),z_k(3,1));
    G_k = [(Ts^2/2)*cos(h) 0;
           (Ts^2/2)*sin(h) 0;
                 Ts*cos(h) 0;
                 Ts*sin(h) 0;
                         0 Ts];

    E = G_k*Q*G_k';
    P_temp = F_k*P_old*(F_k') + E; % update covariance matrix
    yK = [z_k_plus_1_hat(1,1);z_k_plus_1_hat(2,1);z_k_plus_1_hat(3,1);z_k_plus_1_hat(4,1);z_k_plus_1_hat(1,1);z_k_plus_1_hat(2,1);z_k_plus_1_hat(3,1);z_k_plus_1_hat(4,1)]; % compute output prediction estimate
    
    %% %{========================================}%
    %% %{=============== CORRECTION =============}%
    % 8x5 measurement matrix
    H = [eye(4,5); % radar
         eye(4,5)];% camera
    I = eye(5);
    % innovation
    y = [radar_meas;camera_meas]; % must be a clumn vector
    e =  y - yK; % difference between real meas. and predicted output
    Ht = H'; % H transposed, compute just once and use 2 times
    R = [R_radar zeros(4,4);
         zeros(4,4) R_camera];

    S = H*P_temp*Ht + R;  % % residual covariance
    
    % near optimal Kalman Gain
    K =  (P_temp*Ht)/S;
    
    % updated state estimate
    z_k_plus_1 = z_k_plus_1_hat + K*e; % feedback correction
    % updated covariance estimate
    P_k_plus_1 = (I-K*H)*P_temp;
end
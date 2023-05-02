clc

%% simulation parameters

%% estimation
% initial parameters for longitudinal dynamics
th_long = zeros(3,1);

th_long(1,1) = 0.5;     % C1 
th_long(2,1) = 0.125;   % C2 
th_long(3,1) = -0.125;  % C3 

% initial parameters for lateral dynamics
th_lat = zeros(7,1);

th_lat(1,1) = 1270;    % vehicle mass (kg)
th_lat(2,1) = 1536.7;  % vehicle moment of inertia (kg*m^2)
th_lat(3,1) = 1.05;    % distance between center of gravity and front axle (m)
th_lat(4,1) = 1.895;   % distance between center of gravity and rear axle (m)
th_lat(5,1) = 135000;  % front axle cornering stiffness (N/rad)
th_lat(6,1) = 85000;   % rear axle cornering stiffness (N/rad)
th_lat(7,1) = 0.9;     % road - tire coefficient
 

%% longitudinal and lateral models
z0_long = zeros(5,1);
z0_long (1,1) = 20;     % preceding vehicle speed (m/s)
z0_long (2,1) = 20;     % relative distance host to prec. (m)
z0_long (3,1) = 20;     % host speed (m/s)
z0_long (5,1) = 1;      % aggressiveness parameter

z0_lat = zeros(6,1);    
z0_lat (1,1) = 0;    % lateral speed
z0_lat (2,1) = 0;    % yaw rate
z0_lat (3,1) = 0;    % front steering angle
z0_lat (4,1) = 0;    % trajectory tracking offset (e)
z0_lat (5,1) = 0;    % heading angle
z0_lat (6,1) = 0;    % road curvature


dk = 0.1; % sampleTime
Tend = 10;
Tsim = 0:dk:Tend;
Nsim = Tend/dk;
cov_matrix = zeros(2,2); % eye(2)


zsim_long = zeros(5,Nsim+1); % matrix for longitudinal trajectory 
zsim_long(:,1) = z0_long;

zsim_lat = zeros(6,Nsim+1); % matrix for lateral trajectory
zsim_lat(:,1) = z0_lat;

% simulation
for ii = 1:Nsim
    
    model_noise_long = mvnrnd(zeros(2,1),cov_matrix,1)'; % adding white noise

    zsim_long(:,ii+1) = prec_vehicle_long(zsim_long(:,ii),model_noise_long,dk,th_long);
    
    zsim_long(3,ii+1) = provaego(ii).ActorPoses(1).Velocity(1,1);

    % da chiudere in una funzione separata la mediana delle misure distanza
    dimObjdec=size(provaego(ii).ObjectDetections);
    vector=zeros(dimObjdec);
    for a = 1:dimObjdec
        vector(a)=provaego(ii).ObjectDetections{a,1}.Measurement(1,1);
    end

    zsim_long(2,ii+1) = median(vector);


    zsim_lat(:,ii+1) = zsim_lat(:,ii) + dk*prec_vehicle_lat(0, zsim_lat(:,ii),th_lat, zsim_long(1,ii+1));

    % sostituire provaego con prova prec (telecamera veicolo davanti)
    media_e = mean(provaego(ii).LaneDetections.LaneBoundaries(1,1).LateralOffset + provaego(ii).LaneDetections.LaneBoundaries(1,2).LateralOffset)
    zsim_lat(4,ii+1) = media_e;
    

end

figure 
plot(Tsim,zsim_long(1,:)), hold on, grid on
plot(Tsim,zsim_long(2,:)), hold on
plot(Tsim,zsim_long(3,:)), hold on
plot(Tsim,zsim_long(4,:)), hold on
title("longitudinal states")
legend('V_p', 'd', 'V_h', 'C4')

% figure 
% plot(Tsim,zsim_lat(3,:)),hold on, grid on
% plot(Tsim,zsim_lat(4,:)), grid on
% title("lateral")
% 
% figure
% plot(Tsim,zsim_long(1,:)), hold on, grid on
% plot(Tsim,zsim_lat(1,:)), hold on, grid on
% plot(Tsim,zsim_lat(2,:)), hold on
% plot(Tsim,zsim_lat(3,:)),hold on, grid on
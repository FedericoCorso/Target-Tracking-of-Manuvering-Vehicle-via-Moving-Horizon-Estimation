% Function generate measurement vectors
% Inputs:   Con         boolean to choose if you are dealing with
%                       constrained problem
%           Urban       boolean to choose scenario of interest
%           real_meas   boolean to choose to use (set to 1)real measurements or (set to 0) GT + WGN
%           min_time    first time instant of interest
%           max_time    last time instant of interest
%
% Outputs:  ymeas_camera    camera measurements at 10Hz for [x, y, v],
%                           returned as a row vector, size 3xN
%           ymeas_radar     radar measurements at 10Hz for [x, y, v],
%                           returned as a row vector, size 3xN
%           N               number of samples
%           time_vec        time vector for measurement samples, needed for
%                           plots, as a clumn vector, size Nx1
%           GT_state        Ground truth signals for plots, as a column
%                           vector, [x,y,vx,vy,yaw_rate], size Nx5
%           ego_pos         (X,Y) Position of the ego car in the world
%                           coordinates, row vector
%           ego_or          Yaw of the ego car in the world
%                           coordinates, wrt z axis, row vector

function [ymeas_camera,ymeas_radar,N,time_vec,GT_state,min_index,max_index,ego_pos,ego_or] = get_meas(Con,Urban,real_meas,min_time,max_time)
% HW_const_speed = 0; % Highway data scenario
if Urban
    load("Urban_time_vector.mat");
    load("Urban_camera_meas_array.mat");
    load("Urban_camera_speed_array.mat");
    load("Urban_radar_meas_array.mat");
    load("Urban_radar_speed_array.mat");
    load("Urban_GT_array.mat");
    load("Urban_GT_speed_array.mat");
    load("Urban_GT_state.mat");
% elseif HW_const_speed
else
    load("HW_time_vector.mat");
    load("HW_camera_meas_array.mat");
    load("HW_camera_speed_array.mat");
    load("HW_radar_meas_array.mat");
    load("HW_radar_speed_array.mat");
    load("HW_GT_array.mat");
    load("HW_GT_speed_array.mat");
    load("HW_GT_state.mat");
    load("HW_EgoState.mat");
    load("HW_Radar_mean_var.mat");
    load("HW_Camera_mean_var.mat");
end

Ts = 0.1;       % sensor sampling time
Ts_sim = 0.01;  % simulation sampling time, for GT [s]

max_index = max_time/Ts_sim + 1; 
min_index = min_time/Ts_sim + 1;

if Con 
    EgoState = EgoState(:,min_index:max_index);
    EgoState = EgoState(:,1:1/Ts:end);
    ego_pos = EgoState(1:2,:);
    ego_or = EgoState(3,:);
else
    ego_pos = EgoState(1:2,:);
    ego_or = EgoState(3,:);
end

GT_state_new  = GT_state;

if real_meas
    
    % when the camera measurement is NaN replace it with radar measurement
    camera_meas(isnan(camera_meas)) = radar_meas(isnan(camera_meas));
    
    % extract samples of interest from GT
    GT_state_1 = [GT(min_index:max_index,1:2),GT(min_index:max_index,4:5), GT_state_new(min_index:max_index,end)]; % fundamental row
    % downsample GT to sensors sampling time
    GT_state = GT_state_1(1:1/Ts:end,:); 

    % time vector
    time_vec = time_vec(~isnan(radar_meas(min_index:max_index,1)));

    % position data taken from radar
    x_pos_radar = radar_meas(~isnan(radar_meas(min_index:max_index,1)),1);
    y_pos_radar = radar_meas(~isnan(radar_meas(min_index:max_index,2)),2);
    xy_pos_radar = [x_pos_radar y_pos_radar]';

    % position from camera
    x_pos_camera = camera_meas(~isnan(radar_meas(min_index:max_index,1)),1);
    y_pos_camera = camera_meas(~isnan(radar_meas(min_index:max_index,2)),2);
    xy_pos_camera = [x_pos_camera y_pos_camera]';
    
    % speed data
    v_x_radar = radar_meas(~isnan(radar_speed(min_index:max_index)),4);
    v_y_radar = radar_meas(~isnan(radar_speed(min_index:max_index)),5);
    v_x_camera = camera_meas(~isnan(radar_speed(min_index:max_index)),4);
    v_y_camera = camera_meas(~isnan(radar_speed(min_index:max_index)),5);

    %%  building the measurement vector
    ymeas_camera = [xy_pos_camera;v_x_camera';v_y_camera'];
    ymeas_radar = [xy_pos_radar;v_x_radar';v_y_radar'];
    
    N = length(ymeas_radar); % number of meas samples
            
else % synthetic measurements
    
    % time vector
    time_vec = time_vec(min_index:max_index);
    time_vec = time_vec(1:1/Ts:end);
    
    %% create White Gaussian Noises to simulate sensor noises - multiply by 0 to set noises to 0
    
    %% camera WN
    % position noise
    camera_x_noise = sqrt(R_camera_var(1,1))*randn(length(time_vec),1); % lower accuracy of camera on longitudinal axis
    camera_y_noise = sqrt(R_camera_var(2,2))*randn(length(time_vec),1); % higher accuracy of camera on lateral axis
    % camera speed components noise
    camera_v_x_noise = sqrt(R_camera_var(3,3))*randn(length(time_vec),1);
    camera_v_y_noise = sqrt(R_camera_var(4,4))*randn(length(time_vec),1);

    %% radar WN
    % position noise
    radar_x_noise = sqrt(R_radar_var(1,1))*randn(length(time_vec),1); % higher accuracy of radar on longitudinal axis
    radar_y_noise = sqrt(R_radar_var(2,2))*randn(length(time_vec),1); % lower accuracy of radar on lateral axis
    % radar speed components noise
    radar_v_x_noise = sqrt(R_radar_var(3,3))*randn(length(time_vec),1);
    radar_v_y_noise = 0.1*sqrt(R_radar_var(4,4))*randn(length(time_vec),1);
     
    %% camera Outliers
    % create a WGN with 0 mean and std = 3*std of the measurements
    % then you select randomly samples within this signal
    % you add these randomly chosen samples by putting to NaN some of these
    % components
    
    % outliers multiplication factor - otherwise not so relevant
    outl_radar = 3;
    outl_camera = 30;

    % position noise
    camera_x_outliers = outl_camera*sqrt(R_camera_var(1,1))*randn(length(time_vec),1); % lower accuracy of camera on longitudinal axis
    camera_y_outliers = outl_camera*sqrt(R_camera_var(2,2))*randn(length(time_vec),1); % higher accuracy of camera on lateral axis

    % camera speed components noise
    camera_v_x_outliers = outl_camera*sqrt(R_camera_var(3,3))*randn(length(time_vec),1);
    camera_v_y_outliers = outl_camera*sqrt(R_camera_var(4,4))*randn(length(time_vec),1); 
    
    camera_outliers = [camera_x_outliers';camera_y_outliers';camera_v_x_outliers';camera_v_y_outliers'];
     
    %% radar Outliers
    
    % position noise
    radar_x_outliers = outl_radar*sqrt(R_radar_var(1,1))*randn(length(time_vec),1); % higher accuracy of radar on longitudinal axis
    radar_y_outliers = outl_radar*sqrt(R_radar_var(2,2))*randn(length(time_vec),1); % lower accuracy of radar on lateral axis

    % radar speed components noise
    radar_v_x_outliers = outl_radar*sqrt(R_radar_var(3,3))*randn(length(time_vec),1);
    radar_v_y_outliers = outl_radar*sqrt(R_radar_var(4,4))*randn(length(time_vec),1);
    
    radar_outliers = [radar_x_outliers';radar_y_outliers';radar_v_x_outliers';radar_v_y_outliers'];
    
    %% GT
    % extract samples of interest from GT: [x,y,vx,vy,yaw_rate]
    GT_state_1 = [GT(min_index:max_index,1:2),GT(min_index:max_index,4:5), GT_state_new(min_index:max_index,end)]; % fundamental row
    
    % downsample GT
    GT_state = GT_state_1(1:1/Ts:end,:); 

    % positions and velocities from radar
    x_pos_radar = GT_state(:,1) + radar_x_noise;
    y_pos_radar = GT_state(:,2) + radar_y_noise;
    xy_pos_radar = [x_pos_radar y_pos_radar]';%radar_meas(min_index:max_index,1:2)'; % transpose !!!
    v_x_radar = GT_state(:,3) + radar_v_x_noise;
    v_y_radar = GT_state(:,4) + radar_v_y_noise;

    % positions and velocities from camera
    x_pos_camera = GT_state(:,1) + camera_x_noise;
    y_pos_camera = GT_state(:,2) + camera_y_noise;
    xy_pos_camera = [x_pos_camera y_pos_camera]';
    v_x_camera = GT_state(:,3) + camera_v_x_noise;
    v_y_camera = GT_state(:,4) + camera_v_y_noise;


    %%  building the measurement vector
    % inserting outliers
    max_distance = max(sqrt(GT_state(:,1).^2 + GT_state(:,1).^2));
    
    for ind = 1:length(time_vec)
        
        out_radar = binornd(1,0.05*(1/(sqrt(GT_state(ind,1).^2 + GT_state(ind,1).^2)/max_distance)));
        out_camera = binornd(1,0.1*(sqrt(GT_state(ind,1).^2 + GT_state(ind,1).^2)/max_distance));
        
        if out_radar
            
            xy_pos_radar(:,ind) = xy_pos_radar(:,ind)+ radar_outliers(1:2,ind);
            v_x_radar(ind) = v_x_radar(ind) +  radar_outliers(3,ind);
            v_y_radar(ind) = v_y_radar(ind) + radar_outliers(4,ind);

        end

        if out_camera
           xy_pos_camera(:,ind) = xy_pos_camera(:,ind)+ camera_outliers(1:2,ind);
           v_x_camera(ind) = v_x_camera(ind) +  camera_outliers(3,ind);
           v_y_camera(ind) = v_y_camera(ind) + camera_outliers(4,ind);
        end
    
    end

    ymeas_camera = [xy_pos_camera;v_x_camera';v_y_camera'];
    ymeas_radar = [xy_pos_radar;v_x_radar';v_y_radar'];

    
    N = length(ymeas_radar); % number of meas samples

end

end
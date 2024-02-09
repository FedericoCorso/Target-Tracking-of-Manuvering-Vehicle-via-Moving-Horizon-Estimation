% This script creates the driving scenario on which simulation is run and 
% data are synthetically generated.

% the goal of this script is also to generate the ground truth to compare
% against estimation and sensor data to asses performances of our
% estimation algorithm
clear  
close all

addpath("helperfiles/")
addpath("sim_data/")

% to not save simulation data set to 1
dev = 1;

%% SCENARIO CREATION
% to-do implement
% how to specify vehicles physical properties?


scenario = drivingScenario;

roadCenters = ...
    [  0  20  40  49  50 100  50  49  40  20 -20 -40 -49 -50 -100  -50  -49  -40 -20   0
     -50 -50 -50 -50 -50   0  50  50  50  50  50  50  50  50    0  -50  -50  -50 -50 -50
       0   0   0   0   0   0   0   0   0   0   0   0   0   0    0    0    0    0   0   0]';

road(scenario, roadCenters, 'lanes', lanespec(2,"width",4,"marking",[laneMarking('Solid'),laneMarking('Dashed'),laneMarking('Solid')]));

% add vehicles to the track, half of right lane
egoCar = vehicle(scenario,'ClassID',1,'Position',[0 -52 0],'Yaw',0);
precCar = vehicle(scenario,'ClassID',1,'Position',[15 -52 0],'Yaw',0);

% offset from road centerline - assuming lane width is constant
d = 4/2;

% one way point is defined for each road center
roadOffset = [ 0  0  0  0  0  d  0  0  0  0  0  0  0  0 -d  0  0  0  0  0
              -d -d -d -d -d  0  d  d  d  d  d  d  d  d  0 -d -d -d -d -d
               0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0]';

% specify a trajectory with respect to the road center
egoWayPoints = roadCenters + roadOffset; % in the right lane

% trajectory of precedent car
precWayPoints(1:10,:) = roadCenters(1:10,:) + roadOffset(1:10,:); % sum first points
precWayPoints(1,1) = 15;    % change initial waypoint
precWayPoints(11,:)= roadCenters(11,:) - roadOffset(11,:); % L_lane
precWayPoints(12:18,:) = roadCenters(12:18,:) + roadOffset(12:18,:); % R_lane
precWayPoints(19,:)= roadCenters(19,:) - roadOffset(19,:); % L_lane
precWayPoints(20,:) = roadCenters(20,:) + roadOffset(20,:); % R_lane
precWayPoints(end,1) = 15;    % change final waypoint

smoothTrajectory(egoCar,egoWayPoints(:,:), 100);
smoothTrajectory(precCar,precWayPoints(:,:), 100);

% plot(scenario,'Waypoints','on') % for debug

% simulate and show simulation results - needed to check visually

% egoCar.chasePlot
% while advance(scenario)
%   pause(0.001)
% end

%% SYNTHETIC DATA GENERATION
%% radar
create_radar_data;
% plot distance measurements and G.T.
% figure
% plot(distance_meas.Time,distance_meas.realDistance,distance_meas.Time,distance_meas.measDistance)

%% vision
create_camera_data;

%% ground truth data to assess estimation performance
% vx = longitudinal speed of preceeding vehicle (absolute)
% vy = lateral speed of preceeding vehicle (absolute)
% yaw rate of preceeding vehicle (absolute)

% loop for generating ground truth data
real_estim_vars = struct('Time',[],'LongitudinalVel',[], ...
    'LateralVel',[],'YawRate',[],'Yaw',[]);
restart(scenario);
longitudinal_speed_ego = struct('Time',[],'LongitudinalVel',[]);

rng('default'); % repeatibility

while advance(scenario)
    poses = actorPoses(scenario);
    real_estim_vars.Time = [real_estim_vars.Time; scenario.SimulationTime];
    real_estim_vars.LongitudinalVel = [real_estim_vars.LongitudinalVel; (poses(2).Velocity(1,1)*cos(poses(2).Yaw*pi/180) + (poses(2).Velocity(1,2)*sin(poses(2).Yaw*pi/180)))];
    real_estim_vars.LateralVel = [real_estim_vars.LateralVel; (poses(2).Velocity(1,2)*cos(poses(2).Yaw*pi/180) - poses(2).Velocity(1,1)*sin(poses(2).Yaw*pi/180))];
    real_estim_vars.YawRate = [real_estim_vars.YawRate; poses(2).AngularVelocity(1,3)];
    real_estim_vars.Yaw = unwrap([real_estim_vars.Yaw;poses(2).Yaw],180);

    % build measured speed of egovehicle
    longitudinal_speed_ego.Time = [longitudinal_speed_ego.Time; scenario.SimulationTime];
    longitudinal_speed_ego.LongitudinalVel = [longitudinal_speed_ego.LongitudinalVel;(poses(1).Velocity(1,1)*cos(poses(1).Yaw*pi/180) + (poses(1).Velocity(1,2)*sin(poses(1).Yaw*pi/180)))+normrnd(0,1)];
end

%% SAVE DATA
if ~dev
    save("sim_data/"+ datestr(now,'mm_dd_yy_HH_MM'),"distance_meas","longitudinal_speed_ego","camera_meas");
end
% initialize struct for GT data
% may add Time filed to have timestamped data?
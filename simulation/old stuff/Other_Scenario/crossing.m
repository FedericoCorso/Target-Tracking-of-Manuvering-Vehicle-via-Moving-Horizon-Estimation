clear 
close all

scenario = drivingScenario;
% road centers and width
% 1st vertical road
roadCentersV1X = linspace(-300,25,14)';
roadCentersV1Y = zeros(1,length(roadCentersV1X))';
roadCentersV1 = [roadCentersV1X roadCentersV1Y];

% 1st horizontal street
roadCentersH1Y = linspace(-125,25,7)';
roadCentersH1X = zeros(1,length(roadCentersH1Y))';
roadCentersH1 = [roadCentersH1X roadCentersH1Y];

% 2nd veritical road
roadCentersV2X = linspace(-200,25,10)';
roadCentersV2Y = -100*ones(1,length(roadCentersV1X))';
roadCentersV2 = [-25 -100; 25 -100; 50 -100; 75 -100; 200 -100];

% roadwidth = 7.2; Not needed as each lane has by default a 3.6m width

road(scenario,roadCentersV1,'lanes',lanespec([2 2]));
% plot(scenario)

road(scenario,roadCentersH1,'lanes',lanespec([2 2]));
% plot(scenario)

road(scenario,roadCentersV2,'lanes',lanespec([2 2]));
% plot(scenario)

% change lane command
switch_lane = 3.6; % add to the coordinate to change lane. 
jerk = 1.2; % m/s^3

% EGOCAR MOTION
egoCar = vehicle(scenario, 'ClassID', 1);
waypointsX = [-290 -17 -5.4 -5.4 -5.4 10 65 100]';
waypointsY = [-5.4 -5.4 -10 -45 -85 -105.4 -105.4 -105.4]';
waypoints = [waypointsX waypointsY];
speed = [10 0.75 2 5 1 5 12 13];
waittime = [0 0 0 0 0 0 0 0];
yaw = [0 0 -90 -90 -90 0 0 0];
smoothTrajectory(egoCar,waypoints,speed,waittime,'Yaw',yaw,'Jerk',jerk)

%trajectory(turningCar,waypoints,speed,waittime,'Yaw',yaw)
% plot(scenario,'Waypoints','on')
% TARGET MOTION
turningCar = vehicle(scenario,'ClassID',1);
waypointsX = [-250 -10 -1.8 -1.8 -1.8 15 75 150]';
waypointsY = [-1.8 -5.4 -15 -50 -90 -101.8 -105.4 -105.4]';
waypoints = [waypointsX waypointsY];
speed = [14 0 2 6 0 7 13 14];
waittime = [0 6.2 0 0 5 0 0 0];
yaw = [0 0 -90 -90 -90 0 0 0];
smoothTrajectory(turningCar,waypoints,speed,waittime,'Yaw',yaw,'Jerk',jerk)



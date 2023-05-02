% This is a helper function and may be changed or removed without notice.

%   Copyright 2016-2020 The MathWorks, Inc.

% Runs a drivingScenario simulation, update displays, and collects metrics
% from the drivng scenario using helperCollectScenarioMetrics
%
% scenario: A driving scenario object
% egoCar:   Actor in the driving scenario that is used as the ego vehicle
% sensor:   Sensor object used by egoCar to generate detections
% snapTime: Optional. Time in driving scenario when a snapshot should be
%           taken for publishing. When not publishing, this input is
%           ignored. When not specified, no snapshot is taken.
% sideView: Optional. Flag used to indicate that the upper left plot in the
%           driving scenario display should be a side view. When true, a
%           side view of the scenario is displayed, otherwise a top view
%           chasePlot following egoCar is used.
function [metrics, cameraMeasurements]= helperRunSensorDemoScenario(scenario, egoCar,precCar, sensor,sensor_prec, snapTime, sideView)

% Always show the display when not publishing, but when publishing, only
% show the display when it is requested
ispublishing = ~isempty(snapnow('get'));
doDisplay = nargin>5 || ~ispublishing;

if nargin<6
    snapTime = inf;
end

if nargin<7
    sideView = false;
end

% Create driving scenario display
if doDisplay
    [bep, figScene] = helperCreateSensorDemoDisplay(scenario, egoCar, sensor, sideView);
    lmPlotter = findPlotter(bep, 'DisplayName', 'lane markings');
    lbPlotter = findPlotter(bep, 'DisplayName', 'lane detections');
end


metrics = struct;
cameraMeasurements = struct('Time',[],'LateralOffset',[],'HeadingAngle',[],'Curvature',[]);
restart(scenario);
while advance(scenario)
    gTruth = targetPoses(egoCar); % Get target positions in ego vehicle coordinates
    gTruth_prec = targetPoses(precCar);
    % Generate time-stamped sensor detections
    time = scenario.SimulationTime;
    
    if isa(sensor,'visionDetectionGenerator') && strcmpi(sensor.DetectorOutput,'lanes and objects')
      if doDisplay
         [lmv,lmf] = laneMarkingVertices(egoCar);
         plotLaneMarking(lmPlotter, lmv, lmf);
      end
      lookaheadDistance = 0:.5:80; % lane bounaries are detected up to 80m from actor
      % lane boundaries in 'lb' are stored from left to right
      lb = laneBoundaries(egoCar,'XDistance',lookaheadDistance,'AllBoundaries',true); % ground truth for lane boundaries
      lb_prec = laneBoundaries(precCar,'XDistance',lookaheadDistance,'AllBoundaries',true);
      
      [~,~,~,lbdets_prec,isValidLaneTime_prec] = sensor_prec(gTruth_prec,lb_prec,time);
      [dets, ~, isValidTime, lbdets, isValidLaneTime] = sensor(gTruth, lb, time);
      
      if doDisplay && isValidLaneTime
          plotLaneBoundary(lbPlotter,vertcat(lbdets.LaneBoundaries));
      end
    else
      [dets, ~, isValidTime] = sensor(gTruth, time);
    end
    
    
    if isValidTime && isValidLaneTime_prec % consider always moments in which we have acces to both meas
        if doDisplay
            helperUpdateSensorDemoDisplay(bep, egoCar, sensor, dets);
        end
        
        % Collect detection metrics for further analysis
        metrics = helperCollectScenarioMetrics(metrics, gTruth, dets);
        cameraMeasurements.Time = [cameraMeasurements.Time; time];
        
        % find lane of vehicle 
        % lanes are numbered as: | 1 | 2 |
        %                        ^   ^   ^
        % thus lines are:        1   2   3
        current_lane = 2; 
        if (abs(lbdets_prec.LaneBoundaries(1,1).LateralOffset) < abs(lbdets_prec.LaneBoundaries(1,3).LateralOffset))
            current_lane = 1;
        end
        cameraMeasurements.Curvature = [cameraMeasurements.Curvature; (lbdets_prec.LaneBoundaries(1,current_lane).Curvature+lbdets_prec.LaneBoundaries(1,current_lane+1).Curvature)/2];
        cameraMeasurements.HeadingAngle = [cameraMeasurements.HeadingAngle;-(lbdets_prec.LaneBoundaries(1,current_lane).HeadingAngle+lbdets_prec.LaneBoundaries(1,current_lane+1).HeadingAngle)/2];
        
        if current_lane == 1
            cameraMeasurements.LateralOffset = [cameraMeasurements.LateralOffset;...
                ((lbdets_prec.LaneBoundaries(1,current_lane).LateralOffset+lbdets_prec.LaneBoundaries(1,current_lane+1).LateralOffset)/2)...
                + lbdets.LaneBoundaries(1,current_lane).LateralOffset];
        else
            cameraMeasurements.LateralOffset = [cameraMeasurements.LateralOffset;(lbdets_prec.LaneBoundaries(1,current_lane).LateralOffset+lbdets_prec.LaneBoundaries(1,current_lane+1).LateralOffset)/2];
        end
    end
    
    % Take a snapshot for the published example
    if doDisplay
        helperPublishSnapshot(figScene, time>=snapTime);
        if time>=snapTime && ispublishing && isa(sensor,'visionDetectionGenerator') && strcmpi(sensor.DetectorOutput,'lanes and objects')
           doDisplay = false;
        end
    end
end

end

%% CREATE RADAR SYNTHETIC DATA

radarSensor = drivingRadarDataGenerator( ...
    'SensorIndex', 1, ...
    'TargetReportFormat', 'Detections', ...
    'UpdateRate', 10, ...
    'MountingLocation', [egoCar.Wheelbase+egoCar.FrontOverhang 0 0.2], ...
    'FieldOfView', [20 5], ...
    'RangeLimits', [0 150], ...
    'AzimuthResolution', 4, ...
    'RangeResolution', 2.5, ...
    'Profiles', actorProfiles(scenario))

[bep, figScene] = helperCreateSensorDemoDisplay(scenario, egoCar, radarSensor);

% radar simulation
% radar used to get distance from the ego vehicle: sqrt(x^2+y^2)
metrics_radar = struct;                 % Initialize struct to collect scenario metrics
distance_meas = struct('Time',[],'realDistance',[],'measDistance',[]);
while advance(scenario)           % Update vehicle positions
    gTruth = targetPoses(egoCar); % Get target positions in ego vehicle coordinates
    
    % Generate time-stamped radar detections
    time = scenario.SimulationTime;
    [dets, ~, isValidTime] = radarSensor(gTruth, time);
    
    if isValidTime
        % Update Bird's-Eye Plot with detections and road boundaries
        helperUpdateSensorDemoDisplay(bep, egoCar, radarSensor, dets);
        
        % Collect radar detections and ground truth for offline analysis
        metrics_radar = helperCollectScenarioMetrics(metrics_radar, gTruth, dets);
        
        % create distance measurements
        distance_meas.Time = [distance_meas.Time;time];
        
        % compute mean distance
        dist_sum = 0;
        for ii = 1:numel(dets)
            dist_sum = dist_sum + sqrt(metrics_radar.Measurement(ii,1)^2+metrics_radar.Measurement(ii,2)^2);
        end
        
        distance_meas.realDistance = [distance_meas.realDistance; sqrt(gTruth.Position(1,1)^2+gTruth.Position(1,2)^2)];
        distance_meas.measDistance = [distance_meas.measDistance; dist_sum/numel(dets)];
    end
    
    % Take a snapshot for the published example
    % helperPublishSnapshot(figScene, time>=9.1);
    % pause(0.001);
end
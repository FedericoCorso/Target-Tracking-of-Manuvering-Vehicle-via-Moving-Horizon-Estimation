%% CREATE CAMERA DATA

visionSensor = visionDetectionGenerator(...
    'SensorIndex', 1, ...
    'UpdateInterval', 0.1, ...
    'SensorLocation', [0.75*egoCar.Wheelbase 0], ...
    'Height', 1.1, ...
    'Pitch', 1, ...
    'Intrinsics', cameraIntrinsics(800, [320 240], [480 640]), ...
    'BoundingBoxAccuracy', 5, ...
    'MaxRange', 150, ...
    'ActorProfiles', actorProfiles(scenario), ...
    'DetectorOutput','Lanes and objects')

visionSensor_prec = visionDetectionGenerator(...
    'SensorIndex', 2, ...
    'UpdateInterval', 0.1, ...
    'SensorLocation', [0.75*precCar.Wheelbase 0], ...
    'Height', 1.1, ...
    'Pitch', 1, ...
    'Intrinsics', cameraIntrinsics(800, [320 240], [480 640]), ...
    'BoundingBoxAccuracy', 5, ...
    'MaxRange', 150, ...
    'ActorProfiles', actorProfiles(scenario), ...
    'DetectorOutput','Lanes and objects')

[bep, figScene] = helperCreateSensorDemoDisplay(scenario, egoCar, visionSensor);

[metrics_vision, camera_meas] = helperRunSensorDemoScenario(scenario, egoCar,precCar, visionSensor,visionSensor_prec);
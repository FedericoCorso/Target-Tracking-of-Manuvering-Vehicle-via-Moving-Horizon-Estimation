% this script save simulation data from both camera and radar
% Usage: Run AFTER executing the simulation only if you changed some
% parameters of the simulation itself.
save("cameraData.mat","camera_data");
save("radarData.mat","radar_data");
save("GT.mat","ground_truth");

% save in SimulationData folder
save("../SimulationData/cameraData.mat","camera_data");
save("../SimulationData/radarData.mat","radar_data");
save("../SimulationData/GT.mat","ground_truth");
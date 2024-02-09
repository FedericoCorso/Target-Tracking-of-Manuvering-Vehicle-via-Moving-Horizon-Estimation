% this script save simulation data from both camera and radar
% Usage: Run AFTER executing the simulation only if you changed some
% parameters of the simulation itself.
save("cameraData_0_01.mat","camera_data");
save("radarData_0_01.mat","radar_data");
save("GT_0_01.mat","ground_truth");

% save in SimulationData folder
save("../SimulationData/cameraData_0_01.mat","camera_data");
save("../SimulationData/radarData_0_01.mat","radar_data");
save("../SimulationData/GT_0_01.mat","ground_truth");
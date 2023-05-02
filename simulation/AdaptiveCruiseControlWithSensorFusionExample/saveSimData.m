% this script save simulation data from both camera and radar
% run this if you change something in the simulation and you want to save
% data in this folder and in the SimulationData folder
save("cameraData.mat","camera_data");
save("radarData.mat","radar_data");
save("GT.mat","ground_truth");

% save in SimulationData folder
save("../SimulationData/cameraData.mat","camera_data");
save("../SimulationData/radarData.mat","radar_data");
save("../SimulationData/GT.mat","ground_truth");
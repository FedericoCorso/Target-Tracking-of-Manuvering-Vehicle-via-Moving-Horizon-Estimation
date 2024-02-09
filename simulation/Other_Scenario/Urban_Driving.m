% set sim time to 110 s - how to set time for simulation ? 
clear 
close all
clc

%% BEFORE RUNNING CHANGE ALSO IN THE CODE BELOW 'trial' and 'road_type' and set them equal to the ones here.

% 1 - general motion - in Urban
% 2 - uniform, constant speeds
% 3 - uniform, ego const speed, target uniform, pos acc
% 4 - uniform, both uniform acc 
% 5 - turn, const speed - for HW
% 6 - turn, ego const speed, target unif acc - for HW
% '7' - turn, both const acc - for HW
trial = '5';

% 'Urban' - urban scenario: for uniform and general motion patterns
% 'HW' - Highway scenario: for turning dynamics
road_type = "HW";

const_ang_acc = 0; % set to 0 for trajectory with constant yaw rate
wantTargetState = 1;

scenario = drivingScenario;

%% Road - definitio
switch road_type
    case {'Urban'}
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

    case {'HW'}
        % to test with linear angular acceleration
        if const_ang_acc % 6 - 7
            roadCenters = [0 150; 0 40; 0 30; 15 10; 32 -12; 32 -30];
            roadHeadings = [-90; -90; -90; -45; -90; -90];
            roadWidth = 7.2;    % Road width is two lanes, 3.6 meters each
            road(scenario,roadCenters,'Heading',roadHeadings,'Lanes',lanespec(2));
            % plot(scenario);
        else % 5
            % Define a road with a constant radius of curvature, R
            radius = 760;
            roadTurn = pi/3;    % Create a road that turns 60 degrees
            roadWidth = 7.2;    % Road width is two lanes, 3.6 meters each
            numSamples = 200;   % Define the road with sample points
            samples = linspace(0,roadTurn,numSamples)';
            roadCenters = radius*[sin(samples),-cos(samples),zeros(numSamples,1)];
            mainRoad = road(scenario, roadCenters, 'Lanes',lanespec(2));
        end
end

%% Trajectories - Collecting sensor data & ground truth needed for estimation

%% EGOCAR MOTION
egoCar = vehicle(scenario, 'ClassID', 1);

%% TARGET MOTION
targetCar = vehicle(scenario,'ClassID',1);

switch trial
    case {'1'}
        %% 1 - General Motion - ego
        ego_waypointsX = [-290 -17 -5.4 -5.4 -5.4 10 65 100]';
        ego_waypointsY = [-5.4 -5.4 -10 -45 -85 -105.4 -105.4 -105.4]';
        ego_waypoints = [ego_waypointsX ego_waypointsY];
        ego_speed = [10 0.75 2 5 1 5 12 13];
        ego_waittime = [0 0 0 0 0 0 0 0];
        % ego_yaw = [0 0 -90 -90 -90 0 0 0];
        ego_yaw = [NaN NaN NaN NaN NaN NaN NaN NaN]

        %% 1 - General Motion - target
        target_waypointsX = [-250 -10 -1.8 -1.8 -1.8 15 75 150]';
        target_waypointsY = [-1.8 -5.4 -15 -50 -90 -101.8 -105.4 -105.4]';
        target_waypoints = [target_waypointsX target_waypointsY];
        target_speed = [14 0 2 6 0 7 13 14];
        target_waittime = [0 6.2 0 0 5 0 0 0];
        % target_yaw = [0 0 -90 -90 -90 0 0 0];
        target_yaw = [NaN NaN NaN NaN NaN NaN NaN NaN];

        % sim_time = [0,110]; %sim_time interval
    
    case {'2'}
        %% 2 - Uniform Constant speed - ego
        ego_waypointsX = [-290 -50]';
        ego_waypointsY = [-5.4 -5.4]';
        ego_waypoints = [ego_waypointsX ego_waypointsY];
        ego_speed = [10 10];
        ego_waittime = [0 0];
        ego_yaw = [0 0];

        %% 2 - Uniform Constant speed - target
        target_waypointsX = [-250 -10]';
        target_waypointsY = [-5.4 -5.4]';
        target_waypoints = [target_waypointsX target_waypointsY];
        target_speed = [12 12];
        target_waittime = [0 0];
        target_yaw = [0 0];

        % sim_time = []; %sim_time interval
    
    case {'3'}
        %% 3 - Uniform Constant speed - ego
        ego_waypointsX = [-290 -50]';
        ego_waypointsY = [-5.4 -5.4]';
        ego_waypoints = [ego_waypointsX ego_waypointsY];
        ego_speed = [10 10];
        ego_waittime = [0 0];
        ego_yaw = [0 0];

        %% 3 - Uniform Constant Acceleration - target
        target_waypointsX = [-250 -10]';
        target_waypointsY = [-5.4 -5.4]';
        target_waypoints = [target_waypointsX target_waypointsY];
        target_speed = [12 20];
        target_waittime = [0 0];
        target_yaw = [0 0];

        % sim_time = []; %sim_time interval

    case {'4'}
        %% 4 - Uniform Constant Acceleration - ego
        ego_waypointsX = [-290 -50]';
        ego_waypointsY = [-5.4 -5.4]';
        ego_waypoints = [ego_waypointsX ego_waypointsY];
        ego_speed = [10 20];
        ego_waittime = [0 0];
        ego_yaw = [0 0];

        %% 4 - Uniform Constant Acceleration - target
        target_waypointsX = [-250 -10]';
        target_waypointsY = [-5.4 -5.4]';
        target_waypoints = [target_waypointsX target_waypointsY];
        target_speed = [12 20];
        target_waittime = [0 0];
        target_yaw = [0 0];

        % sim_time = []; %sim_time interval

    case {'5'}
        %% 5 - Uniform Constant turn - ego 
        ego_waypoints = ((radius+roadWidth/4)/radius)*roadCenters(:,1:2);
        ego_speed = 25;

        %% 5 - Uniform Constant turn - target
        
        target_waypoints = ego_waypoints(9:end,:);
        target_speed = 30;
        % sim_time = []; %sim_time interval
    
    case {'6'}
        %% 6 - Uniform Constant turn - ego 
        ego_waypoints = ((radius+roadWidth/4)/radius)*roadCenters(:,1:2);
        ego_speed = 14;

        %% 6 - Uniform Constant Acceleration turn - target
        target_waypoints = [115.53142278746 -753.167418322111 0;
    119.482672655111 -752.549059645798 0;
    123.430613840908 -751.909916967936 0;
    127.375137019533 -751.250007987491 0;
    131.316132960316 -750.569350978484 0;
    135.253492530264 -749.867964789485 0;
    139.18710669708 -749.14586884309 0;
    143.116866532186 -748.403083135385 0;
    147.042663213734 -747.639628235391 0;
    150.964388029625 -746.855525284495 0;
    154.881932380516 -746.050795995863 0;
    158.795187782828 -745.225462653841 0;
    162.704045871751 -744.379548113337 0;
    166.608398404245 -743.513075799188 0;
    170.508137262035 -742.62606970551 0;
    174.403154454608 -741.718554395036 0;
    178.2933421222 -740.790554998435 0;
    182.178592538788 -739.842097213615 0;
    186.058798115067 -738.873207305013 0;
    189.933851401434 -737.883912102865 0;
    193.803645090963 -736.874239002468 0;
    197.668072022371 -735.844215963415 0;
    201.527025182996 -734.793871508825 0;
    205.380397711748 -733.723234724554 0;
    209.228082902079 -732.632335258385 0;
    213.069974204931 -731.521203319211 0;
    216.905965231689 -730.389869676198 0;
    220.735949757127 -729.238365657931 0;
    224.559821722349 -728.066723151548 0;
    228.377475237728 -726.874974601856 0;
    232.188804585835 -725.663153010433 0;
    235.993704224368 -724.431291934715 0;
    239.792068789076 -723.179425487066 0;
    243.583793096675 -721.907588333833 0;
    247.368772147761 -720.615815694387 0;
    251.146901129717 -719.304143340146 0;
    254.918075419618 -717.972607593585 0;
    258.682190587126 -716.621245327233 0;
    262.439142397381 -715.250093962649 0;
    266.188826813889 -713.859191469384 0;
    269.931140001406 -712.448576363935 0;
    273.665978328804 -710.807761392883 0;
    277.393238371953 -709.147312479186 0;
    281.112816916576 -707.467269773707 0;
    284.824610961108 -705.767673969896 0;
    288.528517719553 -704.048566302664 0;
    292.224434624328 -702.309988547237 0;
    295.912259329101 -700.551983018002 0;
    299.591889711628 -698.774592567335 0;
    303.263223876579 -696.977860584414 0;
    306.926160158362 -695.161830994017 0;
    310.580597123936 -693.326548255307 0;
    314.226433575622 -691.472057360598 0;
    317.863568553901 -689.598403834113 0;
    321.491901340217 -687.705633730722 0;
    325.111331459759 -685.793793634664 0;
    328.721758684247 -683.86293065826 0;
    332.323083034708 -681.913092440608 0;
    335.91520478424 -679.944327146262 0;
    339.498024460781 -677.956683463901 0;
    343.071442849857 -676.160736920767 0;
    346.635360997332 -674.346010933937 0;
    350.18968021215 -672.512555756312 0;
    353.734302069063 -670.660422159438 0;
    357.269128411362 -668.789661432096 0;
    360.794061353593 -666.900325378886 0;
    364.309003284265 -664.992466318792 0;
    367.813856868557 -663.066137083731 0;
    371.308525051013 -661.121391017091 0;
    374.792911058225 -659.158281972253 0;
    378.266918401519 -657.176864311103 0;
    381.730450879624 -655.387719218312 0;
    385.183412581334 -653.58037575245 0;
    388.625707888168 -651.754889791822 0;
    392.057241477016 -649.911317717128 0;
    395.477918322778 -648.049716409906 0;
    398.887643700996 -646.170143250953 0;
    402.286323190476 -644.272656118739 0;
    405.673862675907 -642.357313387801 0;
    409.050168350459 -640.424173927132 0;
    412.415146718391 -638.473297098542 0;
    415.768704597631 -636.504742755025 0;
    419.110749122362 -634.518571239091 0;
    422.44118774559 -632.514843381103 0;
    425.75992824171 -630.493620497589 0;
    429.066878709058 -628.454964389544 0;
    432.361947572456 -626.398937340719 0;
    435.645043585748 -624.325602115897 0;
    438.916075834327 -622.235021959154 0;
    442.174953737654 -620.127260592108 0;
    445.421587051763 -617.791855896364 0;
    448.655885871761 -615.439398859109 0;
    451.877760634322 -613.069954623945 0;
    455.087122120161 -610.683588804881 0;
    458.283881456507 -608.280367484515 0;
    461.467950119566 -605.8603572122 0;
    464.639239936969 -603.423625002208 0;
    467.797663090216 -600.970238331866 0;
    470.943132117109 -598.500265139696 0;
    474.075559914168 -596.013773823527 0;
    477.194859739052 -593.510833238603 0;
    480.300945212953 -590.991512695678 0;
    483.393730322993 -588.455881959097 0;
    486.473129424601 -585.904011244858 0;
    489.539057243893 -583.335971218675 0;
    492.591428880023 -580.751832994019 0;
    495.630159807542 -578.151668130147 0;
    498.655165878735 -575.535548630119 0;
    501.666363325952 -572.90354693881 0;
    504.663668763928 -570.255735940899 0;
    507.646999192091 -567.592188958851 0;
    510.616271996862 -564.91297975089 0;
    513.571404953943 -562.218182508952 0;
    516.512316230589 -559.507871856633 0;
    519.438924387882 -556.782122847123 0;
    522.35114838298 -554.041010961126 0;
    525.248907571364 -551.284612104771 0;
    528.13212170907 -548.51300260751 0;
    531.000710954913 -545.726259220002 0;
    533.854595872694 -542.924459111993 0;
    536.693697433405 -540.107679870173 0;
    539.517937017413 -537.275999496031 0;
    542.327236416639 -534.429496403694 0;
    545.121517836725 -531.568249417755 0;
    547.900703899184 -528.692337771092 0;
    550.664717643549 -525.801841102673 0;
    553.413482529498 -522.89683945535 0;
    556.146922438978 -519.977413273642 0;
    558.86496167831 -517.043643401509 0;
    561.567524980286 -514.095611080114 0;
    564.254537506255 -511.13339794557 0;
    566.925924848192 -508.157086026682 0;
    569.581613030762 -505.166757742674 0;
    572.221528513367 -502.162495900909 0;
    574.845598192179 -499.144383694592 0;
    577.453749402172 -496.112504700472 0;
    580.045909919128 -493.066942876519 0;
    582.622007961639 -490.00778255961 0;
    585.181972193094 -486.935108463184 0;
    587.725731723658 -483.849005674901 0;
    590.253216112228 -480.749559654285 0;
    592.76435536839 -477.636856230356 0;
    595.259079954357 -474.510981599256 0;
    597.737320786888 -471.37202232186 0;
    600.19900923921 -468.220065321379 0;
    602.644077142912 -465.055197880953 0;
    605.072456789834 -461.877507641236 0;
    607.484080933944 -458.687082597966 0;
    609.878882793199 -455.484011099531 0;
    612.256796051394 -452.26838184452 0;
    614.617754859997 -449.040283879269 0;
    616.961693839976 -445.799806595393 0;
    619.288548083606 -442.547039727313 0;
    621.59825315627 -439.28207334977 0;
    623.890745098239 -436.004997875328 0;
    626.165960426447 -432.715904051876 0;
    628.423836136246 -429.414882960111 0;
    630.664309703152 -426.102026011015 0;
    632.887319084577 -422.777424943326 0;
    635.092802721548 -419.441171821 0;
    637.280699540407 -416.093359030654 0;
    639.450948954508 -412.734079279014 0;
    641.603490865891 -409.363425590348 0;
    643.738265666947 -405.981491303885 0;
    645.855214242071 -402.588370071235 0;
    647.954277969294 -399.184155853793 0;
    650.035398721913 -395.768942920139 0;
    652.098518870093 -392.342825843425 0;
    654.143581282471 -388.905899498759 0;
    656.17052932773 -385.458259060575 0;
    658.179306876173 -382 0];
        
        target_speed = 16.6;
        % sim_time = []; %sim_time interval
    
    case {'7'}
        %% 5 - Uniform Constant turn - ego 
        ego_waypoints = ((radius+roadWidth/4)/radius)*roadCenters(:,1:2);
        

        %% 5 - Uniform Constant turn - target
        
        target_waypoints = ego_waypoints(9:end,:);

        % sim_time = []; %sim_time interval
end

%% Interpolation

switch road_type
    case 'Urban'
        smoothTrajectory(egoCar,ego_waypoints,ego_speed,ego_waittime,'Yaw',ego_yaw,'Jerk',jerk)
        % trajectory(egoCar,waypoints,speed,waittime,'Yaw',yaw)
        
        smoothTrajectory(targetCar,target_waypoints,target_speed,target_waittime,'Yaw',target_yaw,'Jerk',jerk)
        % trajectory(turningCar,waypoints,speed,waittime,'Yaw',yaw)l
    case 'HW'
        smoothTrajectory(egoCar, ego_waypoints,ego_speed); % On right lane
        smoothTrajectory(targetCar,target_waypoints,target_speed); 
end

%% Plot the scenario
plot(scenario,'Waypoints','on')

%% Tracking Simulation
out = sim("Simulation.slx"); % include sim_time

% save relative data
camera_data = out.camera_data;
radar_data = out.radar_data;
ground_truth = out.ground_truth;
ego_state = out.ego_state;

switch road_type
    case 'Urban'
        save("cameraData.mat","camera_data");
        save("radarData.mat","radar_data");
        save("GT.mat","ground_truth");
        save("egoState.mat","ego_state");
        
        % save in SimulationData folder
        save("../SimulationData/UrbanCameraData.mat","camera_data");
        save("../SimulationData/UrbanRadarData.mat","radar_data");
        save("../SimulationData/UrbanGT.mat","ground_truth");
        save("../SimulationData/UrbanEgoState.mat","ego_state");
    case 'HW'
        save("HW_cameraData.mat","camera_data");
        save("HW_radarData.mat","radar_data");
        save("HW_GT.mat","ground_truth");
        save("HW_egoState.mat","ego_state");
        
        % save in SimulationData folder
        save("../SimulationData/HW_CameraData.mat","camera_data");
        save("../SimulationData/HW_RadarData.mat","radar_data");
        save("../SimulationData/HW_GT.mat","ground_truth");
        save("../SimulationData/HW_EgoState.mat","ego_state");
end

%% Target State Retrieval Simulation
if wantTargetState 
    
    clear
    close all
    scenario = drivingScenario;

    % 1 - general motion - in Urban
    % 2 - uniform, constant speeds
    % 3 - uniform, ego const speed, target uniform, pos acc
    % 4 - uniform, both uniform acc 
    % 5 - turn, const speed - for HW
    % 6 - turn, ego const speed, target unif acc - for HW
    % '7' - turn, both const acc - for HW
    trial = '5';
    
    % 'Urban' - urban scenario: for uniform and general motion patterns
    % 'HW' - Highway scenario: for turning dynamics
    road_type = "HW";

    const_ang_acc = 0;

   %% Road - definitio
switch road_type
    case {'Urban'}
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

    case {'HW'}
        % to test with linear angular acceleration
        if const_ang_acc % 6 - 7
            roadCenters = [0 150; 0 40; 0 30; 15 10; 32 -12; 32 -30];
            roadHeadings = [-90; -90; -90; -45; -90; -90];
            roadWidth = 7.2;    % Road width is two lanes, 3.6 meters each
            road(scenario,roadCenters,'Heading',roadHeadings,'Lanes',lanespec(2));
            % plot(scenario);
        else % 5
            % Define a road with a constant radius of curvature, R
            radius = 760;
            roadTurn = pi/3;    % Create a road that turns 60 degrees
            roadWidth = 7.2;    % Road width is two lanes, 3.6 meters each
            numSamples = 200;   % Define the road with sample points
            samples = linspace(0,roadTurn,numSamples)';
            roadCenters = radius*[sin(samples),-cos(samples),zeros(numSamples,1)];
            mainRoad = road(scenario, roadCenters, 'Lanes',lanespec(2));
        end

end

    egoCar = vehicle(scenario, 'ClassID', 1);

    switch trial
        case {'1'} 
            %% 1 - General Motion - target
            waypointsX = [-250 -10 -1.8 -1.8 -1.8 15 75 150]';
            waypointsY = [-1.8 -5.4 -15 -50 -90 -101.8 -105.4 -105.4]';
            waypoints = [waypointsX waypointsY];
            speed = [14 0 2 6 0 7 13 14];
            waittime = [0 6.2 0 0 5 0 0 0];
            target_yaw = [NaN NaN NaN NaN NaN NaN NaN NaN];
        
        case {'2'}
            %% 2 - Uniform Constant speed - target
            waypointsX = [-250 -10]';
            waypointsY = [-5.4 -5.4]';
            waypoints = [waypointsX waypointsY];
            speed = [12 12];
            waittime = [0 0];
            yaw = [0 0];
        
        case {'3'}
            %% 3 - Uniform Constant speed - target
            waypointsX = [-250 -10]';
            waypointsY = [-5.4 -5.4]';
            waypoints = [waypointsX waypointsY];
            speed = [12 20];
            waittime = [0 0];
            yaw = [0 0];
    
        case {'4'}
            %% 4 - Uniform Constant speed - target
            waypointsX = [-250 -10]';
            waypointsY = [-5.4 -5.4]';
            waypoints = [waypointsX waypointsY];
            speed = [12 20];
            waittime = [0 0];
            yaw = [0 0];

        case {'5'}
            %% 5 - Uniform Constant turn - target
            ego_waypoints = ((radius+roadWidth/4)/radius)*roadCenters(:,1:2);
            target_waypoints = ego_waypoints(9:end,:);
            target_speed = 30;
        
        case {'6'}
            %% 6 - Uniform Constant Acceleration turn - target
            target_waypoints = [115.53142278746 -753.167418322111 0;
    119.482672655111 -752.549059645798 0;
    123.430613840908 -751.909916967936 0;
    127.375137019533 -751.250007987491 0;
    131.316132960316 -750.569350978484 0;
    135.253492530264 -749.867964789485 0;
    139.18710669708 -749.14586884309 0;
    143.116866532186 -748.403083135385 0;
    147.042663213734 -747.639628235391 0;
    150.964388029625 -746.855525284495 0;
    154.881932380516 -746.050795995863 0;
    158.795187782828 -745.225462653841 0;
    162.704045871751 -744.379548113337 0;
    166.608398404245 -743.513075799188 0;
    170.508137262035 -742.62606970551 0;
    174.403154454608 -741.718554395036 0;
    178.2933421222 -740.790554998435 0;
    182.178592538788 -739.842097213615 0;
    186.058798115067 -738.873207305013 0;
    189.933851401434 -737.883912102865 0;
    193.803645090963 -736.874239002468 0;
    197.668072022371 -735.844215963415 0;
    201.527025182996 -734.793871508825 0;
    205.380397711748 -733.723234724554 0;
    209.228082902079 -732.632335258385 0;
    213.069974204931 -731.521203319211 0;
    216.905965231689 -730.389869676198 0;
    220.735949757127 -729.238365657931 0;
    224.559821722349 -728.066723151548 0;
    228.377475237728 -726.874974601856 0;
    232.188804585835 -725.663153010433 0;
    235.993704224368 -724.431291934715 0;
    239.792068789076 -723.179425487066 0;
    243.583793096675 -721.907588333833 0;
    247.368772147761 -720.615815694387 0;
    251.146901129717 -719.304143340146 0;
    254.918075419618 -717.972607593585 0;
    258.682190587126 -716.621245327233 0;
    262.439142397381 -715.250093962649 0;
    266.188826813889 -713.859191469384 0;
    269.931140001406 -712.448576363935 0;
    273.665978328804 -710.807761392883 0;
    277.393238371953 -709.147312479186 0;
    281.112816916576 -707.467269773707 0;
    284.824610961108 -705.767673969896 0;
    288.528517719553 -704.048566302664 0;
    292.224434624328 -702.309988547237 0;
    295.912259329101 -700.551983018002 0;
    299.591889711628 -698.774592567335 0;
    303.263223876579 -696.977860584414 0;
    306.926160158362 -695.161830994017 0;
    310.580597123936 -693.326548255307 0;
    314.226433575622 -691.472057360598 0;
    317.863568553901 -689.598403834113 0;
    321.491901340217 -687.705633730722 0;
    325.111331459759 -685.793793634664 0;
    328.721758684247 -683.86293065826 0;
    332.323083034708 -681.913092440608 0;
    335.91520478424 -679.944327146262 0;
    339.498024460781 -677.956683463901 0;
    343.071442849857 -676.160736920767 0;
    346.635360997332 -674.346010933937 0;
    350.18968021215 -672.512555756312 0;
    353.734302069063 -670.660422159438 0;
    357.269128411362 -668.789661432096 0;
    360.794061353593 -666.900325378886 0;
    364.309003284265 -664.992466318792 0;
    367.813856868557 -663.066137083731 0;
    371.308525051013 -661.121391017091 0;
    374.792911058225 -659.158281972253 0;
    378.266918401519 -657.176864311103 0;
    381.730450879624 -655.387719218312 0;
    385.183412581334 -653.58037575245 0;
    388.625707888168 -651.754889791822 0;
    392.057241477016 -649.911317717128 0;
    395.477918322778 -648.049716409906 0;
    398.887643700996 -646.170143250953 0;
    402.286323190476 -644.272656118739 0;
    405.673862675907 -642.357313387801 0;
    409.050168350459 -640.424173927132 0;
    412.415146718391 -638.473297098542 0;
    415.768704597631 -636.504742755025 0;
    419.110749122362 -634.518571239091 0;
    422.44118774559 -632.514843381103 0;
    425.75992824171 -630.493620497589 0;
    429.066878709058 -628.454964389544 0;
    432.361947572456 -626.398937340719 0;
    435.645043585748 -624.325602115897 0;
    438.916075834327 -622.235021959154 0;
    442.174953737654 -620.127260592108 0;
    445.421587051763 -617.791855896364 0;
    448.655885871761 -615.439398859109 0;
    451.877760634322 -613.069954623945 0;
    455.087122120161 -610.683588804881 0;
    458.283881456507 -608.280367484515 0;
    461.467950119566 -605.8603572122 0;
    464.639239936969 -603.423625002208 0;
    467.797663090216 -600.970238331866 0;
    470.943132117109 -598.500265139696 0;
    474.075559914168 -596.013773823527 0;
    477.194859739052 -593.510833238603 0;
    480.300945212953 -590.991512695678 0;
    483.393730322993 -588.455881959097 0;
    486.473129424601 -585.904011244858 0;
    489.539057243893 -583.335971218675 0;
    492.591428880023 -580.751832994019 0;
    495.630159807542 -578.151668130147 0;
    498.655165878735 -575.535548630119 0;
    501.666363325952 -572.90354693881 0;
    504.663668763928 -570.255735940899 0;
    507.646999192091 -567.592188958851 0;
    510.616271996862 -564.91297975089 0;
    513.571404953943 -562.218182508952 0;
    516.512316230589 -559.507871856633 0;
    519.438924387882 -556.782122847123 0;
    522.35114838298 -554.041010961126 0;
    525.248907571364 -551.284612104771 0;
    528.13212170907 -548.51300260751 0;
    531.000710954913 -545.726259220002 0;
    533.854595872694 -542.924459111993 0;
    536.693697433405 -540.107679870173 0;
    539.517937017413 -537.275999496031 0;
    542.327236416639 -534.429496403694 0;
    545.121517836725 -531.568249417755 0;
    547.900703899184 -528.692337771092 0;
    550.664717643549 -525.801841102673 0;
    553.413482529498 -522.89683945535 0;
    556.146922438978 -519.977413273642 0;
    558.86496167831 -517.043643401509 0;
    561.567524980286 -514.095611080114 0;
    564.254537506255 -511.13339794557 0;
    566.925924848192 -508.157086026682 0;
    569.581613030762 -505.166757742674 0;
    572.221528513367 -502.162495900909 0;
    574.845598192179 -499.144383694592 0;
    577.453749402172 -496.112504700472 0;
    580.045909919128 -493.066942876519 0;
    582.622007961639 -490.00778255961 0;
    585.181972193094 -486.935108463184 0;
    587.725731723658 -483.849005674901 0;
    590.253216112228 -480.749559654285 0;
    592.76435536839 -477.636856230356 0;
    595.259079954357 -474.510981599256 0;
    597.737320786888 -471.37202232186 0;
    600.19900923921 -468.220065321379 0;
    602.644077142912 -465.055197880953 0;
    605.072456789834 -461.877507641236 0;
    607.484080933944 -458.687082597966 0;
    609.878882793199 -455.484011099531 0;
    612.256796051394 -452.26838184452 0;
    614.617754859997 -449.040283879269 0;
    616.961693839976 -445.799806595393 0;
    619.288548083606 -442.547039727313 0;
    621.59825315627 -439.28207334977 0;
    623.890745098239 -436.004997875328 0;
    626.165960426447 -432.715904051876 0;
    628.423836136246 -429.414882960111 0;
    630.664309703152 -426.102026011015 0;
    632.887319084577 -422.777424943326 0;
    635.092802721548 -419.441171821 0;
    637.280699540407 -416.093359030654 0;
    639.450948954508 -412.734079279014 0;
    641.603490865891 -409.363425590348 0;
    643.738265666947 -405.981491303885 0;
    645.855214242071 -402.588370071235 0;
    647.954277969294 -399.184155853793 0;
    650.035398721913 -395.768942920139 0;
    652.098518870093 -392.342825843425 0;
    654.143581282471 -388.905899498759 0;
    656.17052932773 -385.458259060575 0;
    658.179306876173 -382 0];
        
        target_speed = 16.6;
        
        case {'7'}
            %% 6 - Uniform Constant Acceleration turn - target
            target_waypoints = [0-roadWidth/4 80;0-roadWidth/4 40; 0-roadWidth/4 30; 15 10-roadWidth/4; 32-roadWidth/4 -12; 32-roadWidth/4 -30];
            target_speed = 12;
            % sim_time = []; %sim_time interval
    end
    
    switch road_type
        case 'Urban'
            smoothTrajectory(egoCar,waypoints,speed,waittime,'Yaw',yaw,'Jerk',jerk)
        case 'HW'
            smoothTrajectory(egoCar, target_waypoints, target_speed); % On right lane
    end
    
    out_TS = sim("Simulation.slx");
    
    switch road_type
        case 'Urban'
            target_state = out_TS.ego_state;
            save("targetState.mat","target_state");
            save("../SimulationData/UrbanTargetState.mat","target_state");
        case 'HW'
            target_state = out_TS.ego_state;
            save("HW_targetState.mat","target_state");
            save("../SimulationData/HW_TargetState.mat","target_state");
    end
end


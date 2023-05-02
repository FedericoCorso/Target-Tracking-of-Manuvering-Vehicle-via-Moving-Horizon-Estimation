% This is a helper function and may be changed or removed without notice.

%   Copyright 2016-2020 The MathWorks, Inc.

% Collects metrics from a driving scenario simulation for offline analysis.

% Assumes the sensor generating the detections is configured to report the
% detections in the ego vehicle's coordinate frame.
%
% Appends the cell array of objectDetections, dets, to the current set of
% detections collected for the driving scenario.

% Creates an 1-by-K struct array for each of the K targets detected. The
% struct array has the following fields:
%   TargetIndex:    A scalar defining the ID of the target whose
%                   measurements are collected on this index of the struct
%                   array
%   Time:   M-by-1 vector defining the time stamps for each detection in seconds
%   GroundTruth:    M-by-6 array with each column [x;y;z;vx;vy;vz] defining
%                   the ground truth position and velocity for the target
%                   from which the detection was generated

%   Measurement:    M-by-6 array with each column [x;y;z;vx;vy;vz] defining
%                   the measured position and velocity

%   MeasurementSigma:   M-by-6 array with each column [x;y;z;vx;vy;vz]
%                       defining the sqrt of the diagonal of the
%                       measurement noise reported for each detection

%   SNR:    Present when detections are generated from radar sensors. An
%           M-by-1 vector defining the signal-to-noise ratio of each
%           detection
%
% In the fieldname descriptions above, M denotes the number of measurements
% that have been collected for the k-th target.
function metrics = helperCollectScenarioMetrics(metrics,targets,dets)

if isempty(dets)
    return % exit function
end

% Create a structure to collect metrics
if ~isfield(metrics,'TargetIndex') % check if the 'metrics' object has field 'TargetIndex' if not, we create the struct
    metrics = createMetricsStruct(targets,dets);
end

hasSNR = isfield(metrics,'SNR');

for iTgt = numel(metrics):-1:1 % filling from last target to first
    thisID = metrics(iTgt).TargetIndex;
    
    % Find detections associated with this ID
    tgtIDs = cellfun(@(d)d.ObjectAttributes{1}.TargetIndex,dets);
    iDets = tgtIDs==metrics(iTgt).TargetIndex; % boolean
    
    if any(iDets) % if elements are non zero
        % Save target's detections
        % - Some sensors will generate multiple detections from a single target
        theseDets = dets(iDets);
        time = cell2mat(cellfun(@(d)d.Time,theseDets,'UniformOutput',false));
        meas = cell2mat(cellfun(@(d)d.Measurement',theseDets,'UniformOutput',false));
        measSigma = cell2mat(cellfun(@(d)sqrt(diag(d.MeasurementNoise))',theseDets,'UniformOutput',false));
        if hasSNR
            snr = cell2mat(cellfun(@(d)d.ObjectAttributes{1}.SNR,theseDets,'UniformOutput',false));
        end
        
        % Find target associated with this ID
        iFnd = [targets.ActorID]==thisID; 
        thisTgt = targets(iFnd); % logical access to array elements
    
        % Save target's ground truth
        groundTruth = [thisTgt.Position thisTgt.Velocity]; % selecting what to put in the ground truth
        groundTruth = repmat(groundTruth,[numel(theseDets) 1]); % for every detection we associate the corresponding gt
        
        metrics(iTgt).Time = [metrics(iTgt).Time;time];
        metrics(iTgt).GroundTruth = [metrics(iTgt).GroundTruth;groundTruth]; % append
        metrics(iTgt).Measurement = [metrics(iTgt).Measurement;meas];
        metrics(iTgt).MeasurementSigma = [metrics(iTgt).MeasurementSigma;measSigma];
        if hasSNR
            metrics(iTgt).SNR = [metrics(iTgt).SNR;snr];
        end
    end
end
end

function metrics = createMetricsStruct(targets,dets)
metrics = struct('TargetIndex',0,'Time',[],'GroundTruth',[],...
    'Measurement',[],'MeasurementSigma',[]); % value initialization

if isfield(dets{1}.ObjectAttributes{1},'SNR') % check if field of struct array is 'SNR'
    metrics.SNR = []; % initialize
end

numTgts = numel(targets);
metrics = repmat(metrics,numTgts,1); % create the array of structure metrics, one for each target
tgtIDs = {targets.ActorID};
[metrics.TargetIndex] = deal(tgtIDs{:}); % create the targetIndexes for each element
end
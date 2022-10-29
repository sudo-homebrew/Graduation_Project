classdef trackerPHD< matlabshared.tracking.internal.fusion.AbstractTracker & fusion.internal.ExportToSimulinkInterface
% trackerPHD Tracking objects using a multi-target PHD filter
%   tracker = trackerPHD('SensorConfigurations',configurations) creates a
%   multi-sensor, multi-object tracker which uses a multi-target
%   Probability Hypothesis Density (PHD) filter to estimate the states of
%   the targets. configurations is a cell array of trackingSensorConfiguration
%   objects. The trackerPHD maintains track estimate and identity by
%   assigning a unique label, TrackID, to each component of the PHD.
%   It uses an Iterated-Corrector approach for each sensor to update
%   the density sequentially.
%
%   A peak from the PHD is labeled as a track, when its weight exceeds a
%   certain threshold, <a href="matlab:help('trackerPHD/ExtractionThreshold')">ExtractionThreshold</a>. The track is given a 'Tentative'
%   status, meaning there is not enough evidence yet to determine if it is
%   of a physical object. When the weight increases beyond the
%   <a href="matlab:help('trackerPHD/ConfirmationThreshold')">ConfirmationThreshold</a>, it's status is changed to 'Confirmed'.
%
%   tracker = trackerPHD('Name',value) creates a trackerPHD by specifying
%   its properties as name-value pairs. See the list of properties below.
%
%   Step method syntax: click on <a href="matlab:help('trackerPHD/stepImpl')">step</a> for more details.
%   The step method of the tracker is responsible for managing all the
%   tracks. It performs the following steps:
%   1. It operates on each sensor sequentially.
%   2. It finds detections with low likelihood from each sensor and uses
%   them for initializing new components in the PHD.
%   3. If the sensor produces multiple detections per object, it generates
%   multiple partitions of the detections to incorporate the unknown origin
%   of the measurement.
%   4. It weighs each partition and the detection clusters belonging to it.
%   5. It assigns and maintains labels (TrackIDs) for each new initiated
%   track originating from the PHD.
%   6. It maintains the PHD within tractable limits by pruning and merging
%   it.
%
%   trackerPHD properties:
%     TrackerIndex                    - Unique identifier of the tracker
%     SensorConfigurations            - Configurations of sensors
%     PartitioningFcn                 - Function to partition detections
%                                       into clusters
%     BirthRate                       - Rate at which new targets are
%                                       born in the scenario per unit
%                                       time
%     DeathRate                       - Rate at which targets die in the
%                                       scenario per unit time
%     AssignmentThreshold             - The threshold that controls the
%                                       list of detection cells, which
%                                       are considered for generating new
%                                       components in the PHD
%     ExtractionThreshold             - The minimum weight of a component
%                                       in PHD to be called a tentative
%                                       track
%     ConfirmationThreshold           - The minimum weight of a tentative
%                                       track to be confirmed
%     DeletionThreshold               - The weight below which a component
%                                       from the PHD is deleted
%     MergingThreshold                - The threshold below which
%                                       components belonging to the same
%                                       target track are merged
%     LabelingThresholds              - Threshold for Label management
%     HasSensorConfigurationsInput    - Update sensor configurations via
%                                       input
%     StateParameters                 - Parameters defining the track state
%     NumTracks                       - Number of tracks. (read-only)
%     NumConfirmedTracks              - Number of confirmed tracks (read-only)
%     MaxNumSensors                   - Define the maximum number of sensors
%     MaxNumTracks                    - Define the maximum number of tracks
%
%   trackerPHD methods:
%     step                       - Initializes, deletes and manages tracks
%     predictTracksToTime        - Predicts the tracks to a time stamp
%     initializeTrack            - Initialize a new track
%     deleteTrack                - Delete an existing track
%     sensorIndices              - Return the list of sensor indices
%     exportToSimulink           - Export the tracker to a Simulink model
%     release                    - Allows property value and input characteristics changes
%     clone                      - Creates a copy of the trackerPHD
%     isLocked                   - Locked status (logical)
%     <a href="matlab:help matlab.System/reset   ">reset</a>                      - Resets states of the trackerPHD
%
%   % EXAMPLE: Construct a tracker and use it to track two objects.
%
%   % Construct a trackerPHD for tracking two objects using 1 sensor.
%   configuration = trackingSensorConfiguration(1);
%
%   % Set the IsValidTime flag of the sensor to true to let the tracker know
%   % that this sensor had the chance to report detections.
%   configuration.IsValidTime = true;
%
%   % Specify properties such as ClutterDensity for the sensor.
%   configuration.ClutterDensity = 1e-7;
%
%   tracker = trackerPHD('SensorConfigurations',configuration);
%   % Update the tracker with some detections from two objects
%   detections = cell(20,1);
%   for i = 1:10
%       detections{i} = objectDetection(0,[200;-30;0] + 0.1*randn(3,1));
%   end
%   for j = 11:20
%       detections{j} = objectDetection(0,[100;5;0] + 0.1*randn(3,1));
%   end
%   [confTracks,tentTracks,allTracks,analysisInfo] = tracker(detections,0);
%
%   % In the first-step, as no previous tracks are present all possible
%   % partitions of the detections are considered for birth.
%   analysisInfo.SensorAnalysisInfo{1}.IsBirthCell;
%
%   % Update the tracker again after 0.1 seconds by assuming that targets
%   % moved at a constant velocity
%   dT = 0.1;
%   for i = 1:20
%       detections{i}.Time = detections{i}.Time + dT;
%       detections{i}.Measurement = detections{i}.Measurement + [1;2;0]*dT;
%   end
%   [confTracks,tentTracks,allTracks] = tracker(detections,dT);
%
%   % In this example, we used the default sensor configuration
%   % FilterInitializationFcn, initcvggiwphd, which uses a constant velocity
%   % model and defines the states as [x;vx;y;vy;z;vy]. To find the
%   % positions, we use
%   positionSelector = [1 0 0 0 0 0;0 0 1 0 0 0;0 0 0 0 1 0];
%   positions = getTrackPositions(confTracks,positionSelector)
%
%   % You can also use the property Extent and MeasurementRate of the track
%   % to check the other states of a Gamma Gaussian Inverse Wishart (GGIW)
%   % state.
%   confTracks(1).MeasurementRate
%
%   % When detections are not available, you can use the function
%   % predictTracksToTime to predict the tracks to a certain time stamp.
%   confPredictedTracks = predictTracksToTime(tracker,'confirmed',0.2);
%
%   % EXAMPLE2: Construct a tracker and use it to track single moving 
%   % object. In this scenario you have a car that is moving at a constant
%   % speed which will be tracked with the help of a non scanning radar
%   % sensor fixed on the tower. In order to track the car you will feed
%   % the detections and sensor configurations generated from the radar 
%   % sensor model to trackerPHD as an input.
%
%   % Create Scenario
%   scenario = trackingScenario;
%   scenario.StopTime = Inf;
%   scenario.UpdateRate = 0;
% 
%   % Create platforms
%   Tower = platform(scenario,'ClassID',3);
%   Tower.Dimensions = struct( ...
%   'Length', 10, ...
%   'Width', 10, ...
%   'Height', 60, ...
%   'OriginOffset', [0 0 30]);
% 
%   Car = platform(scenario,'ClassID',2);
%   Car.Dimensions = struct( ...
%   'Length', 4.7, ...
%   'Width', 1.8, ...
%   'Height', 1.4, ...
%   'OriginOffset', [-0.6 0 0.7]);
%   Car.Trajectory = waypointTrajectory( ...
%   [0 -15 -0.23;0.3 -29.5 -0.23;0.3 -42 -0.39;0.3 -56.5 -0.23; ...
%   -0.3 -78.2 -0.23;4.4 -96.4 -0.23], [0;1.4;2.7;4.1;6.3;8.2], ...
%   'Course', [-88;-89;-89;-92;-84;-71], ...
%   'GroundSpeed', [10;10;10;10;10;10], ...
%   'ClimbRate', [0;0;0;0;0;0], ...
%   'AutoPitch', true, ...
%   'AutoBank', true);
% 
%   % Create sensors
%   NoScanning = fusionRadarSensor('SensorIndex', 1, ...
%   'UpdateRate', 10, ...
%   'MountingAngles', [-90 0 0], ...
%   'FieldOfView', [20 10], ...
%   'ScanMode', 'No scanning', ...
%   'HasINS', true, ...
%   'DetectionCoordinates', 'Scenario', ...
%   'TargetReportFormat','Detections','HasElevation',true);
% 
%   % Assign sensors to platforms
%   Tower.Sensors = NoScanning;
%
%   % Create a theater plot to visualize sensor, sensor coverage,
%   % tracks and detections.
%   tp = theaterPlot('XLim', [-58 58], 'YLim', [-104 12], 'ZLim', [-109 8]);
%   set(tp.Parent,'YDir','reverse', 'ZDir','reverse');
%   view(tp.Parent, -37.5, 30);
%   platp = platformPlotter(tp,'DisplayName','Targets','MarkerFaceColor','k');
%   detp = detectionPlotter(tp,'DisplayName','Detections','MarkerSize',6, ...
%   'MarkerFaceColor',[0.85 0.325 0.098],'MarkerEdgeColor','k','History',10000);
%   covp = coveragePlotter(tp,'DisplayName','Sensor Coverage');
% 
%   % Configure sensor configurations for the scenario.
%   sensorConfig = trackingSensorConfiguration(scenario.Platforms{1}.Sensors{1}, ...
%   'SensorTransformFcn',@cvmeas,'FilterInitializationFcn',@initcvggiwphd);
% 
%   % Configure tracker.
%   tracker = trackerPHD('SensorConfigurations',sensorConfig, ...
%   'PartitioningFcn', @(x)partitionDetections(x,4,10),...
%   'HasSensorConfigurationsInput',true);
% 
%   % Add a trackPlotter.
%   tPlotter = trackPlotter(tp,'DisplayName','Tracks');
% 
%   % Simulation loop
%   while advance(scenario) && ishghandle(tp.Parent)
%
%   % generate sensor data
%   [dets, configs, sensorConfigPIDs] = detect(scenario);
%   
%   % read sensor data
%   allDets = [dets{:}];
%   if ~isempty(allDets)
%   % extract column vector of measurement positions
%      meas = cat(2,allDets.Measurement)';
%   % extract measurement noise
%      measCov = cat(3,allDets.MeasurementNoise);
%   else
%      meas = zeros(0,3);
%      measCov = zeros(3,3,0);
%   end
%   truePoses = platformPoses(scenario);
%   truePosition = vertcat(truePoses(:).Position);
%     
%   % update tracker 
%   [cTracks,tTracks,allTracks] = tracker(dets,configs,scenario.SimulationTime);
% 
%   % update plots
%   plotPlatform(platp,truePosition);
%   plotDetection(detp,meas,measCov);
%   plotCoverage(covp,coverageConfig(scenario));
%     
%   % Update the trackPlotter here:
%   % In this example, we used the sensor configuration generated from
%   % the sensor model FilterInitializationFcn, initcvggiwphd, which uses
%   % a constant velocity model and defines the states as [x;vx;y;vy;z;vy].
%   % To find the positions, we use
%   positionSelector = [1 0 0 0 0 0;0 0 1 0 0 0;0 0 0 0 1 0];
%   positions = getTrackPositions(cTracks,positionSelector);
%   if ~isempty(cTracks)
%      labels = cell(numel(cTracks),1);
%      for i =1:numel(cTracks)
%        labels{i} = {['T',num2str(cTracks(i).TrackID)]};
%      end
%   plotTrack(tPlotter, positions,labels);
%   end
%   drawnow
%   end
%
%   See also: trackingSensorConfiguration, partitionDetections.

     
    %   Copyright 2018-2021 The MathWorks, Inc.

    methods
        function out=trackerPHD
            % Support name-value pair arguments when constructing object
        end

        function out=cloneImpl(~) %#ok<STOUT>
        end

        function out=deleteTrack(~) %#ok<STOUT>
            %deleteTrack  Delete an existing track
            %    deleted = deleteTrack(obj,trackID) deletes the track
            %    specified by trackID from the tracker. The deleted flag
            %    returns true if a track with the same trackID existed and
            %    was deleted. If a track with that trackID did not exist,
            %    the deleted flag is false and a warning is issued.
            %
            % Note: the tracker must be updated at least once to be able to
            % delete a track.
        end

        function out=exportToSimulink(~) %#ok<STOUT>
            % EXPORTTOSIMULINK Export the tracker to a Simulink model.
            %
            % EXPORTTOSIMULINK(TRACKER) exports the tracker, TRACKER, as a
            % Simulink block in a new model with default name.
            %
            % EXPORTTOSIMULINK(..., 'Model', MODEL), allows you to export
            % the tracker to an existing Simulink model MODEL. MODEL can be
            % the name or handle of the Simulink model. If a Simulink model
            % with name MODEL does not exist, a new model is created with
            % name MODEL.
            %
            % EXPORTTOSIMULINK(..., 'BlockName', Name), allows you to
            % specify a name, NAME for the Simulink block.
            %
            % EXPORTTOSIMULINK(..., 'Position', POS), allows you to specify
            % block position, POS, in the model. POS must be a vector of
            % coordinates, in pixels: [left top right bottom]
            %
            % EXPORTTOSIMULINK(...,'OpenModel', TF), allows you to specify
            % a flag, TF, to indicate if the model should be opened after
            % exporting the TRACKER to Simulink or not. The default value
            % is set to true which means the model is always opened.
            %
            % H = EXPORTTOSIMULINK(TRACKER, ...) exports the tracker,
            % TRACKER, as a Simulink block and returns the handle to the
            % block.
        end

        function out=getNumInputsImpl(~) %#ok<STOUT>
            % Define total number of inputs for system with optional inputs
        end

        function out=getPropertyGroups(~) %#ok<STOUT>
            % Define property section(s) for display in Matlab
        end

        function out=initializeTrack(~) %#ok<STOUT>
            % trID = initializeTrack(tracker, track) initializes a a new
            % track in the tracker using the input, track and returns the
            % TrackID associated with it. track must be an objectTrack
            % struct or a struct with similar fields.
            % You may use this syntax if the filter is of the type gmphd.
            %
            % trID = initializeTrack(tracker, track, trackPHDFilter)
            % initializes a new track in the tracker using the input,
            % track, and its associated probability hypothesis density
            % filter, trackPHDFilter. trackPHDFilter must be the same type
            % of filter as used with the tracker. For example, if the
            % tracking is performed using gmphd, the trackPHDFilter must be
            % an object of type gmphd.
            % You may use this syntax if the filter is gmphd or ggiwphd.
            %
            % A warning is issued if the tracker already maintains
            % MaxNumTracks tracks and the returned id is zero, which
            % indicates a failure to initialize the track.
            %
            % Notes:
            % 1. The tracker must be updated at least once to be
            % able to initialize a track.
            % 2. The tracker assigns a TrackID to the track, gives an
            % UpdateTime equal to the last step time, and synchronizes the
            % data in the given track to the initialized track.
        end

        function out=isInputSizeMutableImpl(~) %#ok<STOUT>
            % Return false if input size cannot change
            % between calls to the System object
        end

        function out=loadObjectImpl(~) %#ok<STOUT>
        end

        function out=predictTracksToTime(~) %#ok<STOUT>
            %predictTracksToTime Predicts the tracks to a time stamp
            %   predictedTracks = predictTracksToTime(obj, trackID, time)
            %   predicts the state of the track specified by trackID to
            %   time instant time. time must be greater than the last
            %   update time provided to the tracker in the previous step.
            %
            %   ... = predictTracksToTime(obj, category, time) allows
            %   you to predict the states of all the tracks that match the
            %   category. Valid values for category are 'all', 'confirmed',
            %   or 'tentative'.
            %
            %   ... = predictTracksToTime(...,'WithCovariance',tf) allows
            %   you to specify whether the state covariance of each track
            %   should be predicted as well by setting the tf flag to true
            %   or false. Predicting the covariance is slower than
            %   predicting just the track states. The default is false.
            %
            % Note: the tracker must be updated at least once to be able to
            % predict tracks.
        end

        function out=releaseImpl(~) %#ok<STOUT>
            % Release resources
        end

        function out=resetImpl(~) %#ok<STOUT>
            % Returns the tracker to its initial state
        end

        function out=saveObjectImpl(~) %#ok<STOUT>
            % Set properties in structure s to values in object obj
        end

        function out=sensorIndices(~) %#ok<STOUT>
            %sensorIndices Return the list of sensor indices
            %  indices = sensorIndices(obj) returns the list of SensorIndex
            %  values for each sensor configuration in the
            %  SensorConfigurations property.
        end

        function out=setupImpl(~) %#ok<STOUT>
            % Allocate memory for each PHD. The source for a PHD is the
            % sensor's initialize method.
        end

        function out=stepImpl(~) %#ok<STOUT>
            % step Creates, update and delete tracks.
            % The step method is responsible for managing all the tracks.
            %
            %   confirmedTracks = step(tracker, detections, time)
            %   Update the tracker with a list of detections to the time
            %   instance specified by time. It returns a struct array of
            %   confirmed tracks corrected and predicted to the time instance.
            %
            %   ... = step(tracker, detections, config, time)
            %   HasSensorConfigurationsInput must be true to use this
            %   syntax. The config must be a an array of structs with
            %   SensorIndex and other properties of the
            %   SensorConfigurations which need to be updated at the
            %   current time instance.
            %
            %   [confirmedTracks, tentativeTracks, allTracks] = step(...)
            %   in addition, returns a list of the tentative tracks and all
            %   the tracks.
            %
            %   [...,analysisInformation] = step(...) in addition returns a
            %   struct of analysis information.
            %
            %   Inputs:
            %   tracker          - a trackerPHD
            %   detections       - a cell array of objectDetection objects.
            %   time             - the time to which all the tracks will be
            %                      updated and predicted. A real numeric
            %                      scalar value, greater than the value in
            %                      the previous call.
            %   configs          - an array of structs or a cell array of
            %                      trackingSensorConfiguration objects. If
            %                      provided as a struct, configs can
            %                      contain a subset of properties of
            %                      trackingSensorConfiguration class or the
            %                      configuration output of sensor model.
            %                      The fields on the sensor configuration
            %                      output struct are defined <a
            %                      href="matlab:help('fusion.internal.interfaces.DataStructures/sensorConfigStruct')">here</a>.
            %
            %   Output:
            %   tracks - an array of structs, where each element provides
            %   the track snapshot at the time to which the trackerPHD was
            %   updated. Each track has the following fields:
            %   States of target extracted from the PHD filter. For
            %   example, for a ggiwphd filter, the target states are:
            %
            %   State           - Kinematic State of the target
            %   StateCovariance - Error covariance of kinematic state.
            %   Extent          - Estimated size as a matrix.
            %   MeasurementRate - Estimated number of measurements from the
            %                     target per scan.
            %
            %   The track also has other fields which are populated by the
            %   tracker.
            %   TrackID         - Unique integer that identifies the track.
            %   SourceIndex     - Unique identifier of the tracker,
            %                     TrackerIndex.
            %   UpdateTime      - Time at which the track was updated.
            %   Age             - Age of the track in terms of number of
            %                     hits.
            %   ObjectClassID   - An integer value representing the object
            %                     classification. It is defined as 0 for
            %                     all tracks.
            %   StateParameters - Parameters about the track state
            %                     reference frame set in tracker property,
            %                     StateParameters.
            %   IsConfirmed     - A flag indicating if the track is
            %                     confirmed.
            %   IsSelfReported  - A flag indicating if the track is
            %                     reported by the tracker. It is defined as
            %                     true for all tracks.
            %
            %
            %   analysisInformation - a struct of additional information
            %   that allows you to analyze the tracker update stages.
            %   It includes the following fields:
            %   CorrectionOrder         - A list of SensorIndex in the
            %                             order they updated the PHD
            %   TrackIDsAtStepBeginning - Track IDs when step began
            %   DeletedTrackIDs         - Track IDs deleted during the
            %                             step
            %   TrackIDsAtStepEnd       - Track IDs at the end of step.
            %   SensorAnalysisInfo      - A cell array of structs
            %                             describing the correction stages
            %                             for each sensor as listed in the
            %                             CorrectionOrder.
            %   SensorAnalysisInfo{i} is a struct containing the correction
            %   information from SensorIndex with CorrectionOrder(i). It
            %   has the following fields
            %   SensorIndex             - SensorIndex of the current sensor
            %   DetectionCells          - A logical array of size MxP,
            %                             where M is the number of
            %                             detections from this sensor, and
            %                             P are the number of detection
            %                             cells. A true value at ith row in
            %                             each column represents if
            %                             detection i is included in this
            %                             cell
            %   DetectionLikelihoods    - A value which describes the
            %                             likelihood of the detection cell
            %                             against the current density
            %   IsBirthCell             - A logical vector representing if
            %                             the cell was considered for
            %                             adding birth components in the
            %                             density
            %   NumPartitions           - A scalar value representing the
            %                             number of partitions of
            %                             detections. This is controlled
            %                             by the PartitioningFcn specified
            %                             for the tracker
            %   DetectionProbability    - Probability of detecting
            %                             each component in the density
            %                             with the current sensor
            %   LabelsBeforeCorrection  - Labels/TrackIDs of components
            %                             before update
            %   LabelsAfterCorrection   - Labels/TrackIDs of components
            %                             after update
            %   WeightsBeforeCorrection - Weights of components before
            %                             update
            %   WeightsAfterCorrection  - Weights of components after
            %                             update
        end

        function out=validateInputsImpl(~) %#ok<STOUT>
        end

        function out=validatePropertiesImpl(~) %#ok<STOUT>
            % Validate related or interdependent property values
        end

    end
    properties
        % AssignmentThreshold Threshold for selecting birth detections
        %   Specify a scalar value to control the number of detection cells
        %   which are considered for birth in the predictive birth density.
        %   The AssignmentThreshold is the minimum value of distance
        %   (negative log-likelihood) of a detection (or set of detections
        %   for extended sensors) to be considered for adding new components
        %   in the birth density.
        %
        %   Default: 25
        AssignmentThreshold;

        % BirthRate Birth rate of new components in the density.
        %   Specify a scalar value to indicate the number of components
        %   that are added in the density per unit time. The birth density
        %   is created by using the FilterInitializationFcn of the
        %   trackingSensorConfiguration used with the tracker. The tracker
        %   adds component to the density in two ways.
        %   1. Predictive birth density: This is the density which is
        %   initialized by FilterInitializationFcn when called with no
        %   inputs.
        %   2. Adaptive birth density: This is the density which is
        %   initialized by FilterInitializationFcn when called with
        %   detections. The detections are chosen by the tracker based on
        %   their log-likelihood against the current estimated states of
        %   the targets.
        %   BirthRate corresponds to both predictive and adaptive density
        %   summed together
        %
        %   Default: 1e-3
        BirthRate;

        % ConfirmationThreshold Threshold for track confirmation.
        %   Specify a scalar threshold for confirmation of a track.
        %   ConfirmationThreshold is the minimum weight at which a
        %   tentative track is marked as confirmed.
        %
        %   Default: 0.8
        ConfirmationThreshold;

        %DeathRate Death rate of components in the density.
        % Specify a scalar value to model the death of components in the
        % density per unit time. DeathRate corresponds to the probability
        % of survival of components in the density upon prediction. The
        % survival probability is calculated as
        % (1-DeathRate)^dT, where dT is the prediction interval.
        %
        % Default: 1e-6
        DeathRate;

        % DeletionThreshold Threshold for track deletion.
        %   Specify a scalar threshold for track deletion. The deletion
        %   threshold is the minimum weight of a component in the
        %   PHD filter at which it is pruned.
        %
        %   Default: 1e-3
        DeletionThreshold;

        % ExtractionThreshold Threshold for track initiation
        %   Specify a scalar threshold for initiating a track from PHD
        %   filter. ExtractionThreshold is the minimum weight at which a
        %   component in the density is labelled as a track. The track
        %   starts as a Tentative track if its weight is below
        %   ConfirmationThreshold. Otherwise, it is marked as a Confirmed
        %   track.
        %
        %   Default: 0.5
        ExtractionThreshold;

        % HasSensorConfigurationsInput Update sensor configurations with time
        %   Set this property to true if you want to update the
        %   configurations of the sensor with time.
        %   When this flag is set to true, the tracker must be called with
        %   the configuration input as follows:
        %   ... = step(tracker,detections,config,time)
        %
        %   Default: false
        HasSensorConfigurationsInput;

        % LabelingThresholds Thresholds for Label management.
        %   These thresholds control how the tracker maintains a unique
        %   Label for each component in the density. The tracker uses the
        %   following logic to manage components belonging to the same
        %   TrackID. Consider L components in the density belonging to the
        %   same TrackID
        %   1. The tracker maintains that LabelingThresholds(1) is the
        %   highest attainable weight by a component.
        %   2. If the weight of maximum-weight component (1 out of L) is
        %   greater than LabelingThreshold(2) or if the ratio of
        %   maximum-weight to the sum of all weights is greater than
        %   LabelingThreshold(3), then the maximum-weight components
        %   retains the TrackID and all other components are deleted
        %   irrespective of their weights.
        %   3. If condition in 2. is not satisfied, then all other
        %   components belonging to the same TrackID are unlabeled and the
        %   maximum-weight component retains the TrackID.
        %
        %   Default: [1.1 1 0.8]
        LabelingThresholds;

        %MaxNumSensors  Maximum number of sensors
        %   Set the maximum number of sensors connected to the
        %   trackerPHD as a positive real integer.
        %   This number must be greater than or equal to the highest
        %   SensorIndex value used in the detections input to the
        %   step method.
        %
        %   Default: 20
        MaxNumSensors;

        %MaxNumTracks   Maximum number of tracks
        %   Set the maximum number of tracks the tracker can maintain as a
        %   positive real integer.
        %
        %   Default: 1000
        MaxNumTracks;

        % MergingThreshold Threshold for merging two components. See the <a
        % href="matlab:help('ggiwphd/merge')">merge</a>
        % method of ggiwphd filter. The threshold serves as an input to the
        % merge method.
        %
        %   Default: 25
        MergingThreshold;

        %NumConfirmedTracks    The number of tracks with status 'Confirmed'
        %   The total number of confirmed tracks (IsConfirmed = true) that
        %   trackerPHD is maintaining.
        %
        %   This value is calculated by the tracker and is read-only
        NumConfirmedTracks;

        %NumTracks  Number of tracks
        %   The total number of tracks the trackerPHD is maintaining.
        %
        %   This value is calculated by the tracker and is read-only
        NumTracks;

        % PartitioningFcn A function to compute the partitions of a
        % detections for sensors which report more than one detection per
        % object. See MaxNumDetsPerObject property of the
        % |trackingSensorConfiguration|.
        %
        % Default: 'partitionDetections'
        PartitioningFcn;

        % SensorConfigurations Current configurations of the sensor.
        %   Specify the configurations of the sensor used with the tracker
        %   as trackingSensorConfigurations. You can specify the
        %   SensorConfigurations during construction as a Name,value pair
        %   or set it after construction. There are no default values for
        %   the SensorConfigurations. You must specify them before using
        %   the tracker.
        %
        %   Default: {}
        SensorConfigurations;

        %StateParameters  Parameters used for track state frame transform
        %   Specify the parameters used for track frame transformation from
        %   source (this tracker) frame to a fuser frame.
        %   This property is tunable.
        %
        % Default
        StateParameters;

        %TrackerIndex Unique identifier of the tracker
        %   Specify the unique index associated with this tracker in a
        %   decentralized tracking architecture. This index is used as the
        %   SourceIndex in the tracks output, and serves in track-to-track
        %   fusion. You must define this property to a positive value to
        %   use the track outputs as inputs to a track fuser.
        %
        % Default: uint32(0)
        TrackerIndex;

        pPartitioningFcn;

    end
end

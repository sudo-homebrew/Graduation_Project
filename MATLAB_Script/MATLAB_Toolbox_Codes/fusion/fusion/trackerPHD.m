classdef trackerPHD < matlabshared.tracking.internal.fusion.AbstractTracker ...
        & fusion.internal.ExportToSimulinkInterface
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
    %     reset                      - Resets states of the trackerPHD
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
    %
    
    %   Copyright 2018-2021 The MathWorks, Inc.
    
    % System objects may be called directly like a function instead of using
    % the step method. For example, y = step(obj) and y = obj() are equivalent.
    
    %#codegen
    %#function partitionDetections
    
    properties(Nontunable)
        %TrackerIndex Unique identifier of the tracker
        %   Specify the unique index associated with this tracker in a
        %   decentralized tracking architecture. This index is used as the
        %   SourceIndex in the tracks output, and serves in track-to-track
        %   fusion. You must define this property to a positive value to
        %   use the track outputs as inputs to a track fuser.
        %
        % Default: uint32(0)
        TrackerIndex = uint32(0)
    end
    
    properties
        %StateParameters  Parameters used for track state frame transform
        %   Specify the parameters used for track frame transformation from
        %   source (this tracker) frame to a fuser frame.
        %   This property is tunable.
        %
        % Default
        StateParameters
    end
    
    % Public, tunable properties
    properties(Dependent = true)
        % SensorConfigurations Current configurations of the sensor.
        %   Specify the configurations of the sensor used with the tracker
        %   as trackingSensorConfigurations. You can specify the
        %   SensorConfigurations during construction as a Name,value pair
        %   or set it after construction. There are no default values for
        %   the SensorConfigurations. You must specify them before using
        %   the tracker.
        %
        %   Default: {}
        SensorConfigurations
    end
    
    properties (Nontunable)
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
        BirthRate = 1e-3;
        
        %DeathRate Death rate of components in the density.
        % Specify a scalar value to model the death of components in the
        % density per unit time. DeathRate corresponds to the probability
        % of survival of components in the density upon prediction. The
        % survival probability is calculated as
        % (1-DeathRate)^dT, where dT is the prediction interval.
        %
        % Default: 1e-6
        DeathRate = 1e-6;
        
        % AssignmentThreshold Threshold for selecting birth detections
        %   Specify a scalar value to control the number of detection cells
        %   which are considered for birth in the predictive birth density.
        %   The AssignmentThreshold is the minimum value of distance
        %   (negative log-likelihood) of a detection (or set of detections
        %   for extended sensors) to be considered for adding new components
        %   in the birth density.
        %
        %   Default: 25
        AssignmentThreshold = 25;
        
        % ExtractionThreshold Threshold for track initiation
        %   Specify a scalar threshold for initiating a track from PHD
        %   filter. ExtractionThreshold is the minimum weight at which a
        %   component in the density is labelled as a track. The track
        %   starts as a Tentative track if its weight is below
        %   ConfirmationThreshold. Otherwise, it is marked as a Confirmed
        %   track.
        %
        %   Default: 0.5
        ExtractionThreshold = 0.5;
        
        % ConfirmationThreshold Threshold for track confirmation.
        %   Specify a scalar threshold for confirmation of a track.
        %   ConfirmationThreshold is the minimum weight at which a
        %   tentative track is marked as confirmed.
        %
        %   Default: 0.8
        ConfirmationThreshold = 0.8;
        
        % DeletionThreshold Threshold for track deletion.
        %   Specify a scalar threshold for track deletion. The deletion
        %   threshold is the minimum weight of a component in the
        %   PHD filter at which it is pruned.
        %
        %   Default: 1e-3
        DeletionThreshold = 1e-3;
        
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
        LabelingThresholds = [1.1 1 0.8];
        
        % MergingThreshold Threshold for merging two components. See the <a
        % href="matlab:help('ggiwphd/merge')">merge</a>
        % method of ggiwphd filter. The threshold serves as an input to the
        % merge method.
        %
        %   Default: 25
        MergingThreshold = 25;
    end
    
    properties (Nontunable)
        % PartitioningFcn A function to compute the partitions of a
        % detections for sensors which report more than one detection per
        % object. See MaxNumDetsPerObject property of the
        % |trackingSensorConfiguration|.
        %
        % Default: 'partitionDetections'
        PartitioningFcn = 'partitionDetections';
    end
    
    properties (Access = protected)
        pPartitioningFcn
    end
    
    properties (Nontunable)
        % HasSensorConfigurationsInput Update sensor configurations with time
        %   Set this property to true if you want to update the
        %   configurations of the sensor with time.
        %   When this flag is set to true, the tracker must be called with
        %   the configuration input as follows:
        %   ... = step(tracker,detections,config,time)
        %
        %   Default: false
        HasSensorConfigurationsInput (1, 1) logical = false;
    end
    
    properties (SetAccess = protected, Dependent = true)
        %NumConfirmedTracks    The number of tracks with status 'Confirmed'
        %   The total number of confirmed tracks (IsConfirmed = true) that
        %   trackerPHD is maintaining.
        %
        %   This value is calculated by the tracker and is read-only
        NumConfirmedTracks
        
        %NumTracks  Number of tracks
        %   The total number of tracks the trackerPHD is maintaining.
        %
        %   This value is calculated by the tracker and is read-only
        NumTracks
    end
    
    % Public, non-tunable properties
    properties(Nontunable)
        %MaxNumSensors  Maximum number of sensors
        %   Set the maximum number of sensors connected to the
        %   trackerPHD as a positive real integer.
        %   This number must be greater than or equal to the highest
        %   SensorIndex value used in the detections input to the
        %   step method.
        %
        %   Default: 20
        MaxNumSensors = 20;
        
        %MaxNumTracks   Maximum number of tracks
        %   Set the maximum number of tracks the tracker can maintain as a
        %   positive real integer.
        %
        %   Default: 1000
        MaxNumTracks = 1000
    end
    
    properties (Nontunable, Access = {?trackerPHD,?matlab.unittest.TestCase})
        pDataType
    end
    
    properties(Access = {?trackerPHD,?matlab.unittest.TestCase})
        pDensity;
        pUndetectedDensity;
        pBirthDensity;
        pLabelManager;
        pLastTime = 0;
        pPartitionGenerator;
        pSensorConfigurations;
        pSensorIndices;
        pSampleDetection
        pStateParameters = struct
    end    
    
    methods
        % Constructor
        function obj = trackerPHD(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
        
        function indices = sensorIndices(obj)
            %sensorIndices Return the list of sensor indices
            %  indices = sensorIndices(obj) returns the list of SensorIndex
            %  values for each sensor configuration in the
            %  SensorConfigurations property.
            if ~coder.internal.is_defined(obj.SensorConfigurations)
                indices = zeros(1,0,'uint32');
                return
            end
            numSensors = numel(obj.SensorConfigurations);
            indices = zeros(1,numSensors,'uint32');
            if coder.target('MATLAB')
                indices(1:numSensors) = cellfun(@(c) cast(c.SensorIndex,'uint32'), obj.SensorConfigurations);
            else
                for i = 1:numSensors
                    indices(i) = obj.SensorConfigurations{i}.SensorIndex;
                end
            end
        end
        
        % Predict tracks to time
        function predictedTracks = predictTracksToTime(obj,track,time,varargin)
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
            
            % Check that the tracker has locked
            coder.internal.assert(isLocked(obj),'fusion:trackerPHD:PredictBeforeUpdate')
            narginchk(3,5);
            
            validateattributes(time, {'numeric'}, ...
                {'real', 'finite', 'nonsparse', 'scalar', '>', obj.pLastTime}, ...
                'predictTracksToTime', 'time');
            
            if ischar(track) || isstring(track)
                type = validatestring(track,{'all','confirmed','tentative'},...
                    'predictTracksToTime','category');
                % All is -1, confirmed is -2 and tentative is -3
                id = -(find(strcmpi(type,{'all','confirmed','tentative'})));
            else
                validateattributes(track, {'numeric'}, ...
                    {'real', 'positive', 'scalar', 'integer'}, ...
                    'predictTracksToTime', 'trackID');
                id = track;
            end
            
            withCovariance = false;
            if nargin > 3
                validateattributes(varargin{2},{'logical'},{'scalar'},'trackerPHD');
                withCovariance = varargin{2};
            end
            
            dT = time - obj.pLastTime;
            predictedTracks = obj.pLabelManager.predictTracksToTime(obj.pDensity,id,dT,withCovariance);
        end
        
        function trID = initializeTrack(tracker, track, varargin)
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
            
            narginchk(2,3);
            
            % Validate input track
            validateattributes(track,{'objectTrack','struct'},...
                {'scalar'},'initializeTrack','track');
            
            if isstruct(track)
                objTrk = objectTrack(track);
            else
                objTrk = track;
            end
            coder.internal.assert(isLocked(tracker), 'fusion:trackerPHD:mustUpdateBeforeCall','initializeTrack');
            
            trID = initializeTrack(tracker.pLabelManager, tracker.pDensity, objTrk, varargin{:});
        end
        function deleted = deleteTrack(tracker, trackID)
            %deleteTrack  Delete an existing track
            %    deleted = deleteTrack(obj,trackID) deletes the track
            %    specified by trackID from the tracker. The deleted flag
            %    returns true if a track with the same trackID existed and
            %    was deleted. If a track with that trackID did not exist,
            %    the deleted flag is false and a warning is issued.
            %
            % Note: the tracker must be updated at least once to be able to
            % delete a track.
            %
            
            narginchk(2,2);
            validateattributes(trackID, {'numeric'}, ...
                {'real', 'positive', 'scalar', 'integer'}, ...
                'deleteTrack', 'trackID');
            
            % Assert that the tracker is updated at least once
            coder.internal.assert(isLocked(tracker), 'fusion:trackerPHD:mustUpdateBeforeCall','deleteTrack');
            
            % Delete the track, label manager will throw a warning if the
            % trackID is invalid
            deleted = deleteTrack(tracker.pLabelManager, tracker.pDensity, trackID);
        end
    end
    
    methods(Access = protected)
        %% Setup for the tracker
        function setupImpl(obj,varargin)
            % Allocate memory for each PHD. The source for a PHD is the
            % sensor's initialize method.
            coder.internal.assert(coder.internal.is_defined(obj.pSensorConfigurations),'fusion:trackerPHD:MustDefineBeforeStep','SensorConfigurations');
            sensor = obj.SensorConfigurations{1};
            phd = initialize(sensor);
            
            % Verify that the filter is known.
            coder.internal.assert(isa(phd,'fusion.internal.AbstractPHDFilter'),'fusion:trackerPHD:UnsupportedPHD',class(phd));
            
            % Weights must exist on any PHD filter
            obj.pDataType = class(phd.Weights);
            
            % Nullify the PHD and set detections to allocate memory for
            % detection buffer.
            nullify(phd);
            if isa(varargin{1},'objectDetection')
                dets = matlabshared.tracking.internal.fusion.makeDetectionCells(varargin{1});
            else
                dets = varargin{1};
            end
            
            % The setupImpl path in codegen may be hit multiple times, even
            % when during run-time the tracker will be locked. Guard the
            % error message with a property which is defined during setup
            if ~coder.internal.is_defined(obj.pSampleDetection)
                coder.internal.assert(~isempty(dets),'fusion:trackerPHD:UndefinedSampleDetection');
                obj.pSampleDetection = dets{1};
            end
            
            phd.Detections = {obj.pSampleDetection};
            
            % Assign it to total, undetected and birth density.
            obj.pDensity = clone(phd);
            obj.pUndetectedDensity = clone(phd);
            obj.pBirthDensity = clone(phd);
            
            % Assign labeler and partition generator
            obj.pLabelManager = fusion.internal.LabelManager('LabelingThresholds',obj.LabelingThresholds,...
                'ExtractionThreshold',obj.ExtractionThreshold,...
                'DeletionThreshold',obj.DeletionThreshold,...
                'MergingThreshold',obj.MergingThreshold,...
                'ConfirmationThreshold',obj.ConfirmationThreshold,...
                'MaxNumLabels',obj.MaxNumTracks,...
                'TrackerIndex',obj.TrackerIndex...
                );
            
            obj.pLabelManager.pDataType = obj.pDataType;
            obj.pPartitionGenerator = fusion.internal.PartitionGenerator(...
                'PartitioningFcn',obj.pPartitioningFcn,'pDataType',obj.pDataType);
        end
        
        % Beginning of stepImpl
        % -----------------------------------------------------------------
        function [confTracks,tentTracks,allTracks,analysisInfo] = stepImpl(obj,varargin)
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
            nargoutchk(0,4);
            % Parse step inputs
            [detections, configurations, time] = parseStepInputs(obj,varargin{:});
            
            dT = time - obj.pLastTime;
            coder.internal.assert(dT > 0, ...
                'fusion:trackerPHD:TimeMustIncrease','step');
            
            % Sync configurations with internal configurations
            syncSensorConfigs(obj, configurations);
            
            % Get list of active sensors (IsValidTime = true)
            currentSensors = getCurrentSensors(obj);
            allActiveSensors = obj.pSensorIndices(currentSensors);
            
            % Sort and select available sensors from detections.
            [detectionSources, sensorIndices, timeStamps] = selectAndSortSensors(obj, detections, time);
            
            % Assert that sensorIndices are from defined sensor
            % configurations
            coder.internal.assert(all(ismember(sensorIndices,obj.pSensorIndices)),'fusion:trackerPHD:notDefinedSensor');
            
            % Assert that sensorIndices only contain active sensors.
            coder.internal.assert(all(ismember(sensorIndices,allActiveSensors)),'fusion:trackerPHD:notActiveSensor');
            
            configObjects = obj.SensorConfigurations;
            activeNoDetections = setdiff(allActiveSensors,sensorIndices);
            
            allSensorIndices = [sensorIndices(:);activeNoDetections(:)];
            allTimeStamps = [timeStamps;time*ones(numel(activeNoDetections),1)];
            
            sensorAnalysis = repmat({getSensorAnalysisFormat(obj)},[numel(allSensorIndices) 1]);
            
            % Iterated correction for each sensor
            for i = 1:numel(allSensorIndices)
                sIndex = allSensorIndices(i) == obj.pSensorIndices;
                thisSensorOrigination = detectionSources == allSensorIndices(i);
                if coder.target('MATLAB')
                    sensorConfig = configObjects{sIndex};
                    sensorDetections = {detections{thisSensorOrigination}};
                else
                    detIndices = find(thisSensorOrigination);
                    n = numel(detIndices);
                    sensorDetections = cell(n,1);
                    for k = 1:numel(sensorDetections)
                        sensorDetections{k} = detections{detIndices(k)};
                    end
                    indices = find(sIndex);
                    sensorConfig = configObjects{indices(1)};
                end
                predictDensity(obj,allTimeStamps(i),sensorConfig);
                sensorAnalysis{i} = correctDensity(obj,sensorDetections,sensorConfig);
            end
            
            % Finally, predict everything to current time
            predictDensity(obj,time);
            
            % Extract track information.
            switch nargout
                case 1
                    confTracks = extractState(obj,time);
                case 2
                    [confTracks, tentTracks] = extractState(obj,time);
                case 3
                    [confTracks,tentTracks,allTracks] = extractState(obj,time);
                case 4
                    [confTracks,tentTracks,allTracks, extractionAnalysis] = extractState(obj,time);
                    analysisInfo = struct(...
                        'CorrectionOrder',sensorIndices,...
                        'TrackIDsAtStepBeginning',extractionAnalysis.TrackIDsAtStepBeginning,...
                        'DeletedTrackIDs',extractionAnalysis.DeletedTracksID,...
                        'TrackIDsAtStepEnd',extractionAnalysis.TrackIDsAtStepEnd...
                        );
                    analysisInfo.SensorAnalysisInfo = sensorAnalysis;
            end
            % Add birth density to total density
            scaleAndAddBirthDensity(obj,dT);
        end
        % End of stepImpl
        % -----------------------------------------------------------------
        
        function groups = getPropertyGroups(~)
            % Define property section(s) for display in Matlab
            
            trackerGroup = matlab.mixin.util.PropertyGroup(...
                {'TrackerIndex','SensorConfigurations','PartitioningFcn','MaxNumSensors','MaxNumTracks'});
            
            densityGroup = matlab.mixin.util.PropertyGroup(...
                {'AssignmentThreshold','BirthRate','DeathRate'});
            
            logicGroup = matlab.mixin.util.PropertyGroup(...
                {'ExtractionThreshold','ConfirmationThreshold','DeletionThreshold','MergingThreshold','LabelingThresholds'});
            
            numGroup = matlab.mixin.util.PropertyGroup(...
                {'StateParameters','HasSensorConfigurationsInput','NumTracks', 'NumConfirmedTracks'});
            
            groups = [trackerGroup, densityGroup, logicGroup, numGroup];
        end
        
        function resetImpl(obj)
            % Returns the tracker to its initial state
            
            reset(obj.pLabelManager);
            reset(obj.pPartitionGenerator);
            
            % Reset density
            nullify(obj.pDensity);
            nullify(obj.pUndetectedDensity);
            nullify(obj.pBirthDensity);
            
            % Reset time
            obj.pLastTime = cast(-eps,'like',obj.pLastTime);
            
            resetImpl@matlab.System(obj);
        end
        
        function releaseImpl(obj)
            % Release resources
            release(obj.pLabelManager);
            release(obj.pPartitionGenerator);
            releaseImpl@matlab.System(obj);
        end
        
        function validatePropertiesImpl(obj)
            % Validate related or interdependent property values
            
            % Filter Initialization Validation:
            validateattributes(obj.PartitioningFcn, ...
                {'function_handle','char'}, {'nonempty'}, 'trackerPHD', ...
                'PartitioningFcn');
            
            if isa(obj.PartitioningFcn, 'function_handle')
                obj.pPartitioningFcn = obj.PartitioningFcn;
            else
                obj.pPartitioningFcn = str2func(obj.PartitioningFcn);
            end
            
            % Check interdependent properties
            validateattributes(obj.ExtractionThreshold,...
                {'single','double'},{'real','finite','nonsparse','scalar','<',obj.ConfirmationThreshold},'trackerPHD',...
                'ExtractionThreshold');
            
            validateattributes(obj.DeletionThreshold,...
                {'single','double'},{'real','finite','nonsparse','scalar','<',obj.ExtractionThreshold},'trackerPHD',...
                'DeletionThreshold');
        end
        
        %% Backup/restore functions
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj
            s = saveObjectImpl@matlab.System(obj);
            s.pLastTime = obj.pLastTime;
            if isLocked(obj)
                % Internal subobject
                s.pPartitionGenerator = saveobj(obj.pPartitionGenerator);
                s.pLabelManager = saveobj(obj.pLabelManager);
                % Densities
                s.pDensity = obj.pDensity;
                s.pUndetectedDensity = obj.pUndetectedDensity;
                s.pBirthDensity = obj.pBirthDensity;
                s.pDataType = obj.pDataType;
                s.pSensorConfigurations = obj.pSensorConfigurations;
                s.pSensorIndices = obj.pSensorIndices;
                s.pPartitioningFcn = obj.pPartitioningFcn;
                s.pSampleDetection = obj.pSampleDetection;
                s.pStateParameters = obj.pStateParameters;
            end
        end
        
        function loadObjectImpl(obj,s,wasLocked)
            obj.pLastTime = s.pLastTime;
            if wasLocked
                % Internal subobject
                obj.pLabelManager = fusion.internal.LabelManager.loadobj(s.pLabelManager);
                obj.pPartitionGenerator = fusion.internal.PartitionGenerator.loadobj(s.pPartitionGenerator);
                % Densities
                obj.pDensity = s.pDensity;
                obj.pUndetectedDensity = s.pUndetectedDensity;
                obj.pBirthDensity = s.pBirthDensity;
                obj.pDataType = s.pDataType;
                obj.pSensorConfigurations = s.pSensorConfigurations;
                obj.pSensorIndices = s.pSensorIndices;
                obj.pPartitioningFcn = s.pPartitioningFcn;
                if isfield(s,'pSampleDetection')
                    obj.pSampleDetection = s.pSampleDetection;
                end
                if isfield(s,'pStateParameters')
                    obj.pStateParameters = s.pStateParameters;
                end
            end
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end
        
        function newObj = cloneImpl(obj)
            newObj = cloneImpl@matlab.System(obj);
            if coder.internal.is_defined(obj.pDensity)
                newObj.pDensity = clone(obj.pDensity);
                newObj.pUndetectedDensity = clone(obj.pUndetectedDensity);
                newObj.pBirthDensity = clone(obj.pBirthDensity);
                newObj.pLabelManager = clone(obj.pLabelManager);
                newObj.pPartitionGenerator = clone(obj.pPartitionGenerator);
                newObj.pSampleDetection = obj.pSampleDetection;
                newObj.pDataType = obj.pDataType;
            end
            if coder.internal.is_defined(obj.pSensorConfigurations)
                n = numel(obj.pSensorConfigurations);
                newObj.pSensorConfigurations = cell(n,1);
                for i = 1:n
                    newObj.pSensorConfigurations{i} = clone(obj.pSensorConfigurations{i});
                end
                newObj.pSensorIndices = obj.pSensorIndices;
            end
            if coder.internal.is_defined(obj.pPartitioningFcn)
                newObj.pPartitioningFcn = obj.pPartitioningFcn;
            end
            newObj.pStateParameters = obj.pStateParameters;
        end
        
        %% System Object methods
        function validateInputsImpl(obj, varargin)
            validateattributes(varargin{1},{'cell','objectDetection'},{},'trackerPHD','Detections',1);
            if obj.HasSensorConfigurationsInput
                configs = varargin{2};
                validateattributes(configs, {'struct','cell'}, {'vector'},'step','config',2);
                if iscell(configs)
                    coder.internal.assert(isstruct(configs{1}) || isa(configs{1},'fusion.internal.AbstractTrackingSensorConfiguration'),'fusion:trackerPHD:invalidConfigType',class(configs{1}));
                else
                    coder.internal.assert(isstruct(configs(1)) && isfield(configs(1),'SensorIndex'),'fusion:trackerPHD:invalidConfigType',class(configs(1)));
                end
                validateattributes(varargin{3},{'numeric'},{'scalar','real','finite','nonsparse','nonnegative'},'step','time',4);
            else
                validateattributes(varargin{2},{'numeric'},{'scalar','real','finite','nonsparse','nonnegative'},'step','time',3);
            end
        end
        
        function flag = isInputSizeMutableImpl(obj,index)
            % Return false if input size cannot change
            % between calls to the System object
            flag = false;
            if index == 1
                flag = true;
            end
            if obj.HasSensorConfigurationsInput && index == 2
                flag = true;
            end
        end
        
        function num = getNumInputsImpl(obj)
            % Define total number of inputs for system with optional inputs
            num = 2;
            if obj.HasSensorConfigurationsInput
                num = 3;
            end
        end
    end
    
    methods
        function set.LabelingThresholds(obj,val)
            validateattributes(val,...
                {'single','double'},{'real','finite','nonsparse','vector','numel',3,'decreasing'},'trackerPHD',...
                'LabelingThresholds');
            obj.LabelingThresholds = val;
        end
        
        function set.BirthRate(obj,val)
            validateattributes(val,...
                {'single','double'},{'real','finite','nonsparse','scalar','positive'},'trackerPHD',...
                'BirthRate');
            obj.BirthRate = val;
        end
        
        function set.DeathRate(obj,val)
            validateattributes(val,...
                {'single','double'},{'real','finite','nonsparse','scalar','positive'},'trackerPHD',...
                'DeathRate');
            obj.DeathRate = val;
        end
        
        function set.MaxNumSensors(obj,val)
            validateattributes(val,...
                {'single','double'},{'scalar','integer','positive'},'trackerPHD',...
                'MaxNumSensors');
            obj.MaxNumSensors = val;
        end
        
        function set.MaxNumTracks(obj,val)
            validateattributes(val,...
                {'single','double'},{'scalar','integer','positive'},'trackerPHD',...
                'MaxNumTracks');
            obj.MaxNumTracks = val;
        end
        
        function set.AssignmentThreshold(obj,val)
            validateattributes(val,...
                {'single','double'},{'real','finite','nonsparse','scalar','positive'},'trackerPHD',...
                'AssignmentThreshold');
            obj.AssignmentThreshold = val;
        end
        
        function set.SensorConfigurations(obj,val)
            coder.internal.assert(isa(val,'cell') || isa(val,'struct') || isa(val,'fusion.internal.AbstractTrackingSensorConfiguration'),...
                'fusion:trackerPHD:invalidConfigType',class(val));
            if isstruct(val)
                value = cell(numel(val),1);
                for i = 1:numel(val)
                    sensorConfig = fusion.internal.getConfigurationsFromStruct(val(i));
                    value{i} = sensorConfig{1};
                end
            else
                value = val;
            end
            % Don't have a data type here yet. Use a safe data type
            n = numel(value);
            coder.internal.assert(n<=obj.MaxNumSensors,'fusion:trackerPHD:SensorsGreaterThanMax');
            sensorIndex = zeros(n,1,'uint32');
            isSet = coder.internal.is_defined(obj.pSensorConfigurations);
            coder.internal.assert(~isSet,'fusion:trackerPHD:SensorConfigSet');
            configs = cell(n,1);
            for i = 1:numel(value)
                 if iscell(value)
                    coder.internal.assert(isa(value{i},'fusion.internal.AbstractTrackingSensorConfiguration'),...
                        'fusion:trackerPHD:invalidConfigType',class(value{i}));
                    sensorIndex(i) = value{i}.SensorIndex;
                    configs{i} = clone(value{i});
                else
                    sensorIndex(i) = value(i).SensorIndex;
                    configs{i} = clone(value(i));
                end
            end
            obj.pSensorConfigurations = configs;
            coder.internal.assert(numel(unique(sensorIndex)) == numel(value),'fusion:trackerPHD:ExpectedUniqueSensors');
            obj.pSensorIndices = sensorIndex;
        end
        function val = get.SensorConfigurations(obj)
            val = obj.pSensorConfigurations;
        end
        
        function set.TrackerIndex(obj,value)
            validateattributes(value,{'numeric'},{'real','finite','nonsparse',...
                'nonnegative','integer','scalar'},class(obj),'TrackerIndex')
            obj.TrackerIndex = uint32(value);
        end
        
        function val = get.StateParameters(obj)
            val = obj.pStateParameters;
        end
        
        function set.StateParameters(obj,value)
            setStateParameters(obj,value);
        end
        function val = get.NumTracks(obj)
            val = 0; % initialize double value. Also the value before setup
            if coder.internal.is_defined(obj.pLabelManager)
                val(1) = obj.pLabelManager.pNumLabels;
            end
        end
        function val = get.NumConfirmedTracks(obj)
            val = 0; % initialize double value. Also the value before setup
            if coder.internal.is_defined(obj.pLabelManager)
                val(1) = sum(obj.pLabelManager.pConfirmationFlags);
            end
        end
    end
    
    methods (Access = {?trackerPHD,?matlab.unittest.TestCase})
        function [detections, configurations, time] = parseStepInputs(obj,varargin)
            % Parse step inputs into detections, configurations and time.
            if obj.HasSensorConfigurationsInput
                detections = varargin{1};
                configurations = varargin{2};
                time = varargin{3};
            else
                if iscell(varargin{1})
                    detections = varargin{1};
                else
                    detections = matlabshared.tracking.internal.fusion.makeDetectionCells(varargin{1});
                end
                configurations = obj.SensorConfigurations;
                time = varargin{2};
            end
        end
        
        % Step number 1. Sync sensor configurations with new configs
        function syncSensorConfigs(obj,configurations)
            if obj.HasSensorConfigurationsInput
                for i = 1:numel(configurations)
                    if iscell(configurations)
                        thisConfig = configurations{i};
                    else
                        thisConfig = configurations(i);
                    end
                    id = thisConfig.SensorIndex;
                    internalIndex = obj.pSensorIndices == id;
                    coder.internal.assert(any(internalIndex),'fusion:trackerPHD:unknownSensorIDConfig',i);
                    if coder.target('MATLAB')
                        internalConfig = obj.pSensorConfigurations{internalIndex};
                    else
                        internalIndexLoc = find(internalIndex);
                        internalConfig = obj.pSensorConfigurations{internalIndexLoc(1)};
                    end
                    sync(internalConfig,thisConfig);
                end
            end
        end
        
        function activeSensors = getCurrentSensors(obj)
            if coder.target('MATLAB')
                activeSensors = cellfun(@(x)x.IsValidTime,obj.pSensorConfigurations);
            else
                activeSensors = false(numel(obj.pSensorIndices),1);
                for i = 1:numel(activeSensors)
                    activeSensors(i) = obj.pSensorConfigurations{i}.IsValidTime;
                end
            end
        end
        
        % Step number 2. Sort sensors in order of time.
        function [detectionSources, sensorIndices, timeStamps] = selectAndSortSensors(obj, detections, time)
            if coder.target('MATLAB')
                detectionSources = cellfun(@(x)x.SensorIndex,detections);
                detectionTimes = cellfun(@(x)x.Time,detections);
            else
                n = numel(detections);
                detectionSources = zeros(n,1,obj.pDataType);
                detectionTimes = zeros(n,1,obj.pDataType);
                for i = 1:n
                    detectionSources(i) = detections{i}.SensorIndex;
                    detectionTimes(i) = detections{i}.Time;
                end
            end
            validateDetectionTimeStamps(obj, detectionTimes, time);
            [sensorIndices,ids] = unique(detectionSources);
            sensorTimeStamps = detectionTimes(ids);
            [timeStamps,ind] = sort(sensorTimeStamps);
            sensorIndices = sensorIndices(ind);
        end
        function validateDetectionTimeStamps(obj, detectionTimes, time)
            coder.internal.assert(all(time >= detectionTimes) & all(obj.pLastTime < detectionTimes),'fusion:trackerPHD:DetectionTimeMismatch','step');
        end
        % Step number 3.
        function predictDensity(obj,time,sensorConfig)
            % Predict the density to time.
            dT = time - obj.pLastTime;
            Ps = (1 - obj.DeathRate)^dT;
            predict(obj.pDensity,dT);
            scale(obj.pDensity,Ps);
            obj.pLastTime = time;
            
            if nargin == 3
                % Add predictive birth components
                currentBirthDensity = obj.pBirthDensity;
                predictiveBirth = initialize(sensorConfig);
                append(currentBirthDensity,predictiveBirth);
            end
        end
        
        % Step number 4.
        % Correct density with the correct sensor.
        function analysis = correctDensity(obj,sensorDetections,sensorConfig)
            phd = obj.pDensity;
            undetectedPHD = obj.pUndetectedDensity;
            birthPHD = obj.pBirthDensity;
            
            % Analysis info.
            lBegin = phd.Labels;
            wBegin = phd.Weights;
            
            [states,covs] = sigmaPoints(phd);
            Pd = probDetection(sensorConfig,states,covs);
            
            if ~isempty(sensorDetections)
                isPointSensor = sensorConfig.MaxNumDetsPerObject == 1;
                [detectionIndices,startID,endID] = obj.pPartitionGenerator(sensorDetections,isPointSensor);
                
                phd.Detections = reshape(sensorDetections,numel(sensorDetections),1);
                
                % Several partitions may contain the same cell.
                % Calculate likelihoods of unique cells only to save time.
                [uniqueDetIndicesT,~,ic] = unique(detectionIndices','rows');
                uniqueDetIndices = uniqueDetIndicesT';
                uniqueLikelihoods = likelihood(phd,uniqueDetIndices,sensorConfig);
                % Set the likelihoods of all unique indices.
                cellLikelihoods = uniqueLikelihoods(:,ic);
                
                existWeights = phd.Weights;
                
                Kc = sensorConfig.ClutterDensity;
                
                % Give birth from detection cells whose distance is larger
                % than assignment threshold.
                assignLikelihood = log(sum(bsxfun(@times,exp(cellLikelihoods),existWeights(:)),1));
                birthCells = assignLikelihood < -obj.AssignmentThreshold;
                birthIndices = detectionIndices(:,birthCells);
                for i = 1:sum(birthCells)
                    if coder.target('MATLAB')
                        thisBirthDetections = {sensorDetections{birthIndices(:,i)}};
                    else
                        indices = find(birthIndices(:,i));
                        n = numel(indices);
                        thisBirthDetections = cell(n,1);
                        for k = 1:numel(thisBirthDetections)
                            thisBirthDetections{k} = sensorDetections{indices(k)};
                        end
                    end
                    thisPHD = initialize(sensorConfig,thisBirthDetections);
                    append(birthPHD,thisPHD);
                end
                
                scaledLikelihoods = obj.pPartitionGenerator.evaluatePartitions(cellLikelihoods,detectionIndices,startID,endID,existWeights,Pd,Kc);
                
                % Correct undetected PHD.
                sync(undetectedPHD,phd);
                % Get probability of zero detections
                PzeroDets = probZeroDetections(undetectedPHD, sensorConfig);
                correctUndetected(undetectedPHD,Pd,PzeroDets);
                
                % Correct detected PHD
                correct(phd,detectionIndices,scaledLikelihoods);
                
                % Add detected and undetected density.
                append(phd,undetectedPHD);
                
                
                analysis = struct('SensorIndex',sensorConfig.SensorIndex,...
                    'DetectionCells',detectionIndices,...
                    'DetectionLikelihoods',scaledLikelihoods,...
                    'IsBirthCell',birthCells,...
                    'NumPartitions',numel(startID),...
                    'DetectionProbability',Pd,...
                    'LabelsBeforeCorrection',lBegin,...
                    'LabelsAfterCorrection',phd.Labels,...
                    'WeightsBeforeCorrection',wBegin,...
                    'WeightsAfterCorrection',phd.Weights...
                    );
                
            else
                PzeroDets = probZeroDetections(phd, sensorConfig);
                correctUndetected(phd,Pd,PzeroDets);
                lAfter = phd.Labels;
                wAfter = phd.Weights;
                analysis = struct('SensorIndex',sensorConfig.SensorIndex,...
                    'DetectionCells',false(0,0),...
                    'DetectionLikelihoods',zeros(0,1,obj.pDataType),...
                    'IsBirthCell',false(1,0),...
                    'NumPartitions',0,...
                    'DetectionProbability',Pd,...
                    'LabelsBeforeCorrection',lBegin,...
                    'LabelsAfterCorrection',lAfter,...
                    'WeightsBeforeCorrection',wBegin,...
                    'WeightsAfterCorrection',wAfter...
                    );
            end
        end
        
        function setStateParameters(obj,value)
            validateattributes(value,{'struct'},{},class(obj),'StateParameters')
            obj.pStateParameters = value;
        end
        
        function [confTracks,tentTracks,allTracks,extractionAnalysis] = extractState(obj,time)
            phd = obj.pDensity;
            if nargout == 4
                [confTracks,tentTracks,allTracks,extractionAnalysis] = obj.pLabelManager(phd,obj.StateParameters,time);
            else
                [confTracks,tentTracks,allTracks] = obj.pLabelManager(phd,obj.StateParameters,time);
            end
        end
        
        function scaleAndAddBirthDensity(obj,dT)
            % Add birth density to total density by taking care of birth rate.
            % dT is the total time.
            birthDensity = obj.pBirthDensity;
            totalDensity = obj.pDensity;
            
            % Scale birth density with birth rate.
            sumWeights = sum(birthDensity.Weights);
            if sumWeights > 0
                birthWeight = obj.BirthRate*dT;
                scale(birthDensity,birthWeight/sumWeights);
            end
            % Add it to total density.
            append(totalDensity,birthDensity);
            % Empty the birth density
            nullify(birthDensity);
        end
        
        function sensorAnalysisFormat = getSensorAnalysisFormat(obj)
            % Format of analysis information for the each sensor.
            labels = obj.pDensity.Labels;
            weights = obj.pDensity.Weights;
            Pd = 0*weights; % Can be used as it will be same size.
            
            detCells = false(0);
            detLikelihoods = cast(0,obj.pDataType);
            birthCell = false(0);
            coder.varsize('detCells');
            coder.varsize('detLikelihoods');
            coder.varsize('birthCell');
            
            sensorAnalysisFormat = struct('SensorIndex',obj.pSensorConfigurations{1}.SensorIndex,...
                'DetectionCells',detCells,...
                'DetectionLikelihoods',detLikelihoods,...
                'IsBirthCell',birthCell,...
                'NumPartitions',0,...
                'DetectionProbability',Pd,...
                'LabelsBeforeCorrection',labels,...
                'LabelsAfterCorrection',labels,...
                'WeightsBeforeCorrection',weights,...
                'WeightsAfterCorrection',weights...
                );
        end
        
    end

    methods
        function blkHandle = exportToSimulink(obj,varargin)
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

            %Add tracker block to the model and set parameters that are
            %common between matlab object and Simulink block.
            blkHandle = exportToSimulink@fusion.internal.ExportToSimulinkInterface(obj,varargin{:});

            % Set properties that are exposed differently in Simulink.
            set_param(blkHandle,'TrackerIndexSimulink',num2str(obj.TrackerIndex));
          
            %Preallocate memory for config.            
            config = repmat(toStruct(obj.SensorConfigurations{1}),...
                numel(obj.SensorConfigurations),1);
            for i = 1:numel(obj.SensorConfigurations)
                %SensorConfigurations is a cell array of
                %trackingSensorConfiguration objects.
                config(i) = toStruct(obj.SensorConfigurations{i});
            end
            set_param(blkHandle,'SensorConfigurationExpression',...
                fusion.simulink.internal.addParamInModelPreLoadCallback(blkHandle,...
                'SensorConfiguration',config));
        end
    end   
    
    methods(Static, Hidden)
        function flag = isAllowedInSystemBlock
            flag = false;
        end
    end
end
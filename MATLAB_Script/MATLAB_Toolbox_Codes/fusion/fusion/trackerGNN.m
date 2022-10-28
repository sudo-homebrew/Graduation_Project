classdef (StrictDefaults) trackerGNN <  matlabshared.tracking.internal.fusion.GNNTracker ...
        & fusion.internal.ExportToSimulinkInterface ...
        & fusion.internal.TrackerMemoryManagementUtilities
    %trackerGNN Tracking object using GNN assignment
    %   tracker = trackerGNN creates a multi-sensor, multi-object tracker that
    %   uses a global nearest neighbor (GNN) assignment to maintain a single
    %   hypothesis about the objects it tracks. The trackerGNN initializes,
    %   confirms, corrects, predicts (performs coasting) and deletes tracks.
    %
    %   A track is created with a 'Tentative' status, meaning that there is not
    %   enough evidence for the trackerGNN to determine that the track is of a
    %   physical object. If enough additional detections are assigned to the
    %   tentative track, its status will change to 'Confirmed' (see
    %   ConfirmationThreshold). Alternatively, a track will be confirmed if a
    %   detection with a nonzero ObjectClassID value is assigned to it, as it
    %   means that the sensor is able to classify the physical object.
    %
    %   tracker = trackerGNN('Name', value) creates a trackerGNN object by
    %   specifying its properties as name-value pair arguments. Unspecified
    %   properties have default values. See the list of properties below.
    %
    %   Step method syntax: click on <a href="matlab:help('trackerGNN/stepImpl')">step</a> for more details.
    %   The step method is responsible for managing all the tracks:
    %     1. The method attempts to assign the detections to existing tracks.
    %     2. New tracks are created based on unassigned detections.
    %     3. Tracks that are assigned to detections are updated and confirmed.
    %     4. Tracks that are not assigned to detections are coasted (predicted)
    %        and eventually deleted.
    %
    %   System objects may be called directly like a function instead of using
    %   the step method. For example, y = step(obj) and y = obj() are
    %   equivalent.
    %
    %   trackerGNN properties:
    %     TrackerIndex               - Unique identifier of the tracker
    %     FilterInitializationFcn    - A handle to a function that initializes
    %                                  a tracking filter based on a detection
    %     Assignment                 - The name of the assignment algorithm
    %     CustomAssignmentFcn        - Name of 'Custom' assignment function
    %     AssignmentThreshold        - The threshold that controls the
    %                                  assignment of detections to tracks
    %     AssignmentClustering       - Clustering of detections and tracks
    %                                  for assignment               
    %     TrackLogic                 - Choose track logic: 'History' or 'Score'
    %     ConfirmationThreshold      - Specify the threshold for the
    %                                  confirmation logic
    %     DeletionThreshold          - Specify the threshold for the deletion
    %                                  logic
    %     DetectionProbability       - Probability of detecting a target*
    %     FalseAlarmRate             - Rate of false positive detections*
    %     Beta                       - The rate of new tracks per unit volume*
    %     Volume                     - The volume of the sensor detection bin*
    %     MaxNumTracks               - Define the maximum number of tracks
    %     MaxNumDetections           - Define the maximum number of detections
    %     MaxNumSensors              - Define the maximum number of sensors
    %     OOSMHandling               - Handle out-of-sequence measurement (OOSM)
    %     MaxNumOOSMSteps            - Maximum number of OOSM steps
    %     StateParameters            - Parameters defining the track state
    %     HasDetectableTrackIDsInput - Provide detectable track IDs as an input
    %     HasCostMatrixInput         - Provide cost matrix as an input
    %     NumTracks                  - Number of tracks             (Read-only)
    %     NumConfirmedTracks         - Number of confirmed tracks   (Read-only)
    %     EnableMemoryManagement     - Enable memory management properties
    %     MaxNumDetectionsPerSensor  - Define the maximum number of detections 
    %                                  per sensor
    %     MaxNumDetectionsPerCluster - Define the maximum number of detections 
    %                                  per cluster
    %     MaxNumTracksPerCluster     - Define the maximum number of tracks  
    %                                  per assignment cluster
    %     ClusterViolationHandling   - Handle run-time violation of cluster
    %                                  bounds
    %   * Properties with an asterisk are only used for track score logic.
    %
    %   trackerGNN methods:
    %     <a href="matlab:help('trackerGNN/stepImpl')">step</a>                       - Creates, updates, and deletes the tracks
    %     predictTracksToTime        - Predicts the tracks to a time stamp
    %     getTrackFilterProperties   - Returns the values of filter properties
    %     setTrackFilterProperties   - Sets values to filter properties
    %     initializeTrack            - Initialize a new track
    %     deleteTrack                - Delete an existing track
    %     exportToSimulink           - Export the tracker to a Simulink model
    %     release                    - Allows property value and input characteristics changes
    %     clone                      - Creates a copy of the trackerGNN
    %     isLocked                   - Locked status (logical)
    %     reset                      - Resets states of the trackerGNN
    %
    %   % EXAMPLE: Construct a tracker and use it to track two objects
    %
    %   % Construct a trackerGNN object with the default constant
    %   % velocity Kalman filter initialization function, initcvkf
    %   tracker = trackerGNN('FilterInitializationFcn', @initcvkf, ...
    %       'ConfirmationThreshold', [4 5], ...
    %       'DeletionThreshold', 10);
    %
    %   % Update the tracker with two detections with a nonzero ObjectClassID.
    %   % These detections will create confirmed tracks.
    %   detections = {objectDetection(1, [10;0], 'SensorIndex', 1, ...
    %      'ObjectClassID', 5, 'ObjectAttributes', {struct('ID', 1)}); ...
    %      objectDetection(1, [0;10], 'SensorIndex', 1, ...
    %      'ObjectClassID', 2, 'ObjectAttributes', {struct('ID', 2)})};
    %   time = 2;
    %   tracks = tracker(detections, time);
    %
    %   % In this example, the initcvkf filter initialization function is used.
    %   % The function initializes a 2-D constant velocity model, which assumes
    %   % that the state is: [x; vx; y; vy].
    %   % Because there were two detections, two tracks will be initialized.
    %   % They are confirmed on initialization because their ObjectClassID is
    %   % nonzero, indicating that the sensor was able to classify them.
    %
    %   % To find the position and velocity we use:
    %   positionSelector = [1 0 0 0; 0 0 1 0]; % [x, y]
    %   velocitySelector = [0 1 0 0; 0 0 0 1]; % [vx, vy]
    %
    %   positions = getTrackPositions(tracks, positionSelector)
    %   velocities = getTrackVelocities(tracks, velocitySelector)
    %
    %   See also: objectDetection, objectTrack, trackerJPDA, trackerTOMHT,
    %   trackerPHD
    
    % Copyright 2017-2021 The MathWorks, Inc.
    
    %#codegen
    %#function initsingerekf

    %% Properties
    
    % Public, non-tunable properties
    properties(Nontunable)
        %MaxNumTracks   Maximum number of tracks
        %   Set the maximum number of tracks the tracker can maintain as a
        %   positive real integer.
        %
        %   Default: 100
        MaxNumTracks = 100
        
        %MaxNumOOSMSteps Maximum number of OOSM steps
        %   Set the maximum number of steps that out-of-sequence
        %   measurements (OOSM) algorithms can handle as a positive real
        %   integer. Increasing the value of this property requires more
        %   memory but allows you to call the tracker with OOSMs that have
        %   a larger lag relative to the last timestamp the tracker was
        %   updated. As the lag increases, the impact of the OOSM on the
        %   current state of the track diminishes. A good value of this
        %   property is 3.
        %
        %   Default: 3
        MaxNumOOSMSteps (1, 1) {mustBePositive, mustBeInteger} = 3
    
        %FilterInitializationFcn  Filter initialization function name
        %   Specify the function for initializing the tracking filter used
        %   by a new track. The function must have the following syntax:
        %
        %       filter = filterInitializationFcn(detection)
        %
        %   filter    - a valid tracking filter that implements the motion
        %               and measurement models required for tracking
        %   detection - an objectDetection that initiates the track
        %
        %   Default: @initcvekf
        %   <a href="matlab:edit('initcvekf')">Open initcvekf for more details.</a>
        FilterInitializationFcn = 'initcvekf'
    end
    properties(Hidden,Constant)
        AssignmentSet = matlab.system.StringSet({'MatchPairs','Munkres','Jonker-Volgenant','Auction','Custom'});
        AssignmentFcnSet = {'matchpairs';'assignmunkres';'assignjv';'assignauction'};
        OOSMHandlingSet = matlab.system.StringSet({'Terminate','Neglect','Retrodiction'});
    end
    
    properties(Nontunable)
        %OOSMHandling Handle out-of-sequence measurement (OOSM)
        %   Choose out-of-sequence measurement (OOSM) handling technique
        %   option from [{'Terminate'},'Neglect','Retrodiction']. 
        %   Each detection has a timestamp associated with it, td, and the
        %   tracker has it own timestamp, tt, which is updated with every
        %   call to step. A measurement is considered to be an OOSM if td <
        %   tt.
        %   You can select how the tracker handles the OOSM:
        %     'Terminate'    - The tracker terminates its step on any OOSM.
        %     'Neglect'      - The tracker neglects any OOSM but keeps running.
        %     'Retrodiction' - The tracker uses the retrodiction algorithm.
        OOSMHandling = 'Terminate'
    end
    
    properties(Nontunable)
        %Assignment Assignment algorithm name
        %   Specify the Assignment as one of the
        %   [{'MatchPairs'}|'Munkres'|'Jonker-Volgenant'|'Auction'|'Custom'].
        %   Munkres is the only assignment algorithm that guarantees
        %   optimality, but it is also the slowest, especially
        %   for large numbers of detections and tracks. The other
        %   algorithms do not guarantee optimality but can be faster for
        %   problems ranging from 20 tracks and detections and larger.
        %   Use 'Custom' to define your own assignment function and specify
        %   its name in the CustomAssignmentFcn property.
        %   MatchPairs uses the matchpairs function and is the fastest
        %   algorithm.
        %
        %   See also: matchpairs, assignmunkres, assignjv, assignauction
        %
        %Default: 'MatchPairs'
        Assignment = 'MatchPairs'
        
        %CustomAssignmentFcn Name of 'Custom' assignment function
        %   Specify the function name for the custom assignment. This
        %   function will be used only when Assignment = 'Custom'. The
        %   function must have the following syntax:
        %       [assignment, unTrs, unDets] = f(cost, costNonAssignment)
        %   <a href="matlab:edit('assignmunkres')">Example of valid assignment function.</a>
        CustomAssignmentFcn = ''
    end
    
    properties(Constant, Hidden)
        TrackLogicSet = matlab.system.StringSet({'History';'Score'})
    end
    
    properties(Nontunable)
        %TrackLogic  Type of track confirmation and deletion logic
        %   Specify the TrackLogic as [{'History'}|'Score']. The choices
        %   are:
        %     * 'History': track confirmation and deletion will be based on
        %       the number of times the track has been assigned to a
        %       detection in the last tracker updates.
        %     * 'Score': track confirmation and deletion will be based on
        %       the track score, also known as log-likelihood. A high score
        %       means that the track is more likely to be valid whereas a
        %       low score means that the track is more likely to be false.
        %
        %   Default: 'History'
        TrackLogic = 'History'
        
        %ConfirmationThreshold   Threshold for track confirmation
        %   Specify the threshold for track confirmation. The threshold
        %   depends on the type of track confirmation and deletion logic
        %   you use:
        %     * History: specify the confirmation threshold as [M N], where
        %       a track will be confirmed if it receives at least M out of
        %       N updates.
        %     * Score: specify the confirmation threshold as a scalar. The
        %       track will be confirmed if its score is at least as high as
        %       the confirmation threshold.
        %
        %   Default:
        %       History logic:  [2 3]
        %       Score logic:    20
        ConfirmationThreshold
        
        %DeletionThreshold   Threshold for track deletion
        %   Specify the threshold for track deletion. The threshold depends
        %   on the type of track confirmation and deletion logic you use:
        %     * History: specify the deletion threshold as [P R], where a
        %       track will be deleted if in the last R updates, at least P
        %       times it was not assigned to any detection.
        %     * Score: a track will be deleted if its score decreases by at
        %       least the threshold from the maximum track score.
        %
        %   Default:
        %       History logic:  [5 5]
        %       Score logic:    -7
        DeletionThreshold
    end
    
    properties(Nontunable)
        %DetectionProbability Probability of detection used for track score
        %   Specify the probability of detection expected for the track as
        %   a scalar in the range (0,1)
        %   The probability of detection is used in calculating the track
        %   score when initializing and updating a track.
        %
        %   Default = 0.9
        DetectionProbability = 0.9
        
        %FalseAlarmRate   Rate of false positives used for track score
        %   Specify the expected rate of false positives as a scalar in the
        %   range (0,1)
        %   The false alarm rate is used in calculating the track score
        %   when initializing and updating a track.
        %
        %   Default = 1e-6
        FalseAlarmRate = 1e-6
        
        %Beta   Rate of new tracks per unit volume
        %   Specify the rate of new tracks per unit volume as a positive
        %   value.
        %   The rate of new tracks is used in calculating the track score
        %   during track initialization.
        %
        %   Default = 1
        Beta = 1
        
        %Volume Volume of the sensor's detection bin
        %   Specify the volume of the sensor measurement bin as a positive
        %   scalar.
        %   For example, if a radar produces a 4-D measurement, which
        %   includes azimuth, elevation, range, and range-rate, the 4-D
        %   volume is defined by the radar beam width in angle and the
        %   range and range rate bin widths.
        %   The volume is used in calculating the track score when
        %   initializing and updating a track.
        %
        %   Default = 1
        Volume = 1
    end    
    
    properties (Nontunable)
        %Confirmation threshold [positive scalar]
        pScoreConfThreshold = 20
        
        %Deletion threshold [negative scalar]
        pScoreDelThreshold = -7
    end
    
    properties (Access = protected, Nontunable)
        %pTrackLogic       The type of track logic, in value
        %   1 - History
        %   2 - Score
        pTrackLogic = 1
    end    

    properties (Nontunable, Logical)
        % EnableMemoryManagement Enable memory management properties Set
        % EnableMemoryManagement to true to specify bounds for certain
        % variable-sized arrays used inside the tracker through
        % MaxNumDetectionsPerSensor, MaxNumDetectionsPerCluster and
        % MaxNumTracksPerCluster properties. Specifying bounds allow you to
        % manage the memory footprint of the tracker in generated C/C++
        % code.
        % 
        % Default: false
        EnableMemoryManagement = false;
    end

    properties (Nontunable)
        % AssignmentClustering Configure clusters of detections and tracks
        % Specify AssignmentClusters as one of [{'off'},'on'].
        %
        % If specified as 'off', the tracker solves the global nearest neighbor
        % assignment problem per sensor using a cost matrix of size determined by
        % number of detections in the input and number of tracks maintained by the
        % tracker. Forbidden assignments (cost greater than AssignmentThreshold)
        % have infinite cost of assignment.
        %
        % If AssignmentClustering is specified as 'on', the tracker uses
        % forbidden assignments to break the assignment problem into smaller
        % "clusters". A cluster is a collection of detections (per sensor) and
        % tracks which can be assigned to each other. In this case, the tracker
        % solves the global nearest neighbor assignment problem per
        % cluster. Specifying AssignmentClustering as 'on' also allows you
        % to manage the memory footprint of the tracker in generated C/C++
        % code. The memory management properties can be enabled by setting
        % EnableMemoryManagement property as true.
        % 
        % Default: 'off'
        AssignmentClustering = 'off';

        % MaxNumDetectionsPerSensor Maximum number of detections per sensor
        % Set the maximum number of detections per sensor that can be
        % passed as an input to the tracker as a positive integer. Setting
        % a finite value allows the tracker to more efficiently manage the
        % memory in generated C/C++ code.
        %
        % This property is active when EnableMemoryManagement is true.
        %
        % Default: 100
        MaxNumDetectionsPerSensor = 100;

        % MaxNumDetectionsPerCluster Maximum number of detections per
        % cluster Set the maximum number of detections per cluster that can
        % be expected during run-time as a positive integer. Setting a
        % finite value allows the tracker to bound cluster sizes and reduce
        % the memory footprint of the tracker in generated C/C++ code. If
        % during run-time, the number of detections in a cluster exceeds
        % MaxNumDetectionsPerCluster, the behavior of the tracker is
        % determined by ClusterViolationHandling property.
        %
        % This property is active when AssignmentClustering is 'on' and
        % EnableMemoryManagement is true.
        %
        % Default: 5
        MaxNumDetectionsPerCluster = 5;

        % MaxNumTracksPerCluster Maximum number of tracks per cluster
        % Set the maximum number of tracks per cluster that can be
        % expected during run-time as a positive integer. Setting
        % a finite value allows the tracker to bound cluster sizes and
        % reduce the memory footprint of the tracker in generated C/C++
        % code. If during run-time, the number of tracks in a cluster
        % exceeds MaxNumTracksPerCluster, the behavior of the tracker is
        % determined by ClusterViolationHandling property.
        %
        % This property is active when AssignmentClustering is 'on' and
        % EnableMemoryManagement is true.
        % 
        % Default: 5
        MaxNumTracksPerCluster = 5;

        % ClusterViolationHandling Handle cluster size violations
        % Specify ClusterViolationHandling as one of 
        % ['Terminate'|{'Split and warn'}|'Split'].
        %
        % If specified as 'Terminate', the tracker errors out if cluster
        % bounds specified by MaxNumDetectionsPerCluster and
        % MaxNumTracksPerCluster are violated during run-time. 
        %
        % If specified as 'Split and warn', the tracker splits the
        % size-violating cluster into smaller clusters by using a
        % suboptimal approach. The tracker also throws a warning to
        % indicate a violation. 
        % 
        % If specified as 'Split', the tracker splits the size-violating
        % cluster into smaller clusters by using a suboptimal approach
        % without any warning.
        %
        % This property is active when AssignmentClustering is 'on' and
        % EnableMemoryManagement is true.
        % 
        % Default: 'Split and warn'
        ClusterViolationHandling = 'Split and warn';
    end

    properties (Constant, Hidden)
        AssignmentClusteringSet = matlab.system.StringSet({'off','on'});
        ClusterViolationHandlingSet = matlab.system.StringSet({'Terminate','Split and warn','Split'});
    end

    properties (Nontunable, Access = protected)
        pClusterViolationHandlingType
    end
    
    %% Methods
    methods
        % Constructor
        function obj = trackerGNN(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
        %------------------------------------------------------------------
        function set.TrackLogic(obj,value)
            obj.TrackLogic = value;
            setTrackLogic(obj)
        end
        %------------------------------------------------------------------
        
        function set.ConfirmationThreshold(obj, value)
            setConfThreshold(obj,value)
            obj.ConfirmationThreshold = value;
        end
        %------------------------------------------------------------------
        
        function val = get.ConfirmationThreshold(obj)
            if strcmpi(obj.TrackLogic, 'History')
                val = obj.pHistoryConfThreshold;
            else
                val = obj.pScoreConfThreshold;
            end
        end
        %------------------------------------------------------------------
        
        function set.DeletionThreshold(obj, value)
            setDelThreshold(obj,value);
            obj.DeletionThreshold = value;
        end
        %------------------------------------------------------------------
        
        function val = get.DeletionThreshold(obj)
            if strcmpi(obj.TrackLogic, 'History')
                val = obj.pHistoryDelThreshold;
            else
                val = obj.pScoreDelThreshold;
            end
        end
        %------------------------------------------------------------------
        
        function set.MaxNumTracks(obj, value)
            validateattributes(value, {'numeric'}, ...
                {'positive', 'real', 'integer', 'scalar'}, ...
                'trackerGNN', 'MaxNumTracks');
            obj.MaxNumTracks = value;
        end
        %------------------------------------------------------------------
        function set.pScoreConfThreshold(obj, value)
            validateattributes(value, {'numeric'}, ...
                {'real', 'positive', 'finite', 'nonsparse', 'scalar'}, ...
                'trackerGNN', 'ConfirmationThreshold');
            obj.pScoreConfThreshold = value;
        end
        %------------------------------------------------------------------
        function set.pScoreDelThreshold(obj, value)
            validateattributes(value, {'numeric'}, ...
                {'real', 'finite', 'nonsparse', 'scalar', '<', 0}, ...
                'trackerGNN', 'DeletionThreshold');
            obj.pScoreDelThreshold = value;
        end

        % Memory Management Setters
        function set.MaxNumDetectionsPerSensor(obj, val)
            validateMaxNumDetectionsPerSensor(obj, val);
            obj.MaxNumDetectionsPerSensor = val;
        end

        function set.MaxNumDetectionsPerCluster(obj, val)
            validateMaxNumDetectionsPerCluster(obj, val);
            obj.MaxNumDetectionsPerCluster = val;
        end

        function set.MaxNumTracksPerCluster(obj, val)
            validateMaxNumTracksPerCluster(obj, val);
            obj.MaxNumTracksPerCluster = val;
        end
        %------------------------------------------------------------------
        function values = getTrackFilterProperties(obj, trackID, varargin)
            %getTrackFilterProperties Returns the values of filter properties
            %   values = getTrackFilterProperty(tracker, trackID, properties)
            %   returns the values of the tracking filter properties for
            %   the track whose trackID is given. TrackID must be a valid
            %   TrackID as reported for the tracks in the previous call to
            %   step. Properties is a list of valid names of filter
            %   properties. The returned output, values, is a cell array in
            %   the same order as the list of properties.
            %
            %   Example:
            %   --------
            %   tracker = trackerGNN;
            %   detection = objectDetection(0, [0;0;0]);
            %   [~, tracks] = tracker(detection, 0);
            %   values = getTrackFilterProperties(tracker, 1, 'MeasurementNoise', 'ProcessNoise');
            
            % Method overridden for help text only. Redirects to the superclass
            values = getTrackFilterProperties@matlabshared.tracking.internal.fusion.GNNTracker(obj, trackID, varargin{:});
        end
        %------------------------------------------------------------------
        
        function setTrackFilterProperties(obj, trackID, varargin)
            %setTrackFilterProperties Sets values to filter properties
            %   setTrackFilterProperty(tracker, trackID, 'Name', value)
            %   Use Name-Value pairs to set properties for a track with the
            %   given trackID. TrackID must be a valid TrackID as reported
            %   for the tracks in the previous call to step. You
            %   can specify as many name-value pairs as you wish. Property
            %   names must match the names of public filter properties.
            %
            %   Example:
            %   --------
            %   tracker = trackerGNN;
            %   detection = objectDetection(0, [0;0;0]);
            %   [~, tracks] = tracker(detection, 0);
            %   setTrackFilterProperties(tracker, 1, 'MeasurementNoise', 2, 'ProcessNoise', 5);
            %   values = getTrackFilterProperties(tracker, 1, 'MeasurementNoise', 'ProcessNoise');
            
            % Method overridden for help text only. Redirects to the superclass
            setTrackFilterProperties@matlabshared.tracking.internal.fusion.GNNTracker(obj, trackID, varargin{:});
        end
    end
    
    %% Methods that must be implemented to work with the superclass
    methods (Access = protected)
        function validateFilterOnSetup(~, f)
            cond = isa(f, 'matlabshared.tracking.internal.AbstractTrackingFilter');
            if ~cond
                coder.internal.error('fusion:trackerGNN:InvalidFilter', class(f), mfilename);
            end
        end
        %------------------------------------------------------------------
        
        function initializeTrackDetectability(obj)
            % obj.DetectionProbability is not used when TrackLogic is
            % 'History' (obj.pTrackLogic is 1). So, call the base class in
            % that case and set it to DetectionProbably if not.
            if obj.pTrackLogic == 1
                initializeTrackDetectability@matlabshared.tracking.internal.fusion.GNNTracker(obj)
            else
                obj.pTrackDetectionProbability = obj.DetectionProbability * ones(obj.MaxNumTracks,1,obj.pClassToUse);
            end
        end        
        %------------------------------------------------------------------              
        function setConfThreshold(obj,value)
            if strcmpi(obj.TrackLogic, 'History')
                validateattributes(value, {'numeric'},{},'trackerGNN','ConfirmationThreshold');
                if isscalar(value)
                    modval = [value,value];
                else
                    modval = value;
                end
                validateattributes(modval, {'numeric'}, ...
                    {'real', 'positive', 'nonsparse', 'integer', 'nondecreasing', 'vector', 'numel', 2}, ...
                    'trackerGNN', 'ConfirmationThreshold');
                obj.pHistoryConfThreshold = modval;
            else
                validateattributes(value, {'numeric'}, ...
                    {'real', 'positive', 'finite', 'nonsparse', 'scalar'}, ...
                    'trackerGNN', 'ConfirmationThreshold');
                obj.pScoreConfThreshold = value;
            end
            updateConfirmationThreshold(obj);
        end
        %------------------------------------------------------------------
        
        function setDelThreshold(obj,value)
            if strcmpi(obj.TrackLogic, 'History')
                validateattributes(value, {'numeric'},{},'trackerGNN','DeletionThreshold');
                if isscalar(value)
                    modval = [value, value];
                else
                    modval = value;
                end
                validateattributes(modval, {'numeric'}, ...
                    {'real', 'positive', 'nonsparse', 'integer', 'nondecreasing', 'numel', 2}, ...
                    'trackerGNN', 'DeletionThreshold');
                obj.pHistoryDelThreshold = modval;
            else
                validateattributes(value, {'numeric'}, ...
                    {'real', 'finite', 'nonsparse', 'scalar', '<', 0}, ...
                    'trackerGNN', 'DeletionThreshold');
                obj.pScoreDelThreshold = value;
            end
            updateDeletionThreshold(obj);
        end
        %------------------------------------------------------------------
        
        function setTrackLogic(obj)
            if strcmpi(obj.TrackLogic, 'History')
                obj.pTrackLogic = 1;
            else
                obj.pTrackLogic = 2;
            end
        end
        %------------------------------------------------------------------
        
        function setupAssigner(obj)
            [maxNumDetsPerCluster, maxNumTracksPerCluster] = getClusterBounds(obj);
            obj.cAssigner = matlabshared.tracking.internal.fusion.AssignerGNN(...
                'Assignment', obj.Assignment, ...
                'CustomAssignmentFcn', obj.CustomAssignmentFcn, ...
                'AssignmentThreshold', obj.AssignmentThreshold, ... 
                'AssignmentClustering',obj.AssignmentClustering,...
                'MaxNumRowsPerCluster',maxNumTracksPerCluster,...
                'MaxNumColumnsPerCluster',maxNumDetsPerCluster,...
                'MaxNumRows',obj.MaxNumTracks,...
                'MaxNumColumns',getMaxNumDetectionsPerSensor(obj),...
                'ClusterViolationHandlingType',obj.pClusterViolationHandlingType,...
                'ClusterViolationMessageIdentifiers',getClusterViolationMessages(obj));            
        end

        function msgs = getClusterViolationMessages(obj)
            msgs = cell(2,1);
            msgs{1} = obj.getTrackClusterViolationMsg();
            msgs{2} = obj.getDetectionClusterViolationMsg();
        end
    end
    
    %%
    methods(Access = protected)
        %% Common functions
        
        function validatePropertiesImpl(obj)
            % Validate related or interdependent property values
            
            validatePropertiesImpl@matlabshared.tracking.internal.fusion.GNNTracker(obj);
            validatePropertiesImpl@fusion.internal.TrackerMemoryManagementUtilities(obj);
            validateTrackerProperties(obj);            
        end
        
        function validateTrackerProperties(obj)
            % Check DetectionProbability and FalseAlarmRate when TrackLogic
            % is 'Score'
            if strcmpi(obj.TrackLogic,'Score')
                validateattributes(obj.DetectionProbability, {'double','single'},...
                    {'real','positive','scalar','nonsparse','finite','<',1},mfilename,'DetectionProbability')
                validateattributes(obj.FalseAlarmRate, {'double','single'},...
                    {'real','positive','scalar','nonsparse','finite','<',1},mfilename,'FalseAlarmRate')
            end

            % Check that cost matrix input is disabled with retrodiction
            coder.internal.errorIf(strcmpi(obj.OOSMHandling,'Retrodiction') ...
                && obj.HasCostMatrixInput, 'fusion:trackerGNN:RetrodictionWithCostMatrix', ...
                'OOSMHandling', 'Retrodiction', 'HasCostMatrixInput');
        end

        function setupImpl(obj, detections, varargin)
            % Set up cluster violation handling type
            if ~coder.internal.is_defined(obj.pClusterViolationHandlingType)
                obj.pClusterViolationHandlingType = getClusterViolationHandlingType(obj);
            end

            % Setup the tracker
            setupImpl@matlabshared.tracking.internal.fusion.GNNTracker(obj, detections, varargin{:});
        end

        %------------------------------------------------------------------
        function varargout = stepImpl(obj, detections, varargin)
            %STEP  Creates, updates, and deletes the tracks
            %   The step method is responsible for managing all the
            %   tracks:
            %   1. The method attempts to assign the detections to existing
            %      tracks.
            %   2. New tracks are created based on unassigned detections.
            %   3. Tracks that are assigned to detections are updated and
            %      confirmed.
            %   4. Tracks that are not assigned to detections are coasted
            %      (predicted but not corrected) and eventually deleted.
            %
            %   confirmedTracks = STEP(tracker, detections, time)
            %   Update the tracker with a list of detections to the time
            %   instance specified by time. It returns an array of confirmed
            %   tracks corrected and predicted to the time instance. See the
            %   track output below for description of confirmedTracks.
            %
            %   ... = STEP(..., costMatrix)
            %   HasCostMatrixInput must be true to use this syntax. The
            %   cost matrix must have rows in the same order as the list of
            %   tracks and columns in the same order as the list of
            %   detections. Use the third output of step to get the correct
            %   order of the tracks list. If this is the first call to step
            %   or if there are no previous tracks, the cost matrix should
            %   have a size of [0,numDetections]. Note that a lower cost
            %   value indicates a higher likelihood of assigning a
            %   detection to a track. You may use inf to indicate that
            %   certain detections must not be assigned to certain tracks.
            %
            %   ... = STEP(..., detectableTrackIDs)
            %   HasDetectableTrackIDsInput must be true to use this syntax. The
            %   detectableTrackIDs must be an M-by-1 or M-by-2 matrix. The
            %   first column is a list of track IDs that the sensors report as
            %   detectable. The optional second column is the corresponding
            %   probability of detection, if reported by the sensors. If not
            %   reported, the DetectionProbability property is used.
            %   Tracks, whose IDs are not part of detectableTrackIDs input, are
            %   considered as undetectable, meaning no 'miss' is counted
            %   against them if they are not assigned a detection.
            %
            %   [confirmedTracks, tentativeTracks, allTracks] = STEP(...)
            %   in addition, returns a list of the tentative tracks and all
            %   the tracks. Use allTracks if you want to provide a cost matrix
            %   as it preserves the order of the tracks as required for the
            %   cost matrix input.
            %
            %   [..., analysisInformation] = STEP(...)
            %   in addition, returns a struct of analysis information.
            %
            %   Inputs:
            %     tracker            - a trackerGNN
            %     detections         - a cell array of objectDetection objects
            %     time               - the time to which all the tracks will be
            %                          updated and predicted. A real numeric
            %                          scalar, must be greater than the value
            %                          in the previous call.
            %     costMatrix         - the cost of assigning detections to
            %                          tracks. A real T-by-D matrix, where T is
            %                          the number of allTracks in the previous
            %                          call to step and D is the number of
            %                          detections in the current call. Higher
            %                          cost values mean lower likelihood of
            %                          assignment.
            %     detectableTrackIDs - the IDs of tracks that sensors expect to
            %                          detect, reported as an M-by-1 or M-by-2
            %                          matrix. The first column is of track
            %                          IDs, as reported by the TrackID field of
            %                          the tracks output (see Output below).
            %                          The second column is optional and allows
            %                          you to add the detection probability for
            %                          each track.
            %   Output:
            %     tracks - the output depends on the environment:
            %       MATLAB           - An array of <a href="matlab:help objectTrack">objectTrack</a> objects.
            %       Code generation  - An array of struct with the same fields
            %                          as objectTrack properties.
            %       Simulink         - A 1x1 bus that contains MaxNumTracks
            %                          track buses, each with elements similar
            %                          to objectTrack.
            %
            %     analysisInformation - a struct of additional information that
            %     allows you to analyze the tracker update stages.
            %     It includes the following fields:
            %       OOSMDetectionIndices    - Indices of OOSM detections.
            %       TrackIDsAtStepBeginning - Track IDs when the step began.
            %       CostMatrix              - Cost of assignment matrix.
            %       Assignments             - Assignments returned from the
            %                                 assignment function.
            %       UnassignedTracks        - IDs of unassigned tracks.
            %       UnassignedDetections    - IDs of unassigned detections.
            %       InitiatedTrackIDs       - IDs of tracks initiated during
            %                                 the step.
            %       DeletedTrackIDs         - IDs of tracks deleted during the
            %                                 step.
            %       TrackIDsAtStepEnd       - Track IDs when the step ended.
            %
            %     If OOSMHandling is set to 'Retrodiction', an additional
            %     field is added to the analysisInformation and includes a
            %     struct with the following fields:
            %       DiscardedDetections  - Indices of detections with 
            %                              timestamp beyond the
            %                              MaxNumOOSMSteps.
            %       CostMatrix           - Assignment cost matrix for OOSMs.
            %       Assignments          - Assignment retrurned for OOSMs.
            %       UnassignedDetections - Indices of unassigned OOSMs.
            % 
            %     If AssignmentClustering is set to 'on' and
            %     EnableMemoryManagement is true, two additional fields are
            %     added to analysisInformation:
            %
            %       MaxNumDetectionsPerCluster - Maximum number of
            %                                    detections in the clusters
            %                                    generated during step.
            %       MaxNumTracksPerCluster     - Maximum number of tracks
            %                                    in the clusters generated
            %                                    during the step.
            %
            %   See also: trackerGNN, objectDetection, objectTrack
            
            % Method not overriden to provide the correct help.
            [varargout{1:nargout}] = stepImpl@matlabshared.tracking.internal.fusion.GNNTracker(obj, detections, varargin{:});
        end

        function oosmHandlingInfo = exerciseOOSMHandler(obj, time, varargin)
            if strcmpi(obj.OOSMHandling,'Retrodiction')
                % Prepare the handler
                obj.cOOSMHandler.Tracks = obj.pTracksList;
                obj.cOOSMHandler.UsedSensors = obj.pUsedSensors;
                obj.cOOSMHandler.NumLiveTracks = obj.pNumLiveTracks;
                
                % Step the handler
                oosmHandlingInfo = step(obj.cOOSMHandler, time, varargin{:});
                
                % Initiate new tracks with unassigned retrodicted detections
                dets = detections(obj.cDetectionManager);
                ud = oosmHandlingInfo.UnassignedDetections(oosmHandlingInfo.UnassignedDetections > 0);
                initiateTracks(obj, ud, dets);
                
                % Update list of confirmed tracks
                assignedTrackIDs = oosmHandlingInfo.Assignments(oosmHandlingInfo.Assignments(:,1) > 0,1);
                numAssigned = obj.IntNumel(assignedTrackIDs);
                for i = 1:numAssigned
                    ind = findTrackByID(obj, assignedTrackIDs(i));
                    obj.pConfirmedTracks(ind) = obj.pTracksList{ind}.IsConfirmed;
                end
            else
                oosmHandlingInfo = exerciseOOSMHandler@matlabshared.tracking.internal.fusion.GNNTracker(obj,time,varargin{:});
            end
        end
        
        function assignerInfo = getAssignerInfo(obj)
            if obj.cAssigner.pHasClustering && obj.EnableMemoryManagement
                assignerInfo = struct;
                assignerInfo.MaxNumDetectionsPerCluster = obj.cAssigner.CurrentMaxNumColumnsPerCluster;
                assignerInfo.MaxNumTracksPerCluster = obj.cAssigner.CurrentMaxNumRowsPerCluster;
            else
                assignerInfo = getAssignerInfo@matlabshared.tracking.internal.fusion.GNNTracker(obj);
            end
        end

        function numHistory = numHistorySteps(obj)
            if strcmpi(obj.OOSMHandling,'Retrodiction')
                numHistory = obj.MaxNumOOSMSteps * obj.MaxNumSensors;
            else
                numHistory = numHistorySteps@matlabshared.tracking.internal.fusion.GNNTracker(obj);
            end
        end
        
        function verifyOOSMHandling(obj,filter)
            if strcmpi(obj.OOSMHandling,'Retrodiction')
                coder.internal.assert(isa(filter,'matlabshared.tracking.internal.RetrodictionFilter'),...
                    'fusion:trackerGNN:expectedRetrodictionFilter','OOSMHandling','Retrodiction','FilterInitializationFcn');
            end
        end
        
        function setupOOSMHandler(obj)
            if strcmpi(obj.OOSMHandling,'Retrodiction')
                % Verify that the filter knows how to retrodict
                obj.cOOSMHandler = fusion.internal.OOSMRetrodictionGNN(...
                    'Tracks',obj.pTracksList,...
                    'MaxNumOOSMSteps',obj.MaxNumOOSMSteps,...
                    'DetectionManager',obj.cDetectionManager,...
                    'Assigner',obj.cAssigner,...
                    'Volume',obj.Volume,...
                    'DetectionProbability',obj.DetectionProbability,...
                    'FalseAlarmRate',obj.FalseAlarmRate,...
                    'HasCostMatrixInput',obj.HasCostMatrixInput,...
                    'UsedSensors',obj.pUsedSensors,...
                    'NumLiveTracks',obj.pNumLiveTracks,...
                    'MaxNumSensors',obj.MaxNumSensors,...
                    'HasMeasurementParameters',obj.pHasMeasurementParameters);
            end
        end

        function idx = selectDetections(obj, origSen, sensorID, insequence)
            nMax = getMaxNumDetectionsPerSensor(obj);
            idx = obj.IntFind(origSen == sensorID & insequence);
            coder.varsize('idx',[1 nMax],[0 1]);
            coder.internal.assert(numel(idx) <= nMax, 'fusion:internal:TrackerMemoryManagementUtilities:MaxNumDetectionsPerSensorViolation', sensorID);
        end

        %% Backup/restore functions
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj
            
            % Save the base class information
            s = saveObjectImpl@matlabshared.tracking.internal.fusion.GNNTracker(obj);
            
            % Score confirmation and deletion properties
            s.pScoreConfThreshold   = obj.pScoreConfThreshold;
            s.pScoreDelThreshold    = obj.pScoreDelThreshold;

            if isLocked(obj)
                s.pClusterViolationHandlingType = obj.pClusterViolationHandlingType;
            end
        end
        
        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s
            obj.pScoreConfThreshold         = s.pScoreConfThreshold;
            obj.pScoreDelThreshold          = s.pScoreDelThreshold;
            
            % Remove fields that are not used in previous releases
            fns = {'pDetections','pOriginatingSensor','pVersion','pMaxNumDetections'};
            for i = 1:numel(fns)
                if isfield(s, fns{i})
                    s = rmfield(s, fns{i});
                end
            end

            if wasLocked
                if isfield(s, 'pClusterViolationHandlingType')
                    obj.pClusterViolationHandlingType = s.pClusterViolationHandlingType;
                else % older release
                    obj.pClusterViolationHandlingType = fusion.internal.ClusterViolationHandlingType(2); % Default
                end
            end

            loadObjectImpl@matlabshared.tracking.internal.fusion.GNNTracker(obj,s,wasLocked);
        end
        
        function newTracker = cloneImpl(obj)
            %clone Creates a copy of the trackerGNN
            %   newTracker = clone(tracker) returns a copy of the tracker
            %   object.
            
            % Copy public properties
            newTracker = cloneImpl@matlabshared.tracking.internal.fusion.GNNTracker(obj);
            
            if ~coder.internal.is_defined(obj.cDetectionManager) ... %Only happens after setup
                && ~coder.target('MATLAB') % In codegen, these don't get set with the call to cloneImpl above
                % Confirmation and deletion properties:
                newTracker.pScoreConfThreshold   = obj.pScoreConfThreshold;
                newTracker.pScoreDelThreshold    = obj.pScoreDelThreshold;
                newTracker.pClusterViolationHandlingType = obj.pClusterViolationHandlingType;
            end
        end
        
        %% System Object methods
         function flag = isInputSizeMutableImpl(~, index)
            % Return false if input size is not allowed to change while
            % system is running
            flag = true; % All inputs except time are varsize
            if index == 2 % time is an input
                flag = false;
            end
         end                
        
        function list = oosmGroup(~)
            list = matlab.mixin.util.PropertyGroup({'OOSMHandling','MaxNumOOSMSteps'});
        end
        
        function groups = getPropertyGroupsImpl(obj)
            groups = getPropertyGroups(obj);
        end

        function groups = getPropertyGroups(obj)
            % Define property section(s) for display in Matlab
            
            trackerGroup = matlab.mixin.util.PropertyGroup(...
                {'TrackerIndex', 'FilterInitializationFcn','MaxNumTracks',...
                'MaxNumDetections','MaxNumSensors'});
            
            assignmentGroup = matlab.mixin.util.PropertyGroup({'Assignment', ...
                'CustomAssignmentFcn', 'AssignmentThreshold','AssignmentClustering'});
            
            logicGroup = matlab.mixin.util.PropertyGroup(...
                {'TrackLogic', 'ConfirmationThreshold', 'DeletionThreshold', ...
                'DetectionProbability', 'FalseAlarmRate', 'Volume', 'Beta'});
            
            ioGroup = matlab.mixin.util.PropertyGroup(...
                {'HasCostMatrixInput', 'HasDetectableTrackIDsInput', ...
                'StateParameters'});
            
            numGroup = matlab.mixin.util.PropertyGroup(...
                {'NumTracks', 'NumConfirmedTracks'});
           
            memGroup = getPropertyGroups@fusion.internal.TrackerMemoryManagementUtilities(obj);

            groups = [trackerGroup, assignmentGroup oosmGroup(obj), logicGroup, ioGroup, numGroup, memGroup];
        end
        
        function flag = isInactivePropertyImpl(obj, prop)
            % Return false if property is visible based on object
            % configuration, for the command line and System block dialog
            flag = isInactivePropertyImpl@matlabshared.tracking.internal.fusion.TrackManager(obj,prop);            
            
            flag = flag | (strcmp(prop, 'CustomAssignmentFcn') && ~strcmp(obj.Assignment,'Custom'));
            
            flag = flag | (strcmpi(prop, 'MaxNumOOSMSteps') && ~strcmpi(obj.OOSMHandling,'Retrodiction'));

            if ~strcmp(obj.TrackLogic,'Score') % Not a score track logic
                flag = flag | any(strcmp(prop, ...
                    {'DetectionProbability', 'FalseAlarmRate', 'Volume',...
                    'Beta','pScoreConfThreshold','pScoreDelThreshold'}));
            else
                flag = flag | any(strcmp(prop, {'pHistoryConfThreshold','pHistoryDelThreshold'}));
            end

            flag = flag || isInactivePropertyImpl@fusion.internal.TrackerMemoryManagementUtilities(obj,prop);
        end       
    end

    % Memory management methods
    methods (Access = protected)
        function tf = hasAssignmentClustering(obj)
            tf = strcmpi(obj.AssignmentClustering,'on');
        end

        function out = getMaxNumDetectionsPerSensor(obj)
            % Divert implementation to memory management to enable bounds
            % per sensor when possible.
            out = getMaxNumDetectionsPerSensor@fusion.internal.TrackerMemoryManagementUtilities(obj);
        end
    end

    methods(Static, Hidden)
        function flag = isAllowedInSystemBlock
            flag = false;
        end
    end
end

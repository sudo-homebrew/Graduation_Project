classdef (StrictDefaults) trackerTOMHT < matlabshared.tracking.internal.fusion.TrackManager ...
        & fusion.internal.ExportToSimulinkInterface
%trackerTOMHT Track-oriented multi-hypothesis tracker
%   tracker = trackerTOMHT creates a multi-hypothesis, multi-sensor,
%   multi-object tracker that uses track-oriented multi-hypothesis. The
%   trackerTOMHT initializes, confirms, corrects, predicts (performs
%   coasting) and deletes tracks. A track is created with a 'Tentative'
%   status, meaning that there is not enough evidence for the trackerTOMHT
%   to determine that the track is of a physical object. If enough
%   additional detections are assigned to the tentative track, its status
%   will change to 'Confirmed' (see ConfirmationThreshold). Alternatively,
%   a track will be confirmed if a detection with a nonzero ObjectClassID
%   value is assigned to it, as it means that the sensor is able to
%   classify the physical object.
%
%   tracker = trackerTOMHT('Name', value) creates a trackerTOMHT object by
%   specifying its properties as name-value pair arguments. Unspecified
%   properties have default values. See the list of properties below.
%
%   Step method syntax: click on <a href="matlab:help trackerTOMHT/stepImpl">step</a> for more details.
%
%   System objects may be called directly like a function instead of
%   using the step method. For example, y = step(obj) and y = obj() are
%   equivalent.
%
%   trackerTOMHT properties:
%     TrackerIndex                - Unique identifier of the tracker
%     FilterInitializationFcn     - A handle to a function that initializes
%                                   a tracking filter based on a detection
%     MaxNumTracks                - Define the maximum number of tracks
%     MaxNumDetections            - Define the maximum number of detections
%     MaxNumSensors               - Define the maximum number of sensors
%     OOSMHandling                - Handle out-of-sequence measurements (OOSM)
%     StateParameters             - Parameters defining the track state
%     MaxNumHypotheses            - Define the maximum number of hypotheses
%     MaxNumTrackBranches         - Define the maximum number of branches
%                                   (track hypotheses) per track
%     MaxNumHistoryScans          - Define the maximum number of scans kept
%                                   in the track history
%     AssignmentThreshold         - The threshold that controls the
%                                   assignment of detections to tracks
%     ConfirmationThreshold       - The required track score for confirmation
%     DeletionThreshold           - The drop from track maximum score before
%                                   deletion
%     DetectionProbability        - Probability of detection
%     FalseAlarmRate              - Rate of false positive detections
%     Beta                        - The rate of new tracks per unit volume
%     Volume                      - The volume of the sensor detection bin
%     MinBranchProbability        - Minimum global probability to avoid pruning
%     NScanPruning                - Choose n-scan pruning method
%     HasCostMatrixInput          - Provide cost matrix as an input
%     HasDetectableBranchIDsInput - Provide detectable branch IDs as an input
%     OutputRepresentation        - Choice of track output method
%     HypothesesToOutput          - An array of hypotheses to output
%     NumTracks                   - Number of tracks            (read only)
%     NumConfirmedTracks          - Number of confirmed tracks  (read only)
%
%   trackerTOMHT methods:
%     <a href="matlab:help trackerTOMHT/stepImpl">step</a>                      - Creates, updates, and deletes the tracks
%     getBranches               - Returns the current list of track branches
%     getTrackFilterProperties  - Returns the values of filter properties
%     setTrackFilterProperties  - Sets values to filter properties
%     predictTracksToTime       - Predicts the tracks to a time stamp
%     initializeTrack           - Initialize a new track
%     deleteTrack               - Delete a track
%     initializeBranch          - Initialize a new track branch
%     deleteBranch              - Delete a track branch
%     exportToSimulink          - Export the tracker to a Simulink model
%     release                   - Allows property value and input characteristics changes
%     clone                     - Creates a copy of the trackerTOMHT
%     isLocked                  - Locked status (logical)
%     reset                     - Resets states of the trackerTOMHT
%
%   % EXAMPLE: Construct a tracker and use it to track two objects
%
%   % Construct a trackerTOMHT object with the default constant
%   % velocity Kalman filter initialization function, initcvkf
%   tracker = trackerTOMHT('FilterInitializationFcn', @initcvkf, ...
%       'ConfirmationThreshold', 20, ...
%       'DeletionThreshold', -7, ...
%       'MaxNumHypotheses', 10);
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
%   See also: objectDetection, objectTrack, trackerGNN, trackerJPDA,
%   trackerPHD, assignTOMHT, trackBranchHistory, clusterTrackBranches,
%   compatibleTrackBranches, pruneTrackBranches

%   References:
%   [1] J.R. Werthmann, "A Step-by-Step Description of a Computationally
%       Efficient Version of Multiple Hypothesis Tracking", SPIE Vol. 1698
%       Signal and Processing of Small Targets, pp 288-300, 1992.
%   [2] Samuel Blackman and Robert Popoli, "Design and Analysis of Modern
%       Tracking Systems", Artech House, 1999.

% Copyright 2018-2021 The MathWorks, Inc.

%#codegen
%#function initsingerekf

    properties(Nontunable)
        %FilterInitializationFcn    Name of function to initialize filter
        %   Specify the function for initializing the tracking filter used
        %   by a new track.  The function must have the following syntax:
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
    
    % Tracker capacity
    properties(Nontunable)
        %MaxNumTracks   Maximum number of tracks
        %   Set the maximum number of tracks the tracker can maintain as a
        %   positive real integer.
        %
        %   Default: 100
        MaxNumTracks = 100
        
        %MaxNumTrackBranches  Maximum number of track branches per track
        %   Set the maximum number of track branches (hypotheses) allowed
        %   for each track. Higher values allow the tracker to maintain
        %   more ambiguity, but increase computational load.
        %
        %   Default: 3
        MaxNumTrackBranches (1, 1) {mustBePositive, mustBeInteger, mustBeNonsparse} = 3
        
        %MaxNumSensors Maximum number of sensors
        %   Set the maximum number of tracks the tracker can maintain as a
        %   positive real integer.
        %   This number must be greater than or equal to the highest
        %   SensorIndex value used in the detections input to the
        %   step method. This property determines how many sets of
        %   ObjectAttributes each track can have and is used in the
        %   management of track branches.
        %
        %   Default: 20
        MaxNumSensors = 20
        
        %MaxNumHypotheses   Maximum number of hypotheses to maintain
        %   Set the maximum number of global hypotheses maintained by the
        %   tracker in cases of ambiguity. Higher values allow the tracker
        %   to maintain more ambiguity, but increase computational load.
        %
        %   Default: 5
        MaxNumHypotheses (1, 1) {mustBePositive, mustBeInteger, mustBeNonsparse} = 5
        
        %MaxNumHistoryScans  Maximum number of scans maintained in the branch history
        %   Set the maximum number of scans maintained in the branch
        %   history. The number of track history scans is typically between
        %   2 and 6. Higher values increase computational load.
        %
        %   Default: 4
        MaxNumHistoryScans (1, 1) {mustBePositive, mustBeInteger, mustBeNonsparse} = 4
    end
    
    properties(Nontunable)
        %HasDetectableBranchIDsInput Enable detectable branch IDs input
        %   Set this property to true if you want to provide the list of
        %   detectable branch IDs. Use this list to inform the tracker of
        %   branches that the sensors expected to detect and, optionally,
        %   the probability of detection for each track ID.
        %
        %   Default: false
        HasDetectableBranchIDsInput (1, 1) logical = false
        
        %HasCostMatrixInput Enable cost matrix input
        %   Set this property to true if you want to provide the assignment
        %   cost matrix as an input in the call to step
        %
        %   Default: false
        HasCostMatrixInput (1, 1) logical = false
    end
    
    properties(Nontunable) % Assignment parameters
        %AssignmentThreshold Threshold for assigning detections to tracks
        %   Specify the threshold that controls the assignment of a
        %   detection to a track, the creation of a new branch from
        %   detection, and the creation of a new branch from unassigned
        %   track. Specify this value as a scalar or a vector [C1,C2,C3,C4].
        %   The values must satisfy the relation: C1 <= C2 <= C3 <= C4.
        %   C1 defines a distance that if a track has an assigned detection
        %   with lower distance than C1, the track is no longer considered
        %   unassigned and does not create an unassigned track branch.
        %   C2 defines a distance that if a detection has been assigned to
        %   a track with lower distance than C2, the detection is no longer
        %   considered unassigned and does not create a new track branch.
        %   C3 defines the maximum distance for assigning a detection to a
        %   track.
        %   C4 defines the combinations of {track, detection} for which an
        %   accurate normalized cost calculation is performed. Initially, a
        %   coarse estimate of the normalized distance is done, and only
        %   combinations whose distance is less than C4 are calculated
        %   accurately.
        %   If specified as a scalar, the specified value will be expanded
        %   to [0.3, 0.7, 1, Inf]*value.
        %   If specified as [C1,C2,C3], it will be expanded by adding a
        %   fourth element, C4=Inf*C3.
        %
        %   Tips:
        %   1) Increase the value C3 if there are detections that should be
        %   assigned to tracks but are not. Decrease the value if there are
        %   detections that are assigned to tracks they should not be
        %   assigned to (too far).
        %   2) Increasing the values C1 and C2 can help control the number
        %   of track branches that are created. However, doing so reduces
        %   the number of branches (hypotheses) each track has.
        %   3) Increase the values of C4 if there are combinations of track
        %   and detection that should be calculated for assignment but are
        %   not. Decrease it if cost calculation takes too much time.
        %
        %   Default: [0.3, 0.7, 1, Inf]*30.0
        AssignmentThreshold = [0.3, 0.7, 1, Inf]*30.0
    end
    
    % Pruning parameters
    properties(Nontunable)
        %ConfirmationThreshold   Threshold for track confirmation
        %   Specify the threshold for track confirmation as a positive
        %   scalar value. Any track with a score higher than this threshold
        %   will be confirmed.
        %
        %   Default: 20
        ConfirmationThreshold = 20
        
        %DeletionThreshold   Threshold for track deletion
        %   Specify the threshold for track deletion as a negative scalar
        %   value. A track will be deleted if its score decreases, relative
        %   to its maximum score, by more than this threshold.
        %
        %   Default: -7
        DeletionThreshold = -7
        
        %MinBranchProbability   Minimum probability required to keep a branch
        %   Define the minimum probability required to keep the branch.
        %   Any branch with probability lower than this will be pruned.
        %   Must be a real positive scalar less than 1. Typical values are
        %   0.001 to 0.005.
        %
        %   Default: 0.001
        MinBranchProbability = 0.001
    end
    properties(Hidden,Constant)
        OOSMHandlingSet = matlab.system.StringSet({'Terminate','Neglect'});
        NScanPruningSet = matlab.system.StringSet({'None','Hypothesis'});
    end
    
    properties(Nontunable)
        %OOSMHandling Handle out-of-sequence measurement (OOSM)
        %   Choose out-of-sequence measurement (OOSM) handling technique
        %   option from [{'Terminate'},'Neglect']. Each detection has a
        %   timestamp associated with it, td, and the tracker has it own
        %   timestamp, tt, which is updated with every call to step. A
        %   measurement is considered to be an OOSM if td < tt. 
        %   You can select how the tracker handles the OOSM:
        %     'Terminate' - The tracker terminates its step on any OOSM.
        %     'Neglect'   - The tracker neglects any OOSM but keeps running.
        OOSMHandling = 'Terminate'
        
        %NScanPruning - Choose n-scan pruning method
        %   Choose n-scan pruning option from [{'None'},'Hypothesis'].
        %   In n-scan pruning, branches that belong to the same track are
        %   pruned (deleted) if, in the n scans history, they contradict
        %   the most likely branch for the same track. The most likely
        %   branch is defined in one of two ways:
        %     'None'       - No N-scan pruning us done.
        %     'Hypothesis' - The branch is in the most likely hypothesis.
        %
        %   Default: 'None'
        NScanPruning = 'None'
    end
    
    % Parameters related to track score
    properties(Nontunable)
        %DetectionProbability  The probability of detection used for track score
        %   Specify the probability of detection expected for the track as
        %   a scalar in the range (0,1)
        %   The probability of detection is used in calculating the track
        %   score when initializing and updating a track.
        %
        %   Default = 0.9
        DetectionProbability = 0.9
        
        %FalseAlarmRate    Rate of false positives used for track score
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
        
        %Volume The volume of the sensor's detection bin
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
    
    properties(Hidden,Constant)
        %OutputRepresentationSet - A set of output methods
        OutputRepresentationSet = matlab.system.StringSet({'Hypothesis','Tracks','Clusters'});
    end
    
    properties(Nontunable)
        %OutputRepresentation - Choice of track output method
        %   Choose the output method from a list of the following methods:
        %     'Hypothesis'  - Outputs each branches that are in certain
        %                     hypotheses. If you choose this option, list
        %                     the hypotheses to output using the property
        %                     HypothesesToOutput.
        %     'Tracks'      - Outputs the centroid of each track based on
        %                     its track branches.
        %     'Clusters'    - Outputs the centroid of each cluster. Similar
        %                     to 'Tracks' output, but includes all the
        %                     tracks within a cluster.
        %
        %   Default: 'Tracks'
        OutputRepresentation = 'Tracks';
    end
    
    properties
        %HypothesesToOutput - An array of hypotheses to output
        %   Choose which hypotheses to output as an array of indices. The
        %   indices must all be less than or equal to the maximum number of
        %   hypotheses provided by the tracker. This property is tunable.
        %
        % Default: 1
        HypothesesToOutput = 1;
    end
    
    %% Internal properties
    properties(Access = {?trackerTOMHT, ?matlab.unittest.TestCase}) % Sub-objects used by the tracker
        %cCostCalculator An object responsible for calculating cost
        cCostCalculator
        
        %cBranchManager A track-oriented branch history manager
        cBranchManager
        
        %cOutputter     Provides MHT outputting
        cOutputter
    end
    
    properties(Access = protected)
        %pLastTimeStamp     Time stamp of the last tracker update
        pLastTimeStamp
        
        %pBranchIDs         A list of track branch IDs. For each track in
        %                   pTracksList, keep the associated track ID
        pBranchIDs
        
        %pHyps      Keeps the hypotheses matrix for predictTracksToTime
        pHyps
        
        %pClusters  Keeps the clusters matrix for predictTracksToTime
        pClusters
        
        %pProbs     Keeps the branch probabilities for predictTracksToTime
        pProbs
        
        %pWasDetectable Lets the tracker know which tracks were detectable
        %               by the sensors
        pWasDetectable
        
        %pTrackDetectionProbability Detection probability for tracks
        %   Used if DetectableTrackIDs is enabled.
        pTrackDetectionProbability
    end
    
    properties (Access=protected, Hidden)
        pVersion = ver('fusion');
    end
    
    properties(Access = {?trackerTOMHT, ?matlab.unittest.TestCase},Constant)
        %constAssignmentExpansion Expands a scalar AssignmentThreshold
        constAssignmentExpansion = [0.3 0.7 1 Inf];
    end    
    
    %% Common functions
    methods
        function obj = trackerTOMHT(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
    end
    
    %% Public methods
    methods
        function values = getTrackFilterProperties(obj, branchID, varargin)
            %getTrackFilterProperties Returns the values of filter properties
            %   values = getTrackFilterProperties(tracker,branchID,properties)
            %   returns the values of the tracking filter properties for
            %   the branch whose branchID is given. BranchID must be a
            %   valid BranchID as reported in the list of branches obtained
            %   by the getBranches method. Properties is a list of valid
            %   names of filter properties. The returned output, values, is
            %   a cell array in the same order as the list of properties.
            %
            %   Example:
            %   --------
            %   tracker = trackerTOMHT;
            %   detection = objectDetection(0, [0;0;0]);
            %   tracker(detection, 0); % Call the tracker to initiate a track
            %   branches = getBranches(tracker);
            %   branchID = branches(1).BranchID;
            %   values = getTrackFilterProperties(tracker, branchID, 'MeasurementNoise', 'ProcessNoise');
            validateattributes(branchID, {'numeric'}, ...
                {'real', 'positive', 'scalar', 'finite', 'integer'}, 'getTrackFilterProperties', ...
                'trackID');
            for i = 1:numel(varargin)
                validateattributes(varargin{i}, {'char','string'}, {'nonempty'}, ...
                    'getTrackFilterProperties', 'properties');
            end
            
            numProps = numel(varargin);
            values = cell(numProps, 1);
            index = findTrackByID(obj, branchID);
            if isempty(index)
                coder.internal.warning('fusion:trackerTOMHT:TrackIDNotFound', branchID);
                for i = 1:numProps
                    values{i} = [];
                end
            else
                for i = 1:numProps
                    values{i} = obj.pTracksList{index(1)}.Filter.(varargin{i});
                end
            end
        end
        %------------------------------------------------------------------
        
        function setTrackFilterProperties(obj, branchID, varargin)
            %setTrackFilterProperties Sets values to filter properties
            %   setTrackFilterProperties(tracker, branchID, 'Name', value)
            %   Use Name-Value pairs to set properties for a branch with
            %   the given branchID. BranchID must be a valid BranchID as
            %   reported by the getBranches method. You can specify as many
            %   name-value pairs as you wish. Property names must match the
            %   names of public filter properties.
            %
            %   Example:
            %   --------
            %   tracker = trackerTOMHT;
            %   detection = objectDetection(0, [0;0;0]);
            %   tracker(detection, 0); % Call the tracker to initiate a track
            %   branches = getBranches(tracker);
            %   branchID = branches(1).BranchID;
            %   setTrackFilterProperties(tracker, branchID, 'MeasurementNoise', 2, 'ProcessNoise', 5);
            %   values = getTrackFilterProperties(tracker, branchID, 'MeasurementNoise', 'ProcessNoise');
            validateattributes(branchID, {'numeric'}, ...
                {'real', 'positive', 'scalar', 'finite', 'integer'}, 'setTrackFilterProperties', ...
                'trackID');
            
            index = findTrackByID(obj, branchID);
            if isempty(index)
                coder.internal.warning('fusion:trackerTOMHT:TrackIDNotFound', branchID);
            else
                numProps = numel(varargin);
                coder.internal.errorIf(mod(numProps,2) > 0, 'fusion:trackerTOMHT:OddNameValuePairs');
                for i = 1:2:numProps % Note: property name and value validations are done by the filter
                    validateattributes(varargin{i}, {'char','string'}, {'nonempty'}, ...
                        'setTrackFilterProperties', 'Name');
                    filter = obj.pTracksList{index(1)}.Filter;
                    filter.(varargin{i}) = varargin{i+1};
                end
            end
        end
        %------------------------------------------------------------------
        
        function branches = getBranches(obj)
            %getBranches Get the current list of track branches
            %   branches = getBranches(tracker) returns the list of track
            %   branches maintained by the tracker. Use getBranches to
            %   obtain the current state list of branches before using
            %   other methods that require identifying a branch by its
            %   BranchID.
            %
            %   Note: the tracker must be updated at least once to be able
            %   to provide track branches. You can use isLocked(tracker) to
            %   check if the tracker has been updated.
            %
            %   Example:
            %   --------
            %   tracker = trackerTOMHT;
            %   detection = objectDetection(0, [0;0;0]);
            %   tracker(detection, 0); % Call the tracker to initiate a track
            %   branches = getBranches(tracker);
            
            % Has the tracker locked yet?
            coder.internal.assert(isLocked(obj),'fusion:trackerTOMHT:PredictBeforeUpdate','getBranches');
            
            list = true(obj.pNumLiveTracks,1);
            branches = getTracks(obj, list);
        end
        %------------------------------------------------------------------
        
        function predictedTracks = predictTracksToTime(obj,type,ID,time,varargin)
            %predictTracksToTime Predicts the tracks to a time stamp
            %   predictedTracks = predictTracksToTime(obj,type,ID,time)
            %   predicts the branch or track specified by ID to time
            %   instant, time. type must be either 'branch' or 'track'.
            %   time must be greater than the last update time provided to
            %   the tracker in the previous step.
            %
            %   ... = predictTracksToTime(obj,type,category,time) allows
            %   you to predict all the tracks that match the category.
            %   Valid values for category are 'all', 'confirmed', or
            %   'tentative'.
            %
            %   ... = predictTracksToTime(...,'WithCovariance',tf) allows
            %   you to specify whether the state covariance of each track
            %   should be predicted as well by setting the tf flag to true
            %   or false. Predicting the covariance is slower than
            %   predicting just the track states. The default is false.
            %
            % Note: the tracker must be updated at least once to be able to
            % predict tracks. You can use isLocked(tracker) to check if the
            % tracker has been updated.
            
            % Has the tracker locked yet?
            coder.internal.assert(isLocked(obj),'fusion:trackerTOMHT:PredictBeforeUpdate','predictTracksToTime');
            
            %Input validation
            narginchk(4,6);
            validateattributes(time, {'numeric'}, ...
                {'real', 'finite', 'nonsparse', 'scalar', '>', obj.pLastTimeStamp}, ...
                'predictTracksToTime', 'time');
            typ = validatestring(type,{'branch','track'},'predictTracksToTime','type');
            
            % Check flag if varargin is not empty
            if ~isempty(varargin)
                coder.internal.assert(numel(varargin)==2,'fusion:trackerTOMHT:PredictToTimeNargin',4,6)
                validatestring(varargin{1},{'WithCovariance'},mfilename);
                validateattributes(varargin{2},{'numeric','logical'},...
                    {'scalar','binary'},mfilename,'tf');
                withCov = varargin{2};
            else
                withCov = false;
            end
            
            if strcmpi(typ,'branch')
                if ischar(ID) || isstring(ID)
                    categ = validatestring(ID,{'all','confirmed','tentative'},...
                        'predictTracksToTime','category');
                    if strcmpi(categ,'all')
                        list = true(1,obj.pNumLiveTracks);
                    elseif strcmpi(categ,'confirmed')
                        list = obj.pConfirmedTracks;
                    else
                        list = ~obj.pConfirmedTracks(1:obj.pNumLiveTracks);
                    end
                else
                    validateattributes(ID, {'numeric'}, ...
                        {'real', 'positive', 'scalar', 'integer'}, ...
                        'predictTracksToTime', 'ID');
                    list = false(1,obj.pNumLiveTracks);
                    ind = findTrackByID(obj, ID);
                    if isempty(ind)
                        coder.internal.warning('fusion:trackerTOMHT:TrackIDNotFound', ID);
                    else
                        list(ind) = true;
                    end
                end
                
                %We have a list of tracks we want to predict. Get them using
                %getTracks and then predict them
                predictedTracks = getTracks(obj, list);
                if isempty(predictedTracks)
                    return
                end
                
                if withCov
                    predictedTracks = predictTracksWithCov(obj,predictedTracks,time);
                else
                    predictedTracks = predictTracksWithoutCov(obj,predictedTracks,time);
                end
            else % By consistent representation of tracks
                if ischar(ID) || isstring(ID) % By category
                    categ = validatestring(ID,{'all','confirmed','tentative'},...
                        'predictTracksToTime','category');
                    list = true(1,obj.pNumLiveTracks); % All branches must be passed
                else % By track ID: get the cluster for that track ID.
                    categ = 'ID';
                    validateattributes(ID, {'numeric'}, ...
                        {'real', 'positive', 'scalar', 'integer'}, ...
                        'predictTracksToTime', 'ID');
                    list = (obj.pTrackIDs(1:obj.pNumLiveTracks)==ID);
                    if ~any(list)
                        coder.internal.warning('fusion:trackerTOMHT:TrackIDNotFound', ID);
                    end
                end
                
                %We have a list of tracks we want to predict. Get them using
                %getTracks and then predict them
                predictedBranches = getTracks(obj, list);
                if isempty(predictedBranches)
                    % That's OK, because it is an empty struct with the
                    % correct fields
                    predictedTracks = predictedBranches;
                    return
                end
                
                if withCov
                    predictedBranches = predictTracksWithCov(obj,predictedBranches,time);
                else
                    predictedBranches = predictTracksWithoutCov(obj,predictedBranches,time);
                end
                
                switch lower(categ)
                    case 'confirmed'
                        predictedTracks = obj.cOutputter(predictedBranches,obj.pHyps,obj.pProbs,obj.pClusters);
                    case 'tentative'
                        [~,predictedTracks] = obj.cOutputter(predictedBranches,obj.pHyps,obj.pProbs,obj.pClusters);
                    case 'all'
                        [~,~,predictedTracks] = obj.cOutputter(predictedBranches,obj.pHyps,obj.pProbs,obj.pClusters);
                    otherwise % This is by ID.
                        predictedTracks = predictedBranches;
                end
            end
        end
        function deleted = deleteTrack(obj, trackID)
            %deleteTrack  Delete a track managed by the tracker
            %    deleted = deleteTrack(obj,trackID) deletes the track
            %    specified by trackID and all its branches from the
            %    tracker. The deleted flag returns true if a track with the
            %    same trackID existed and was deleted. If a track with that
            %    trackID did not exist, the deleted flag is false and a
            %    warning is issued.
            %
            % Note: the tracker must be updated at least once to be able to
            % delete a track.
            
            % Has the tracker locked yet?
            coder.internal.assert(isLocked(obj),'fusion:trackerTOMHT:PredictBeforeUpdate','deleteTrack');
            
            %Input validation
            narginchk(2,2);
            validateattributes(trackID, {'numeric'}, ...
                {'real', 'positive', 'scalar', 'integer'}, 'deleteTrack', ...
                'trackID');
            
            % 1. Create a list of branches that have to be deleted
            toDelete1 = find(obj.pTrackIDs == trackID);
            
            % Warning if not deleted
            if isempty(toDelete1)
                coder.internal.warning('fusion:trackerTOMHT:TrackIDNotFound', trackID);
                deleted = false;
                return
            end
            
            % 2. Create a list of trackBranchHistory rows to be deleted
            history = getHistory(obj.cBranchManager,'Matrix');
            history(toDelete1,:) = [];
            obj.cBranchManager.TrackHistory = history;
            
            % 3. Delete the track branches from track list and history
            deleteTracks(obj,obj.pBranchIDs,toDelete1,true);
            deleted = true;
        end
        function deleted = deleteBranch(obj, branchID)
            %deleteBranch  Delete a track branch managed by the tracker
            %    deleted = deleteBranch(obj,branchID) deletes the track
            %    branch specified by branchID from the tracker.
            %    The deleted flag returns true if a track with the same
            %    branchID existed and was deleted. If a track with that
            %    branchID did not exist, the deleted flag is false and a
            %    warning is issued.
            %
            % Note: the tracker must be updated at least once to be able to
            % delete a track.
            
            % Has the tracker locked yet?
            coder.internal.assert(isLocked(obj),'fusion:trackerTOMHT:PredictBeforeUpdate','deleteBranch');
            
            %Input validation
            narginchk(2,2);
            validateattributes(branchID, {'numeric'}, ...
                {'real', 'positive', 'scalar', 'integer'}, 'deleteBranch', ...
                'branchID');
            
            % 1. Create a list of branches that have to be deleted
            toDelete1 = find(obj.pBranchIDs == branchID);
            
            % Warning if not deleted
            if isempty(toDelete1)
                coder.internal.warning('fusion:trackerTOMHT:TrackIDNotFound', branchID);
                deleted = false;
                return
            end
            
            % 2. Create a list of trackBranchHistory rows to be deleted
            history = getHistory(obj.cBranchManager,'Matrix');
            history(toDelete1,:) = [];
            obj.cBranchManager.TrackHistory = history;
            
            % 3. Delete the track branches from track list and history
            deleteTracks(obj,obj.pBranchIDs,toDelete1,true);
            deleted = true;
        end
        function trackID = initializeTrack(obj, track, varargin)
            %initializeTrack Initialize a new track in the tracker
            %   id = initializeTrack(obj,track) initializes a new
            %   track in the tracker using the track and returns
            %   the TrackID associated with it. track must be an
            %   objectTrack object or a struct with similar fields, with
            %   properties of the same size and type as the ones returned
            %   by the tracker.
            %   You may use this syntax if the filter is any of the
            %   following types: trackingABF, trackingKF, trackingEKF,
            %   trackingUKF, trackingCKF, or trackingMSCEKF.
            %
            %   id = initializeTrack(obj,track,filter)   initializes a new
            %   track in the tracker using a filter, which must be a
            %   filter of the same type (including sizes and types)
            %   as the FilterInitializationFcn property returns.
            %   You must uses this syntax if the filter is either
            %   trackingPF, trackingIMM, or trackingGSF.
            %
            %   A warning is issued if the tracker already maintains
            %   MaxNumTracks tracks and the returned id is zero, which
            %   indicates a failure to initialize the track.
            %
            % Notes:
            %   1. The tracker must be updated at least once to be able to
            %      initialize a track.
            %   2. The tracker assigns a TrackID to the track, gives an
            %      UpdateTime equal to the last step time, and synchronizes
            %      the data in the given track to the initialized track.
            
            % Validate inputs
            narginchk(2,3);
            trackObj = validateInitInputs(obj,track,'initializeTrack',varargin{:});
            
            trackObj.UpdateTime = obj.pLastTimeStamp;
            % Get last track ID and last branch ID from the branch manager
            [trackID, branchID] = getLastIDs(obj.cBranchManager);
            trackID = trackID + 1;
            
            tf = initializeTrack@matlabshared.tracking.internal.fusion.TrackManager(obj,trackID,trackObj,varargin{:});
            
            if tf
                branchID = branchID + 1;
                obj.pTracksList{obj.pNumLiveTracks}.BranchID = branchID;
                obj.pBranchIDs(obj.pNumLiveTracks) = branchID;
                
                % Add a row in the history to reflect this track and branch
                history = getHistory(obj.cBranchManager,'Matrix');
                newHistoryLine = zeros(1,size(history,2),'like',history);
                newHistoryLine(1,1:3) = [trackID, 0, branchID];
                obj.cBranchManager.TrackHistory = [history;newHistoryLine];
                
                % Set last track ID and last branch ID back to branch manager
                setLastIDs(obj.cBranchManager,trackID,branchID)
            else
                trackID = zeros(1,'like',trackID);
                coder.internal.warning('fusion:trackerTOMHT:MaxNumTracksReached', 'MaxNumTracks');
            end
        end
        function branchID = initializeBranch(obj, trackID, branch, varargin)
            %initializeBranch Initialize a new track branch in the tracker
            %   id = initializeBranch(obj, trackID, branch) initializes a
            %   new track branch in the tracker. The branch is added to the
            %   track specified by trackID with the information contained
            %   in branch, which must be an objectTrack object or a struct
            %   with similar fields, with properties of the same size and
            %   type as the ones returned by the tracker. The initialized
            %   branch ID is returned.
            %   You may use this syntax if the filter is any of the
            %   following types: trackingABF, trackingKF, trackingEKF,
            %   trackingUKF, trackingCKF, or trackingMSCEKF.
            %
            %   id = initializeTrack(obj, trackID, branch, filter)
            %   initializes a new track branch in the tracker using a
            %   filter, which must be a filter of the same type (including
            %   sizes and types) as the FilterInitializationFcn property
            %   returns.
            %   You must uses this syntax if the filter is either
            %   trackingPF, trackingIMM, or trackingGSF.
            %
            %   A warning is issued if the tracker already maintains
            %   MaxNumTracks tracks and the returned id is zero, which
            %   indicates a failure to initialize the track branch.
            %
            % Notes:
            %   1. The tracker must be updated at least once to be able to
            %      initialize a track.
            %   2. The tracker assigns a BranchID to the branch, gives an
            %      UpdateTime equal to the last step time, and synchronizes
            %      the data in the given track to the initialized branch.
            
            % Input validation
            narginchk(3,4);
            trackObj = validateInitInputs(obj,branch,'initializeBranch',varargin{:});
            trackObj.UpdateTime = obj.pLastTimeStamp;
            validateattributes(trackID, {'numeric'}, ...
                {'real', 'positive', 'scalar', 'integer'}, 'initializeBranch', ...
                'trackID');
            
            % Validate that the track ID even exists in the tracker
            thisTrackBranches = (obj.pTrackIDs == trackID);
            if ~any(thisTrackBranches)
                coder.internal.warning('fusion:trackerTOMHT:TrackIDNotFound', trackID);
                branchID = zeros(1,'like',obj.pBranchIDs);
                return
            end
            
            % Validate that the number of branches for this track does not
            % exceed MaxNumBranches
            if sum(thisTrackBranches) >= obj.MaxNumTrackBranches
                coder.internal.warning('fusion:trackerTOMHT:MaxNumTracksReached', 'MaxNumTrackBranches');
                branchID = zeros(1,'like',obj.pBranchIDs);
                return
            end
            
            % Initialize the branch
            [~, branchID] = getLastIDs(obj.cBranchManager);
            tf = initializeTrackImpl(obj,trackID,trackObj,varargin{:});
            
            if tf
                branchID = branchID + 1;
                obj.pTracksList{obj.pNumLiveTracks}.BranchID = branchID;
                obj.pBranchIDs(obj.pNumLiveTracks) = branchID;
                
                % Add a row in the history to reflect this track and branch
                history = getHistory(obj.cBranchManager,'Matrix');
                newHistoryLine = zeros(1,size(history,2),'like',history);
                newHistoryLine(1,1:3) = [trackID, 0, branchID];
                obj.cBranchManager.TrackHistory = [history;newHistoryLine];
                
                % Set last track ID and last branch ID back to branch manager
                setLastIDs(obj.cBranchManager,trackID,branchID)
            else
                branchID = zeros(1,'like',trackID);
                coder.internal.warning('fusion:trackerTOMHT:MaxNumTracksReached', 'MaxNumTracks');
            end
        end
        function set.AssignmentThreshold(obj,value)
            if isscalar(value)
                validateattributes(value,{'numeric'},...
                    {'real','finite','nonsparse','vector','nrows',1},...
                    mfilename,'AssignmentThreshold');
                obj.AssignmentThreshold = value(1,1) * obj.constAssignmentExpansion;
            elseif numel(value)==3 % [C1,C2,C3]
                validateattributes(value,{'numeric'},...
                    {'real','finite','nonsparse','vector','nondecreasing','numel',3},...
                    mfilename,'AssignmentThreshold');
                obj.AssignmentThreshold = [value(:)',value(end)*obj.constAssignmentExpansion(end)];
            else % [C1,C2,C3,C4]
                validateattributes(value,{'numeric'},...
                    {'real','nonsparse','vector','nondecreasing','numel',4},...
                    mfilename,'AssignmentThreshold');
                validateattributes(value(1:3),{'numeric'},{'finite'},...
                    mfilename,'AssignmentThreshold(1:3)');
                obj.AssignmentThreshold = (value(:))';
            end
        end
        function set.DetectionProbability(obj,value)
            validateProbProps(obj,value,'DetectionProbability')
            obj.DetectionProbability = value;
        end
        function set.FalseAlarmRate(obj,value)
            validateProbProps(obj,value,'FalseAlarmRate')
            obj.FalseAlarmRate = value;
        end
        function set.MinBranchProbability(obj,value)
            validateProbProps(obj,value,'MinBranchProbability')
            obj.MinBranchProbability = value;
        end
        function set.HypothesesToOutput(obj,value)
            validateHypothesesToOutput(obj,value);
            obj.HypothesesToOutput = value;
        end
        function set.ConfirmationThreshold(obj,value)
            validateattributes(value, {'numeric'}, ...
                {'real','positive','finite','nonsparse','scalar'}, ...
                'trackerTOMHT', 'ConfirmationThreshold');
            obj.ConfirmationThreshold = value;
        end
        function set.DeletionThreshold(obj,value)
            validateattributes(value, {'numeric'}, ...
                {'real','finite','nonsparse','scalar','<',0,}, ...
                'trackerTOMHT', 'DeletionThreshold');
            obj.DeletionThreshold = value;
        end
    end
    %% System object functions
    methods(Access = protected)
        function validateProbProps(~,value,propName)
            validateattributes(value,{'double','single'},{'real','nonsparse',...
                'finite','positive','scalar','<',1},mfilename,propName)
        end
        function [conf,tent,all,info] = stepImpl(obj,detections,varargin)
            %STEP  Creates, updates, and deletes the tracks
            %   The step method is responsible for managing all the tracks:
            %     1. The method attempts to assign the detections to existing
            %        tracks.
            %     2. The trackerTOMHT allows for multiple hypotheses about the
            %        assignment of detections to tracks.
            %     3. Unassigned detections result in the creation of new
            %        tracks.
            %     4. Assignments of detections to tracks create new branches
            %        for the assigned tracks.
            %     5. Unassigned tracks are coasted (predicted).
            %     6. All the branches are scored and branches with low initial
            %        scores are pruned.
            %     7. Clusters of branches that share detections (incompatible
            %        branches) in their history are generated.
            %     8. Global hypotheses of compatible branches are formulated
            %        and scored.
            %     9. Branches are scored based on their existence in the global
            %        hypotheses. Low scored branches are pruned.
            %    10. Additional pruning is done based on N-scan history.
            %
            %   confirmedTracks = STEP(tracker, detections, time)
            %   Update the tracker with a list of detections to the time
            %   instance specified by time. It returns a struct array of
            %   confirmed tracks corrected and predicted to the time
            %   instance. See the track output below for description of
            %   confirmedTracks.
            %
            %   ... = STEP(..., costMatrix)
            %   HasCostMatrixInput must be true to use this syntax. The
            %   cost matrix must have rows in the same order as the list of
            %   branches and columns in the same order as the list of
            %   detections. Use the getBranches() method to get the correct
            %   order of the tracks list. If this is the first call to step
            %   or if there are no previous tracks, the cost matrix should
            %   have a size of [0,numDetections]. Note that a lower cost
            %   value indicates a higher likelihood of assigning a
            %   detection to a track. You may use inf to indicate that
            %   certain detections must not be assigned to certain tracks.
            %
            %   ... = STEP(..., detectableBranchIDs)
            %   HasDetectableBranchIDsInput must be true to use this
            %   syntax. The detectableBranchIDs must be an M-by-1 or M-by-2
            %   matrix. The first column is a list of branch IDs that the
            %   sensors report as detectable. The optional second column is
            %   the corresponding probability of detection, if reported by
            %   the sensors. If not reported, the DetectionProbability
            %   property is used. Branches, whose IDs are not part of
            %   detectableBranchIDs input, are considered as undetectable,
            %   meaning no 'miss' is counted against them if they are not
            %   assigned a detection.
            %
            %   [confirmedTracks, tentativeTracks, allTracks] = STEP(...)
            %   in addition, returns a list of the tentative tracks and all
            %   the tracks.
            %
            %   [..., analysisInformation] = STEP(...)
            %   in addition, returns a struct of analysis information.
            %
            %   Inputs:
            %     tracker            - a trackerTOMHT
            %     detections         - a cell array of objectDetection objects
            %     time               - the time to which all the tracks will
            %                          be updated and predicted. A real
            %                          numeric scalar, must be greater than
            %                          the value in the previous call.
            %     costMatrix         - the cost of assigning detections to
            %                          tracks. A real B-by-D matrix, where
            %                          B is the number of allBranches
            %                          output of the getBranches method.
            %     detectableTrackIDs - the branch IDs that sensors expect to
            %                          detect, reported as an M-by-1 or
            %                          M-by-2 matrix. The first column is
            %                          of branch IDs, as reported by the
            %                          BranchID field of the tracks output
            %                          (see Output below). The second
            %                          column is optional and allows you to
            %                          add the detection probability for
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
            %       BranchIDsAtStepBeginning  - Branch IDs when the step began.
            %       CostMatrix                - Cost of assignment matrix.
            %       Assignments               - Assignments returned from
            %                                   assignTOMHT.
            %       UnassignedTracks          - IDs of unassigned branches
            %                                   returned from trackerTOMHT.
            %       UnassignedDetections      - IDs of unassigned detections
            %                                   returned from trackerTOMHT.
            %       InitialBranchHistory      - Branch history after branching,
            %                                   before pruning.
            %       InitialBranchScores       - Branch scores before pruning.
            %       KeptBranchHistory         - Branch history after initial
            %                                   pruning.
            %       KeptBranchScores          - Branch scores after initial
            %                                   pruning.
            %       Clusters                  - A logical array that maps
            %                                   branches to clusters. Branches
            %                                   belong in the same cluster if
            %                                   they share detections in their
            %                                   history or belong to the same
            %                                   track, either directly or
            %                                   through other branches. Such
            %                                   branches are incompatible.
            %       TrackIncompatibility      - Branch incompatibility matrix.
            %                                   The (i,j) element is true if
            %                                   the i-th and j-th branches have
            %                                   shared detections in their
            %                                   history or belong to the same
            %                                   track.
            %       GlobalHypotheses          - A logical matrix that maps
            %                                   branches to global hypotheses.
            %                                   Branches can belong in the same
            %                                   hypotheses if they are
            %                                   compatible.
            %       GlobalHypScores           - The total score of global
            %                                   hypotheses.
            %       PrunedBranches            - A logical array of branches
            %                                   that the pruneTrackBranches
            %                                   determined to be pruned.
            %       GlobalBranchProbabilities - Global probability of each
            %                                   branch, taking into account
            %                                   their existence in the global
            %                                   hypotheses.
            %       BranchesDeletedByPruning  - Branches that were deleted by
            %                                   the trackerTOMHT.
            %       BranchIDsAtStepEnd        - Branch IDs when the step ended.
            %
            %   See also: trackerTOMHT, objectDetection, objectTrack
            
            % Process inputs
            time = processInputs(obj, detections, varargin{:});
            
            % Run core algorithm
            if nargout==4
                [hypothesis, cluster, probability, keptScores, info] ...
                    = coreAlgorithm(obj,  varargin{:});
            else
                [hypothesis, cluster, probability, keptScores] ...
                    = coreAlgorithm(obj,  varargin{:});
            end
            % Process output
            [conf,tent,all] = getTrackerOutputs(obj, time, ...
                hypothesis, cluster, probability, keptScores);
        end
        
        function time = processInputs(obj, detections, time, varargin)
            % In MATLAB: time is always the 1st part of varargin
            
            % Error out if the time input is not greater than obj.
            coder.internal.errorIf(time <= obj.pLastTimeStamp, ...
                'fusion:trackerTOMHT:TimeMustIncrease','step');
            
            % Pre-process detections (format, time, sensor index)
            step(obj.cDetectionManager, detections, obj.pLastTimeStamp, time);
        end
        
        function [hyps1, clu1, prob1, keptScores, info] = coreAlgorithm(obj, varargin)
            branchIDsAtStart = obj.pBranchIDs(1:obj.pNumLiveTracks)';
            
            dets = detections(obj.cDetectionManager);
            % Update list of unique sensor IDs that tracker has seen
            updateUsedSensors(obj,originatingSensor(obj.cDetectionManager));
            
            % Calculate the cost matrix
            isInseq = isInSequence(obj.cDetectionManager);
            if numel(varargin) > 1 && obj.HasCostMatrixInput
                % Validate a cost matrix input
                if (numel(varargin) > (1 + obj.HasCostMatrixInput + obj.HasDetectableBranchIDsInput))
                    coder.internal.error('MATLAB:TooManyInputs');
                end
                validateattributes(varargin{end-obj.HasDetectableBranchIDsInput}, {'numeric'}, {'real', ...
                    'nonsparse', '2d', 'nrows', obj.pNumLiveTracks, ...
                    'ncols', numel(dets)}, 'step', 'costMatrix');
                c = getCostMatrixFromInput(obj,varargin{:}); % Goes to info
                costMatrix = c(:,isInseq); % Ignore OOSMs and used in processing
            else
                c = zeros(obj.pNumLiveTracks,numDetections(obj.cDetectionManager),obj.pClassToUse);
                costMatrix = calcCostMatrix(obj,dets); % Ignore OOSMs and used in processing
                c(:,isInseq) = costMatrix; % Goes to info
            end
            
            % Process detectableTrackIDs with or without input
            trackDetectability(obj, varargin{:})
            
            % Call the assigner to get the updated history.
            [assignments,unassignedTracks,unassignedDetections] = assignDetections(obj,costMatrix);
            usedOriginatingSensors = originatingSensorToUsedSensor(obj);
            history = step(obj.cBranchManager,assignments,...
                unassignedTracks,unassignedDetections,usedOriginatingSensors);
            
            % Score track hypotheses
            scores = scoreTrackHypotheses(obj,history,dets);
            
            % Prune tracks based on prior scores
            [keptHistory,keptScores,toPrune] = pruneTrackHypotheses(obj,history,scores);
            
            % Formulate clusters from tracks
            [clusters, incompTracks] = clusterTracks(obj, keptHistory);
            
            % Formulate global hypotheses
            [hyps,hypsScores] = formulateGlobalHypotheses(obj,clusters,incompTracks,keptScores);
            
            % Initiate tracks
            initiatedTracks = initiateTracksFromHistory(obj, keptHistory, dets);
            
            % Update assigned tracks and check confirmation
            updatedAssignedTracks = updateAssignedTracksFromHistory(obj, keptHistory, dets);
            
            % Update unassigned tracks
            updatedUnassignedTracks = updateUnassignedTracks(obj, keptHistory);
            
            % Prune low probability tracks and n-scan pruning
            actualTracksInHistory = initiatedTracks | updatedAssignedTracks | updatedUnassignedTracks;
            keptHistory = keptHistory(actualTracksInHistory,:);
            keptScores = keptScores(actualTracksInHistory,:);
            hyps = hyps(actualTracksInHistory,:);
            [prunedTracks,posteriorProbs,pruningInfo] = pruneTracks(obj,keptHistory,keptScores,hyps);
            
            % Delete pruned tracks
            deleteTracks(obj,history(:,3),toPrune,true);
            toDelete1 = prunedTracks;
            deleted1 = deleteTracks(obj,keptHistory(:,3),toDelete1,true);
            hyps1 = hyps(~deleted1,:);
            clu1 = clusters(~deleted1,:);
            prob1 = posteriorProbs(~deleted1,:);
            hist1 = keptHistory(~deleted1,:);
            
            % Find and delete tracks not mentioned in the history
            deleteTracksNotInHistory(obj,hist1);
            
            % Check track confirmation
            confirmTracks(obj);
            
            % Add analysis information to the output
            if nargout == 5
                branchIDsAtEnd = obj.pBranchIDs(1:obj.pNumLiveTracks);
                info = struct(...
                    'OOSMDetectionIndices',oosmIndices(obj.cDetectionManager),...
                    'BranchIDsAtStepBeginning', branchIDsAtStart, ...
                    'CostMatrix', c, ...
                    'Assignments', assignments, ...
                    'UnassignedTracks', unassignedTracks, ...
                    'UnassignedDetections', unassignedDetections, ...
                    'InitialBranchHistory', history, ...
                    'InitialBranchScores', scores, ...
                    'KeptBranchHistory', keptHistory, ...
                    'KeptBranchScores', keptScores, ...
                    'Clusters', clusters, ...
                    'TrackIncompatibility', incompTracks, ...
                    'GlobalHypotheses', hyps, ...
                    'GlobalHypScores', hypsScores, ...
                    'DetailedPruningInfo', pruningInfo, ...
                    'PrunedBranches', prunedTracks, ...
                    'GlobalBranchProbabilities', posteriorProbs, ...
                    'BranchesDeletedByPruning', keptHistory(deleted1,3), ...
                    'BranchIDsAtStepEnd', branchIDsAtEnd);
            else
                info = {};
            end
        end

        function costMatrix = getCostMatrixFromInput(obj,varargin)
            %Extract cost matrix from input.
            costMatrix = varargin{end-obj.HasDetectableBranchIDsInput};
        end
        
        function usedOriginatingSensors = originatingSensorToUsedSensor(obj)
            % Converts from originating sensors, which may be non
            % consecutive, to used sensors, which must be consecutive.
            origSen = originatingSensor(obj.cDetectionManager);
            usedOriginatingSensors = zeros(1,numel(origSen));
            for i = 1:numel(origSen)
                usedOriginatingSensors(i) = find(origSen(i) == obj.pUsedSensors,1,'first');
            end
        end
        
        function [conf,tent,all] = getTrackerOutputs(obj, varargin)
            time  = varargin{1};
            hyps1 = varargin{2};
            clu1  = varargin{3};
            prob1 = varargin{4};
            keptScores = varargin{5};
            % Predict tracks to update time
            predictTracks(obj,time);
            
            % Update tracker timestamp
            obj.pLastTimeStamp = cast(time,'like',obj.pLastTimeStamp);
            
            % Outputs:
            % Output tracks
            outputList = true(obj.pNumLiveTracks,1);
            tracks = getTracks(obj, outputList);
            
            % Reorder the hypotheses by score:
            if coder.target('MATLAB')
                ts = [tracks.TrackLogicState]; % Returns a vector
                trackScores = ts(1:2:end-1)';
            else
                trackScores = zeros(obj.pNumLiveTracks,1,'like',keptScores);
                for ii = 1:obj.pNumLiveTracks
                    trackScores(ii) = tracks(ii).TrackLogicState(1);
                end
            end
            hypScores = trackScores(:,1)' * hyps1;
            [~,I] = sort(hypScores,'descend');
            
            outputHyps = hyps1(:,I);
            % Remove clusters if the branches in them are pruned
            if size(clu1,2) > 0
                outputClusters = clu1(:,sum(clu1)>0);
            else % Nothing to remove, already zero columns (clusters)
                outputClusters = clu1;
            end
            outputProbability = prob1;
            
            obj.pHyps = outputHyps;
            obj.pClusters = outputClusters;
            obj.pProbs = outputProbability;
            
            [conf,tent,all] = organizeTrackOutputs(obj,tracks);
        end
        
        function [conf,tent,all] = organizeTrackOutputs(obj,tracks)
            outputHyps = obj.pHyps;
            outputProbability = obj.pProbs;
            outputClusters = obj.pClusters;
            [conf,tent,all] = obj.cOutputter(tracks,outputHyps,outputProbability,outputClusters);
        end
        
        function setupImpl(obj, detections, varargin)
            % Perform one-time calculations, such as computing constants
            % Prepare the time stamp
            
            time = varargin{1};
            obj.pLastTimeStamp = cast(-eps,'like',time);
            
            % Set the track manager up
            obj.pMaxNumBranches = obj.MaxNumTracks * obj.MaxNumTrackBranches;
            setupImpl@matlabshared.tracking.internal.fusion.TrackManager(obj,detections);
            
            % Set the sub-objects up
            obj.cCostCalculator = matlabshared.tracking.internal.fusion.AssignmentCostCalculator(...
                'MaxAssignmentCost', obj.AssignmentThreshold(end));
            setup(obj.cCostCalculator, obj.pTracksList, {obj.pSampleDetection}, ...
                obj.pNumLiveTracks, zeros(0,1,'uint32'));
            
            obj.cBranchManager = trackBranchHistory(...
                'MaxNumSensors',obj.MaxNumSensors,...
                'MaxNumHistoryScans',obj.MaxNumHistoryScans, ...
                'MaxNumTracks', obj.MaxNumTracks, ...
                'MaxNumTrackBranches', obj.MaxNumTrackBranches);
            obj.cOutputter = fusion.internal.mhtOutputting(...
                'OutputRepresentation', obj.OutputRepresentation, ...
                'HypothesesToOutput', obj.HypothesesToOutput);
            
            obj.pTrackDetectionProbability = obj.DetectionProbability * ones(obj.pMaxNumBranches,1);
        end
        
        function [trackLogic, type] = createLogic(obj)
            trackLogic = trackScoreLogic(...
                'ConfirmationThreshold', obj.ConfirmationThreshold, ...
                'DeletionThreshold', obj.DeletionThreshold);
            type = 2;
        end
        
        function validateFilterOnSetup(~, f)
            cond = isa(f, 'matlabshared.tracking.internal.AbstractTrackingFilter');
            if ~cond
                coder.internal.error('fusion:trackerTOMHT:InvalidFilter', class(f), mfilename);
            end
        end
        
        function resetImpl(obj)
            % Returns the tracker to its initial state
            
            % Reset sub-objects
            reset(obj.cBranchManager);
            reset(obj.cCostCalculator);
            reset(obj.cOutputter);
            
            % Reset tracks and DetectionManager
            resetImpl@matlabshared.tracking.internal.fusion.TrackManager(obj);
            obj.pLastTimeStamp = cast(-eps, 'like', obj.pLastTimeStamp); %Has to be negative to run tracker from t=0
            
            % Reset branches
            obj.pBranchIDs = zeros(1,obj.pMaxNumBranches,'uint32');
            
            obj.pWasDetectable = true(obj.pMaxNumBranches,1);
        end
        
        function releaseImpl(obj)
            % Release resources, such as file handles
            releaseImpl@matlabshared.tracking.internal.fusion.TrackManager(obj);
            release(obj.cBranchManager);
            release(obj.cCostCalculator);
            release(obj.cOutputter);
        end
        
        function validatePropertiesImpl(obj)
            % Validate related or interdependent property values
            
            % Filter Initialization Validation:
            validateattributes(obj.FilterInitializationFcn, ...
                {'function_handle','char','string'}, {'nonempty'}, 'trackOrientedMHobj', ...
                'FilterInitializationFcn');
            
            if isa(obj.FilterInitializationFcn, 'function_handle')
                obj.pFilterInitializationFcn = obj.FilterInitializationFcn;
            else
                obj.pFilterInitializationFcn = str2func(obj.FilterInitializationFcn);
            end
            
            % Check the filter initialization function syntax
            isInvalid = ~matlabshared.tracking.internal.fusion.isValidInitFcn(obj.pFilterInitializationFcn);
            if isInvalid
                coder.internal.error('fusion:trackerTOMHT:InvalidFilterInitFcn', 'FilterInitializationFcn');
            end
        end
        
        function validateHypothesesToOutput(obj,value)
            validateattributes(value,{'numeric'},...
                {'finite','real','positive','nonsparse','integer','vector'},...
                mfilename,'HypothesesToOutput')
            coder.internal.assert(numel(value)<=obj.MaxNumHypotheses,...
                'fusion:trackerTOMHT:NumHypothesesOutput')
        end
        
        function processTunedPropertiesImpl(obj)
            % Perform actions when tunable properties change
            % between calls to the System object
            
            % Process HypothesesToOutput property
            obj.cOutputter.HypothesesToOutput = obj.HypothesesToOutput;
        end
        
        function flag = isInactivePropertyImpl(obj,prop)
            % Return false if property is visible based on object
            % configuration, for the command line and System block dialog
            
            flag = isInactivePropertyImpl@matlabshared.tracking.internal.fusion.TrackManager(obj,prop);
            
            if ~strcmpi(obj.OutputRepresentation,'Hypothesis')
                flag = flag | any(strcmpi(prop,{'HypothesesToOutput'}));
            end
        end
        
        function validateInputsImpl(obj,~,varargin)
            % Validate inputs to the step method at initialization
            % Validate time input
            if numel(varargin) > 0
                validateattributes(varargin{1},{'single','double'}, ...
                    {'real','finite','nonsparse','scalar','nonnegative'}, ...
                    mfilename, 'time')
            end
            
            if obj.HasDetectableBranchIDsInput
                detectables = getDetectableBranchIDsFromInput(obj,varargin{:});
                validateattributes(detectables, {'numeric'}, {'real', ...
                    'nonsparse', '2d'}, 'step', 'detectableBranchIDs');
            end
        end
        
        function detectables = getDetectableBranchIDsFromInput(~,varargin)
            %In Matlab detectable track IDs is always last input
            detectables = varargin{end};
        end
        
        function num = getNumInputsImpl(obj)
            % Define total number of inputs for system with optional inputs
            num = 2;
            if obj.HasCostMatrixInput
                num = num + 1;
            end
            if obj.HasDetectableBranchIDsInput
                num = num + 1;
            end
        end
        
        function num = getNumOutputsImpl(~)
            % Define total number of outputs for system with optional
            % outputs
            
            % In MATLAB, always have 4 outputs
            num = 4;
        end
    end
    
    %% Methods Called as Part of stepImpl
    methods(Access = protected)
        function trackDetectability(obj, varargin)
            % If HasDetectableBranchIDsInput is true, use
            % detectableBranchIDs to calculate obj.pWasDetectable and
            % obj.pTrackDetectionProbability. Otherwise, we use the default
            % that all tracks are detectable and that all tracks have
            % obj.DetectionProbability
            
            % Reset the pWasDetectable and pTrackDetectionProbability from
            % last step
            obj.pTrackDetectionProbability = obj.DetectionProbability * ones(obj.pMaxNumBranches,1);
            
            % Validate detectable track IDs input
            if obj.HasDetectableBranchIDsInput
                detectables = getDetectableBranchIDsFromInput(obj,varargin{:});
                obj.pWasDetectable(1:obj.pNumLiveTracks) = false;
                if numel(detectables)>0 % Validate only if nonempty
                    if size(detectables,2)==1 % Column of track IDs
                        validateattributes(detectables, {'numeric'}, ...
                            {'real','positive','column','integer'}, ...
                            'step', 'detectableTrackIDs');
                        trackIDs = detectables(:,1);
                        trackPDs = obj.DetectionProbability*ones(numel(trackIDs),1);
                    else % Optional column of PDs
                        validateattributes(detectables, {'numeric'}, ...
                            {'real','ncols',2}, 'step', 'detectableTrackIDs');
                        validateattributes(detectables(:,1), {'numeric'}, ...
                            {'positive','column','integer'}, ...
                            'step', 'detectableTrackIDs');
                        trackIDs = detectables(:,1);
                        validateattributes(detectables(:,2), {'numeric'}, ...
                            {'nonnegative','column','<',1}, ...
                            'step', 'detectionProbability');
                        trackPDs = detectables(:,2);
                    end
                    for i = 1:numel(trackIDs)
                        ind = findTrackByID(obj,trackIDs(i));
                        if ~isempty(ind)
                            obj.pWasDetectable(ind) = true;
                            obj.pTrackDetectionProbability(ind) = trackPDs(i);
                        end
                    end
                end
                obj.pTrackDetectionProbability(~obj.pWasDetectable) = 0;
            else
                % All tracks are detectable if HasDetectableBranchIDsInput is false
                obj.pWasDetectable(1:obj.pNumLiveTracks) = true;
            end
        end
        %------------------------------------------------------------------
        
        function costMatrix = calcCostMatrix(obj,dets)
            % This function calculates the cost matrix for the assignment
            % function. Ever row represents a single track, every column
            % represents a single detection. SensorDetections is a list of
            % indices to the detections from a specific sensor based on
            % Detection.SensorIndex
            % To calculate the cost matrix, the distance method of the
            % track's tracking filter is used. It provides a result that is
            % normalized to the size of the state covariance matrix
            % projected to the measurement space. See the tracking filter's
            % distance method help for more details.
            inseqInds = inSequenceIndices(obj.cDetectionManager);
            costMatrix = step(obj.cCostCalculator, obj.pTracksList, dets, obj.pNumLiveTracks, inseqInds);
        end
        %------------------------------------------------------------------
        
        function [assignments,unassignedTracks,unassignedDetections] = assignDetections(obj,costMatrix)
            % This method provides the assignments, unassigned tracks, and
            % unassigned detections as a function of the cost matrix. We
            % use the track assigner to get the assignment. However, the
            % assigner returns track indices (row indices in the
            % costMatrix). These need to be converted to track IDs.
            
            % Get the assignments from the assigner object
            [assignments,unassignedTracks,unassignedDetections] = ...
                assignTOMHT(costMatrix,cast(obj.AssignmentThreshold(1:3),'like',costMatrix));
            
            inSeqInds = inSequenceIndices(obj.cDetectionManager);
            % Convert to track IDs
            if ~isempty(assignments)
                assignments(:,1) = obj.pBranchIDs(assignments(:,1));
                assignments(:,2) = inSeqInds(1,assignments(:,2));
            end
            
            if ~isempty(unassignedTracks)
                unassignedTracks(:) = obj.pBranchIDs(unassignedTracks(:));
            end
            
            if ~isempty(unassignedDetections)
                unassignedDetections(:) = inSeqInds(1,unassignedDetections);
            end
        end
        %------------------------------------------------------------------
        
        % Score track hypotheses
        function scores = scoreTrackHypotheses(obj,history,dets)
            %scoreTrackHypotheses For every track hypothesis, calculate the
            % track score.
            % List has three cases: new tracks initiated by a detection,
            % existing tracks with no detections, and tracks with
            % detections in the gate. The score is calculated based on
            % these three cases
            
            numTrackHyps = size(history,1);
            scores = zeros(numTrackHyps,2);
            numSensors = obj.MaxNumSensors;
            
            for i = 1:numTrackHyps
                % Case 1: new tracks based on detection
                if (history(i,2)==0) % No parent track
                    scores(i,1) = fusion.internal.trackScoreInit(obj.Volume, obj.Beta, obj.DetectionProbability, obj.FalseAlarmRate);
                    % Case 2: existing tracks not assigned a detection
                elseif all((history(i,4:numSensors+3)==0))
                    ind = findTrackByID(obj,history(i,2));
                    scoreIncrement = fusion.internal.trackScoreUpdate(false, obj.pTrackDetectionProbability(ind), obj.FalseAlarmRate);
                    score = output(obj.pTracksList{ind(1,1)}.TrackLogic);
                    scores(i,1) = score(1) + scoreIncrement(1,1);
                    scores(i,2) = score(2);
                else % Case 3: at least one, but possibly more, detections are assigned
                    inds = ~(history(i,4:numSensors+3)==0);
                    is = (4:numSensors+3);
                    indices = is(inds);
                    numDets = numel(indices);
                    if coder.target('MATLAB')
                        detections = [dets{history(i,indices)}];
                        detectionTimes = [detections.Time];
                    else
                        detectionTimes = zeros(numDets,1,'like',dets{1}.Time);
                        for k = 1:numDets
                            detectionTimes(k) = dets{history(i,indices(k))}.Time;
                        end
                    end
                    [~,I] = sort(detectionTimes,'ascend');
                    
                    ind = findTrackByID(obj,history(i,2));
                    track = obj.pTracksList{ind(1,1)};
                    
                    score = output(track.TrackLogic);
                    scores(i,:) = score;
                    isLinearKalmanFilter = track.pIsLinearKalmanFilter;
                    motionModel = track.pMotionModel;
                    for k = 1:numDets
                        detection = dets{history(i,indices(I(k)))};
                        measurement = detection.Measurement;
                        if obj.pHasMeasurementParameters % Parameters can be different
                            if iscell(detection.MeasurementParameters)
                                parameters = detection.MeasurementParameters;
                            else
                                parameters = {detection.MeasurementParameters};
                            end
                        else
                            parameters = {};
                        end
                        sync(obj.pDistFilter, track.Filter);
                        dt = detection.Time - track.Time;
                        obj.pDistFilter.MeasurementNoise = detection.MeasurementNoise;
                        matlabshared.tracking.internal.fusion.predictTrackFilter(...
                            obj.pDistFilter, isLinearKalmanFilter, motionModel, dt);
                        
                        if track.pIsLinearKalmanFilter || isempty(parameters)
                            l = likelihood(obj.pDistFilter, measurement(:));
                        else
                            l = likelihood(obj.pDistFilter, measurement(:), parameters);
                        end
                        
                        scoreIncrement = fusion.internal.trackScoreUpdate(true, obj.Volume, l, obj.DetectionProbability, obj.FalseAlarmRate);
                        scores(i,1) = scores(i,1) + scoreIncrement;
                    end
                end
            end
        end
        %------------------------------------------------------------------
        
        % Prune tracks
        function [keptHistory,keptScores,toPrune] = pruneTrackHypotheses(obj,history,scores)
            %pruneTrackHypotheses   Prunes the track hypotheses to reduce
            %   their number before formulating the overall hypotheses
            keptList = ((scores(:,1)-scores(:,2)) >= obj.DeletionThreshold);
            keptHistory = history(keptList,:);
            keptScores = scores(keptList,:);
            toPrune = ~keptList;
        end
        %------------------------------------------------------------------
        
        % Cluster tracks
        function [clusters, incompatibleTracks] = clusterTracks(~, keptHistory)
            [clusters, incompatibleTracks] = clusterTrackBranches(keptHistory);
        end
        %------------------------------------------------------------------
        
        % Formulate global hypotheses
        function [hyps, hypScores] = formulateGlobalHypotheses(obj,clusters,incompatibleTracks,scores)
            % Formulate the global hypotheses using the hypotheses manager
            [hyps,hypScores] = compatibleTrackBranches(clusters,incompatibleTracks,scores,obj.MaxNumHypotheses);
        end
        %------------------------------------------------------------------
        
        % Prune tracks
        function [toPrune,posteriorProbs,info] = pruneTracks(obj,branchHistory,branchScores,hyps)
            % Use the track pruning object to prune the tracks based on
            % their hypotheses and n-scan pruning.
            [toPrune,posteriorProbs,info] = pruneTrackBranches(...
                branchHistory,branchScores,hyps, ...
                'MaxNumTrackBranches',obj.MaxNumTrackBranches, ...
                'MinBranchProbability',obj.MinBranchProbability, ...
                'NScanPruning', obj.NScanPruning, ...
                'NumSensors', obj.MaxNumSensors);
        end
        %------------------------------------------------------------------
        
        % Initiate tracks
        function initiatedTracks = initiateTracksFromHistory(obj, keptHistory, dets)
            %This method initiates a new track based on an existing
            %post-processed detection, when this detection
            %cannot be associated with any existing track in the tracks
            %list.
            %History holds all the information about the track to be
            %created, including its TrackID and BranchID. The ParentID is
            %0, but will be replaced with the BranchID.
            
            % Find branches in history that are new
            numTracks = size(keptHistory,1);
            initiatedTracks = false(numTracks,1);
            firstInitiatedTrack = find((keptHistory(:,2)==0),1,'first');
            allBranches = 1:numTracks;
            newBranches = allBranches((keptHistory(:,2)==0));
            
            for i=1:numel(newBranches)
                % Get the relevant detection.
                historyRow = newBranches(i);
                detInds = (4:obj.MaxNumSensors+3);
                lastScan = keptHistory(historyRow,detInds);
                nonZeros  = ~(lastScan==0);
                detID = keptHistory(historyRow,detInds(nonZeros));
                Det = dets{detID(1,1)};
                
                % From history, get the new TrackID and new BranchID
                newTrackID = keptHistory(historyRow,1);
                newBranchID = keptHistory(historyRow,3);
                
                % There is a pre-allocated memory for the new track and the
                % number of tracks is smaller than MaxNumTracks
                tf = initiateTrack(obj, newTrackID, Det);
                if tf
                    % Add the branch ID to the list of branch IDs
                    obj.pBranchIDs(obj.pNumLiveTracks) = newBranchID;
                    obj.pTracksList{obj.pNumLiveTracks}.BranchID = newBranchID;
                    initiatedTracks(firstInitiatedTrack+i-1) = true;
                else  % Failed to initiate track because pTracksList is full
                    coder.internal.warning('fusion:trackerTOMHT:MaxNumTracksReached', 'MaxNumTracks');
                    break
                end
            end
        end
        %------------------------------------------------------------------
        
        % Update unassigned tracks
        function updatedUnassignedTracks = updateUnassignedTracks(obj, history)
            % Unassigned tracks: call the updateNotAssociated method on
            % each track
            updatedUnassignedTracks = false(size(history,1),1);
            unassignedTracks = find(all((history(:,3+(1:obj.MaxNumSensors))==0),2));
            for i = 1:numel(unassignedTracks)
                ind = findTrackByID(obj,history(unassignedTracks(i),3));
                if ~isempty(ind)
                    updateNotAssociated(obj.pTracksList{ind(1,1)}, obj.pTrackDetectionProbability(ind(1,1)), obj.FalseAlarmRate);
                    updatedUnassignedTracks(i) = true;
                end
            end
        end
        %------------------------------------------------------------------
        
        % Update assigned tracks
        function updatedAssignedTracks = updateAssignedTracksFromHistory(obj, history, dets)
            % Updating an assigned track means that we are spawning a new
            % track and update it with the assigned detections. This
            % provides multiple hypotheses (the null hypothesis is that the
            % track was not assigned).
            % For each track hypothesis, history holds the detections that
            % were assigned to the track.
            checkUnassignedDets = (find((history(:,2)==0),1,'last'));
            checkUnassignedTrks = find(all((history(:,3+(1:obj.MaxNumSensors))==0),2),1,'last');
            
            if ~isempty(checkUnassignedDets) % If there are unassigned detections
                firstAssignedTrack = checkUnassignedDets(1,1)+1;
            elseif ~isempty(checkUnassignedTrks) % If there are unassigned tracks
                firstAssignedTrack = checkUnassignedTrks(1,1)+1;
            else % If none, all are assigned
                firstAssignedTrack = 1;
            end
            
            updatedAssignedTracks = false(size(history,1),1);
            assignedHistory = history(firstAssignedTrack(1,1):end,4:obj.MaxNumSensors+3);
            assignedBranchIDs = history(firstAssignedTrack(1,1):end,2);
            newBranchIDs = history(firstAssignedTrack(1,1):end,3);
            numAssigned = numel(assignedBranchIDs);
            for i = 1:numAssigned
                % Find the originating track by track ID
                id = assignedBranchIDs(i);
                index = findTrackByID(obj, id);
                if isempty(index)
                    continue
                end
                % Get the detections associated with this track
                nonZeroDets = assignedHistory(i,~(assignedHistory(i,:)==0));
                
                % Initiate a new track based on the originating track
                newTrackNumber = obj.pNumLiveTracks + 1;
                
                % There is a pre-allocated memory for the new track
                if newTrackNumber <= numel(obj.pTracksList)
                    obj.pNumLiveTracks = obj.pNumLiveTracks + 1;
                    
                    % Synchronize the new branch with the existing branch
                    thisTrack = obj.pTracksList{newTrackNumber};
                    thatTrack = obj.pTracksList{index(1,1)};
                    sync(thisTrack, thatTrack);
                    
                    % Provide a new branch ID
                    obj.pTracksList{newTrackNumber}.BranchID = newBranchIDs(i);
                    
                    % Update the branch with the assigned detections
                    trackDetections = extractDetectionsForTrack(obj, nonZeroDets, dets);
                    correct(obj.pTracksList{newTrackNumber}, trackDetections, obj.pUsedSensors, obj.Volume, obj.DetectionProbability, obj.FalseAlarmRate);
                    
                    % Update the list of branches maintained by tracker
                    obj.pBranchIDs(newTrackNumber) = newBranchIDs(i); % Add to list of track IDs.
                    obj.pTrackIDs(newTrackNumber) = obj.pTrackIDs(index(1,1)); % Relate this track to the same tree ID
                    obj.pConfirmedTracks(newTrackNumber) = obj.pTracksList{newTrackNumber}.IsConfirmed;
                    updatedAssignedTracks(i+firstAssignedTrack(1,1)-1) = true;
                else
                    coder.internal.warning('fusion:trackerTOMHT:MaxNumTracksReached', 'MaxNumTracks');
                    break
                end
            end
        end
        %------------------------------------------------------------------
        
        function trackDetections = extractDetectionsForTrack(~, detectionsForCorrect, dets)
            numTrackDetections = numel(detectionsForCorrect);
            trackDetections = repmat({dets{detectionsForCorrect(1)}}, [numTrackDetections, 1]);
            for i = 2:numTrackDetections
                trackDetections{i} = dets{detectionsForCorrect(i)};
            end
        end
        %------------------------------------------------------------------
        
        function index = findTrackByID(obj, branchID)
            % Returns the index of a track specified by its branchID. If
            % the track does not exist, it returns empty
            
            index = find(branchID == obj.pBranchIDs, 1, 'first');
        end
        %------------------------------------------------------------------
        
        function deleted = deleteTracks(obj,tracks,toDelete,byID)
            %This function 'removes' old tracks from the list of tracks
            %  1. Every track that is not assigned to a detection is
            %     coasted (predicted and the lack of assigned detection is
            %     registered).
            %  2. The track is checked for deletion.
            %  3. If the track is to be deleted, 'remove' it.
            %
            %  Note: For performance purposes, the track is not really removed.
            %  It is moved to the end of the tracks list and its content is
            %  'nullified'. In addition, it does not appear in the list of
            %  live tracks and if it was a confirmed track its confirm
            %  status flag is cleared.
            %
            % The method returns a Boolean list of tracks that were deleted
            
            % tracks - a list of branch IDs or indices.
            % toDelete - a boolean array, element is true if the branch needs to be deleted
            % byID - indicates if the tracks are branch IDs or indices.
            
            tracksToDelete = tracks(toDelete);
            indicesToDelete = false(obj.pMaxNumBranches,1);
            deleted = toDelete;
            if byID
                numTracks = numel(tracksToDelete);
                for i = 1:numTracks
                    ind = findTrackByID(obj,tracksToDelete(i));
                    if ~isempty(ind)
                        indicesToDelete(ind(1,1)) = true;
                    else
                        deleted(i) = false;
                    end
                end
            else
                indicesToDelete(tracksToDelete) = true;
            end
            recycleTracks(obj, indicesToDelete);
            if coder.target('MATLAB')
                obj.pBranchIDs = [obj.pBranchIDs(~indicesToDelete),obj.pBranchIDs(indicesToDelete)];
            else
                indices = find(indicesToDelete);
                numInds = numel(indices);
                for i = numInds:-1:1
                    currentInd = indices(i);
                    obj.pBranchIDs(currentInd:end) = [obj.pBranchIDs(currentInd+1:end), uint32(0)];
                end
            end
        end
        
        %------------------------------------------------------------------
        function deleted = deleteTracksNotInHistory(obj,keptHistory)
            inHistory = keptHistory(:,3); % Track IDs in history
            n = obj.pNumLiveTracks;
            toDelete = false(n,1);
            for i = 1:n
                toDelete(i) = isempty(find(obj.pBranchIDs(i)==inHistory, 1));
            end
            deleted = deleteTracks(obj,obj.pBranchIDs,toDelete,true);
        end
        %------------------------------------------------------------------
        
        function confirmTracks(obj)
            % Checks the confirmation of all the live tracks
            for i = 1:obj.pNumLiveTracks
                if ~obj.pTracksList{i}.IsConfirmed
                    obj.pTracksList{i}.IsConfirmed = checkPromotion(obj.pTracksList{i});
                    obj.pConfirmedTracks(i) = obj.pTracksList{i}.IsConfirmed;
                end
            end
        end
        %------------------------------------------------------------------
        
        function predictTracks(obj,time)
            % Predict all the tracks to the current tracker time
            for i = 1:obj.pNumLiveTracks
                predict(obj.pTracksList{i},time);
            end
        end
    end
    
    methods(Access = protected)
        function group = getPropertyGroups(obj)
            group = getPropertyGroupsLongImpl(obj);
        end
        function groups = getPropertyGroupsLongImpl(obj)
            capacityList = {'TrackerIndex',...
                'FilterInitializationFcn','AssignmentThreshold', ...
                'MaxNumTracks','MaxNumDetections','MaxNumSensors'};
            capacityGrp = matlab.mixin.util.PropertyGroup(capacityList);
            
            hypothesesList = {'MaxNumHypotheses','MaxNumHistoryScans',...
                'MaxNumTrackBranches'};
            hypothesesGrp = matlab.mixin.util.PropertyGroup(hypothesesList);
            
            scoreList = {'ConfirmationThreshold','DeletionThreshold',...
                'DetectionProbability','FalseAlarmRate','Beta','Volume'};
            scoreGrp = matlab.mixin.util.PropertyGroup(scoreList);
            
            pruningList = {'MinBranchProbability','NScanPruning'};
            pruningGrp = matlab.mixin.util.PropertyGroup(pruningList);
            
            inputList = {'HasCostMatrixInput','HasDetectableBranchIDsInput',...
                'StateParameters'};
            inputGrp = matlab.mixin.util.PropertyGroup(inputList);
            
            outputList = {'OutputRepresentation','HypothesesToOutput'};
            outputGrp = matlab.mixin.util.PropertyGroup(outputList);
            
            dependentList = {'NumTracks','NumConfirmedTracks'};
            dependentGrp = matlab.mixin.util.PropertyGroup(dependentList);
            
            groups = [capacityGrp,oosmGroup(obj),hypothesesGrp,scoreGrp,pruningGrp,inputGrp,outputGrp,dependentGrp];
        end
    end
    
    methods(Static, Hidden)
        function flag = isAllowedInSystemBlock
            flag = false;
        end
    end
    
    %% Save, Load, Clone Methods
    methods (Access = protected)
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj
            
            % Set public properties and states
            s = saveObjectImpl@matlabshared.tracking.internal.fusion.TrackManager(obj);
            
            % Set private and protected properties
            % Only save private properties if the tracker is locked
            s.pVersion = obj.pVersion;
            if isLocked(obj)
                s.pLastTimeStamp             = obj.pLastTimeStamp;
                s.pWasDetectable             = obj.pWasDetectable;
                s.pTrackDetectionProbability = obj.pTrackDetectionProbability;
                s.pBranchIDs                 = obj.pBranchIDs;
                s.pHyps                      = obj.pHyps;
                s.pClusters                  = obj.pClusters;
                s.pProbs                     = obj.pProbs;
                
                % Save internal sub-objects
                s.cCostCalculator   = saveobj(obj.cCostCalculator);
                s.cBranchManager    = saveobj(obj.cBranchManager);
                s.cOutputter        = saveobj(obj.cOutputter);
            end
        end
        function s = loadSubObjects(obj,s,wasLocked)
            %Load internal sub-objects
            if wasLocked
                obj.cCostCalculator = ...
                    matlabshared.tracking.internal.fusion.AssignmentCostCalculator.loadobj(s.cCostCalculator);
                s = rmfield(s,'cCostCalculator');
                obj.cBranchManager = trackBranchHistory.loadobj(s.cBranchManager);
                s = rmfield(s,'cBranchManager');
                obj.cOutputter = fusion.internal.mhtOutputting.loadobj(s.cOutputter);
                s = rmfield(s,'cOutputter');
            end
        end
        
        function loadObjectImpl(obj,s,wasLocked)
            % Load object
            
            % The fields pDetections and pOriginatingSensor were removed in 21a
            if isfield(s,'pDetections')
                s = rmfield(s,'pDetections');
            end
            if isfield(s,'pOriginatingSensor')
                s = rmfield(s,'pOriginatingSensor');
            end
            s = loadSubObjects(obj,s,wasLocked);
            loadObjectImpl@matlabshared.tracking.internal.fusion.TrackManager(obj,s,wasLocked);
            if wasLocked
                s = rmLoadedFieldNames(obj,s);
            end
            fn = fieldnames(s);
            for m = 1:numel(fn)
                if isprop(obj,fn{m})
                    obj.(fn{m}) = s.(fn{m});
                end
            end
        end
        
        function newTracker = cloneImpl(obj)
            % Copy public properties
            newTracker = cloneImpl@matlabshared.tracking.internal.fusion.TrackManager(obj);
            
            if coder.internal.is_defined(obj.cDetectionManager) % These properties are defined only after setup
                newTracker.pVersion                     = obj.pVersion;
                newTracker.pWasDetectable               = obj.pWasDetectable;
                newTracker.pTrackDetectionProbability   = obj.pTrackDetectionProbability;
                newTracker.pLastTimeStamp               = obj.pLastTimeStamp;
                newTracker.pBranchIDs                   = obj.pBranchIDs;
                newTracker.pHyps                        = obj.pHyps;
                newTracker.pClusters                    = obj.pClusters;
                newTracker.pProbs                       = obj.pProbs;
                
                % Clone internal sub-objects
                newTracker.cCostCalculator   = clone(obj.cCostCalculator);
                newTracker.cBranchManager    = clone(obj.cBranchManager);
                newTracker.cOutputter        = clone(obj.cOutputter);
                newTracker.cDetectionManager = clone(obj.cDetectionManager);
            end
        end

        function flag = isInputSizeMutableImpl(~, index)
            % Return false if input size is not allowed to change while
            % system is running
            flag = true; % All inputs except time are varsize
            if index == 2 % time is an input
                flag = false;
            end
        end
        
        function flag = isInputComplexityMutableImpl(~, ~)
            flag = true;
        end
    end
end

function sampleDetection = getDetectionFromInput(input)
% Returns a sample detection from a variety of input types. Supports cell
% array of objects, objects array and struct array.
if isempty(input)
    coder.internal.error('fusion:trackerTOMHT:UndefinedSampleDetection');
elseif iscell(input)
    sampleDetection = input{1};
else
    sampleDetection = input(1);
end
end

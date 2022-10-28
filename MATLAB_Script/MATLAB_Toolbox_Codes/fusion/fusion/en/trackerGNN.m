classdef trackerGNN< matlabshared.tracking.internal.fusion.GNNTracker & fusion.internal.ExportToSimulinkInterface & fusion.internal.TrackerMemoryManagementUtilities
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
%     <a href="matlab:help matlab.System/reset   ">reset</a>                      - Resets states of the trackerGNN
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

    methods
        function out=trackerGNN
            % Support name-value pair arguments when constructing object
        end

        function out=cloneImpl(~) %#ok<STOUT>
            %clone Creates a copy of the trackerGNN
            %   newTracker = clone(tracker) returns a copy of the tracker
            %   object.
        end

        function out=exerciseOOSMHandler(~) %#ok<STOUT>
        end

        function out=getAssignerInfo(~) %#ok<STOUT>
        end

        function out=getClusterViolationMessages(~) %#ok<STOUT>
        end

        function out=getMaxNumDetectionsPerSensor(~) %#ok<STOUT>
            % Divert implementation to memory management to enable bounds
            % per sensor when possible.
        end

        function out=getPropertyGroups(~) %#ok<STOUT>
            % Define property section(s) for display in Matlab
        end

        function out=getPropertyGroupsImpl(~) %#ok<STOUT>
        end

        function out=getTrackFilterProperties(~) %#ok<STOUT>
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
        end

        function out=hasAssignmentClustering(~) %#ok<STOUT>
        end

        function out=initializeTrackDetectability(~) %#ok<STOUT>
            % obj.DetectionProbability is not used when TrackLogic is
            % 'History' (obj.pTrackLogic is 1). So, call the base class in
            % that case and set it to DetectionProbably if not.
        end

        function out=isInactivePropertyImpl(~) %#ok<STOUT>
            % Return false if property is visible based on object
            % configuration, for the command line and System block dialog
        end

        function out=isInputSizeMutableImpl(~) %#ok<STOUT>
            % Return false if input size is not allowed to change while
            % system is running
        end

        function out=loadObjectImpl(~) %#ok<STOUT>
            % Set properties in object obj to values in structure s
        end

        function out=numHistorySteps(~) %#ok<STOUT>
        end

        function out=oosmGroup(~) %#ok<STOUT>
        end

        function out=saveObjectImpl(~) %#ok<STOUT>
            % Set properties in structure s to values in object obj
        end

        function out=selectDetections(~) %#ok<STOUT>
        end

        function out=setConfThreshold(~) %#ok<STOUT>
        end

        function out=setDelThreshold(~) %#ok<STOUT>
        end

        function out=setTrackFilterProperties(~) %#ok<STOUT>
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
        end

        function out=setTrackLogic(~) %#ok<STOUT>
        end

        function out=setupAssigner(~) %#ok<STOUT>
        end

        function out=setupImpl(~) %#ok<STOUT>
            % Set up cluster violation handling type
        end

        function out=setupOOSMHandler(~) %#ok<STOUT>
        end

        function out=stepImpl(~) %#ok<STOUT>
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
        end

        function out=validateFilterOnSetup(~) %#ok<STOUT>
        end

        function out=validatePropertiesImpl(~) %#ok<STOUT>
            % Validate related or interdependent property values
        end

        function out=validateTrackerProperties(~) %#ok<STOUT>
            % Check DetectionProbability and FalseAlarmRate when TrackLogic
            % is 'Score'
        end

        function out=verifyOOSMHandling(~) %#ok<STOUT>
        end

    end
    properties
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
        Assignment;

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
        AssignmentClustering;

        %Beta   Rate of new tracks per unit volume
        %   Specify the rate of new tracks per unit volume as a positive
        %   value.
        %   The rate of new tracks is used in calculating the track score
        %   during track initialization.
        %
        %   Default = 1
        Beta;

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
        ClusterViolationHandling;

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
        ConfirmationThreshold;

        %CustomAssignmentFcn Name of 'Custom' assignment function
        %   Specify the function name for the custom assignment. This
        %   function will be used only when Assignment = 'Custom'. The
        %   function must have the following syntax:
        %       [assignment, unTrs, unDets] = f(cost, costNonAssignment)
        %   <a href="matlab:edit('assignmunkres')">Example of valid assignment function.</a>
        CustomAssignmentFcn;

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
        DeletionThreshold;

        %DetectionProbability Probability of detection used for track score
        %   Specify the probability of detection expected for the track as
        %   a scalar in the range (0,1)
        %   The probability of detection is used in calculating the track
        %   score when initializing and updating a track.
        %
        %   Default = 0.9
        DetectionProbability;

        % EnableMemoryManagement Enable memory management properties Set
        % EnableMemoryManagement to true to specify bounds for certain
        % variable-sized arrays used inside the tracker through
        % MaxNumDetectionsPerSensor, MaxNumDetectionsPerCluster and
        % MaxNumTracksPerCluster properties. Specifying bounds allow you to
        % manage the memory footprint of the tracker in generated C/C++
        % code.
        % 
        % Default: false
        EnableMemoryManagement;

        %FalseAlarmRate   Rate of false positives used for track score
        %   Specify the expected rate of false positives as a scalar in the
        %   range (0,1)
        %   The false alarm rate is used in calculating the track score
        %   when initializing and updating a track.
        %
        %   Default = 1e-6
        FalseAlarmRate;

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
        FilterInitializationFcn;

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
        MaxNumDetectionsPerCluster;

        % MaxNumDetectionsPerSensor Maximum number of detections per sensor
        % Set the maximum number of detections per sensor that can be
        % passed as an input to the tracker as a positive integer. Setting
        % a finite value allows the tracker to more efficiently manage the
        % memory in generated C/C++ code.
        %
        % This property is active when EnableMemoryManagement is true.
        %
        % Default: 100
        MaxNumDetectionsPerSensor;

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
        MaxNumOOSMSteps;

        %MaxNumTracks   Maximum number of tracks
        %   Set the maximum number of tracks the tracker can maintain as a
        %   positive real integer.
        %
        %   Default: 100
        MaxNumTracks;

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
        MaxNumTracksPerCluster;

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
        OOSMHandling;

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
        TrackLogic;

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
        Volume;

        pClusterViolationHandlingType;

        %Confirmation threshold [positive scalar]
        pScoreConfThreshold;

        %Deletion threshold [negative scalar]
        pScoreDelThreshold;

        %pTrackLogic       The type of track logic, in value
        %   1 - History
        %   2 - Score
        pTrackLogic;

    end
end

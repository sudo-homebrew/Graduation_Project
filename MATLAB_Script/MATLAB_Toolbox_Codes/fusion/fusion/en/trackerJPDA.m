classdef trackerJPDA< matlabshared.tracking.internal.fusion.TrackManager & fusion.internal.ExportToSimulinkInterface & fusion.internal.TrackerMemoryManagementUtilities
%trackerJPDA Joint Probabilistic Data Association tracker
%   tracker = trackerJPDA creates a single-scan, multi-sensor,
%   multi-object tracker that uses Joint Probabilistic Data Association
%   to assign detections to each track. trackerJPDA applies a
%   probabilistic assignment where multiple detections from the same
%   sensor can be assigned to each track. trackerJPDA initializes,
%   confirms, corrects, predicts (performs coasting) and deletes
%   tracks. A track is created with a 'Tentative' status, meaning that
%   there is not enough evidence for the trackerJPDA to determine that
%   the track is of a physical object. If enough additional detections
%   are assigned to the tentative track, its status will change to
%   'Confirmed' (see ConfirmationThreshold). Alternatively, a track
%   will be confirmed if a detection with a nonzero ObjectClassID value
%   is assigned to it, as it means that the sensor is able to classify
%   the physical object.
%
%   tracker = trackerJPDA('Name', value) creates a trackerJPDA object
%   by specifying its properties as name-value pair arguments.
%   Unspecified properties have default values. See the list of
%   properties below.
%
%   Step method syntax: click on <a href="matlab:help('trackerJPDA/stepImpl')">step</a>
%
%   System objects may be called directly like a function instead of
%   using the step method. For example, y = step(obj) and y = obj() are
%   equivalent.
%
%   trackerJPDA properties:
%     TrackerIndex                - Unique identifier of the tracker
%     FilterInitializationFcn     - A handle to a function that initializes
%                                   a tracking filter based on a detection
%     MaxNumEvents                - Value of k for k-best JPDA
%     EventGenerationFcn          - A handle to a function that generates
%                                   the feasible joint event matrices from
%                                   a validation matrix.
%     MaxNumTracks                - Define the maximum number of tracks
%     MaxNumDetections            - Define the maximum number of detections
%     MaxNumSensors               - Define the maximum number of sensors
%     OOSMHandling                - Handle out-of-sequence measurement (OOSM)
%     MaxNumOOSMSteps             - Maximum number of OOSM steps
%     StateParameters             - Parameters defining the track state
%     AssignmentThreshold         - The threshold that controls the
%                                   assignment of detections to tracks
%     DetectionProbability        - Probability of detection
%     InitializationThreshold     - The probability threshold that allows
%                                   assigned detections to initialize a new track
%     TrackLogic                  - Choose track logic: 'History' or
%                                   'Integrated'
%     ConfirmationThreshold       - The required threshold for confirmation
%     DeletionThreshold           - The threshold below which a track is
%                                   deleted
%     HitMissThreshold            - The required threshold to 'hit' the
%                                   TrackLogic or register a 'miss'(*)
%     ClutterDensity              - Spatial density of clutter measurements
%     NewTargetDensity            - Spatial density of new targets (**)
%     DeathRate                   - Time rate of target deaths (**)
%     InitialExistenceProbability - Initial probability of track existence
%                                   (read only)(**)
%     HasCostMatrixInput          - Provide cost matrix as an input
%     HasDetectableTrackIDsInput  - Provide detectable track IDs as an
%                                   input
%     NumTracks                   - Number of tracks (read only)
%     NumConfirmedTracks          - Number of confirmed tracks (read only)
%     TimeTolerance               - Absolute tolerance between time
%                                   stamps of cluster detections (seconds)
%     EnableMemoryManagement      - Enable memory management properties
%     MaxNumDetectionsPerSensor   - Define the maximum number of detections 
%                                   per sensor
%     MaxNumDetectionsPerCluster  - Define the maximum number of detections 
%                                   per cluster
%     MaxNumTracksPerCluster      - Define the maximum number of tracks  
%                                   per assignment cluster
%     ClusterViolationHandling    - Handle run-time violation of cluster
%                                   bounds
%   * Properties are only used for track history logic.
%   ** Properties are only used for track integrated logic.
%
%   trackerJPDA methods:
%     <a href="matlab:help('trackerJPDA/stepImpl')">step</a>                       - Creates, updates, and deletes the tracks
%     predictTracksToTime        - Predicts the tracks to a time stamp
%     getTrackFilterProperties   - Returns the values of filter properties
%     setTrackFilterProperties   - Sets values to filter properties
%     initializeTrack            - Initialize a new track
%     deleteTrack                - Delete a track
%     exportToSimulink           - Export the tracker to a Simulink model
%     release                    - Allows property value and input characteristics changes
%     clone                      - Creates a copy of the trackerJPDA
%     isLocked                   - Locked status (logical)
%     <a href="matlab:help matlab.System/reset   ">reset</a>                      - Resets states of the trackerJPDA
%
%   % EXAMPLE: Construct a tracker and use it to track two objects
%   % Construct a trackerJPDA object with a default constant
%   % velocity Extended Kalman Filter and 'History' track logic.
%   % Increase AssignmentThreshold to 100 to let tracks be jointly
%   % associated
%   tracker = trackerJPDA('TrackLogic','History', 'AssignmentThreshold',100,...
%       'ConfirmationThreshold', [4 5], ...
%       'DeletionThreshold', 10);
%
%   % Create noisy detections of two objects moving along the y = 10,
%   % and the y = -10 axis with constant velocity in the x direction
%   pos_true = [0 0 ; 40 -40 ; 0 0];
%   V_true = 5*[cosd(-30) cosd(30)  ; sind(-30) sind(30) ;0 0];
%
%   % Create a theater plot to visualize tracks and detections
%   tp = theaterPlot('XLimits',[-1 150],'YLimits',[-50 50]);
%   trackP = trackPlotter(tp,'DisplayName','Tracks','MarkerFaceColor','g','HistoryDepth',0);
%   detectionP = detectionPlotter(tp,'DisplayName','Detections','MarkerFaceColor','r');
%   % To find the position and velocity, use:
%   positionSelector = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 0 0]; % [x, y, 0]
%   velocitySelector = [0 1 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 0 ]; % [vx, vy, 0]
%
%   dt = 0.2;
%   for time = 0:dt:30
%       % update the true position of objects
%       pos_true = pos_true + V_true*dt;
%       % create noisy detections of the two objects
%       detection(1) = objectDetection(time,pos_true(:,1)+1*randn(3,1));
%       detection(2) = objectDetection(time,pos_true(:,2)+1*randn(3,1));
%
%       % step the tracker through time with the detections
%       [confirmed,tentative,alltracks,info] = tracker(detection,time);
%
%       % Get positions, velocity and label info for the plot
%       [pos,cov] = getTrackPositions(confirmed,positionSelector);
%       vel = getTrackVelocities(confirmed,velocitySelector);
%       meas = cat(2,detection.Measurement);
%       measCov = cat(3,detection.MeasurementNoise);
%       % Update the track plot only if there are any tracks
%       if numel(confirmed)>0
%           labels = arrayfun(@(x)num2str([x.TrackID]),confirmed,'UniformOutput',false);
%           trackP.plotTrack(pos,vel,cov,labels);
%       end
%       detectionP.plotDetection(meas',measCov);
%       drawnow;
%
%       % Look at the information output every 5 seconds:
%       if time>0 && mod(time,8) == 0
%           disp(['At time t = ' num2str(time) 'seconds,']);
%           disp('The cost of assignment was: ')
%           disp(info.CostMatrix);
%           disp(['Number of clusters: ' num2str(numel(info.Clusters))]);
%           if numel(info.Clusters) == 1
%               disp('-----------------------------')
%               disp('The two tracks were in the same cluster.')
%               disp(info.Clusters{1})
%               disp('Marginal Probabilities of association:')
%               disp(info.Clusters{1}.MarginalProbabilities)
%           end
%       end
%   end
%
%   See also: objectDetection, objectTrack, trackerGNN, trackerTOMHT,
%   trackerPHD, jpdaEvents

     
    % Copyright 2018-2021 The MathWorks, Inc.

    methods
        function out=trackerJPDA
            % You call setProperties in the constructor to let
            % a user specify public properties of object as
            % name-value pairs.
        end

        function out=algorithmRetrodiction(~) %#ok<STOUT>
            % Calculate cost matrix for Out-Of-Sequence detections
        end

        function out=assemblePosteriorLikelihoodMatrix(~) %#ok<STOUT>
            % Allocate memory for posterior likelihood matrix
        end

        function out=associationProbabilities(~) %#ok<STOUT>
            % Computes the a-posteriori probabilities of individual track events i.e. P(X^t, X^t_i | Z^k).
            % Inputs are the current validation Matrix and Feasible Joint Events
            % posterior probabilities P(Xi|Z^k).
        end

        function out=calcEventAndProbabilities(~) %#ok<STOUT>
            % Compute conditional association likelihood.
        end

        function out=calcEventProbabilities(~) %#ok<STOUT>
            % Return the posterior probabilities of each Feasible Joint
            % Event (FJE) conditioned on the cluster detections.
        end

        function out=cloneImpl(~) %#ok<STOUT>
            %clone Creates a copy of the trackerJPDA
            %   newTracker = clone(tracker) returns a copy of the tracker
            %   object.
        end

        function out=clusterAssignUpdate(~) %#ok<STOUT>
            %clusterAssignUpdate  creates all the track clusters based on live
            %tracks in the tracker and the incoming detections.
            %This function executes the following:
            % 1 - For each sensor, calculate a cost matrix of assigning
            %     detections from this sensor to the current tracks.
            %     From each cost matrix, using the assignment threshold,
            %     each track is matched with detections that fall in their
            %     assignment (or validation) gate.
            % 2 - Create clusters of tracks with shared measurements.
            %     For each cluster, extract the cluster validation matrix
            %     and calculate all the feasible joint association events
            %     using EventGenerationFcn.
            % 3 - For each joint event, calculate its posterior marginal
            %     probability.
            % 4 - For each track in the cluster, calculate the marginal
            %     posterior probabilities of association to each measurement
            %     (we use the variable name 'beta' or 'jpda' to refer to
            %     these probabilities).
            % 5 - Update tracks in cluster with a jpda filter correction,
            %     i.e. call the method 'correctjpda'.
        end

        function out=clusterDetections(~) %#ok<STOUT>
            %clusterDetection Determine which detections do not fall in the
            % gate of any track, unassignedDets, which track do not have
            % any detections in their validation gate, unassignedTracks,
            % and which tracks have connected gates i.e. form a cluster.
            % The jpda algorithm should be applied cluster by cluster.
            % clusters is the cell array containing the validated tracks
            % and detections indices.
        end

        function out=clusterViolationMsgs(~) %#ok<STOUT>
        end

        function out=coastUnassignedTracks(~) %#ok<STOUT>
        end

        function out=coreAlgorithm(~) %#ok<STOUT>
            % Keep track IDs at the beginning of the step
        end

        function out=correctTrack(~) %#ok<STOUT>
            % correctTrack applies the appropriate correction to a track
        end

        function out=createClusters(~) %#ok<STOUT>
            %CREATECLUSTERS go over all detections, one sensor at a time to
            % create clusters. Updates the unassigned track counter and the
            % flags for unassigned detections.
        end

        function out=deleteOldTracks(~) %#ok<STOUT>
            %This function 'removes'  tracks
            %based on their current probability of target existence
        end

        function out=deleteTrack(~) %#ok<STOUT>
            %deleteTrack  Delete a track managed by the tracker
            %    deleted = deleteTrack(obj,trackID) deletes the track
            %    specified by trackID from the tracker. The deleted
            %    flag returns true if a track with the same trackID existed
            %    and was deleted. If a track with that trackID did not
            %    exist, the deleted flag is false and a warning is issued.
            %
            % Note: the tracker must be updated at least once to be able to
            % delete a track.
        end

        function out=extractDetectionsForCluster(~) %#ok<STOUT>
        end

        function out=findTracksByIDs(~) %#ok<STOUT>
        end

        function out=getClusterInfo(~) %#ok<STOUT>
        end

        function out=getCostMatrix(~) %#ok<STOUT>
            % Validate cost matrix input or calculate
        end

        function out=getCostMatrixFromInput(~) %#ok<STOUT>
            %Extract cost matrix from input.
        end

        function out=getDetectableTrackIDsFromInput(~) %#ok<STOUT>
            %In Matlab detectable track IDs is always last input
        end

        function out=getMaxNumInputDetections(~) %#ok<STOUT>
        end

        function out=getNumInputsImpl(~) %#ok<STOUT>
            % Define total number of inputs for system with optional inputs
        end

        function out=getNumOutputsImpl(~) %#ok<STOUT>
            % In MATLAB, all four outputs are always available.
        end

        function out=getOOSMInfo(~) %#ok<STOUT>
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
            %   tracker = trackerJPDA;
            %   detection = objectDetection(0, [0;0;0]);
            %   [~, tracks] = tracker(detection, 0);
            %   values = getTrackFilterProperties(tracker, 1, 'MeasurementNoise', 'ProcessNoise');
        end

        function out=getTrackerInfo(~) %#ok<STOUT>
            % Inlining assists coder to optimize the output variable when
            % info is not an output
        end

        function out=getTracksPTE(~) %#ok<STOUT>
        end

        function out=hasAssignmentClustering(~) %#ok<STOUT>
            % Yes, always true
        end

        function out=identLowProbDet(~) %#ok<STOUT>
            % This method returns the detections that are not assigned to
            % any track with a probability larger than the threshold.
        end

        function out=initializeTrack(~) %#ok<STOUT>
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
        end

        function out=initializeTracks(~) %#ok<STOUT>
            %This function initializes a new track based on an existing
            %post-processed detection, when this detection cannot be
            %associated with any existing track in the TracksBank
        end

        function out=isInactivePropertyImpl(~) %#ok<STOUT>
            % Return false if property is visible based on object
            % configuration, for the command line and System block dialog
        end

        function out=isInfoRequested(~) %#ok<STOUT>
        end

        function out=isInputComplexityMutableImpl(~) %#ok<STOUT>
        end

        function out=isInputSizeMutableImpl(~) %#ok<STOUT>
            % Return false if input size is not allowed to change while
            % system is running
        end

        function out=jpdaClusterUpdate(~) %#ok<STOUT>
            %JPDACLUSTERUPDATE Perform JPDA on each cluster and correct track estimates
        end

        function out=likelihoodMatrix(~) %#ok<STOUT>
            % Return an MxN matrix of likelihood values
            % such that lhoodMatrix(i,j) is the likelihood of the ith
            % detection in the current cluster detection array being
            % associated with the jth target in the current cluster tracks
            % cell array
        end

        function out=loadObjectImpl(~) %#ok<STOUT>
            % Set properties in object obj to values in structure s
        end

        function out=numHistorySteps(~) %#ok<STOUT>
            % Implement numHistorySteps in the Retrodiction case.
        end

        function out=predictTrackStruct(~) %#ok<STOUT>
            % Predicts the track's struct output. We use the distance
            % filter to predict the track's struct output to time.
        end

        function out=predictTracks(~) %#ok<STOUT>
            %This function uses tracking filters to predict the tracks into
            %the next time instance
            % The track's predict method and the tracking filters must
            % support predict(track, t).
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

        function out=preprocessDetections(~) %#ok<STOUT>
            % Process the coming detections and separate their origin by
            % source
            % detections can be cell array (especially in
            % codegen) or a regular array of objectDetections or structs.
            % If detections are not cell array, convert them to cell array.
            % Codegen only supports cell arrays of objects
        end

        function out=processInputs(~) %#ok<STOUT>
            % In MATLAB: time is always the 1st part of varargin
        end

        function out=processOutputs(~) %#ok<STOUT>
        end

        function out=releaseImpl(~) %#ok<STOUT>
            % Release resources, such as file handles
        end

        function out=resetImpl(~) %#ok<STOUT>
            % Returns the tracker to its initial state.
        end

        function out=saveObjectImpl(~) %#ok<STOUT>
            % Set properties in structure s to values in object obj
        end

        function out=selectDetections(~) %#ok<STOUT>
        end

        function out=sensorValidationMatrix(~) %#ok<STOUT>
            % Calculate the cost matrix or validate a cost matrix input
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
            %   tracker = trackerJPDA;
            %   detection = objectDetection(0, [0;0;0]);
            %   [~, tracks] = tracker(detection, 0);
            %   setTrackFilterProperties(tracker, 1, 'MeasurementNoise', 2, 'ProcessNoise', 5);
            %   values = getTrackFilterProperties(tracker, 1, 'MeasurementNoise', 'ProcessNoise');
        end

        function out=setTrackLogic(~) %#ok<STOUT>
        end

        function out=setupImpl(~) %#ok<STOUT>
            % Setup properties that will not be modified later on
            % Perform one-time calculations, such as computing constants
        end

        function out=sortClusterByTime(~) %#ok<STOUT>
            % Get sorting index of each cluster using the TimeStamp field
            % of the cluster
        end

        function out=stepImpl(~) %#ok<STOUT>
            %STEP  Creates, updates, and deletes the tracks
            %   The step method is responsible for managing all the tracks:
            %   1. The method attempts to assign the detections to existing
            %      tracks.
            %   2. New tracks are created based on unassigned detections
            %      and detections with low association probabilities.
            %   3. Tracks that are assigned to detections are updated and
            %      confirmed using a JPDA update.
            %   4. Tracks that are not assigned to detections are coasted
            %      (predicted but not corrected).
            %   5. Tracks are deleted based on the number of scans without
            %      association with 'History' logic or based on their
            %      probability of existence with 'Integrated' track logic.
            %
            %   confirmedTracks = STEP(tracker, detections, time) Update
            %   the tracker with a list of detections to the time instance
            %   specified by time. It returns a struct array of confirmed
            %   tracks corrected and predicted to the time instance. See
            %   the track output below for description of confirmedTracks.
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
            %   HasDetectableTrackIDsInput must be true to use this syntax.
            %   The detectableTrackIDs must be an M-by-1 or M-by-2 matrix.
            %   The first column is a list of track IDs that the sensors
            %   report as detectable. The optional second column is the
            %   corresponding probability of detection, if reported by the
            %   sensors. If not reported, the DetectionProbability property
            %   is used. Tracks, whose IDs are not part of
            %   detectableTrackIDs input, are considered as undetectable,
            %   meaning no 'miss' is counted against them if they are not
            %   assigned a detection.
            %
            %   [confirmedTracks, tentativeTracks, allTracks] = STEP(...)
            %   in addition, returns a list of the tentative tracks and all
            %   the tracks. Use allTracks if you want to provide a cost
            %   matrix as it preserves the order of the tracks as required
            %   for the cost matrix input.
            %
            %   [..., analysisInformation] = STEP(...)
            %   in addition, returns a struct of analysis information.
            %
            %   Inputs:
            %     tracker            - a trackerJPDA
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
            %       TrackIDsAtStepBeginning    - Track IDs when the step began.
            %       UnassignedTracks           - Track IDs of unassigned
            %                                    tracks.
            %       UnassignedDetections       - Indices of unassigned
            %                                    detections in the 'detections'
            %                                    input of step().
            %       CostMatrix                 - Cost of assignment matrix.
            %       Clusters                   - Cell array of cluster
            %                                    reports.(*)
            %       initializedTrackIDs        - IDs of tracks initialized 
            %                                    during the step.
            %       DeletedTrackIDs            - IDs of tracks deleted
            %                                    during the step.
            %       TrackIDsAtStepEnd          - Track IDs when the step ended.
            %
            %     If OOSMHandling is set to 'Retrodiction', an additional
            %     field is added to the analysisInformation and includes a
            %     struct with the following fields:
            %       DiscardedDetections  - Indices of detections with 
            %                              timestamp beyond MaxNumOOSMSteps.
            %       CostMatrix           - Assignment cost matrix for OOSMs. 
            %       Clusters             - Cell array of cluster reports,
            %                              restricted to OOSMs.(*)
            %       UnassignedDetections - Indices of unassigned OOSMs.
            %
            %       (*) Each cluster report is a struct that contains:
            %       DetectionIndices     - Indices of clustered detections.
            %       TrackIDs             - Track IDs of clustered tracks.
            %       ValidationMatrix     - Validation matrix of the
            %                              cluster. See jpdaEvents for
            %                              more details.
            %       SensorIndex          - Index of the originating sensor
            %                              of the clustered detections.
            %       TimeStamp            - Mean time stamp of clustered
            %                              detections.
            %       MarginalProbabilities- Matrix of marginal posterior
            %                              joint association probabilities.
            %
            %   If EnableMemoryManagement is true, two additional fields
            %   are added to the analysisInformation:
            %
            %       MaxNumDetectionsPerCluster - Maximum number of
            %                                    detections in the clusters
            %                                    generated during step.
            %       MaxNumTracksPerCluster     - Maximum number of tracks
            %                                    in the clusters generated
            %                                    during the step.
            %
            %   See also: trackerJPDA, objectDetection, objectTrack
        end

        function out=trackDetectability(~) %#ok<STOUT>
            % If HasDetectableTrackIDsInput is true, use detectableTrackIDs
            % to calculate obj.pWasDetectable and
            % obj.pTrackDetectionProbability. Otherwise, we use the default
            % that all tracks are detectable and that all tracks have
            % obj.DetectionProbability
        end

        function out=updateAssignedTracks(~) %#ok<STOUT>
            % updateAssignedTracks Updates the tracks that were assigned
            % with detections and grouped in clusters
        end

        function out=updateClusterTracks(~) %#ok<STOUT>
            % updates the filters of all confirmed tracks using the current
            % detections and the calculated data association coefficients
        end

        function out=validateClusterDetectionsTime(~) %#ok<STOUT>
        end

        function out=validateConfDel(~) %#ok<STOUT>
        end

        function out=validateDetectableTrackIDs(~) %#ok<STOUT>
        end

        function out=validateEventGenerationFcn(~) %#ok<STOUT>
            % If MaxNumEvents is not finite
        end

        function out=validateFilterOnSetup(~) %#ok<STOUT>
        end

        function out=validateInputsImpl(~) %#ok<STOUT>
            % Validate time input
        end

        function out=validatePropertiesImpl(~) %#ok<STOUT>
            % Validate related or interdependent property values
        end

        function out=validateTimeInput(~) %#ok<STOUT>
        end

        function out=verifyOOSMHandling(~) %#ok<STOUT>
        end

    end
    properties
        %AssignmentThreshold   Threshold for assigning detections to tracks
        %   Specify the threshold that controls the assignment of a
        %   detection to a track as a real scalar or a 1x2 vector [C1,C2],
        %   where C1 <= C2. If specified as a scalar, this value is used as
        %   C1, and C2 is Inf.
        %   Initially, a coarse estimation is done to verify which
        %   combinations of {track,detection} require an accurate
        %   normalized distance calculation. Only combinations whose coarse
        %   normalized distance is lower than C2 are calculated.
        %   Detections can only be assigned to a track if their normalized
        %   distance from the track is less than C1.
        %   See the distance method of each tracking filter for explanation
        %   of the distance calculation.
        %   Increase the values if there are detections that should be
        %   assigned to tracks but are not. Decrease the values if there
        %   are detections that are assigned to tracks they should not be
        %   assigned to (too far).
        %
        %   Default: [1 Inf] * 30.0
        AssignmentThreshold;

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
        % This property is active when EnableMemoryManagement is true
        %
        % Default: 'Split and warn'
        ClusterViolationHandling;

        %ClutterDensity Spatial density of clutter measurements
        %   Specify the expected number of false positives per unit volume
        %   as a real scalar. The clutter density is used as the parameter
        %   of a Poisson clutter model. It is also used in calculating the
        %   initial probability of track existence when TrackLogic is set
        %   to 'Integrated'.
        %
        %   Default = 1e-6
        ClutterDensity;

        %ConfirmationThreshold   Threshold for track confirmation
        %   Specify the threshold for track confirmation. The threshold
        %   depends on the type of track confirmation and deletion logic
        %   you use:
        %     * History: specify the confirmation threshold as [M N], where
        %       a track will be confirmed if it receives at least M out of
        %       N updates.
        %     * Integrated: specify the confirmation threshold as a scalar
        %       in the range (0,1). The track will be confirmed if its
        %       probability of existence is at least as high as the
        %       confirmation threshold.
        %
        %   Default:
        %       History logic:     [2 3]
        %       Integrated logic:  0.95
        ConfirmationThreshold;

        %DeathRate Time Rate of target deaths.
        %   Specify the rate at which true targets disappear as a scalar in
        %   the range (0,1). DeathRate is related to the probability of
        %   track existence (PTE) via:
        %      PTE(t+dt) = (1-DeathRate)^dt * PTE(t)
        %   dt is the time interval since the previous update time t.
        %   This property is only used when TrackLogic is set to
        %   'Integrated'
        %
        %   Default = 0.01;
        DeathRate;

        %DeletionThreshold   Threshold for track deletion
        %   Specify the threshold for track deletion. The threshold depends
        %   on the type of track confirmation and deletion logic you use:
        %	  * History: specify the deletion threshold as [P R], where a
        %       track will be deleted if in the last R updates, at least P
        %       times it was not assigned to any detection.
        %	  * Integrated: specify the deletion threshold as a scalar in
        %       the range (0,1). A track will be deleted if its probability
        %       of existence drops below the threshold.
        %
        %   Default:
        %       History logic:	  [5 5]
        %       Integrated logic:  0.1
        DeletionThreshold;

        %DetectionProbability Probability of detection used for track score
        %   Specify the probability of detection expected for the track as
        %   a scalar in the range (0,1). The probability of detection is
        %   used in the marginal posterior probabilities of association and
        %   in the probability of track existence when initializing and
        %   updating a track.
        %
        %   Default = 0.9
        DetectionProbability;

        % EnableMemoryManagement Enable memory management properties Set
        % EnableMemoryManagement to true to specify bounds for certain
        % variable-sized arrays used inside the tracker using
        % MaxNumDetectionsPerSensor, MaxNumDetectionsPerCluster and
        % MaxNumTracksPerCluster properties. Specifying bounds allow you to
        % manage the memory footprint of the tracker in generated C/C++
        % code.
        % 
        % Default: false
        EnableMemoryManagement;

        %EventGenerationFcn Feasible Joint Events generation function name
        % Specify the function for generating feasible joint event matrices
        % corresponding to admissible assignments.
        % trackerJPDA clusters tracks and calls EventGenerationFcn on a
        % validation matrix for each cluster.
        % The function must have the following syntax if the
        % MaxNumEvents property is not finite
        %
        %       FJE = EventGenerationFcn(validationMatrix)
        %
        %   validationMatrix  - a m-by-n+1 binary numeric or logical matrix
        %   FJE               - a m-by-n+1-by-p logical array of feasible
        %                       joint events
        %
        % The function must have the following syntax if the
        % MaxNumEvents property is finite. This allows trackerJPDA to work
        % with k-best events.
        %
        %       [FJE, FJEProbs] = EventGenerationFcn(likelihoodMatrix, k)
        %  
        %   likelihoodMatrix - a m+1-by-n+1 matrix of posterior likelihood
        %                      of associations. The first row defines the
        %                      the likelihood of leaving tracks unassigned
        %                      and the first column defines the likelihood
        %                      of leaving detections unassigned.
        %   k                - Maximum number of feasible events to
        %                      generate.
        %   FJE              - a m-by-n+1-by-p logical array of feasible
        %                      joint events, where p must be less than k.
        %   FJEProbs         - A p-element vector of event probabilities. 
        %                      The sum of probabilities must be equal to 1.
        %   
        %   Default: 'jpdaEvents'
        %   <a href="matlab:edit('jpdaEvents')">Open jpdaEvents for more details.</a>
        EventGenerationFcn;

        %FilterInitializationFcn  Filter initialization function name
        %   Specify the function for initializing the tracking filter used
        %   by a new track. The function must have the following syntax:
        %
        %       filter = filterInitializationFcn(detection)
        %
        %   filter    - a valid tracking filter that implements the motion
        %               and measurement models required for tracking
        %   detection - an objectDetection that initializes the track
        %
        %   Default: @initcvekf
        %   <a href="matlab:edit('initcvekf')">Open initcvekf for more details.</a>
        FilterInitializationFcn;

        %HasCostMatrixInput Enable cost matrix input
        %   Set this property to true if you want to provide the assignment
        %   cost matrix as an input in the call to step
        %
        %   Default: false
        HasCostMatrixInput;

        %HasDetectableTrackIDsInput Enable detectable track IDs input
        %   Set this property to true if you want to provide the list of
        %   detectable track IDs. Use this list to inform the tracker of
        %   tracks that the sensors expected to detect and, optionally, the
        %   probability of detection for each track ID.
        %
        %   Default: false
        HasDetectableTrackIDsInput;

        %HitMissThreshold Threshold for registering a 'Hit' or a 'Miss'
        %   Specify the threshold that controls whether a track will
        %   register a 'hit' or a 'miss' as a scalar in the range (0,1).
        %   This property is only used when TrackLogic is set to 'History'.
        %   The track logic will register a miss and the track will be
        %   coasted if the sum of the marginal probabilities of assignments
        %   is below the value of HitMissThreshold. Otherwise, the track
        %   history logic will register a 'hit'. In either case, since the
        %   track is assigned detections, its state and state covariance is
        %   corrected.
        %
        %   Default: 0.2
        HitMissThreshold;

        %InitialExistenceProbability initial probability of track existence
        %This property depends on ClutterDensity, NewTargetDensity, and
        %DetectionProbability through the following equation:
        % InitialExistenceProbability = NewTargetDensity*DetectionProbability/ ...
        % (ClutterDensity+NewTargetDensity*DetectionProbability)
        InitialExistenceProbability;

        %InitializationThreshold Threshold to initialize a track
        %   Specify the probability threshold to initialize new tracks as a
        %   scalar in the range (0,1). For example, if a detection in the
        %   current scan was not assigned to an existing track with a joint
        %   probability of assignment greater than the
        %   InitializationThreshold value, this detection will be used to
        %   initialize a new track. This allows initiating tracks that are
        %   within the assignment gate of an existing track.
        %
        %   Default: 0
        InitializationThreshold;

        % MaxNumDetectionsPerCluster Maximum number of detections per cluster
        % Set the maximum number of detections per cluster that can be
        % expected during run-time as a positive integer. Setting
        % a finite value allows the tracker to bound cluster sizes and
        % reduces the memory footprint of the tracker in generated C/C++
        % code. If during run-time, the number of detections in a cluster
        % exceeds MaxNumDetectionsPerCluster, the behavior of the tracker
        % is determined by ClusterViolationHandling property.
        %
        % This property is active when EnableMemoryManagement is true.
        %
        % Default: 5
        MaxNumDetectionsPerCluster;

        % MaxNumDetectionsPerSensor Maximum number of detections per sensor
        % Set the maximum number of detections per sensor that can be
        % passed as an input to the tracker as a positive integer. Setting
        % a finite value allows the tracker to put efficient bounds on
        % local variables for C/C++ code generation. 
        %
        % This property is active when EnableMemoryManagement is true.
        %
        % Default: 100
        MaxNumDetectionsPerSensor;

        %MaxNumEvents Value of k for k-best JPDA
        % Specifying a finite value for this property allows you to run a
        % k-best approximation to JPDA tracker. The tracker will generate
        % maximum of k events per cluster. See property <a href="matlab:help
        % trackerJPDA\EventGenerationFcn">EventGenerationFcn</a> to see how
        % this property affects the syntax of EventGenerationFcn.
        %
        % Default: inf
        MaxNumEvents;

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

        %MaxNumSensors  Maximum number of sensors
        %   Set the maximum number of sensors connected to the
        %   trackerJPDA as a positive real integer.
        %   This number should be greater than or equal to the highest
        %   SensorIndex value used in the detections input to the
        %   step method. This property determines how many sets of
        %   ObjectAttributes each track can have.
        %
        %   Default: 20
        MaxNumSensors;

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
        % reduces the memory footprint of the tracker in generated C/C++
        % code. If during run-time, the number of tracks in a cluster
        % exceeds MaxNumTracksPerCluster, the behavior of the tracker is
        % determined by ClusterViolationHandling property.
        %
        % This property is active when EnableMemoryManagement is true.
        % 
        % Default: 5
        MaxNumTracksPerCluster;

        %NewTargetDensity Spatial density of new targets.
        %   Specify the expected number of new tracks per unit volume as a
        %   real scalar. It is used in calculating the initial probability
        %   of track existence when TrackLogic is set to 'Integrated'. This
        %   property is not used when TrackLogic is set to 'History'.
        %
        %   Default = 1e-5
        NewTargetDensity;

        %OOSMHandling Handle out-of-sequence measurement (OOSM)
        %   Choose out-of-sequence measurement (OOSM) handling technique
        %   option from [{'Terminate'},'Neglect','Retrodiction']. Each
        %   detection has a timestamp associated with it, td, and the
        %   tracker has it own timestamp, tt, which is updated with every
        %   call to step. A measurement is considered to be an OOSM if td < tt. 
        %   You can select how the tracker handles the OOSM:
        %     'Terminate'    - The tracker terminates its step on any OOSM.
        %     'Neglect'      - The tracker neglects any OOSM but keeps running.
        %     'Retrodiction' - The tracker uses the retrodiction algorithm. 
        OOSMHandling;

        %TimeTolerance Absolute tolerance between time stamps of detections
        %   Specify the tolerance in seconds between time stamps of
        %   detections originating from the same sensor. trackerJPDA
        %   expects detections from a sensor to have identical time stamps
        %   when they are grouped in the same cluster. Detections with time
        %   stamp differences within the TimeTolerance will be used to
        %   update the track estimate to their average time.
        %
        %   Default: 1e-5
        TimeTolerance;

        %TrackLogic  Type of track confirmation and deletion logic
        %   Specify the TrackLogic as [{'History'}|'Integrated']. The choices
        %   are:
        %     * 'History': track confirmation and deletion will be based on
        %       the number of times the track has been assigned to a
        %       detection in the last tracker updates.
        %     * 'Integrated': track confirmation and deletion will be based
        %       on the probability of track existence.
        %
        %   Default: 'History'
        TrackLogic;

        pClusterViolationHandlingType;

        %pCostMatrix holds the current cost Matrix for analysis information
        pCostMatrix;

        % Stores the observed values of maximum number of detections and
        % tracks per cluster to send to info.
        pCurrentMaxNumDetectionsPerCluster;

        pCurrentMaxNumTracksPerCluster;

        %pEventGenerationFcn handle of the function used to generate the
        %feasible joint events during the joint probability data
        %association coefficients calculations.
        pEventGenerationFcn;

        %Confirmation threshold [M N]
        pHistoryConfThreshold;

        %Deletion threshold [P Q]
        pHistoryDelThreshold;

        %Confirmation threshold (Probability)
        pIntegratedConfThreshold;

        %Deletion threshold (Probability)
        pIntegratedDelThreshold;

        % pLastTimeStamp Keeps the last time to which the tracker was updated
        pLastTimeStamp;

        %pLastTrackID ID of the last track initialized by the tracker
        %Needed to keep the TrackID unique
        pLastTrackID;

        % pMaxNumClusters A scalar value that defines an upper bound on
        % number of clusters. This value is calculated using MaxNumTracks
        % and MaxNumDetections
        pMaxNumClusters;

        % pMaxNumClustersPerSensor A scalar value that defines an upper
        % bound of number of clusters per sensor. 
        pMaxNumClustersPerSensor;

        %pSensorValidationMatrix holds the current validation matrix with
        %all current tracks and with the detections from a single sensor
        pSensorValidationMatrix;

        %pTrackLogic The type of track logic
        %	1 - History
        %	3 - Integrated
        pTrackLogic;

        %pUseRetrodiction A scalar logical value. True when OOSMHandling is
        %set to 'Retrodiction'.
        pUseRetrodiction;

    end
end

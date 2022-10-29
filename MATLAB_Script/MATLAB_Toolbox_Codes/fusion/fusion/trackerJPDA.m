classdef (StrictDefaults) trackerJPDA < matlabshared.tracking.internal.fusion.TrackManager ...
        & fusion.internal.ExportToSimulinkInterface ...
        & fusion.internal.TrackerMemoryManagementUtilities
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
    %     reset                      - Resets states of the trackerJPDA
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
    
    % References:
    % [1] Bar-Shalom, Yaakov, and Xiao-Rong Li. Multitarget-multisensor
    %     tracking: principles and techniques. Vol. 19. Storrs, CT: YBs, 1995.
    % [2] Challa, Subhash, Mark R. Morelande, Darko Musicki, and Robin J.
    %     Evans. Fundamentals of object tracking. Cambridge University Press,
    %     2011.
    
    % Copyright 2018-2021 The MathWorks, Inc.
    
    %#function jpdaEvents
    %#function initsingerekf
    %#codegen
    
    properties(Nontunable)
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
        FilterInitializationFcn = 'initcvekf'
        
        %MaxNumEvents Value of k for k-best JPDA
        % Specifying a finite value for this property allows you to run a
        % k-best approximation to JPDA tracker. The tracker will generate
        % maximum of k events per cluster. See property <a href="matlab:help
        % trackerJPDA\EventGenerationFcn">EventGenerationFcn</a> to see how
        % this property affects the syntax of EventGenerationFcn.
        %
        % Default: inf
        MaxNumEvents = inf;
        
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
        EventGenerationFcn = 'jpdaEvents'
    end
    
    properties(Nontunable)
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
        TrackLogic = 'History'
        
        %DetectionProbability Probability of detection used for track score
        %   Specify the probability of detection expected for the track as
        %   a scalar in the range (0,1). The probability of detection is
        %   used in the marginal posterior probabilities of association and
        %   in the probability of track existence when initializing and
        %   updating a track.
        %
        %   Default = 0.9
        DetectionProbability = 0.9
        
        %ClutterDensity Spatial density of clutter measurements
        %   Specify the expected number of false positives per unit volume
        %   as a real scalar. The clutter density is used as the parameter
        %   of a Poisson clutter model. It is also used in calculating the
        %   initial probability of track existence when TrackLogic is set
        %   to 'Integrated'.
        %
        %   Default = 1e-6
        ClutterDensity = 1e-6
        
        %NewTargetDensity Spatial density of new targets.
        %   Specify the expected number of new tracks per unit volume as a
        %   real scalar. It is used in calculating the initial probability
        %   of track existence when TrackLogic is set to 'Integrated'. This
        %   property is not used when TrackLogic is set to 'History'.
        %
        %   Default = 1e-5
        NewTargetDensity = 1e-5
        
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
        DeathRate = 0.01;
        
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
        ConfirmationThreshold
        
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
        DeletionThreshold
        
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
        AssignmentThreshold = [1 Inf] * 30.0
        
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
        HitMissThreshold = 0.2;
        
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
        InitializationThreshold = 0
        
        %TimeTolerance Absolute tolerance between time stamps of detections
        %   Specify the tolerance in seconds between time stamps of
        %   detections originating from the same sensor. trackerJPDA
        %   expects detections from a sensor to have identical time stamps
        %   when they are grouped in the same cluster. Detections with time
        %   stamp differences within the TimeTolerance will be used to
        %   update the track estimate to their average time.
        %
        %   Default: 1e-5
        TimeTolerance = 1e-5;
    end
    
    properties(Nontunable)
        %MaxNumTracks   Maximum number of tracks
        %   Set the maximum number of tracks the tracker can maintain as a
        %   positive real integer.
        %
        %   Default: 100
        MaxNumTracks = 100
        
        %MaxNumSensors  Maximum number of sensors
        %   Set the maximum number of sensors connected to the
        %   trackerJPDA as a positive real integer.
        %   This number should be greater than or equal to the highest
        %   SensorIndex value used in the detections input to the
        %   step method. This property determines how many sets of
        %   ObjectAttributes each track can have.
        %
        %   Default: 20
        MaxNumSensors = 20
        
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
    end
    
    properties(Hidden,Constant)
        OOSMHandlingSet = matlab.system.StringSet({'Terminate','Neglect','Retrodiction'});
        TrackLogicSet = matlab.system.StringSet({'History';'Integrated'})
    end
    
    properties(Nontunable)
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
        OOSMHandling = 'Terminate'        
    end
    
    properties(Nontunable)
        %HasDetectableTrackIDsInput Enable detectable track IDs input
        %   Set this property to true if you want to provide the list of
        %   detectable track IDs. Use this list to inform the tracker of
        %   tracks that the sensors expected to detect and, optionally, the
        %   probability of detection for each track ID.
        %
        %   Default: false
        HasDetectableTrackIDsInput (1, 1) logical = false
        
        %HasCostMatrixInput Enable cost matrix input
        %   Set this property to true if you want to provide the assignment
        %   cost matrix as an input in the call to step
        %
        %   Default: false
        HasCostMatrixInput (1, 1) logical = false
    end
    
    properties(Access = protected)
        %pCostMatrix holds the current cost Matrix for analysis information
        pCostMatrix
        
        %pSensorValidationMatrix holds the current validation matrix with
        %all current tracks and with the detections from a single sensor
        pSensorValidationMatrix
        
        %pLastTrackID ID of the last track initialized by the tracker
        %Needed to keep the TrackID unique
        pLastTrackID
        
        % pLastTimeStamp Keeps the last time to which the tracker was updated
        pLastTimeStamp
        
        %pEventGenerationFcn handle of the function used to generate the
        %feasible joint events during the joint probability data
        %association coefficients calculations.
        pEventGenerationFcn
    end
    
    properties(Access = {?matlab.unittest.TestCase, ?trackerJPDA})
        % Keeps history of all timestamps
        pTimeStampHistory
    end
    
    properties(Access = protected, Nontunable)
        %pTrackLogic The type of track logic
        %	1 - History
        %	3 - Integrated
        pTrackLogic = 1
        
        % pMaxNumClusters A scalar value that defines an upper bound on
        % number of clusters. This value is calculated using MaxNumTracks
        % and MaxNumDetections
        pMaxNumClusters;

        % pMaxNumClustersPerSensor A scalar value that defines an upper
        % bound of number of clusters per sensor. 
        pMaxNumClustersPerSensor;

        %pUseRetrodiction A scalar logical value. True when OOSMHandling is
        %set to 'Retrodiction'.
        pUseRetrodiction = false
    end

    properties(Access = private)
        %pAssignmentThreshold Holds the value of AssignmentThreshold
        pAssignmentThreshold
        
        %pWasDetectable	Lets the tracker know which tracks were detectable
        % by the sensors
        pWasDetectable
        
        %pTrackDetectionProbability Detection probability for particular
        % tracks. Used if DetectableTrackIDs is enabled.
        pTrackDetectionProbability
        
        %pClusterTypeDef Type definition for cluster structure
        pClusterTypeDef
    end
    
    properties(Nontunable)
        %Confirmation threshold (Probability)
        pIntegratedConfThreshold = 0.95
        
        %Deletion threshold (Probability)
        pIntegratedDelThreshold = 0.1
        
        %Confirmation threshold [M N]
        pHistoryConfThreshold = [2 3]
        
        %Deletion threshold [P Q]
        pHistoryDelThreshold = [5 5]
    end    
    
    properties(SetAccess = private, Dependent)
        %InitialExistenceProbability initial probability of track existence
        %This property depends on ClutterDensity, NewTargetDensity, and
        %DetectionProbability through the following equation:
        % InitialExistenceProbability = NewTargetDensity*DetectionProbability/ ...
        % (ClutterDensity+NewTargetDensity*DetectionProbability)
        InitialExistenceProbability
    end   
    
    properties(Access = private)
        %cCostCalculator - cost calculation object
        cCostCalculator
    end
    
    properties(Access=protected, Hidden)
        pVersion = ver('fusion');
    end
    
    properties (Access = protected)
        % Stores the observed values of maximum number of detections and
        % tracks per cluster to send to info.
        pCurrentMaxNumDetectionsPerCluster = uint32(0);
        pCurrentMaxNumTracksPerCluster = uint32(0);
    end

    properties (Nontunable, Logical)
        % EnableMemoryManagement Enable memory management properties Set
        % EnableMemoryManagement to true to specify bounds for certain
        % variable-sized arrays used inside the tracker using
        % MaxNumDetectionsPerSensor, MaxNumDetectionsPerCluster and
        % MaxNumTracksPerCluster properties. Specifying bounds allow you to
        % manage the memory footprint of the tracker in generated C/C++
        % code.
        % 
        % Default: false
        EnableMemoryManagement = false;
    end
    
    properties (Nontunable)
        % MaxNumDetectionsPerSensor Maximum number of detections per sensor
        % Set the maximum number of detections per sensor that can be
        % passed as an input to the tracker as a positive integer. Setting
        % a finite value allows the tracker to put efficient bounds on
        % local variables for C/C++ code generation. 
        %
        % This property is active when EnableMemoryManagement is true.
        %
        % Default: 100
        MaxNumDetectionsPerSensor = 100;

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
        MaxNumDetectionsPerCluster = 5;

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
        % This property is active when EnableMemoryManagement is true
        %
        % Default: 'Split and warn'
        ClusterViolationHandling = 'Split and warn';
    end

    properties (Constant, Hidden)
        ClusterViolationHandlingSet = matlab.system.StringSet({'Terminate','Split and warn','Split'});
    end
    
    properties (Nontunable, Access = protected)
        pClusterViolationHandlingType
    end

    properties(Access = private, Constant = true)
        %constAssignmentExpansion A constant to expand AssignmentThreshold
        constAssignmentExpansion = [1,Inf];
        
        %constTimeTol A constant to validate detections times
        constTimeTol = 1e-5;
        
    end
    % ------------------------------------------------------------------
    % ------------------------------------------------------------------
    
    %% Methods
    
    %% Public
    methods
        % Constructor -----------------------------------------------------
        function obj = trackerJPDA(varargin)
            % You call setProperties in the constructor to let
            % a user specify public properties of object as
            % name-value pairs.
            setProperties(obj,nargin,varargin{:})
        end
        % Getters and Setters----------------------------------------------
        function set.TrackLogic(obj,value)
            obj.TrackLogic = value;
            setTrackLogic(obj)
        end
        
        function setTrackLogic(obj)
            if strcmpi(obj.TrackLogic, 'History')
                obj.pTrackLogic = 1;
            else
                obj.pTrackLogic = 3;
            end
        end
        
        function val= get.InitialExistenceProbability(obj)
            if obj.pTrackLogic == 3 % Integrated
                bPd = obj.NewTargetDensity*obj.DetectionProbability;
                val = bPd/(obj.ClutterDensity+bPd);
            else
                val = 1;
            end
        end
        
        function set.AssignmentThreshold(obj,value)
            if isscalar(value)
                validateattributes(value,{'numeric'},...
                    {'real','finite','nonsparse','vector','nrows',1},...
                    mfilename,'AssignmentThreshold');
                obj.AssignmentThreshold = value(1,1) * obj.constAssignmentExpansion;
            else
                validateattributes(value,{'numeric'},...
                    {'real','nonsparse','vector','nondecreasing','numel',2},...
                    mfilename,'AssignmentThreshold');
                validateattributes(value(1),{'numeric'},{'finite'},...
                    mfilename,'AssignmentThreshold(1)');
                obj.AssignmentThreshold = (value(:))';
            end
        end
        
        function set.pIntegratedConfThreshold(obj,value)
            validateattributes(value, {'numeric'}, ...
                {'positive', 'real', 'scalar','<',1}, ...
                'trackerJPDA', 'ConfirmationThreshold');
            obj.pIntegratedConfThreshold = value;
        end
        
        function set.pIntegratedDelThreshold(obj,value)
            validateattributes(value, {'numeric'}, ...
                {'positive', 'real', 'scalar','<',1}, ...
                'trackerJPDA', 'DeletionThreshold');
            obj.pIntegratedDelThreshold = value;
        end
        
        function set.pHistoryConfThreshold(obj,value)
            if isscalar(value)
                modval = [value,value];
            else
                modval = value;
            end
            validateattributes(modval, {'numeric'}, ...
                {'positive', 'real', 'integer', 'nondecreasing', 'numel', 2}, ...
                'trackerJPDA', 'ConfirmationThreshold');
            obj.pHistoryConfThreshold = modval;
        end
        
        function set.pHistoryDelThreshold(obj,value)
            if isscalar(value)
                modval = [value, value];
            else
                modval = value;
            end
            validateattributes(modval, {'numeric'}, ...
                {'positive', 'real', 'integer', 'nondecreasing', 'numel', 2}, ...
                'trackerJPDA', 'DeletionThreshold');
            obj.pHistoryDelThreshold = modval;
        end
        
        function set.ConfirmationThreshold(obj, value)
            setConfThreshold(obj,value)
            obj.ConfirmationThreshold = value;
        end
        
        function setConfThreshold(obj,value)
            if strcmpi(obj.TrackLogic, 'History')
                if isscalar(value)
                    modval = [value,value];
                else
                    modval = value;
                end
                validateattributes(modval, {'numeric'}, ...
                    {'positive', 'real', 'integer', 'nondecreasing', 'numel', 2}, ...
                    'trackerJPDA', 'ConfirmationThreshold');
                obj.pHistoryConfThreshold = modval;
            else
                validateattributes(value, {'numeric'}, ...
                    {'positive', 'real', 'scalar','<',1}, ...
                    'trackerJPDA', 'ConfirmationThreshold');
                obj.pIntegratedConfThreshold = value;
            end
            if coder.internal.is_defined(obj.pTracksList)
                for i = obj.IntOne:obj.IntNumel(obj.pTracksList)
                    obj.pTracksList{i}.TrackLogic.ConfirmationThreshold = obj.ConfirmationThreshold;
                end
            end
        end
        
        function set.DetectionProbability(obj,value)
            validateattributes(value, {'double','single'},...
                {'real','positive','scalar','nonsparse','finite','<',1},mfilename,'DetectionProbability');
            obj.DetectionProbability = value;
        end
        
        function set.ClutterDensity(obj,value)
            validateattributes(value, {'double','single'},...
                {'real','positive','scalar','nonsparse','finite','<',1},mfilename,'ClutterDensity');
            obj.ClutterDensity = value;
        end
        
        function set.InitializationThreshold(obj,value)
            validateattributes(value, {'double','single'},...
                {'real','nonnegative','scalar','nonsparse','finite','<',1},mfilename,'InitializationThreshold');
            obj.InitializationThreshold = value;
        end
        
        function set.TimeTolerance(obj,value)
            validateattributes(value, {'double','single'},...
                {'real','nonnegative','scalar','nonsparse','finite'},mfilename,'TimeTolerance');
            obj.TimeTolerance = value;
        end
        
        function val = get.ConfirmationThreshold(obj)
            if strcmpi(obj.TrackLogic, 'History')
                val = obj.pHistoryConfThreshold;
            else
                val = obj.pIntegratedConfThreshold;
            end
        end
        
        function set.DeletionThreshold(obj, value)
            setDelThreshold(obj,value);
            obj.DeletionThreshold = value;
        end
        
        function setDelThreshold(obj,value)
            if strcmpi(obj.TrackLogic, 'History')
                if isscalar(value)
                    modval = [value, value];
                else
                    modval = value;
                end
                validateattributes(modval, {'numeric'}, ...
                    {'positive', 'real', 'integer', 'nondecreasing', 'numel', 2}, ...
                    'trackerJPDA', 'DeletionThreshold');
                obj.pHistoryDelThreshold = modval;
            else
                validateattributes(value, {'numeric'}, ...
                    {'positive', 'real', 'scalar','<',1}, ...
                    'trackerJPDA', 'DeletionThreshold');
                obj.pIntegratedDelThreshold = value;
            end
            if coder.internal.is_defined(obj.pTracksList)
                for i = obj.IntOne:obj.IntNumel(obj.pTracksList)
                    obj.pTracksList{i}.TrackLogic.DeletionThreshold = obj.DeletionThreshold;
                end
            end
        end
        
        function validateConfDel(obj)
            coder.internal.assert(obj.DeletionThreshold <= obj.ConfirmationThreshold, ...
                'fusion:trackerJPDA:confdel','DeletionThreshold','ConfirmationThreshold');
        end
        
        function val = get.DeletionThreshold(obj)
            if strcmpi(obj.TrackLogic, 'History')
                val = obj.pHistoryDelThreshold;
            else
                val = obj.pIntegratedDelThreshold;
            end
        end
        
        function set.MaxNumTracks(obj, value)
            validateattributes(value, {'numeric'}, ...
                {'positive', 'real', 'integer', 'scalar'}, ...
                'trackerJPDA', 'MaxNumTracks');
            obj.MaxNumTracks = value;
        end
        
        function set.MaxNumEvents(obj, value)
            % First validate the value is numeric
            validateattributes(value,{'numeric'},{},...
                'trackerJPDA','MaxNumEvents');
            % If value is finite, add additional validations
            if isfinite(value)
                validateattributes(value,{'numeric'},...
                {'scalar','nonsparse','integer','positive'},...
                'trackerJPDA','MaxNumEvents');
            end
            obj.MaxNumEvents = value;
        end

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
            %   tracker = trackerJPDA;
            %   detection = objectDetection(0, [0;0;0]);
            %   [~, tracks] = tracker(detection, 0);
            %   values = getTrackFilterProperties(tracker, 1, 'MeasurementNoise', 'ProcessNoise');
            validateattributes(trackID, {'numeric'}, ...
                {'real', 'positive', 'scalar', 'finite', 'integer'}, 'getTrackFilterProperties', ...
                'trackID');
            for i = 1:numel(varargin)
                validateattributes(varargin{i}, {'char','string'}, {'nonempty'}, ...
                    'getTrackFilterProperties', 'properties');
            end
            
            numProps = numel(varargin);
            values = cell(numProps, 1);
            index = findTrackByID(obj, trackID);
            if isempty(index)
                coder.internal.warning('fusion:trackerJPDA:TrackIDNotFound', trackID);
                for i = 1:numProps
                    values{i} = [];
                end
            else
                for i = 1:numProps
                    values{i} = obj.pTracksList{index(1)}.Filter.(varargin{i});
                end
            end
        end
        
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
            %   tracker = trackerJPDA;
            %   detection = objectDetection(0, [0;0;0]);
            %   [~, tracks] = tracker(detection, 0);
            %   setTrackFilterProperties(tracker, 1, 'MeasurementNoise', 2, 'ProcessNoise', 5);
            %   values = getTrackFilterProperties(tracker, 1, 'MeasurementNoise', 'ProcessNoise');
            validateattributes(trackID, {'numeric'}, ...
                {'real', 'positive', 'scalar', 'finite', 'integer'}, 'setTrackFilterProperties', ...
                'trackID');
            
            index = findTrackByID(obj, trackID);
            if isempty(index)
                coder.internal.warning('fusion:trackerJPDA:TrackIDNotFound', trackID);
            else
                numProps = numel(varargin);
                coder.internal.errorIf(mod(numProps,2) > 0, 'fusion:trackerJPDA:OddNameValuePairs');
                for i = 1:2:numProps % Note: property name and value validations are done by the filter
                    validateattributes(varargin{i}, {'char','string'}, {'nonempty'}, ...
                        'setTrackFilterProperties', 'Name');
                    filter = obj.pTracksList{index(1)}.Filter;
                    filter.(varargin{i}) = varargin{i+1};
                end
            end
        end
        
        function predictedTracks = predictTracksToTime(obj, track, time, varargin)
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
            
            % Has the tracker locked yet?
            coder.internal.assert(isLocked(obj),'fusion:trackerJPDA:PredictBeforeUpdate','predictTracksToTime');
            
            %Input validation
            narginchk(3,5);
            validateattributes(time, {'numeric'}, ...
                {'real', 'finite', 'nonsparse', 'scalar', '>', obj.pLastTimeStamp}, ...
                'predictTracksToTime', 'time');
            
            % Assist coder with bounded size
            n = obj.pNumLiveTracks;
            assert(n <= obj.MaxNumTracks);
            
            if ischar(track) || isstring(track)
                type = validatestring(track,{'all','confirmed','tentative'},...
                    'predictTracksToTime','category');
                if strcmpi(type,'all')
                    list = true(1,n);
                elseif strcmpi(type,'confirmed')
                    list = obj.pConfirmedTracks;
                else
                    list = ~obj.pConfirmedTracks(1:n);
                end
            else
                validateattributes(track, {'numeric'}, ...
                    {'real', 'positive', 'scalar', 'integer'}, ...
                    'predictTracksToTime', 'trackID');
                list = false(1,n);
                ind = findTrackByID(obj, track);
                if isempty(ind)
                    coder.internal.warning('fusion:trackerJPDA:TrackIDNotFound', track);
                else
                    list(ind) = true;
                end
            end
            
            % Check flag if varargin is not empty
            if ~isempty(varargin)
                coder.internal.assert(numel(varargin)==2,'fusion:trackerJPDA:PredictToTimeNargin',3,5)
                validatestring(varargin{1},{'WithCovariance'},mfilename);
                validateattributes(varargin{2},{'numeric','logical'},...
                    {'scalar','binary'},mfilename,'tf');
                withCov = varargin{2};
            else
                withCov = false;
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
        end
        %------------------------------------------------------------------
        
        function deleted = deleteTrack(obj, trackID)
            %deleteTrack  Delete a track managed by the tracker
            %    deleted = deleteTrack(obj,trackID) deletes the track
            %    specified by trackID from the tracker. The deleted
            %    flag returns true if a track with the same trackID existed
            %    and was deleted. If a track with that trackID did not
            %    exist, the deleted flag is false and a warning is issued.
            %
            % Note: the tracker must be updated at least once to be able to
            % delete a track.
            
            % Has the tracker locked yet?
            coder.internal.assert(isLocked(obj),'fusion:trackerJPDA:PredictBeforeUpdate','deleteTrack');
            
            %Input validation
            narginchk(2,2);
            
            % Use the internal manager to delete the track
            deleted = deleteTrack@matlabshared.tracking.internal.fusion.TrackManager(obj, trackID);
            
            % Warning if not deleted
            if ~deleted
                coder.internal.warning('fusion:trackerJPDA:TrackIDNotFound', trackID);
            end
        end
        %------------------------------------------------------------------
                
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
            trackID = obj.pLastTrackID + 1;
            tf = initializeTrack@matlabshared.tracking.internal.fusion.TrackManager(obj,trackID,trackObj,varargin{:});
            if tf
                obj.pLastTrackID = trackID;
            else
                trackID = zeros(1,'like',trackID);
                coder.internal.warning('fusion:trackerJPDA:MaxNumTracksReached', 'MaxNumTracks');
            end
        end
    end
    
    %% Protected
    methods(Access=protected)
        %% Common functions
        
        %---Release -------------------------------------------------------
        
        function releaseImpl(obj)
            % Release resources, such as file handles
            releaseImpl@matlabshared.tracking.internal.fusion.TrackManager(obj);
            obj.pLastTrackID = uint32(0);
            obj.pLastTimeStamp = cast(-eps, 'like', obj.pLastTimeStamp); %Has to be negative to run tracker from t=0
        end
        
        function validatePropertiesImpl(obj)
            % Validate related or interdependent property values
            
            % Filter Initialization Validation:
            validateattributes(obj.FilterInitializationFcn, ...
                {'function_handle','char','string'}, {'nonempty'}, 'trackerJPDA', ...
                'FilterInitializationFcn');
            
            if isa(obj.FilterInitializationFcn, 'function_handle')
                obj.pFilterInitializationFcn = obj.FilterInitializationFcn;
            else
                obj.pFilterInitializationFcn = str2func(obj.FilterInitializationFcn);
            end
            % Check the filter initialization function syntax
            isInvalid = ~matlabshared.tracking.internal.fusion.isValidInitFcn(obj.pFilterInitializationFcn);
            if isInvalid
                coder.internal.error('fusion:trackerJPDA:InvalidFilterInitFcn', 'FilterInitializationFcn');
            end
            
            % Event generation validation
            validateattributes(obj.EventGenerationFcn, ...
                {'function_handle','char'}, {'nonempty'}, 'trackerJPDA', ...
                'EventGenerationFcn');
            if isa(obj.EventGenerationFcn, 'function_handle')
                obj.pEventGenerationFcn = obj.EventGenerationFcn;
            else
                obj.pEventGenerationFcn = str2func(obj.EventGenerationFcn);
            end
            
            if ~isfinite(obj.MaxNumEvents)
                isValid = matlabshared.tracking.internal.fusion.isValidInitFcn(obj.pEventGenerationFcn);
                coder.internal.assert(isValid,'fusion:trackerJPDA:InvalidEventGenFcn', 'EventGenerationFcn');
            else
                isValid = isValidKBestEventFcn(obj.pEventGenerationFcn);
                coder.internal.assert(isValid, 'fusion:trackerJPDA:InvalidKBestEventGenFcn', 'EventGenerationFcn');
            end

            % Check that cost matrix input is disabled with retrodiction
            coder.internal.errorIf(strcmpi(obj.OOSMHandling,'Retrodiction') ...
                && obj.HasCostMatrixInput, 'fusion:trackerJPDA:RetrodictionWithCostMatrix', ...
                'OOSMHandling', 'Retrodiction', 'HasCostMatrixInput');

            % Check that track logic is history with retrodiction
            coder.internal.errorIf(strcmpi(obj.TrackLogic,'Integrated') ...
                && strcmpi(obj.OOSMHandling,'Retrodiction'), 'fusion:trackerJPDA:RetrodictionWithIntegrated', ...
                'OOSMHandling', 'Retrodiction', 'TrackLogic', 'History');

            
            % Validate properties used with Integrated logic
            if strcmp(obj.TrackLogic,'Integrated')
                validateattributes(obj.DeathRate, {'double','single'},...
                    {'real','nonnegative','scalar','nonsparse','finite','<',1},mfilename,'DeathRate');
                validateattributes(obj.NewTargetDensity, {'double','single'},...
                    {'real','nonnegative','scalar','nonsparse','finite','<',1},mfilename,'NewTargetDensity');
            end
            
            % Check Conf threshold > Del threshold
            if strcmpi(obj.TrackLogic,'Integrated')
                validateConfDel(obj);
            end            
        end
        
        function validateInputsImpl(obj,~,varargin)
            % Validate time input
            validateTimeInput(obj,varargin{:});
            if obj.HasDetectableTrackIDsInput
                validateDetectableTrackIDs(obj,varargin{:});
            end
        end
        
        function validateTimeInput(obj,varargin)
            time = varargin{1};
            % Validate inputs to the step method at initialization
            validateattributes(time, {'numeric'}, {'real', 'nonsparse', ...
                'finite', 'scalar', 'nonnegative'}, 'step', 'time');
            if coder.internal.is_defined(obj.pLastTimeStamp)
                % Error out if the time input is not greater than obj.
                coder.internal.errorIf(time <= obj.pLastTimeStamp, ...
                    'fusion:trackerJPDA:TimeMustIncrease','step');
            end
        end

        function validateDetectableTrackIDs(obj,varargin)
            detectables = getDetectableTrackIDsFromInput(obj,varargin{:});
            validateattributes(detectables, {'numeric'}, {'real', ...
                'nonsparse', '2d'}, 'step', 'detectableTrackIDs');
        end

        function setupImpl(obj, detections, varargin)
            % Setup properties that will not be modified later on
            % Perform one-time calculations, such as computing constants
            
            obj.pMaxNumBranches = obj.MaxNumTracks;
            setupImpl@matlabshared.tracking.internal.fusion.TrackManager(obj, detections);            
            
            % Validate event generation function
            validateEventGenerationFcn(obj);            
            
            %pClusterTypeDef
            obj.pClusterTypeDef = struct('DetectionIndices',zeros(1,0,'uint32'),...
                'TrackIDs',zeros(1,0,'like', obj.pTracksList{1}.TrackID),...
                'ValidationMatrix',zeros(0,0,'logical'),...
                'SensorIndex',uint32(0),...
                'TimeStamp',cast(0,obj.pClassToUse),...
                'MarginalProbabilities',zeros(0,0,obj.pClassToUse));
            
            obj.pTrackDetectionProbability = obj.DetectionProbability * ones(obj.MaxNumTracks,1,obj.pClassToUse);
            
            % Set the cost calculator object
            obj.cCostCalculator = matlabshared.tracking.internal.fusion.AssignmentCostCalculator(...
                'MaxAssignmentCost', obj.AssignmentThreshold(end));

            % Set up the cost calculator to make it ready to use
            setup(obj.cCostCalculator, obj.pTracksList, {obj.pSampleDetection}, ...
                obj.pNumLiveTracks, zeros(0,1,'uint32'));
            
            obj.pMaxNumClustersPerSensor = min(obj.MaxNumTracks,getMaxNumDetectionsPerSensor(obj));
            obj.pMaxNumClusters = min(obj.pMaxNumClustersPerSensor*obj.MaxNumSensors, getMaxNumInputDetections(obj));           
            obj.pClusterViolationHandlingType = getClusterViolationHandlingType(obj);

            if strcmpi(obj.OOSMHandling, 'Retrodiction')
                obj.pUseRetrodiction = true;
                obj.pTimeStampHistory = zeros(1,obj.MaxNumOOSMSteps,obj.pClassToUse);
            end
            
        end
        
        % Validate filter supports JPDA correction
        function validateFilterOnSetup(~, f)
            coder.internal.assert(fusion.internal.isJPDATrackingFilter(f), ...
                'fusion:trackerJPDA:InvalidJPDAFilterInitFcn', ...
                'FilterInitializationFcn');
        end

        function verifyOOSMHandling(obj,filter)
            if strcmpi(obj.OOSMHandling,'Retrodiction')
                coder.internal.assert(isa(filter,'matlabshared.tracking.internal.RetrodictionFilter'),...
                    'fusion:trackerJPDA:ExpectedRetrodictionFilter','OOSMHandling','Retrodiction','FilterInitializationFcn');
            end
        end

        % Validate EventGenerationFcn
        function validateEventGenerationFcn(obj)
            % If MaxNumEvents is not finite
            if ~isfinite(obj.MaxNumEvents)
                % Try to validate EventGenerationFcn on a prototype validation
                % matrix
                valMatrix = ones(2);
                fje = obj.pEventGenerationFcn(valMatrix);
                % verify that fje is logical, has the correct size
                [fjex,fjey,~]=size(fje);
                cond = (islogical(fje) && fjex==2 && fjey==2);
                coder.internal.assert(cond,'fusion:trackerJPDA:InvalidEventGenFcn', 'EventGenerationFcn');
            else
                % Try to validate EventGenerationFcn on a protoype
                % likelihood matrix and k
                lhoodMatrix = ones(5,5,obj.pClassToUse);
                k = obj.MaxNumEvents;
                [fje, fjeprobs] = obj.pEventGenerationFcn(coder.ignoreConst(coder.ignoreSize(lhoodMatrix)),k);
                % verify that fje is logical, has the correct size
                [fjex,fjey,nfje]=size(fje);
                cond = (islogical(fje) && fjex==4 && fjey==5 && nfje==k && numel(fjeprobs)==k);
                coder.internal.assert(cond,'fusion:trackerJPDA:InvalidKBestEventGenFcn', 'EventGenerationFcn');
            end
        end
        
        %---Reset ---------------------------------------------------------
        function resetImpl(obj)
            % Returns the tracker to its initial state.
            
            % Reset tracks
            resetImpl@matlabshared.tracking.internal.fusion.TrackManager(obj);
            
            % Reset cost calculator
            reset(obj.cCostCalculator);      

            sts = getSampleTime(obj);   
            if sts.SampleTime > 0
                obj.pLastTimeStamp = cast(-sts.SampleTime, 'like', obj.pLastTimeStamp);
            else
                obj.pLastTimeStamp = cast(-eps, 'like', obj.pLastTimeStamp); %Has to be negative to run tracker from t=0
            end
            obj.pLastTrackID = uint32(0);
            obj.pWasDetectable = true(obj.MaxNumTracks,1);
            obj.pAssignmentThreshold = obj.AssignmentThreshold;

            if obj.pUseRetrodiction
                obj.pTimeStampHistory = zeros(1,obj.MaxNumOOSMSteps,obj.pClassToUse);
            end
        end
        
        %---Step ----------------------------------------------------------
        
        function varargout = stepImpl(obj, detsIn, varargin)
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
            
            %Process inputs.
            [dets,time] = processInputs(obj,detsIn,varargin{:});
            
            %Run the core algorithm.
            info = coreAlgorithm(obj,dets,time,nargout,varargin{:});
            
            %Format outputs.
            [varargout{1:nargout}] = processOutputs(obj,info);
        end

        function infoTracker = coreAlgorithm(obj,dets,time,nOutArgs,varargin)
            % Keep track IDs at the beginning of the step
            trIDsBeg = getLiveTrackIDs(obj);
            numLiveTracksBeg = obj.pNumLiveTracks;

                        
            % Cluster and assign out-of-sequence detections to tracks
            if obj.pUseRetrodiction
                [discardedDetections, unassignedDetectionIdOOSM, ...
                    clustersOOSM, oosmCost] = ...
                    algorithmRetrodiction(obj,dets,time);
            end
            
            % TrackIDs after initialization with OOSM
            trIDsAfterOOSM = getLiveTrackIDs(obj);
            trIDsInitAtOOSM = obj.pTrackIDs(numLiveTracksBeg+1:obj.pNumLiveTracks);

            % Get cost matrix for in-sequence detections and live tracks
            costMatrix = getCostMatrix(obj, dets, varargin{:});

            % Process detectableTrackIDs with or without input
            trackDetectability(obj, varargin{:})

            % Predict all tracks to the first future sensor time
            inseq = obj.cDetectionManager.isInSequence;
            if any(inseq)
                dTimes = detectionTimes(obj.cDetectionManager);
                tmin = min(dTimes(inseq));
                predictTracks(obj,tmin);
            else
                predictTracks(obj,time);
            end

            % Cluster and assign in-sequence detections to tracks
            [unassignedDetectionId, unassignedTrackId,clusters] = ...                
                clusterAssignUpdate(obj,costMatrix,dets,inseq);
            
            % Initialize tracks for unassigned detections.
            trIDsInitInSequence = initializeTracks(obj,unassignedDetectionId,dets);
            initTrackIDs = [trIDsInitAtOOSM,trIDsInitInSequence];

            % Coast tracks that were not assigned any detections
            coastUnassignedTracks(obj,unassignedTrackId)

            % Predict all tracks to the first sensor time
            predictTracks(obj,time);

            % Delete tracks with low probability of existence / logic
            delTrIDs = deleteOldTracks(obj);

            % Add the analysis information struct
            if ~isempty(unassignedTrackId)
                unasTrs = trIDsAfterOOSM(unassignedTrackId);
            else
                unasTrs = zeros(1,0,'like',obj.pTrackIDs);
            end

            if isInfoRequested(obj,nOutArgs)
                infoTracker = getTrackerInfo(obj, trIDsBeg, ...
                    unasTrs, ...
                    unassignedDetectionId,...
                    costMatrix,...
                    clusters,...
                    initTrackIDs,delTrIDs);

                if obj.pUseRetrodiction
                    infoTracker.OOSMHandling = getOOSMInfo(obj,...
                        discardedDetections, unassignedDetectionIdOOSM,...
                        oosmCost, clustersOOSM);
                end
            else
                infoTracker = repmat(struct,0,1);
            end
        end

        function [dets,time] = processInputs(obj,inDetections,varargin)
            % In MATLAB: time is always the 1st part of varargin
            time = varargin{1};
            % Error out if the time input is not greater than obj.pLastTimeStamp
            coder.internal.errorIf(time <= obj.pLastTimeStamp, ...
                'fusion:trackerJPDA:TimeMustIncrease','step');
           
            % Pre-process the incoming detections
            preprocessDetections(obj, inDetections, time);
            dets = detections(obj.cDetectionManager);
        end

        function varargout = processOutputs(obj,info)
            if nargout > 3
                [varargout{1:3}] = formatTrackOutputs(obj,3);
                varargout{4} = info;
            else
                [varargout{1:nargout}] = formatTrackOutputs(obj,nargout);
            end
        end

        function infoTracker = getTrackerInfo(obj, trIDsBeg, unasTrs, ...
                unassignedDetectionId,costMatrix,clusters,...
                initTrIDs,delTrIDs)

            % Inlining assists coder to optimize the output variable when
            % info is not an output
            coder.inline('always');

            clusterInfo = getClusterInfo(obj, clusters);

            infoTracker = struct(...
                'OOSMDetectionIndices',oosmIndices(obj.cDetectionManager),...
                'TrackIDsAtStepBeginning',trIDsBeg, ...
                'UnassignedTracks',unasTrs, ...
                'UnassignedDetections',cast(unassignedDetectionId,'uint32'), ...
                'CostMatrix',costMatrix(:,~obj.cDetectionManager.isOOSM), ...
                'Clusters', {clusterInfo}, ...
                'InitializedTrackIDs',initTrIDs, ...
                'DeletedTrackIDs',delTrIDs, ...
                'TrackIDsAtStepEnd',obj.pTrackIDs(1:obj.pNumLiveTracks));
            if obj.EnableMemoryManagement
                infoTracker.MaxNumDetectionsPerCluster = obj.pCurrentMaxNumDetectionsPerCluster;
                infoTracker.MaxNumTracksPerCluster = obj.pCurrentMaxNumTracksPerCluster;
            end
        end

        function oosmInfo = getOOSMInfo(obj,discardedDetections, unassignedDetectionIdOOSM,costMatrix, clusters)
            coder.inline('always');

            % Output cluster only info if requested
            clusterInfoOOSM = getClusterInfo(obj, clusters);

            oosmInfo = struct(...
                'DiscardedDetections',discardedDetections, ...
                'CostMatrix', costMatrix(:,obj.cDetectionManager.isOOSM),...
                'Clusters',{clusterInfoOOSM},...
                'UnassignedDetections',cast(unassignedDetectionIdOOSM,'uint32'));

        end

        %% StepImpl Methods

        function [discardedDetections, unassignedDetectionIdOOSM, clusterOOSM, costMatrix] = ...
                algorithmRetrodiction(obj,dets, time)

            % Calculate cost matrix for Out-Of-Sequence detections
            
            detTimes = obj.cDetectionManager.detectionTimes;
            oldestTime = min(obj.pTimeStampHistory);
            oldDets = detTimes < oldestTime;
            discardedDetections = uint32(reshape(find(oldDets),1,[]));

            isoosm = isOOSM(obj.cDetectionManager);
            numDets = numel(find(isoosm));

            % Get retro Cost matrix
            n = obj.pNumLiveTracks;
            m = numDetections(obj.cDetectionManager);
            assert(n <= obj.MaxNumTracks);
            assert(m<=obj.getMaxNumInputDetections);

            costMatrix = zeros(n,m,obj.pClassToUse);
            detIndices = obj.IntFind(isoosm & ~oldDets);
            costMatrix(:,detIndices) = retroCostMatrix(obj.cCostCalculator, obj.pTracksList, ...
                dets, n, detIndices);

            [minTimeStampHistory, indMinTimeStamp] = min(obj.pTimeStampHistory);
            maxTimeStampHistory = max(obj.pTimeStampHistory);
            inOOSMRange = detTimes <= maxTimeStampHistory & detTimes >= minTimeStampHistory;
            
            clusterOOSM = repmat(obj.pClusterTypeDef,1,0);
            coder.varsize('clusterOOSM',[1 obj.pMaxNumClusters]);
            isUnassignedDet = false(1, numDets);
            ONE = obj.IntOne;

            if any(isoosm) && any(inOOSMRange)
                isIntervalUsed = false(1,obj.MaxNumOOSMSteps);
                historyTimes = sort([obj.pTimeStampHistory maxTimeStampHistory+eps(maxTimeStampHistory)]);
                for i = ONE:obj.MaxNumOOSMSteps
                    inInterval = detTimes >= historyTimes(i) & detTimes < historyTimes(i+1);
                    if any(inInterval)
                        detSel = isoosm & inInterval;
                        [unassignedDets, ~,cluster] = ...
                            clusterAssignUpdate(obj,costMatrix,dets,detSel);
                        if ~isempty(cluster)
                            clusterOOSM = [clusterOOSM, cluster]; %#ok<AGROW> 
                            isIntervalUsed(i) = true;
                        end
                        isUnassignedDet(unassignedDets) = true;
                    end
                end
            end

            % Summarize assignment results over oosm time intervals
            detInds = ONE:obj.IntNumel(isUnassignedDet);
            unassignedDetectionIdOOSM = detInds(1,isUnassignedDet);

            % Initialize tracks from unassigned OOSMs
            initializeTracks(obj,unassignedDetectionIdOOSM,dets);

            % Maintains the update times at each step in MaxNumOOSMSteps
            % Timestamp history is a buffer that gets rewritten - the
            % oldest timestamp is replaced by the newest
            obj.pTimeStampHistory(indMinTimeStamp) = time;
        end
        
        function [unassignedDets, trueUnassignedTrackId,clusters] = ...
                clusterAssignUpdate(obj,overallCostMatrix,dets, detSelector)
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
            
            ONE = obj.IntOne();
            [clusters, isUnassignedDetection, unassignedTrackCounter] = ...
                createClusters(obj,overallCostMatrix, detSelector);
            
            [clusters, isUnassignedDetection] = jpdaClusterUpdate(obj,clusters,isUnassignedDetection,dets);
			
            detInds = ONE:obj.IntNumel(isUnassignedDetection);
            unassignedDets = detInds(1,isUnassignedDetection);
            origSen = originatingSensor(obj.cDetectionManager, detSelector);
            trueUnassignedTrackId = ...
                obj.IntFind(unassignedTrackCounter==obj.IntNumel(unique(origSen)));

            
        end

        function flag = isInfoRequested(~,numOutputs)
            flag =  numOutputs > 3;
        end

        function [clusters, isUnassignedDetection, unassignedTrackCounter] = ...
                createClusters(obj,overallCostMatrix, detSelector)
            %CREATECLUSTERS go over all detections, one sensor at a time to
            % create clusters. Updates the unassigned track counter and the
            % flags for unassigned detections.

            % Force coder not to inline this
            coder.inline('never');

            ZERO = obj.IntZero();
            ONE = obj.IntOne();
            N = obj.pNumLiveTracks;
            assert(N <= obj.MaxNumTracks);
            
            % Initialize arrays
            isUnassignedDetection = false(1,numel(detSelector));
            unassignedTrackCounter = repmat(ZERO,ONE,N);
            sensorIDs = unique(originatingSensor(obj.cDetectionManager,detSelector));
            sensorIDs = sensorIDs(sensorIDs>0); % Remove bus detections with SensorIndex = 0
            clusters = repmat(obj.pClusterTypeDef,1,0);
            coder.varsize('clusters',[1 obj.pMaxNumClusters]);
            obj.pCurrentMaxNumDetectionsPerCluster(1) = 0;
            obj.pCurrentMaxNumTracksPerCluster(1) = 0;
            for s = ONE:obj.IntNumel(sensorIDs)
                % Retrieve detections from the current originating sensor
                sensorDetectionsId = selectDetections(obj,sensorIDs(s),detSelector);
                
                % Calculate validation matrix and cost matrix for this sensor
                [validationMatrix, costMatrix] = sensorValidationMatrix(obj,sensorDetectionsId,overallCostMatrix);
                
                % Apply DFS algorithm to bin the tracks into clusters. This
                % allows to reduce the validation matrix size
                [unassignedDet,unassignedTrack,thisSensorClusters] = clusterDetections(obj,validationMatrix,costMatrix, sensorIDs(s), detSelector);
                
                % Store information across sensor iterations:
                isUnassignedDetection(sensorDetectionsId(unassignedDet)) = true;
                unassignedTrackCounter(unassignedTrack) = unassignedTrackCounter(unassignedTrack)+ONE;
                clusters = [clusters thisSensorClusters];%#ok<AGROW>
            end
        end
        
        function [validationMatrix, costMatrix] = sensorValidationMatrix(obj,detIndices,overallCost)
            % Calculate the cost matrix or validate a cost matrix input
            costMatrix = overallCost(:,detIndices)';
            % gating threshold
            gate = obj.AssignmentThreshold(1);
            m = costMatrix < gate; % gating
            M = obj.IntNumel(detIndices);
            validationMatrix = [true(M,1) m]; % M by T matrix. Transp
        end
        
        function [unassignedDets,unassignedTracks,clusters] = clusterDetections(obj,valMatrix, costMatrix, sensorID, detSelector)
            %clusterDetection Determine which detections do not fall in the
            % gate of any track, unassignedDets, which track do not have
            % any detections in their validation gate, unassignedTracks,
            % and which tracks have connected gates i.e. form a cluster.
            % The jpda algorithm should be applied cluster by cluster.
            % clusters is the cell array containing the validated tracks
            % and detections indices.
            
            % We parse the validation matrix of the entire space
            Omega = valMatrix(:,2:end); % remove column associated with Track0

            [maxNumDetsPerCluster, maxNumTracksPerCluster] = getClusterBounds(obj);
            
            [detections2cluster, tracks2cluster, nAllClusters, ...
                maxDetsPerCluster, maxTracksPerCluster, Omega] = ...
                fusion.internal.constrainedConnectedTracks(...
                costMatrix, Omega,...
                maxNumDetsPerCluster, ...
                maxNumTracksPerCluster, ...
                obj.pClusterViolationHandlingType, ...
                obj.clusterViolationMsgs());

            obj.pCurrentMaxNumDetectionsPerCluster = max(obj.pCurrentMaxNumDetectionsPerCluster,uint32(maxDetsPerCluster));
            obj.pCurrentMaxNumTracksPerCluster = max(obj.pCurrentMaxNumTracksPerCluster,uint32(maxTracksPerCluster));
            
            % Update omega as cluster size enforcement may modify
            % the event matrix. 
            unassignedDets = obj.IntFind(all(~Omega,2));   % an unassigned detection is a null row
            unassignedTracks = obj.IntFind(all(~Omega,1)); % an unassigned Track is a null column
            
            % each unassigned Track and Detection will create a degenerate
            % cluster of a single track and single detection
            nClusters = nAllClusters - obj.IntNumel(unassignedTracks) - obj.IntNumel(unassignedDets);  % form the clusters
            clusterNumero = tracks2cluster;
            clusterNumero(unassignedTracks) = [];
            clusterNumero = unique(clusterNumero);

            % Create variable size cluster structure
            assert(nClusters <= obj.pMaxNumClustersPerSensor);
            clusters = repmat(obj.pClusterTypeDef,1,nClusters);
            coder.varsize('clusters(:).DetectionIndices',[1 maxNumDetsPerCluster],[0 1]);
            coder.varsize('clusters(:).TrackIDs',[1 maxNumTracksPerCluster],[0 1]);
            coder.varsize('clusters(:).ValidationMatrix',[maxNumDetsPerCluster maxNumTracksPerCluster + 1],[1 1]);
            coder.varsize('clusters(:).MarginalProbabilities',[maxNumDetsPerCluster+1,maxNumTracksPerCluster],[1 1]);

            sensorDetIds = selectDetections(obj, sensorID, detSelector) ;
            detTimes = obj.cDetectionManager.detectionTimes;
            for c=1:nClusters % note that among the nClusters some are unassignedTr
                clustdetid = find(detections2cluster==clusterNumero(c));
                clusters(c).DetectionIndices = cast(sensorDetIds(clustdetid),'uint32');
                clusters(c).TrackIDs = trackIDs(obj,find(tracks2cluster==clusterNumero(c)))';
                clusters(c).ValidationMatrix = ...
                    valMatrix(clustdetid,[1,1+find(tracks2cluster==clusterNumero(c))]);
                clusters(c).SensorIndex = cast(sensorID,'uint32');
                clusters(c).TimeStamp = mean(detTimes(clusters(c).DetectionIndices));
            end
        end
        
        function [clusters, isUnassignedDetection] = jpdaClusterUpdate(obj,clusters,isUnassignedDetection, dets)
            %JPDACLUSTERUPDATE Perform JPDA on each cluster and correct track estimates
            
            % Sort clusters by their mean time stamp.
			sorting = sortClusterByTime(obj, clusters);
            
            for cl = 1:numel(clusters)
                clIdx = sorting(cl);
                % Define important quantities for the current cluster
                clusterDetectionId = clusters(clIdx).DetectionIndices;
                validateClusterDetectionsTime(obj,clusterDetectionId)
                clusterTrackId = findTracksByIDs(obj,clusters(clIdx).TrackIDs);
                cMatrix = clusters(clIdx).ValidationMatrix;
                
                % Formulate joint events and their probabilities
                [FJE, FJEprob] = calcEventAndProbabilities(obj, clusterDetectionId, clusterTrackId, cMatrix, dets);
                
                % Compute joint probabilistic data association for each
                % {track,detection} pair. Also computes posterior Existence
                % and updates the track logic
                posteriorMarginalProbabilities = ...
                    associationProbabilities(obj,FJE,FJEprob,clusterDetectionId,clusterTrackId);
                
                % Identify detections that are likely to come from a new target
                lowDetectionIdx = identLowProbDet(obj,posteriorMarginalProbabilities,clusterDetectionId);
                % add these detections to allUnassignedDe
                isUnassignedDetection(lowDetectionIdx) = true;
                
                % Update analysis information for that cluster:
                clusters(clIdx).MarginalProbabilities = posteriorMarginalProbabilities;
                
                % Update tracks in clusters
                updateAssignedTracks(obj,clusters(clIdx),dets)
            end
        end

        function sorting = sortClusterByTime(obj, clusters)
            % Get sorting index of each cluster using the TimeStamp field
            % of the cluster
            ZERO = obj.IntZero();
			ONE = obj.IntOne();
            sorting = repmat(ZERO,ONE,obj.IntNumel(clusters));
            if ~isempty(clusters)
                if coder.target('MATLAB')
                    [~,sorting(:)] = sort([clusters.TimeStamp]);
                else
                    timestamps = zeros(1,numel(clusters),obj.pClassToUse);
                    for k=ONE:obj.IntNumel(timestamps)
                        timestamps(k) = clusters(k).TimeStamp;
                    end
                    [~,sorting(:)] = sort(timestamps);
                end
            end
        end

        function clusterInfo = getClusterInfo(obj, clusters)
            sorting = sortClusterByTime(obj, clusters);
            if coder.target('MATLAB')
                clusterInfo = num2cell(clusters(sorting));
            else
                clusterInfo = cell(1,numel(clusters));
                for k = 1:numel(clusterInfo)
                    clusterInfo{k} = clusters(sorting(k));
                end
            end
        end
        
        function preprocessDetections(obj, dets, updateTime)
            % Process the coming detections and separate their origin by
            % source
            % detections can be cell array (especially in
            % codegen) or a regular array of objectDetections or structs.
            % If detections are not cell array, convert them to cell array.
            % Codegen only supports cell arrays of objects
            
            step(obj.cDetectionManager, dets, obj.pLastTimeStamp, updateTime);
            origSen = originatingSensor(obj.cDetectionManager);
            numDets = numDetections(obj.cDetectionManager);
            if numDets >0
                sensorIDs = unique(origSen);
                updateUsedSensors(obj,sensorIDs);
            end
        end
        
        function costMatrix = getCostMatrix(obj, dets, varargin)
            % Validate cost matrix input or calculate
            if numel(varargin) > 1 && obj.HasCostMatrixInput
                maxnargin = 3 + obj.HasCostMatrixInput + obj.HasDetectableTrackIDsInput;
                narginchk(2,maxnargin);
                costMatrix = getCostMatrixFromInput(obj,varargin{:});
                validateattributes(costMatrix, ...
                    {'numeric'}, {'real','nonsparse', '2d', 'nrows', obj.pNumLiveTracks, ...
                    'ncols', numel(dets)}, 'step', 'costMatrix');
            else
                detIndices = inSequenceIndices(obj.cDetectionManager);
                n = obj.pNumLiveTracks;
                m = numDetections(obj.cDetectionManager);
                assert(n <= obj.MaxNumTracks);
                assert(m <= getMaxNumInputDetections(obj));
                
                costMatrix = zeros(n,m,obj.pClassToUse);
                costMatrix(:,detIndices) = step(obj.cCostCalculator, obj.pTracksList, ...
                    dets, n, detIndices);
            end
        end

        function costMatrix = getCostMatrixFromInput(obj,varargin)
            %Extract cost matrix from input.
            costMatrix = varargin{end-obj.HasDetectableTrackIDsInput};
        end
        
        function updateAssignedTracks(obj,cluster,dets)
            % updateAssignedTracks Updates the tracks that were assigned
            % with detections and grouped in clusters
            if~isempty(cluster)
                posteriorMarginalProbabilities = cluster.MarginalProbabilities;
                clusterDetectionId = cluster.DetectionIndices;
                clusterTrackId = findTracksByIDs(obj,cluster.TrackIDs);
                % Update all tracks with correctjpda. These tracks cannot appear in another cluster but may be updated again in the next sensor loop
                updateClusterTracks(obj,posteriorMarginalProbabilities,clusterDetectionId,clusterTrackId,dets);
            end
        end
        
        function trackDetectability(obj, varargin)
            % If HasDetectableTrackIDsInput is true, use detectableTrackIDs
            % to calculate obj.pWasDetectable and
            % obj.pTrackDetectionProbability. Otherwise, we use the default
            % that all tracks are detectable and that all tracks have
            % obj.DetectionProbability
            
            % Reset the pWasDetectable and pTrackDetectionProbability from
            % last step
            obj.pTrackDetectionProbability = obj.DetectionProbability * ones(obj.MaxNumTracks,1,obj.pClassToUse);
            
            % Validate detectable track IDs input
            if obj.HasDetectableTrackIDsInput
                % Find the input port number in simulink and number of
                % inputs in matlab. Extract the detectableTrackIDs from
                % varargin and assign them as detectables
                detectables = getDetectableTrackIDsFromInput(obj,varargin{:});                
                obj.pWasDetectable(1:obj.pNumLiveTracks) = false;
                if numel(detectables)>0 % Validate only if nonempty
                    if size(detectables,2)==1 % Column of track IDs
                        validateattributes(detectables, {'numeric'}, ...
                            {'real','positive','column','integer'}, ...
                            'step', 'detectableTrackIDs');
                        trackIDs = detectables(:,1);
                        trackPDs = obj.DetectionProbability*ones(numel(trackIDs),1,obj.pClassToUse);
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
            else
                % All tracks are detectable if HasDetectableTrackIDsInput is false
                obj.pWasDetectable(1:obj.pNumLiveTracks) = true;
            end
        end

        function detectables = getDetectableTrackIDsFromInput(~,varargin)
            %In Matlab detectable track IDs is always last input
            detectables = varargin{end};
        end
        
        function lhoodMatrix = likelihoodMatrix(obj,clusterDetectionId,clusterTrackId,cMatrix,dets)
            % Return an MxN matrix of likelihood values
            % such that lhoodMatrix(i,j) is the likelihood of the ith
            % detection in the current cluster detection array being
            % associated with the jth target in the current cluster tracks
            % cell array
            
            valid = cMatrix;
            N = numel(clusterTrackId);
            M = numel(clusterDetectionId);
            
            lhoodMatrix = zeros(M,N,obj.pClassToUse);
            for row =1:M
                for col =2:N+1 % col starts at 2 to skip the cluster column in the validation matrix
                    if valid(row,col)
                        trackId = col-1;
                        det = dets{clusterDetectionId(row)};
                        track = obj.pTracksList{clusterTrackId(trackId)};
                        z = det.Measurement;
                        filter = track.Filter;
                        filter.MeasurementNoise = dets{clusterDetectionId(row)}.MeasurementNoise;
                        
                        if obj.pTracksList{trackId}.pIsLinearKalmanFilter || ...
                                isstruct(det) && ~isfield(det,'MeasurementParameters') ||...
                                isa(det,'objectDetection') && isempty(det.MeasurementParameters)
                            lhoodMatrix(row,col-1)=likelihood(filter,z);
                        else
                            MP = dets{clusterDetectionId(row)}.MeasurementParameters;
                            if isstruct(MP)
                                lhoodMatrix(row,col-1)=likelihood(filter,z,{MP});
                            else
                                lhoodMatrix(row,col-1)=likelihood(filter,z,MP);
                            end
                        end
                    end
                end
            end
        end
        
        function updateClusterTracks(obj,posterior,clusterDetectionId,clusterTrackId, allDets)
            % updates the filters of all confirmed tracks using the current
            % detections and the calculated data association coefficients
            
            % posterior was computed with detections in this cluster:
            dets = extractDetectionsForCluster(obj, clusterDetectionId, allDets);
            detTimes = obj.cDetectionManager.detectionTimes;
            isoosm = isOOSM(obj.cDetectionManager);

            for trackID=1:numel(clusterTrackId)
                track = obj.pTracksList{clusterTrackId(trackID)};
                % mean cluster timestamp
                mtime = mean(detTimes(clusterDetectionId));
                isRetro = isoosm(clusterDetectionId);
                dets{1}.Time = mtime; % pass mtime value through first detection time stamp
                % correct the track with all detections in the cluster
                correctTrack(obj,track, dets,posterior(:,trackID),isRetro);
                
                % update confirmed status for the tracker
                obj.pConfirmedTracks(clusterTrackId(trackID)) = obj.pTracksList{clusterTrackId(trackID)}.IsConfirmed;
            end
        end
        
        function correctTrack(obj, track, dets, betas, isRetro)
            % correctTrack applies the appropriate correction to a track
            if ~isRetro
                correctjpda(track, dets, betas, obj.pUsedSensors, obj.HitMissThreshold);
            elseif obj.pUseRetrodiction
                retroCorrectJPDA( track, dets, betas, obj.pUsedSensors, obj.HitMissThreshold);
            end
                
            
        end
        
        function clusterDets = extractDetectionsForCluster(~, clusterDetectionId, dets)
            numTrackDetections = numel(clusterDetectionId);
            clusterDets = repmat({dets{clusterDetectionId(1)}}, [numTrackDetections, 1]);
            for i = 2:numTrackDetections
                clusterDets{i} = dets{clusterDetectionId(i)};
            end
        end
        
        function newTrackIDs = initializeTracks(obj, OverallUnassigned, dets)
            %This function initializes a new track based on an existing
            %post-processed detection, when this detection cannot be
            %associated with any existing track in the TracksBank
            
            lastTrackInd =  obj.pNumLiveTracks;
            ZERO = obj.IntZero();
            ONE = obj.IntOne();
            
            while numel(OverallUnassigned)
                Det = dets{OverallUnassigned(1)};
                
                % Create the track
                NewTrackNumber = obj.pNumLiveTracks + ONE;
                
                newTrackID = obj.pLastTrackID + 1;
                tf = initiateTrack(obj, newTrackID, Det);
                if tf
                    obj.pLastTrackID = newTrackID;
                else  % Failed to initialize track because pTracksList is full
                    coder.internal.warning('fusion:trackerJPDA:MaxNumTracksReached', 'MaxNumTracks');
                    break
                end
                
                % Multi-sensor case:
                % Eliminate any unassigned detections within the new track
                % gate to avoid unnecessary track initialization. Use the ones
                % that are within the gate to update the initialized track.
                
                if ~obj.HasCostMatrixInput
                    costMatrix = zeros(1, numel(OverallUnassigned),obj.pClassToUse);
                    track = obj.pTracksList{NewTrackNumber};
                    initDetection = dets{OverallUnassigned(1)};
                    initSensor = initDetection.SensorIndex;
                    origSen = originatingSensor(obj.cDetectionManager);
                    sensorsInUnassigned = origSen(1,OverallUnassigned);
                    sameSensor = (sensorsInUnassigned == initSensor) | (sensorsInUnassigned == 0);
                    costMatrix(sameSensor) = inf;
                    costMatrix(~sameSensor) = distance(track, dets, OverallUnassigned(~sameSensor(:)), obj.pHasMeasurementParameters);
                    
                    checkedUnassigned = zeros(size(OverallUnassigned), 'like', OverallUnassigned);
                    m = ZERO;
                    for k = 2:numel(OverallUnassigned)
                        if costMatrix(1,k) <= obj.AssignmentThreshold(1) % detections were unassigned so correct suffices
                            %  Use measurement parameters if they are given
                            det = dets{OverallUnassigned(k)};
                            correctjpda(track,{det}, cast([1 0],obj.pClassToUse), obj.pUsedSensors); % correct track with detection
                        else
                            m = m + ONE;
                            checkedUnassigned(m) = OverallUnassigned(k);
                        end
                    end
                    obj.pConfirmedTracks(NewTrackNumber) = obj.pTracksList{NewTrackNumber}.IsConfirmed;
                    
                    if m > 0
                        OverallUnassigned = checkedUnassigned;
                        OverallUnassigned(m+1:end) = [];
                    else
                        break
                    end
                else % Eliminate the detection that initialized the track
                    OverallUnassigned(1) = [];
                    obj.pConfirmedTracks(NewTrackNumber) = obj.pTracksList{NewTrackNumber}.IsConfirmed;
                end
            end
            newTrackIDs = obj.pTrackIDs(lastTrackInd+ONE:obj.pNumLiveTracks);
        end
        
        function posterior = associationProbabilities(obj, FJE, FJEprobs,clusterDetectionId,clusterTrackId)
            % Computes the a-posteriori probabilities of individual track events i.e. P(X^t, X^t_i | Z^k).
            % Inputs are the current validation Matrix and Feasible Joint Events
            % posterior probabilities P(Xi|Z^k).
            
            ONE = obj.IntOne();
            TWO = obj.IntIndex(2);
            % number of Measurements
            M = obj.IntNumel(clusterDetectionId);
            % number of Tracks
            T= obj.IntNumel(clusterTrackId);
            % number of Feasible Joint Events
            nFJE = size(FJE,3);
            
            % probability and gate probability
            Pd = obj.pTrackDetectionProbability(clusterTrackId(:));
            Pg = gateProbability(obj)*ones(T,ONE,obj.pClassToUse);
            
            % Retrieve each track existence probability
            Pte = getTracksPTE(obj,clusterTrackId);
            
            posterior = zeros(M+ONE,T,obj.pClassToUse);
            posterior_end = zeros(T,ONE,obj.pClassToUse);
            for e=1:nFJE
                % Get FJE matrix and its probability
                X = FJE(:,TWO:T+ONE,e);
                Px = FJEprobs(e);
                
                % find which individual association occurs in X
                assigned = [X>0;false(ONE,size(X,2))];
                unassignedT = all(~X,1);
                % add contribution of this event to posterior
                posterior(assigned) = posterior(assigned) + Px;
                n = obj.IntLogicalSum(unassignedT);
                assert(n <= obj.MaxNumTracks);
                I = ones(n,ONE,obj.pClassToUse);
                if strcmp(obj.TrackLogic,'Integrated')
                    num = (I-Pd(unassignedT).*Pg(unassignedT)).*Pte(unassignedT);
                    den = I-Pd(unassignedT).*Pg(unassignedT).*Pte(unassignedT)+eps(obj.pClassToUse)*I;
                    ratio = num./den;
                else
                    ratio = I;
                end
                posterior_end(unassignedT) = posterior_end(unassignedT)+ ratio*Px;
            end
            posterior(M+1,:) = posterior_end';
            myround = @(x,n)round(x.*10.^n)./(10.^n); % round(x,n) does not support code generation
            posterior = myround(posterior,9);
        end
        
        function [FJE, FJEProb] = calcEventAndProbabilities(obj, clusterDetectionId, clusterTrackId, validationMatrix, dets)
            % Compute conditional association likelihood. 
            lhoodMatrix = likelihoodMatrix(obj, clusterDetectionId, clusterTrackId, validationMatrix, dets);
            
            % If number of events are not finite, generate all the feasible
            % joint events using the validationMatrix.
            if ~isfinite(obj.MaxNumEvents)
                %jpdaEvents will use double-precision as validation is a logical matrix. 
                FJE = obj.pEventGenerationFcn(validationMatrix);
                FJEProb = calcEventProbabilities(obj, FJE, lhoodMatrix, clusterDetectionId, clusterTrackId);
            % If number of events are finite, provide the posterior
            % likelihood matrix
            else
                posteriorLikelihoodMatrix = assemblePosteriorLikelihoodMatrix(obj, lhoodMatrix, clusterDetectionId, clusterTrackId);
                [FJE, FJEProb] = obj.pEventGenerationFcn(posteriorLikelihoodMatrix, obj.MaxNumEvents);
            end
        end
        
        function posteriorLikelihoodMatrix = assemblePosteriorLikelihoodMatrix(obj, lhoodMatrix, clusterDetectionId, clusterTrackId)
            % Allocate memory for posterior likelihood matrix
            M = numel(clusterDetectionId);
            N = numel(clusterTrackId);
            posteriorLikelihoodMatrix = zeros(M + 1, N + 1, 'like', lhoodMatrix);
            
            % Pd for each track in the cluster
            Pd = obj.pTrackDetectionProbability(clusterTrackId);
            
            % Pg
            Pg = gateProbability(obj);
            
            % Existence probability
            Pte = getTracksPTE(obj,clusterTrackId);
            
            % spatial clutter density in the cluster
            lambda = obj.ClutterDensity;
            
            % Assemble posterior likelihood matrix 
            
            % 1, 1 is dummy-dummy, its likelihood is 1
            posteriorLikelihoodMatrix(1,1) = 1;
            
            % First column is clutter density
            posteriorLikelihoodMatrix(2:end,1) = lambda;
            
            % First row is effective miss probability
            posteriorLikelihoodMatrix(1,2:end) = 1 - Pd.*Pte.*Pg;
            
            % Rest is likelihood matrix of detection and measurement
            % with posterior due to gating, existence and detection 
            % Pd*Pte*Pg*(Lij/Pg) = Pd*Pte*Lij;
            posteriorLikelihoodMatrix(2:end,2:end) = bsxfun(@times,(Pd.*Pte)',lhoodMatrix);
        end
        
        function eventProb = calcEventProbabilities(obj,FJE,lhoodMatrix,clusterDetectionId,clusterTrackId)
            % Return the posterior probabilities of each Feasible Joint
            % Event (FJE) conditioned on the cluster detections.
            
            % Main reference is p.317 eq 6.2.4-4 [1]
            
            M = numel(clusterDetectionId);
            nFeasibleEvents = size(FJE,3);
            
            % Retrieve pd for the tracks in cluster
            Pd = obj.pTrackDetectionProbability(clusterTrackId);
            Pg = gateProbability(obj);
            
            % Retrieve each track existence probability if using
            if strcmp(obj.TrackLogic,'Integrated')
                Pte = getTracksPTE(obj,clusterTrackId);
            else % deterministic version, all live tracks are existing objects
                Pte =ones(numel(clusterTrackId),1,obj.pClassToUse);
            end
            
            eventProbabilities=zeros(1,nFeasibleEvents,obj.pClassToUse);
            lambda = obj.ClutterDensity; % spatial clutter density
            for event=1:nFeasibleEvents
                X = FJE(:,:,event);  % current feasible event
                deltap1 = cast(sum(X,1),obj.pClassToUse);
                delta=deltap1(2:end)';
                tau = cast(sum(X(:,2:end),2),'logical'); % tau(meas)=1 if the meas is not clutter
                % Compute likelihood of this event detections knowing the current state estimate
                lhood=ones(1,obj.pClassToUse); % initialize
                % likelihood of joint event is product of likelihoods of individual associations
                for meas = 1:M
                    if tau(meas) % this detection comes from a target
                        trackidx = find(X(meas,2:end));
                        lhood = lhood/Pg/lambda*lhoodMatrix(meas,trackidx(1));
                    end
                end
                
                % compute prior distribution
                product_detected= prod((Pte.*Pg.*Pd).^delta);
                product_undetected = prod((1-Pd.*Pg.*Pte).^(1-delta));
                prior = product_detected*product_undetected;
                
                % posterior ~ likelihood * prior
                eventProbabilities(event)= lhood*prior;
            end
            eventProb = eventProbabilities/(sum(eventProbabilities)+eps);
        end
        
        function pte = getTracksPTE(obj,clusterTrackId)
            T = numel(clusterTrackId);
            if strcmp(obj.TrackLogic,'History')
                pte = ones(T,1,obj.pClassToUse);
                return
            else
                pte = zeros(T,1,obj.pClassToUse);
            end
            for t = 1:T
                pte(t) = obj.pTracksList{clusterTrackId(t)}.TrackLogic.ExistenceProbability;
            end
        end
        
        function indices = selectDetections(obj,ID,detSelector)
            nMax = getMaxNumDetectionsPerSensor(obj);
            indices =  obj.IntFind(originatingSensor(obj.cDetectionManager)==ID & detSelector);
            coder.varsize('indices',[1 nMax],[0 1]);
            coder.internal.assert(numel(indices) <= nMax, 'fusion:internal:TrackerMemoryManagementUtilities:MaxNumDetectionsPerSensorViolation', ID);
        end
        
        function indices = findTracksByIDs(obj,trackIDs)
            indices = zeros(1,numel(trackIDs),'like',trackIDs);
            for i =1:numel(indices)
                indices(i) = obj.findTrackByID(trackIDs(i));
            end
        end
        
        function weakDetectionIdx = identLowProbDet(obj,jpda_beta,clusterDetectionId)
            % This method returns the detections that are not assigned to
            % any track with a probability larger than the threshold.
            
            jpda_beta(end,:) =[]; % discard null coefficient for what comes next
            isWeak =all(jpda_beta<obj.InitializationThreshold,2);
            weakDetectionIdx = clusterDetectionId(isWeak);
        end
        
        function coastUnassignedTracks(obj,unassignedTracks)
            for count = 1: numel(unassignedTracks)
                if obj.pWasDetectable(unassignedTracks(count))
                    updateNotAssociated(obj.pTracksList{unassignedTracks(count)},...
                        obj.pTrackDetectionProbability(unassignedTracks(count)));
                end
            end
        end
        
        function delTrIDs = deleteOldTracks(obj)
            %This function 'removes'  tracks
            %based on their current probability of target existence
            
            ONE = obj.IntOne();
            NumberOfTracks = obj.pNumLiveTracks;
            assert(NumberOfTracks <= obj.MaxNumTracks);
            unassigned = ONE:NumberOfTracks;
            
            toDelete = false(1,obj.MaxNumTracks);
            for i=NumberOfTracks:-ONE:ONE
                if obj.pWasDetectable(unassigned(i))
                    toDelete(unassigned(i)) = checkDeletion(obj.pTracksList{unassigned(i)});
                end
            end
            delTrIDs = obj.pTrackIDs(toDelete);
            recycleTracks(obj, toDelete);
        end
        
        function predictTracks(obj, t)
            %This function uses tracking filters to predict the tracks into
            %the next time instance
            % The track's predict method and the tracking filters must
            % support predict(track, t).
            
            if t > obj.pLastTimeStamp
                for i=1:obj.pNumLiveTracks
                    track = obj.pTracksList{i};
                    if t > track.Time
                        predict(track, t);
                    end
                end
            end
            obj.pLastTimeStamp = t;
        end
        
        function track = predictTrackStruct(obj, track, time, numStates)
            % Predicts the track's struct output. We use the distance
            % filter to predict the track's struct output to time.
            
            % Use the fact that all filters are the same.
            isLinearKalmanFilter = obj.pTracksList{1}.pIsLinearKalmanFilter;
            motionModel = obj.pTracksList{1}.pMotionModel;
            
            dt = time - track.UpdateTime;
            matlabshared.tracking.internal.fusion.predictTrackFilter(...
                obj.pDistFilter, isLinearKalmanFilter, motionModel, dt);
            P = obj.pDistFilter.StateCovariance;
            xx = obj.pDistFilter.State(:);
            track.State = xx(1:numStates,1);
            track.StateCovariance = P(1:numStates,1:numStates);
            track.UpdateTime = time;
        end
        
        function validateClusterDetectionsTime(obj,detIndices)
            detTimes = obj.cDetectionManager.detectionTimes;
            thisDetTimes = detTimes(detIndices);
            maxTime = max(thisDetTimes(:));
            minTime = min(thisDetTimes(:));
            origSen = originatingSensor(obj.cDetectionManager);
            sensorID = origSen(detIndices(1));
            tol = cast(obj.TimeTolerance,'like',detTimes);
            coder.internal.errorIf(maxTime - minTime > tol,...
                'fusion:trackerJPDA:SensorTimeTolerance',...
                sensorID,'TimeTolerance');
        end
        
        function numHistory = numHistorySteps(obj)
            % Implement numHistorySteps in the Retrodiction case.
            if strcmpi(obj.OOSMHandling,'Retrodiction')
                numHistory = obj.MaxNumOOSMSteps * obj.MaxNumSensors;
            else
                numHistory = numHistorySteps@matlabshared.tracking.internal.fusion.TrackManager(obj);
            end
        end
        
        %% Save / Load / Clone Impl
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj
            
            % Save the base class information
            s = saveObjectImpl@matlabshared.tracking.internal.fusion.TrackManager(obj);
            s.pEventGenerationFcn         = obj.pEventGenerationFcn;
               
            % Confirmation and deletion properties
            s.pHistoryConfThreshold    = obj.pHistoryConfThreshold;
            s.pHistoryDelThreshold     = obj.pHistoryDelThreshold;
            s.pIntegratedConfThreshold = obj.pIntegratedConfThreshold;
            s.pIntegratedDelThreshold  = obj.pIntegratedDelThreshold;
            
            % Set private and protected properties
            % Only save the tracks list if the tracker is locked
            s.pAssignmentThreshold                  = obj.pAssignmentThreshold;
            s.pLastTrackID                          = obj.pLastTrackID;
            s.pLastTimeStamp                        = obj.pLastTimeStamp;
            s.pVersion                              = obj.pVersion;
            s.pWasDetectable                        = obj.pWasDetectable;
            s.pTrackDetectionProbability            = obj.pTrackDetectionProbability;
            s.pClusterTypeDef                       = obj.pClusterTypeDef;
            s.cCostCalculator                       = obj.cCostCalculator;
            s.pMaxNumClusters                       = obj.pMaxNumClusters;
            s.pMaxNumClustersPerSensor              = obj.pMaxNumClustersPerSensor;
            s.pCurrentMaxNumTracksPerCluster        = obj.pCurrentMaxNumTracksPerCluster;
            s.pCurrentMaxNumDetectionsPerCluster    = obj.pCurrentMaxNumDetectionsPerCluster;
            s.pClusterViolationHandlingType         = obj.pClusterViolationHandlingType;
            s.pTimeStampHistory                     = obj.pTimeStampHistory;
            s.pUseRetrodiction                      = obj.pUseRetrodiction;
        end
        
        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s
            obj.pAssignmentThreshold             = s.pAssignmentThreshold;
            obj.pLastTimeStamp                   = s.pLastTimeStamp;
            if isfield(s, 'pOriginatingSensor')
                s = rmfield(s, 'pOriginatingSensor');
            end
            if isfield(s, 'pDetectionTimes')
                s = rmfield(s, 'pDetectionTimes');
            end
            obj.pClusterTypeDef                  = s.pClusterTypeDef;
            obj.pLastTrackID                     = s.pLastTrackID;           
            
            if isfield(s,'MaxNumDetectionsPerCluster')
                obj.MaxNumDetectionsPerCluster = s.MaxNumDetectionsPerCluster;
            end
            
            if isfield(s,'MaxNumTracksPerCluster')
                obj.MaxNumTracksPerCluster = s.MaxNumTracksPerCluster;
            end
            
            if isfield(s,'pMaxNumClusters')
                obj.pMaxNumClusters = s.pMaxNumClusters;
            else
                obj.pMaxNumClusters = inf;
            end

            if isfield(s,'pMaxNumClustersPerSensor')
                obj.pMaxNumClustersPerSensor = s.pMaxNumClustersPerSensor;
            else
                obj.pMaxNumClustersPerSensor = inf;
            end

            if isfield(s,'pCurrentMaxNumTracksPerCluster')
                obj.pCurrentMaxNumTracksPerCluster = s.pCurrentMaxNumTracksPerCluster;
            end % Has a default value
                

            if isfield(s,'pCurrentMaxNumDetectionsPerCluster')
                obj.pCurrentMaxNumDetectionsPerCluster = s.pCurrentMaxNumDetectionsPerCluster;
            end % Has a default value                 

            if isfield(s,'pClusterViolationHandlingType')
                obj.pClusterViolationHandlingType = s.pClusterViolationHandlingType;
            else
                obj.pClusterViolationHandlingType = fusion.internal.ClusterViolationHandlingType(2);
            end
            
            if isfield(s, 'pTimeStampHistory')
                obj.pTimeStampHistory = s.pTimeStampHistory;
                obj.pUseRetrodiction  = s.pUseRetrodiction;
            end % Has a default value
            obj.pWasDetectable             = s.pWasDetectable;
            obj.pTrackDetectionProbability = s.pTrackDetectionProbability;
            obj.pHistoryConfThreshold      = s.pHistoryConfThreshold;
            obj.pHistoryDelThreshold       = s.pHistoryDelThreshold;
            obj.pIntegratedConfThreshold   = s.pIntegratedConfThreshold;
            obj.pIntegratedDelThreshold    = s.pIntegratedDelThreshold;
            obj.cCostCalculator            = s.cCostCalculator;
            obj.pEventGenerationFcn        = s.pEventGenerationFcn;
            loadObjectImpl@matlabshared.tracking.internal.fusion.TrackManager(obj,s,wasLocked);
        end
        
        function newTracker = cloneImpl(obj)
            %clone Creates a copy of the trackerJPDA
            %   newTracker = clone(tracker) returns a copy of the tracker
            %   object.
            
            % Copy public properties
            newTracker = cloneImpl@matlabshared.tracking.internal.fusion.TrackManager(obj);
            
            if coder.internal.is_defined(obj.cDetectionManager) %Only happens after setup
                % Next, copy the rest of the private properties
                newTracker.pEventGenerationFcn                  = obj.pEventGenerationFcn;
                newTracker.pLastTimeStamp                       = obj.pLastTimeStamp;
                newTracker.pLastTrackID                         = obj.pLastTrackID;
                newTracker.pAssignmentThreshold                 = obj.AssignmentThreshold;
                newTracker.pWasDetectable                       = obj.pWasDetectable;
                newTracker.pTrackDetectionProbability           = obj.pTrackDetectionProbability;
                newTracker.pClusterTypeDef                      = obj.pClusterTypeDef;
                newTracker.pVersion                             = obj.pVersion;
                newTracker.cCostCalculator                      = clone(obj.cCostCalculator);
                newTracker.pMaxNumClusters                      = obj.pMaxNumClusters;
                newTracker.pMaxNumClustersPerSensor             = obj.pMaxNumClustersPerSensor;
                newTracker.pCurrentMaxNumDetectionsPerCluster   = obj.pCurrentMaxNumDetectionsPerCluster;
                newTracker.pCurrentMaxNumTracksPerCluster       = obj.pCurrentMaxNumTracksPerCluster;
                newTracker.pClusterViolationHandlingType        = obj.pClusterViolationHandlingType;
            elseif ~coder.target('MATLAB') % In codegen, these don't get set with the call to cloneImpl above
                % Confirmation and deletion properties:
                newTracker.pHistoryConfThreshold     = obj.pHistoryConfThreshold;
                newTracker.pHistoryDelThreshold      = obj.pHistoryDelThreshold;
                newTracker.pIntegratedConfThreshold  = obj.pIntegratedConfThreshold;
                newTracker.pIntegratedDelThreshold   = obj.pIntegratedDelThreshold;
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

        function flag = isInputComplexityMutableImpl(~, ~)
            flag = true;
        end
        
        function num = getNumInputsImpl(obj)
            % Define total number of inputs for system with optional inputs
            num = 2; %In MATLAB, detection and time inputs are always available. 
            if obj.HasCostMatrixInput
                num = num + 1;
            end
            if obj.HasDetectableTrackIDsInput
                num = num + 1;
            end
        end
        
        function num = getNumOutputsImpl(~)
            % In MATLAB, all four outputs are always available.
            num = 4;
        end

        function groups = getPropertyGroupsImpl(obj)
            groups = getPropertyGroups(obj);
        end
        
        function groups = getPropertyGroups(obj)
            % Define property section(s) for display in Matlab
            
            trackerGroup = matlab.mixin.util.PropertyGroup(...
                {'TrackerIndex','FilterInitializationFcn',...
                'MaxNumEvents','EventGenerationFcn',...
                'MaxNumTracks','MaxNumDetections','MaxNumSensors',...
                'TimeTolerance'});
            
            assignmentGroup = matlab.mixin.util.PropertyGroup(...
                {'AssignmentThreshold', ...
                'InitializationThreshold',...
                'DetectionProbability',...
                'ClutterDensity'});
            
            logicGroup = matlab.mixin.util.PropertyGroup(...
                {'TrackLogic', 'ConfirmationThreshold', 'DeletionThreshold', ...
                'HitMissThreshold', 'NewTargetDensity','DeathRate',...
                'InitialExistenceProbability'});
            
            ioGroup = matlab.mixin.util.PropertyGroup(...
                {'HasCostMatrixInput', 'HasDetectableTrackIDsInput',...
                'StateParameters'});
            
            numGroup = matlab.mixin.util.PropertyGroup(...
                {'NumTracks', 'NumConfirmedTracks'});

            oosmGroup = matlab.mixin.util.PropertyGroup({'OOSMHandling','MaxNumOOSMSteps'});
            
            memManagementGroups = getPropertyGroups@fusion.internal.TrackerMemoryManagementUtilities(obj);

            groups = [trackerGroup, assignmentGroup, oosmGroup, logicGroup, ioGroup, numGroup memManagementGroups];
        end
        
        function flag = isInactivePropertyImpl(obj, prop)
            % Return false if property is visible based on object
            % configuration, for the command line and System block dialog
            
            flag = isInactivePropertyImpl@matlabshared.tracking.internal.fusion.TrackManager(obj,prop);           

            if strcmp(obj.TrackLogic,'History') % Not a score track logic
                flag = flag | any(strcmp(prop, ...
                    {'NewTargetDensity','pIntegratedConfThreshold',...
                    'pIntegratedDelThreshold',...
                    'InitialExistenceProbability',...
                    'DeathRate'}));
            else
                flag = flag | any(strcmp(prop, {'pHistoryConfThreshold','pHistoryDelThreshold',...
                    'HitMissThreshold'}));
            end

            flag = flag || isInactivePropertyImpl@fusion.internal.TrackerMemoryManagementUtilities(obj,prop);
            flag = flag | (strcmpi(prop, 'MaxNumOOSMSteps') && ~strcmpi(obj.OOSMHandling,'Retrodiction'));
        end              
    end

    % Memory management methods
    methods (Access = protected)
        function out = getMaxNumInputDetections(obj)
            out = obj.MaxNumDetections;
        end

        function tf = hasAssignmentClustering(~)
            % Yes, always true
            tf = true;
        end
    end        
    
    methods(Static, Access = protected)
        function msgs = clusterViolationMsgs()
            msgs = cell(2,1);
            msgs{1} = trackerJPDA.getDetectionClusterViolationMsg();
            msgs{2} = trackerJPDA.getTrackClusterViolationMsg();
        end
    end
    %% Static
    methods(Static, Hidden)
        function flag = isAllowedInSystemBlock
            flag = false;
        end
        
        function [clustDets,clustTracks,numClusters] = connectedTracks(A)  
            [clustDets,clustTracks,numClusters] = fusion.internal.connectedTracks(A);
        end
    end
end

function out = gateProbability(obj)
% chi2 distribution
Pg = gammainc(sqrt(obj.AssignmentThreshold(1))/2,numel(obj.pSampleDetection.Measurement)/2);
out = cast(real(Pg),obj.pClassToUse);
end

function tf = isValidKBestEventFcn(fcn)
tf = ~((nargin(fcn) ~= 2 && nargin(fcn) > 0) ...  % Allow anonymous function
        || (nargout(fcn) ~= 2 && nargout(fcn) > 0)); % Allow anonymous function
end


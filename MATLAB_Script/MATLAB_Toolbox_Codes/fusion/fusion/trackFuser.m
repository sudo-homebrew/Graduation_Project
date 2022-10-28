classdef (StrictDefaults) trackFuser < fusion.internal.FuserManager ...
        & fusion.internal.ExportToSimulinkInterface
%trackFuser   Single-hypothesis track-to-track fuser
%  fuser = trackFuser creates a track-to-track fuser that uses the
%  global nearest neighbor (GNN) algorithm to maintain a single hypothesis
%  about the objects it tracks. 
%
%  fuser = trackFuser('Name',value) creates a trackFuser object by
%  specifying its properties as name-value pair arguments. Unspecified
%  properties have default values. See the list of properties below.
%
%  Step method syntax:
%    conf = step(obj,localTracks,tFusion) outputs the list of confirmed
%    central tracks. The input localTracks is an array of tracks where each
%    element is an objectTrack object or a struct containing fields with
%    the same names as the objectTrack properties. The state of the local
%    track must be defined by a Gaussian distribution, described by State
%    and StateCovariance.
%    The input tFusion is the time to which the tracks are predicted at the
%    end of the step.
%    The confirmed tracks output, conf, is an array of tracks where each
%    element is an objectTrack object or a struct containing fields with
%    the same names as the objectTrack properties.
%
%    [conf,tent,all] = step(...), additionally, lets you output the
%    list of tentative central tracks, tent, and the list of all central
%    tracks, all. The tentative and all tracks outputs are arrays of tracks
%    where each element is an objectTrack object or a struct containing
%    fields with the same names as the objectTrack properties.
%
%    [..., info] = step(...), additionally, lets you output analysis
%    information about the step. It is a struct with the following fields:
%      TrackIDsAtStepBeginning    - IDs of central tracks when the step began.
%      CostMatrix                 - Cost of assignment matrix.
%      Assignments                - Assignments returned from the assignment 
%                                   function.
%      UnassignedCentralTracks    - IDs of unassigned central tracks.
%      UnassignedLocalTracks      - IDs of unassigned local tracks.
%      NonInitializingLocalTracks - IDs of local tracks that were
%                                   unassigned but were not used to
%                                   initialize a central track.
%      InitializedCentralTrackIDs - IDs of central tracks initialized 
%                                   during the step.
%      UpdatedCentralTrackIDs     - IDs of central tracks updated during
%                                   the step.
%      DeletedTrackIDs            - IDs of central tracks deleted during the
%                                   step.
%      TrackIDsAtStepEnd          - IDs of central tracks when the step ended.
%
%  System objects may be called directly like a function instead of using
%  the step method. For example, y = step(obj) and y = obj() are 
%  equivalent.
%
%  trackFuser properties:
%    FuserIndex                 - Index of the track fuser
%    MaxNumCentralTracks        - Maximum number of fused tracks
%    MaxNumSources              - Maximum number of source configurations
%    SourceConfigurations       - Configuration of source systems
%    Assignment                 - Assignment algorithm name
%    CustomAssignmentFcn        - Name of 'Custom' assignment algorithm
%    AssignmentThreshold        - Threshold for track-to-track assignment
%    StateTransitionFcn         - A function to predict track state
%    StateTransitionJacobianFcn - Jacobian of StateTransitionFcn
%    ProcessNoise               - The process noise covariance matrix
%    HasAdditiveProcessNoise    - True if the process noise is additive
%    StateParameters            - Parameters about the track state reference frame
%    ConfirmationThreshold      - Threshold for track confirmation
%    DeletionThreshold          - Threshold for track deletion
%    FuseConfirmedOnly          - Fuse only confirmed local tracks
%    FuseCoasted                - Fuse coasted local tracks
%    StateFusion                - Algorithm used in track states fusion
%    CustomStateFusionFcn       - Custom state fusion function
%    StateFusionParameters      - Parameters used by StateFusion algorithm
%    NumCentralTracks           - Number of central-level tracks           (read only)
%    NumConfirmedCentralTracks  - Number of confirmed central-level tracks (read only) 
%    
%  trackFuser methods:
%    step                - Creates, updates, and deletes the tracks
%    predictTracksToTime - Predicts the tracks to a time stamp
%    initializeTrack     - Initialize a new track in the track fuser
%    deleteTrack         - Delete an existing track in the track fuser
%    sourceIndices       - Return the list of fuser source indices
%    exportToSimulink    - Export the fuser to a Simulink model
%    release             - Allows property value and input characteristics changes
%    clone               - Creates a copy of the trackerFuser
%    isLocked            - Locked status (logical)
%    reset               - Resets states of the trackerFuser
%
%   % EXAMPLE: Construct a fuser and use to fuse tracks from two sources
%   
%   % Define two sources: internal and external. Note that you have to use
%   % a different SourceIndex for each.
%   internalSource = fuserSourceConfiguration(1,'IsInternalSource',true);
%   externalSource = fuserSourceConfiguration(2,'IsInternalSource',false);
%
%   % Construct a trackFuser, identified as number 3. The fuser has the two
%   % sources defined above. Use default assignment and state transition
%   % models. Use the 'Cross' StateFusion model StateFusionParameters = 0.4
%   fuser = trackFuser('FuserIndex', 3, 'MaxNumSources', 2, ...
%      'SourceConfigurations', {internalSource;externalSource}, ...
%      'StateFusion', 'Cross', 'StateFusionParameters', 0.4);
%
%   % Update the fuser with two tracks, one from each source. Use a 3-D
%   % constant velocity state. For the first track create a large
%   % covariance in the x axis and for the second track create a large
%   % covariance in the y axis.
%   tracks = [...
%      objectTrack('SourceIndex', 1, 'State', [10;0;0;0;0;0], ...
%           'StateCovariance', diag([100,1000,1,10,1,10])); ...
%      objectTrack('SourceIndex', 2, 'State', [10;0;0;0;0;0], ...
%           'StateCovariance', diag([1,10,100,1000,1,10]))];
%   time = 0;
%   confirmedTracks = fuser(tracks, time);
%
%   % In this example, a 3-D constant velocity model is used, which assumes
%   % that the state is: [x; vx; y; vy; z; vz].
%   % Because there were two tracks close to each other, one track will be
%   % initialized and it will be confirmed with the 2-out-of-3 default
%   % confirmation.
%
%   % Get the positions, position covariances:
%   positionSelector = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0]; % [x, y, z]
%   [inputPos, inputCov] = getTrackPositions(tracks, positionSelector);
%   [outputPos, outputCov] = getTrackPositions(confirmedTracks, positionSelector);
%
%   %Plot the results using trackPlotter
%   tPlotter = theaterPlot('XLim',[0, 20],'YLim',[-10, 10],'ZLim',[-10, 10]);
%   tPlotter1 = trackPlotter(tPlotter,'DisplayName','Input Tracks','MarkerEdgeColor','blue');
%   tPlotter2 = trackPlotter(tPlotter,'DisplayName','Fused Tracks','MarkerEdgeColor','green');
%   plotTrack(tPlotter1, inputPos, inputCov)
%   plotTrack(tPlotter2, outputPos, outputCov)
%   title('Cross-covariance fusion')
%
%  See also: objectTrack, fuserSourceConfiguration, fusexcov, fusecovint,
%  constvel, constveljac

% Copyright 2019-2021 The MathWorks, Inc.

%#codegen

    properties(Nontunable)
        %FuserIndex - Unique index of the track fuser
        %  Specify the track fuser index as a positive integer scalar
        %
        % Default: 1
        FuserIndex = 1
    end
    
    properties(Dependent)
        %SourceConfigurations  Configurations of source systems
        %  Specify the configurations of the source systems that report
        %  tracks to the track fuser as fuserSourceConfiguration. You can
        %  specify the SourceConfigurations during construction as a
        %  Name,value pair or set it after construction. 
        %
        %  Default: a 1-by-MaxNumSources cell array of default
        %  fuserSourceConfiguration objects
        SourceConfigurations
    end
    
    properties(Nontunable)
        %MaxNumSourceConfiguration Maximum number of source configurations
        %   Set the maximum number of sources configurations that the fuser
        %   can maintain as a positive real integer.
        %
        %   Default: 20
        MaxNumSources = 20
        
        %MaxNumCentralTracks   Maximum number of fused (central-level) tracks
        %  Set the maximum number of central-level tracks the fuser can
        %  maintain as a positive real integer.
        %
        %  Default: 100
        MaxNumCentralTracks = 100
    end
    
    properties(Hidden,Constant)
        AssignmentSet  = matlab.system.StringSet({'MatchPairs','Munkres','Jonker-Volgenant','Auction','Custom'});
        StateFusionSet = matlab.system.StringSet({'Cross','Intersection','Custom'});
    end
    
    properties(Nontunable)
        %Assignment Assignment algorithm name
        %   Specify the Assignment as one of the
        %   [{'MatchPairs'}|'Munkres'|'Jonker-Volgenant'|'Auction'|'Custom'].
        %   Munkres is the only assignment algorithm that guarantees
        %   optimality, but it is also the slowest, especially
        %   for large numbers of local and central tracks. The other
        %   algorithms do not guarantee optimality but can be faster for
        %   problems ranging from 20 local and central tracks and larger. 
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
        %     [assignment, unCentral, unLocal] = f(cost, costNonAssignment)
        %   <a href="matlab:edit('assignmunkres')">Example of valid assignment function.</a>
        CustomAssignmentFcn = ''

        %AssignmentThreshold   Threshold for track-to-track assignment
        %   Specify the threshold that controls the assignment of a
        %   local track to a central track as a real scalar or a 1x2 vector
        %   [C1,C2], where C1 <= C2. If specified as a scalar, this value
        %   is used as C1, and C2 is Inf*C1.
        %   Initially, a coarse estimation is done to verify which
        %   combinations of {central track,local track} require an accurate
        %   normalized distance calculation. Only combinations whose coarse
        %   normalized distance is lower than C2 are calculated.
        %   Local tracks can only be assigned to a central track if their
        %   normalized distance from the central track is less than C1.
        %   See the distance method of each tracking filter for explanation
        %   of the distance calculation.
        %   Increase the values if there are local tracks that should be
        %   assigned to central tracks but are not. Decrease the values if
        %   there are local tracks that are assigned to central tracks they
        %   should not be assigned to (too far).
        %
        %   Default: [1 Inf] * 30.0
        AssignmentThreshold = [1 Inf] * 30.0
    end
    
    properties(Nontunable)
        %FuseConfirmedOnly  Fuse only confirmed local tracks
        %   Set this value to true if you want to fuse only confirmed
        %   local tracks. Set it to false if you want to fuse all local
        %   tracks regardless of their IsConfirmed state.
        %
        %   Default: true
        FuseConfirmedOnly (1, 1) logical = true
        
        %FuseCoasted  Fuse coasted local tracks
        %   Set this value to true if you want to fuse local tracks with
        %   status IsCoasted = true. Set it to false if you want to only
        %   fuse local tracks that are not coasted.
        %
        %   Default: false
        FuseCoasted (1, 1) logical = false
    end
    
    properties(Nontunable)
        %StateFusion  Algorithm used in track states fusion
        %   Specify the state and state covariance fusion algorithm as one
        %   of the [{'Cross'}|'Intersection'|'Custom'], where:
        %     'Cross'        - uses cross-covariance fusion algorithm
        %     'Intersection' - uses covariance intersection algorithm
        %     'Custom'       - allows you specify a fusion function
        %
        %   Use the StateFusionParameters property to specify additional
        %   parameters used by the state fusion algorithm.
        %
        %   Default: 'Cross'
        StateFusion = 'Cross'
        
        %CustomStateFusionFcn  Custom state fusion function
        %   Specify a state fusion function to be used when StateFusion is
        %   set to 'Custom'. The function must have the following syntax:
        %     [fusedState,fusedCov] = f(trackState,trackCov,fuseParams)
        %
        %   trackState is an N-by-M matrix, where N is the dimension of the
        %   state and M is the number of tracks, and their corresponding 
        %   covariance matrices, trackCov, an N-by-N-by-M matrix. The
        %   optional parameters, fuseParams, is as defined in the
        %   StateFusionParameters property.
        %   The function returns a single fused state, fusedState, an
        %   N-by-1 vector, and its covariance, fusedCov, an N-by-N matrix.
        %
        %   Default: ''
        CustomStateFusionFcn = ''
        
        %StateFusionParameters Parameters used by state fusion algorithm
        %   Specify optional parameters used by the state fusion algorithm.
        %   By default, this property is empty. You can specify these
        %   parameters in any way, as long as they match the optional
        %   arguments of your state fusion algorithm.
        %
        %   Default: none
        StateFusionParameters 
    end
    properties (Nontunable, Hidden)
        %AttributeFusionFcn  Function used for attributes fusion
        %   Specify the attribute fusion function to use. This function
        %   takes the list of ObjectAttributes from the central track and
        %   the local tracks associated with it and returns a fused set of
        %   new ObjectAttributes to the central track.
        %
        %   The default function simply returns the central attributes.
        %
        %   Default: ''
        AttributeFusionFcn = ''
    end
    properties (Nontunable)
        %TrackLogic is assumed to be history-based
        %ConfirmationThreshold   Threshold for track confirmation
        %   Specify the threshold for track confirmation  as [M N], where
        %   a track will be confirmed if it receives at least M out of N
        %   updates.
        %
        %   Default: [2 3]
        ConfirmationThreshold = [2 3]
        
        %DeletionThreshold   Threshold for track deletion
        %   Specify the threshold for track deletion as [P R], where a
        %   track will be deleted if in the last R updates, at least P
        %   times it was not assigned to any local track.
        %
        %   Default: [5 5]
        DeletionThreshold = [5 5]
    end
    
    % Sub-objects used by this object
    properties(Access = private)
        %cFuser - Fuser object
        cFuser
    end
    
    properties(Access = {?trackFuser, ?matlab.unittest.TestCase})
        %cAssigner - Assigner object
        cAssigner
    end
    
    properties(Access=protected)
        pLastTrackID = uint32(0)
    end
    
    properties(Access = {?trackFuser, ?matlab.unittest.TestCase}, Constant = true)
        %constAssignmentExpansion A constant to expand AssignmentThreshold
        constAssignmentExpansion = [1,Inf];
    end
    
    methods
        function obj = trackFuser(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
            if ~isa(obj,'fusion.simulink.trackFuser')
                ind = fusion.internal.findProp('SourceConfigurations',varargin{:});
                if ind <= nargin-1
                    obj.SourceConfigurations = varargin{ind+1};
                elseif coder.target('MATLAB')
                    obj.SourceConfigurations = getDefaultConfiguration(obj);
                else
                    coder.internal.error('fusion:trackFuser:ConfigurationsCGSet','SourceConfigurations');
                end
            end
        end
        
        function id = initializeTrack(obj,inputTrack)
            %initializeTrack Initialize a new track in the track fuser
            %   id = initializeTrack(obj,inputTrack) initializes a new
            %   track in the track fuser and returns the TrackID associated
            %   with it. A warning is issued if the track fuser already
            %   maintains MaxNumCentralTracks tracks and the returned id is
            %   zero, which indicates a failure to initialize the track.
            %
            % Notes: 
            %   1. The tracker must be updated at least once to be able to
            %      initialize a track.
            %   2. The fuser assigns a TrackID to the track, gives an
            %      UpdateTime equal to the last step time, and initializes
            %      the track logic with a single hit.
            
            % Has the fuser locked yet?
            coder.internal.assert(isLocked(obj),...
                'fusion:trackFuser:ObjectNotLocked','initializeTracks');
            
            validateattributes(inputTrack,{'objectTrack','struct'},...
                {'scalar'},'initializeTracks','inputTrack');
            if isstruct(inputTrack)
                % This will error out if the struct is not the right one
                t = objectTrack(inputTrack);
            else
                t = inputTrack;
            end
            
            newTrackIndex = obj.pNumLiveTracks + 1;
            if newTrackIndex <= obj.MaxNumCentralTracks
                coder.internal.assert(numel(t.State)==obj.pStateSize,...
                    'fusion:trackFuser:NumStatesMismatch','inputTrack',obj.pStateSize)
                obj.pNumLiveTracks  = obj.pNumLiveTracks + 1;
                obj.pLastTrackID = obj.pLastTrackID + 1;
                obj.pTracksList{newTrackIndex} = t;
                id = obj.pLastTrackID;
                obj.pTracksList{newTrackIndex}.TrackID = id;
                obj.pTracksList{newTrackIndex}.UpdateTime = obj.pLastTimeStamp;
                
                % Register the track
                obj.pTrackIDs(newTrackIndex) = id;
                obj.pConfirmedTracks(newTrackIndex) = t.IsConfirmed;
                
                % Initialize the track logic
                init(obj.pTrackLogics{newTrackIndex})
            else
                coder.internal.warning('fusion:trackFuser:MaxNumTracksReached', 'MaxNumTracks');
                id = uint32(0);
            end
        end
        
        function deleted = deleteTrack(obj,trackID)
            %deleteTrack  Delete a track managed by the track fuser
            %    deleted = deleteTrack(obj,trackID) deletes the track
            %    specified by trackID from the track fuser. The deleted
            %    flag returns true if a track with the same trackID existed
            %    and was deleted. If a track with that trackID did not
            %    exist, the deleted flag is false and a warning is issued.
            %
            % Note: the tracker must be updated at least once to be able to
            % delete a track.
            
            % Has the fuser locked yet?
            coder.internal.assert(isLocked(obj),...
                'fusion:trackFuser:ObjectNotLocked','deleteTrack');
            
            validateattributes(trackID,{'numeric'},{'real','finite','nonsparse',...
                'positive','integer','scalar'},'deleteTrack','trackID')
            
            ind = findTrackByID(obj, trackID);
            
            if isempty(ind)
                coder.internal.warning('fusion:trackFuser:TrackIDNotFound',trackID)
                deleted = false;
            else
                toDelete = false(1,obj.pMaxNumBranches);
                toDelete(ind) = true;
                reset(obj.pTrackLogics{ind});
                recycleTracks(obj, toDelete);
                deleted = true;
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
        
        function set.ConfirmationThreshold(obj,value)
            validateattributes(value,{'numeric'},...
                {'real','finite','nonsparse','integer'},mfilename,...
                'ConfirmationThreshold');
            if isscalar(value)
                modval = [value,value];
            else
                validateattributes(value,{'numeric'},{'numel',2},mfilename,...
                    'ConfirmationThreshold');
                modval = value(:)';
            end
            obj.ConfirmationThreshold = modval;    
        end
        
        function set.DeletionThreshold(obj,value)
            validateattributes(value,{'numeric'},...
                {'real','finite','nonsparse','integer'},mfilename,...
                'ConfirmationThreshold');
            if isscalar(value)
                modval = [value,value];
            else
                validateattributes(value,{'numeric'},{'numel',2},mfilename,...
                    'ConfirmationThreshold');
                modval = value(:)';
            end
            obj.DeletionThreshold = modval;    
        end
        
        function set.SourceConfigurations(obj,value)
            setSources(obj,value);
        end
        
        function value = get.SourceConfigurations(obj)
            value = getSources(obj);
        end
    end
    
    methods(Access=protected)
        function [confTracks,tentTracks,allTracks,info] = stepImpl(obj,localTracks,tFusion)
            info = coreAlgorithm(obj,localTracks,tFusion);
            % Output
            allTracks = formatOutput(obj,(1:obj.pNumLiveTracks));
            confTracks = allTracks(obj.pConfirmedTracks,1);
            tentTracks = allTracks(~obj.pConfirmedTracks(1:obj.pNumLiveTracks),1);
        end
        
        function info = coreAlgorithm(obj,localTracks,tFusion)
            % Check that the time is not in the past
            coder.internal.assert(tFusion > obj.pLastTimeStamp, ...
                'fusion:trackFuser:TimeMustIncrease',mfilename);
            
            % Check that the localTracks time stamps are all less than
            % tFusion
            times = getInputTrackTimes(obj, localTracks);
            coder.internal.assert(all(times <= tFusion + obj.pConstantTimeTolerance) ...
                && all(times >= obj.pLastTimeStamp), ...
                'fusion:trackFuser:TrackFusionTimeMismatch', 'UpdateTime', 'tFusion')
            
            % Track IDs at beginning of step
            prevNumLive = obj.pNumLiveTracks;
            prevInds = (1:prevNumLive);
            initialTrackIDs = (obj.pTrackIDs(1:prevNumLive));
            
            % Collect source system IDs
            collectSourceIDs(obj,localTracks);
            
            % Calculate distance to local tracks
            costMatrix = distance(obj,localTracks,obj.AssignmentThreshold(1));
            
            % Assign
            [assigned, unassignedCentral, unassignedSource] = assign(obj,costMatrix);
            assignedTrIDs = trackIDs(obj, assigned(:,1));
            unassignedTrIDs = trackIDs(obj, unassignedCentral);
            
            % Separate unassigned local tracks that can initialize a
            % central track from the ones that cannot
            isInitializing = getInitializing(obj,localTracks,unassignedSource);
            
            % Initialize new central tracks
            initializeCentralTracks(obj,localTracks, unassignedSource(isInitializing));
            
            initializedTracks = (obj.pTrackIDs(prevNumLive+1:obj.pNumLiveTracks));
            
            % Update assigned central tracks (fuse)
            updated = fuseAssigned(obj,localTracks,assigned);
            
            % Update unassigned central track (coast)
            deleted = coastUnassigned(obj,unassignedCentral,prevInds(~updated));
            
            % Predict all central tracks to tFusion
            predictTracks(obj,cast(tFusion,obj.pClassToUse));
            
            % Register the last time stamp the fuser has been updated to
            obj.pLastTimeStamp = cast(tFusion,obj.pClassToUse);
            
            finalTrackIDs = (obj.pTrackIDs(1:obj.pNumLiveTracks));
            
            if isempty(unassignedSource)
                noninit = unassignedSource;
            else
                noninit = unassignedSource(~isInitializing,1);
            end
            
            info = struct(...
                'TrackIDsAtStepBeginning', initialTrackIDs, ...
                'CostMatrix', costMatrix, ...
                'Assignments', [assignedTrIDs(:),assigned(:,2)], ...
                'UnassignedCentralTracks', unassignedTrIDs, ...
                'UnassignedLocalTracks', unassignedSource,...
                'NonInitializingLocalTracks', noninit,...
                'InitializedCentralTrackIDs', initializedTracks, ...
                'UpdatedCentralTrackIDs', initialTrackIDs(updated), ...
                'DeletedTrackIDs', initialTrackIDs(1,deleted(1:prevNumLive)), ...
                'TrackIDsAtStepEnd', finalTrackIDs);
        end
        function times = getInputTrackTimes(obj, localTracks)
            if isempty(localTracks)
                times = zeros(1,0,obj.pClassToUse);
                return
            end
            if coder.target('MATLAB')
                times = [localTracks.UpdateTime];
            else
                n = numel(localTracks);
                times = zeros(1,n,obj.pClassToUse);
                for i = 1:n
                    times(i) = cast(localTracks(i).UpdateTime,obj.pClassToUse);
                end
            end
        end
        
        function [overallAssignments, overallUnassignedCentralTracks, ...
                overallUnassignedLocalTracks] = assign(obj,costMatrix)
            % Go one local system at a time and assign using the assigner
            numSourceTrks = size(costMatrix,2);
            numCentralTrks = obj.pNumLiveTracks;
            if numSourceTrks == 0
                overallAssignments = repmat(uint32([0 0]), [numSourceTrks, 1]);
                overallUnassignedCentralTracks = cast((1:numCentralTrks)', 'uint32');
                overallUnassignedLocalTracks = repmat(uint32(0), [numSourceTrks, 1]);
                obj.cAssigner.pAssignmentResults = {overallAssignments, ...
                    overallUnassignedCentralTracks, overallUnassignedLocalTracks};
                return
            end
            
            % Initialize the output
            sourceInds = (1:numel(obj.pSourceConfigIDs));
            usedSources = any(obj.pUsedConfigIDs,1);
            sourceIDs = sourceInds(usedSources);
            numSources = sum(usedSources);
            overallAssignments = zeros(numCentralTrks * numSources, 2, 'uint32');
            overallUnassignedLocalTracks = zeros(numSourceTrks, 1, 'uint32');
            overallUnassignedCentralTracks = cast((1:numCentralTrks)', 'uint32');
            
            % Initialize important parameters
            lastAssigned = 0;
            lastUnassigned = 0;            
            for s = 1:numSources
                sourceIndex = find(obj.pUsedConfigIDs(:,sourceIDs(s)));
                if numSourceTrks==0
                    break
                end
                
                thisSourceCM = cast(costMatrix(:, sourceIndex),'like',obj.pTracksList{1}.State);

                [assignments, unassignedCentralTracks, unassignedlocalTracks] = ...
                    step(obj.cAssigner, thisSourceCM);            
                
                numAssigned = size(assignments, 1);  
                if numAssigned
                    overallAssignments(lastAssigned+1: lastAssigned+numAssigned, 1) = assignments(:,1);
                    overallAssignments(lastAssigned+1: lastAssigned+numAssigned, 2) = sourceIndex(assignments(:,2));
                    lastAssigned = lastAssigned + numAssigned;
                end
                
                numUnassigned = size(unassignedlocalTracks, 1);
                if numUnassigned
                    overallUnassignedLocalTracks(lastUnassigned+1: lastUnassigned+numUnassigned) = sourceIndex(unassignedlocalTracks(:,1))';
                    lastUnassigned = lastUnassigned + numUnassigned;
                end
                
                numUnassignedTracks = size(unassignedCentralTracks, 1);
                if numUnassignedTracks
                    overallUnassignedCentralTracks = intersect(overallUnassignedCentralTracks, sort(unassignedCentralTracks));
                else
                    overallUnassignedCentralTracks = unassignedCentralTracks;
                end
            end
            overallUnassignedLocalTracks = overallUnassignedLocalTracks(overallUnassignedLocalTracks(:,1) > 0);
            overallUnassignedLocalTracks = unique(overallUnassignedLocalTracks);
            overallAssignments = overallAssignments(overallAssignments(:,1) > 0, :);
        end
        
        function isInitializing = getInitializing(obj,localTracks,unassignedSource)
            n = numel(unassignedSource);
            isInitializing = true(n,1);
            for i = 1:n
                thisSource = getConfigByTrack(obj,localTracks(unassignedSource(i)));
                isInitializing(i) = thisSource.IsInitializingCentralTracks;
            end
        end
        
        function initializeCentralTracks(obj,localTracks,unassignedlocalTracks)
            % Initialize the central tracks
            allSourceInds = zeros(numel(unassignedlocalTracks),1,obj.pClassToUse);
            for i = 1:numel(allSourceInds)
                allSourceInds(i) = localTracks(unassignedlocalTracks(i)).SourceIndex;
            end
            
            while numel(unassignedlocalTracks) > 0
                if obj.pNumLiveTracks < obj.MaxNumCentralTracks % Room for tracks
                    % Convert local track to central track
                    thisTrack = localTracks(unassignedlocalTracks(1));
                    thisConfig = getConfigByTrack(obj,thisTrack);
                    
                    % Do not create a central track if it's not your own source
                    % or if the track is coasted. This is to avoid rumors.
                    if ~thisConfig.IsInternalSource && thisTrack.IsCoasted
                        break
                    end
                    centralTrack = ensureTrack(obj, transformToCentral(thisConfig,thisTrack));
                    obj.pNumLiveTracks = obj.pNumLiveTracks + 1;
                    obj.pLastTrackID   = obj.pLastTrackID + 1;
                    centralTrack.TrackID = obj.pLastTrackID;
                    centralTrack.SourceIndex = uint32(obj.FuserIndex);
                    centralTrack.Age = uint32(1);
                    centralTrack.StateParameters = obj.StateParameters;
                    
                    % Add self reporting and object class ID
                    obj.pTracksList{obj.pNumLiveTracks} = centralTrack;
                    centralTrack.IsSelfReported = getSelfReporting(obj,localTracks,unassignedlocalTracks(1));
                    isr = centralTrack.IsSelfReported;
                    centralTrack.ObjectClassID = getClassID(obj,localTracks,obj.pNumLiveTracks,unassignedlocalTracks(1));
                    
                    % Track logic
                    init(obj.pTrackLogics{obj.pNumLiveTracks});
                    centralTrack.IsConfirmed = checkConfirmation(obj.pTrackLogics{obj.pNumLiveTracks}) ...
                        || (centralTrack.ObjectClassID > 0);
                    
                    % Register track
                    obj.pTrackIDs(obj.pNumLiveTracks) = obj.pLastTrackID;
                    obj.pTracksList{obj.pNumLiveTracks} = centralTrack;
                    obj.pConfirmedTracks(obj.pNumLiveTracks) = centralTrack.IsConfirmed;
                    
                    % Multi-sensor case:
                    % Eliminate any unassigned local tracks within the new track
                    % gate to avoid unnecessary track initiation. Use the ones
                    % that are within the gate to update the initiated track.
                    
                    costMatrix = zeros(1, size(unassignedlocalTracks, 1));
                    initTrack = localTracks(unassignedlocalTracks(1));
                    initSource = initTrack.SourceIndex;
                    sameSensor = (allSourceInds == initSource);
                    costMatrix(sameSensor) = inf;
                    costMatrix(~sameSensor) = distanceToCentralTrack(obj,localTracks,centralTrack, unassignedlocalTracks(~sameSensor(:)));
                    
                    checkedUnassigned = unassignedlocalTracks;
                    remainingInds = allSourceInds;
                    
                    assignedToNewTrack = costMatrix(1,:)< obj.AssignmentThreshold(1);
                    if any(assignedToNewTrack)
                        nAssigned = sum(assignedToNewTrack);
                        thidInd = repmat(obj.pNumLiveTracks,nAssigned,1);
                        fuseAssigned(obj,localTracks,[thidInd,unassignedlocalTracks(assignedToNewTrack)]);
                        checkedUnassigned = unassignedlocalTracks(~assignedToNewTrack);
                        remainingInds = allSourceInds(~assignedToNewTrack);
                        obj.pTracksList{obj.pNumLiveTracks}.IsSelfReported = isr || ...
                            obj.pTracksList{obj.pNumLiveTracks}.IsSelfReported;
                    end
                    
                    % Done with first one
                    checkedUnassigned(1) = [];
                    remainingInds(1) = [];
                    if ~isempty(remainingInds)
                        unassignedlocalTracks = checkedUnassigned;
                        allSourceInds = remainingInds;
                    else
                        break
                    end
                else % Cannot initialize another track. Return
                    coder.internal.warning('fusion:trackFuser:MaxNumTracksReached', 'MaxNumTracks');
                    return
                end
            end
        end
        
        function out = ensureTrack(~, in)
            % Since we're not preprocessing local tracks anymore, they can
            % be either struct or an objectTrack. This method ensures the
            % output is an objectTrack
            if isstruct(in)
                out = objectTrack(in);
            else
                out = in;
            end
        end
        
        function cost = distanceToCentralTrack(obj,localTracks,initializedCentralTrack,localInds)
            % thisTrack is a track that already initialized a central track
            
            if isempty(localInds)
                cost = zeros(1,0,'like',initializedCentralTrack.State);
                return
            end
            
            cost = zeros(1,numel(localInds),'like',initializedCentralTrack.State);
            for i = 1:numel(localInds)
                thisLocalTrack = localTracks(localInds(i));
                
                % First predict the initialized central track to local
                % track time
                tempCentralTrack = initializedCentralTrack;
                tempCentralTrack.StateParameters = thisLocalTrack.StateParameters;
                tempCentralTrack.SourceIndex = cast(thisLocalTrack.SourceIndex,'like',tempCentralTrack.SourceIndex);
                dt = thisLocalTrack.UpdateTime - initializedCentralTrack.UpdateTime;
                [x,P] = fusion.internal.gaussEKFilter.predict(...
                    tempCentralTrack.State,tempCentralTrack.StateCovariance,...
                    obj.ProcessNoise,obj.pStateTransitionFcn,obj.pStateTransitionJacobianFcn,...
                    obj.HasAdditiveProcessNoise,dt);
                tempCentralTrack.State = x;
                tempCentralTrack.StateCovariance = P;
                
                % Now, transform to source frame
                thisSource = getConfigByTrack(obj,thisLocalTrack);
                tempLocalTrack = transformToLocal(thisSource,tempCentralTrack);
                allStates = cat(2,tempLocalTrack.State(:),thisLocalTrack.State(:));
                allCovars = cat(3,tempLocalTrack.StateCovariance,thisLocalTrack.StateCovariance);
                allCosts = (fusion.internal.gaussNormDiff(allStates,allCovars,true));
                cost(1,i) = allCosts(2);
            end
        end
        
        function updated = fuseAssigned(obj,localTracks,assignments)
            % Fuse information from the local tracks into the central
            % tracks and update the central tracks
            
            % Move all the local tracks from local frame to central frame
            numAssigned = size(assignments,1);
            transformedTracks = repmat({obj.pTracksList{1}},[1,numAssigned]);
            for i = 1:numAssigned
                thisTrack = localTracks(assignments(i,2));
                thisConfig = getConfigByTrack(obj,thisTrack);
                transformedTracks{i} = ensureTrack(obj, transformToCentral(thisConfig,thisTrack));
            end
            
            % Update the rest of the central track properties
            uniqueAssigned = unique(assignments(:,1));
            numUnique = numel(uniqueAssigned);
            updated = false(1,numUnique);
            for i = 1:numUnique
                thisTrackIndex = uniqueAssigned(i);
                inAssigned = (thisTrackIndex==assignments(:,1));
                assignedSourceInds = find(inAssigned);
                toFuse = getToFuse(obj, transformedTracks, assignedSourceInds);
                if any(toFuse)
                    obj.pTracksList{thisTrackIndex}.Age = obj.pTracksList{thisTrackIndex}.Age + sum(toFuse);
                    obj.pTracksList{thisTrackIndex}.ObjectClassID = getClassID(obj,localTracks,thisTrackIndex,assignments(inAssigned,2));
                
                    obj.pTracksList{thisTrackIndex} = fuse(obj.cFuser,obj.pTracksList{thisTrackIndex},transformedTracks,assignedSourceInds(toFuse));
                
                    % Update track logic and confirmation
                    for j = 1:sum(toFuse)
                        hit(obj.pTrackLogics{thisTrackIndex});
                    end
                    obj.pTracksList{thisTrackIndex}.IsConfirmed = ...
                        obj.pTracksList{thisTrackIndex}.IsConfirmed ...
                        || checkConfirmation(obj.pTrackLogics{thisTrackIndex}) ...
                        || (obj.pTracksList{thisTrackIndex}.ObjectClassID > 0);
                    obj.pConfirmedTracks(thisTrackIndex) = obj.pTracksList{thisTrackIndex}.IsConfirmed;
                    
                    obj.pTracksList{thisTrackIndex}.IsCoasted = getCoasting(obj,localTracks,assignments(inAssigned,2));
                    obj.pTracksList{thisTrackIndex}.IsSelfReported = getSelfReporting(obj,localTracks,assignments(inAssigned,2));
                    obj.pTracksList{thisTrackIndex}.StateParameters = obj.StateParameters;
                    obj.pTracksList{thisTrackIndex}.ObjectAttributes = fuseAttributes(obj,localTracks,thisTrackIndex,assignments(inAssigned,2));
                    updated(i) = true;
                end
            end
        end
        
        function tf = getCoasting(~,localTracks,assignedlocalTracks)
            numAssigned = numel(assignedlocalTracks);
            tf = false;
            i = 0;
            while ~tf && i < numAssigned
                i = i + 1;
                thisSourceTrack = localTracks(assignedlocalTracks(i));
                tf = tf || thisSourceTrack.IsCoasted;
            end
        end
        
        function tf = getSelfReporting(obj,localTracks,assignedlocalTracks)
            % A track is considered self-reported if at least one of the
            % local tracks assigned to it comes from a source that is
            % considered internal. 
            numAssigned = numel(assignedlocalTracks);
            tf = false;
            i = 0;
            while ~tf && i < numAssigned
                i = i + 1;
                thisSourceTrack = localTracks(assignedlocalTracks(i));
                thisSource = getConfigByTrack(obj,thisSourceTrack);
                tf = tf || (thisSource.IsInternalSource && thisSourceTrack.IsSelfReported);
            end
        end
        
        function id = getClassID(obj,localTracks,i,assignedlocalTracks)
            % Get the new Class ID of the track from the local tracks
            
            % Class ID can change only if it is unknown
            % If Class ID is unknown, accept the first known class ID
            numAssigned = numel(assignedlocalTracks);
            id = obj.pTracksList{i}.ObjectClassID;
            j = 0;
            while id == 0 && j < numAssigned
                j = j + 1;
                thisSourceTrack = localTracks(assignedlocalTracks(j));
                id = thisSourceTrack.ObjectClassID;
            end
        end
        
        function toFuse = getToFuse(obj, transformedTracks, assignedSourceInds)
            % Choose which local tracks to fuse based on their properties
            % and on the user choices
            numTracks = numel(assignedSourceInds);
            toFuse = true(numTracks,1);
            for i = 1:numTracks
                ind = assignedSourceInds(i);
                track = transformedTracks{ind};
                toFuse(i) = ((track.IsConfirmed || ~obj.FuseConfirmedOnly) && ...
                    (~track.IsCoasted || obj.FuseCoasted)) || ...
                    track.IsSelfReported;
            end
        end
        
        function attributes = fuseAttributes(obj,localTracks,i,assignedlocalTracks)
            % Invokes the AttributesFusion function
            
            thisTrack = obj.pTracksList{i};
            thisObjAttr = thisTrack.ObjectAttributes;
            
            % The other attributes
            numAssigned = numel(assignedlocalTracks);
            otherAttributes = cell(1,numAssigned);
            for i = 1:numAssigned
                if isa(localTracks(assignedlocalTracks(i)),'objectTrack') || ...
                        (isstruct(localTracks(assignedlocalTracks(i))) && ...
                        isfield(localTracks(assignedlocalTracks(i)), 'ObjectAttributes'))
                    otherAttributes{i} = localTracks(assignedlocalTracks(i)).ObjectAttributes;
                end
            end
            
            attributes = thisObjAttr;
        end
        
        function toDelete = coastUnassigned(obj,unassignedTracks,notUpdated)
            % This method adds the coast unassigned information
            unassignedTracks = unique([unassignedTracks', notUpdated]);
            numUnassigned = numel(unassignedTracks);
            toDelete = false(1,obj.pMaxNumBranches);
            for i = 1:numUnassigned
                obj.pTracksList{unassignedTracks(i)}.Age = obj.pTracksList{unassignedTracks(i)}.Age + 1;
                obj.pTracksList{unassignedTracks(i)}.IsCoasted = true;
                miss(obj.pTrackLogics{unassignedTracks(i)});
                toDelete(unassignedTracks(i)) = checkDeletion(obj.pTrackLogics{unassignedTracks(i)},...
                    ~obj.pTracksList{unassignedTracks(i)}.IsConfirmed,...
                    obj.pTracksList{unassignedTracks(i)}.Age);
                if toDelete(unassignedTracks(i))
                    reset(obj.pTrackLogics{unassignedTracks(i)});
                end
            end
            recycleTracks(obj, toDelete);
        end
        
        function predictTracks(obj,tFusion)
            % This method predicts all the tracks to the update time
            
            for i = 1:obj.pNumLiveTracks
                currentTime = obj.pTracksList{i}.UpdateTime;
                dt = tFusion - currentTime;
                if dt>0
                    [x,P] = fusion.internal.gaussEKFilter.predict(...
                        obj.pTracksList{i}.State,obj.pTracksList{i}.StateCovariance,...
                        obj.ProcessNoise,obj.pStateTransitionFcn,...
                        obj.pStateTransitionJacobianFcn,obj.HasAdditiveProcessNoise,dt);
                    track = obj.pTracksList{i};
                    track.State = x;
                    track.StateCovariance = P;
                    track.UpdateTime = cast(tFusion, obj.pClassToUse);
                    obj.pTracksList{i} = track;
                end
            end
        end

        function setupImpl(obj,localTracks,~,varargin)
            % Perform one-time calculations, such as computing constants
            
            % There has to be at least one track in the input. Only call if
            % setup has not been done.
            if ~coder.internal.is_defined(obj.pMaxNumBranches)
                coder.internal.errorIf(isempty(localTracks),'fusion:trackFuser:EmptyTrackInput');
                obj.pMaxNumBranches = obj.MaxNumCentralTracks;
                if ~strcmpi(class(obj),'fusion.simulink.trackFuser')
                    setupImpl@fusion.internal.FuserManager(obj,localTracks);
                end
            end
            
            % At this time, properties are already validated. Just make
            % sure that StateFusionParameters are fully spelled out in the
            % case of Intersection
            if strcmpi(obj.StateFusion,'Intersection') && ...
                    ~isempty(obj.StateFusionParameters)
                if strncmpi(obj.StateFusionParameters(1),'d',1) % 'det'
                    obj.StateFusionParameters = 'det';
                elseif strncmpi(obj.StateFusionParameters(1),'t',1) % 'trace'
                    obj.StateFusionParameters = 'trace';
                end
            end
            
            % Set the cost calculator object
            obj.cAssigner = matlabshared.tracking.internal.fusion.AssignerGNN(...
                'Assignment', obj.Assignment,...
                'CustomAssignmentFcn', obj.CustomAssignmentFcn,...
                'AssignmentThreshold', obj.AssignmentThreshold);
            
            % Set the fuser object with the correct fusion algorithm
            if coder.target('MATLAB')
                fusionInd = getIndex(obj.StateFusionSet,obj.StateFusion);
                switch fusionInd
                    case 1 % Cross
                        fuseFcn = @fusexcov;
                    case 2 % Intersection
                        fuseFcn = @fusecovint;
                    otherwise
                        fuseFcn = obj.CustomStateFusionFcn;
                end
            else % Code generation doesn't allow to use methods from matlab.system.StringSet
                switch lower(obj.StateFusion)
                    case 'cross'
                        fuseFcn = @fusexcov;
                    case 'intersection'
                        fuseFcn = @fusecovint;
                    otherwise
                        fuseFcn = obj.CustomStateFusionFcn;
                end
            end
            
            if coder.internal.is_defined(obj.StateFusionParameters)
                obj.cFuser = fusion.internal.Fuserxcov(...
                    obj.pStateTransitionFcn, obj.pStateTransitionJacobianFcn,...
                    obj.ProcessNoise, obj.HasAdditiveProcessNoise, fuseFcn, ...
                    obj.StateFusionParameters);
            else
                obj.cFuser = fusion.internal.Fuserxcov(...
                    obj.pStateTransitionFcn, obj.pStateTransitionJacobianFcn,...
                    obj.ProcessNoise, obj.HasAdditiveProcessNoise, fuseFcn, []);
            end
            
            % Set the track logics
            obj.pTrackLogics = cell(1,obj.pMaxNumBranches);
            for i = coder.unroll(1:obj.pMaxNumBranches)
                obj.pTrackLogics{i} = trackHistoryLogic(...
                    'ConfirmationThreshold', obj.ConfirmationThreshold, ...
                    'DeletionThreshold', obj.DeletionThreshold);
            end
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            
            % Reset tracks
            resetImpl@fusion.internal.FuserManager(obj);
            reset(obj.cAssigner);
            for i = coder.unroll(1:obj.pMaxNumBranches)
                reset(obj.pTrackLogics{i});
            end
            obj.pLastTrackID = uint32(0);
        end

        function releaseImpl(obj)
            % Release resources, such as file handles
            
            releaseImpl@fusion.internal.FuserManager(obj);
            release(obj.cAssigner);
            for i = coder.unroll(1:obj.pMaxNumBranches)
                reset(obj.pTrackLogics{i});
            end
            obj.pLastTrackID = uint32(0);
        end

        function defaultConfig = getDefaultConfiguration(obj)
            defaultConfig = cell(1,obj.MaxNumSources);
            for i = coder.unroll(1:obj.MaxNumSources)
                defaultConfig{i} = fuserSourceConfiguration('SourceIndex',i);
            end
        end
        
        % Methods related to object display
        function groups = getPropertyGroups(~)
            % Define property section(s) for display in Matlab
            
            fuserGroup = matlab.mixin.util.PropertyGroup(...
                {'FuserIndex', 'MaxNumCentralTracks', 'MaxNumSources','SourceConfigurations'});
            
            assignProps = matlabshared.tracking.internal.fusion.AssignerGNN.getDispList;
            assignGroup = matlab.mixin.util.PropertyGroup(assignProps);
            
            predictGroup = matlab.mixin.util.PropertyGroup(...
                {'StateTransitionFcn','StateTransitionJacobianFcn',...
                 'ProcessNoise','HasAdditiveProcessNoise','StateParameters'});
            
            logicGroup = matlab.mixin.util.PropertyGroup(...
                {'ConfirmationThreshold', 'DeletionThreshold'});
            
            fusionGroup = matlab.mixin.util.PropertyGroup(...
                {'FuseConfirmedOnly','FuseCoasted',...
                'StateFusion','CustomStateFusionFcn',...
                'StateFusionParameters'});
            
            numGroup = matlab.mixin.util.PropertyGroup(...
                {'NumCentralTracks', 'NumConfirmedCentralTracks'});
            
            groups = [fuserGroup,assignGroup,predictGroup,logicGroup,fusionGroup,numGroup];
        end
        
        function flag = isInactivePropertyImpl(obj, prop)
            % Return false if property is visible based on object 
            % configuration, for the command line and System block dialog            
            flag = strcmp(prop, 'CustomAssignmentFcn') && ~strcmp(obj.Assignment,'Custom');
            
            flag = flag | (strcmp(prop, 'CustomStateFusionFcn') && ~strcmp(obj.StateFusion,'Custom'));
        end

        function flag = isInputSizeMutableImpl(~, idx)
        % All inputs except time are varsize
            if idx == 2
                flag = false;
            else
                flag = true;
            end
        end
        
        function flag = isInputComplexityMutableImpl(~, ~)
            flag = true;
        end

        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s

            % Set private and protected properties
            obj.cAssigner    = s.cAssigner;
            obj.cFuser       = s.cFuser;
            obj.pLastTrackID = s.pLastTrackID;

            % Set public properties and states
            loadObjectImpl@fusion.internal.FuserManager(obj,s,wasLocked);
        end

        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj

            % Set public properties and states
            s = saveObjectImpl@fusion.internal.FuserManager(obj);
            s.cAssigner    = obj.cAssigner;
            s.cFuser       = obj.cFuser;
            s.pLastTrackID = obj.pLastTrackID;
        end
    end
    
    methods
        function blkHandle = exportToSimulink(obj,varargin)
            % EXPORTTOSIMULINK Export the fuser to a Simulink model.
            %
            % EXPORTTOSIMULINK(FUSER) exports the fuser, FUSER, as a
            % Simulink block in a new model with default name.
            %
            % EXPORTTOSIMULINK(..., 'Model', MODEL), additionally, allows
            % you to export the fuser to an existing Simulink model MODEL.
            % MODEL can be the name or handle of the Simulink
            % model. If a Simulink model with name MODEL does not exist, a
            % new model is created with name MODEL.
            %
            % EXPORTTOSIMULINK(..., 'BlockName', NAME), additionally,
            % allows you to specify a name, NAME for the Simulink block.
            %
            % EXPORTTOSIMULINK(..., 'Position', POS), additionally, allows
            % you to specify block position, POS, in the model. POS must be
            % a vector of coordinates, in pixels: [left top right bottom]
            %
            % EXPORTTOSIMULINK(...,'OpenModel', TF), allows you to specify
            % a flag, TF, to indicate if the model should be opened after
            % exporting the FUSER to Simulink or not. The default value
            % is set to true which means the model is always opened.
            %
            % H = EXPORTTOSIMULINK(FUSER, ...) exports the fuser, FUSER,
            % as a Simulink block and returns the handle to the
            % block.

            %Add fuser block to the model and set parameters that are common
            %between matlab object and Simulink block.
            blkHandle = exportToSimulink@fusion.internal.ExportToSimulinkInterface(obj,varargin{:});

            % Set properties that are exposed differently in Simulink.
            if strcmpi(obj.StateFusion,'Cross') && ~isempty(obj.StateFusionParameters)
                set_param(blkHandle,'StateFusionParamSource','Property');
                set_param(blkHandle,'StateFusionCross',mat2str(obj.StateFusionParameters));
            elseif strcmpi(obj.StateFusion,'Intersection') && ~isempty(obj.StateFusionParameters)
                set_param(blkHandle,'StateFusionParamSource','Property');
                set_param(blkHandle,'StateFusionIntersection',obj.StateFusionParameters);
            elseif strcmpi(obj.StateFusion,'Custom') && ~isempty(obj.StateFusionParameters)
                set_param(blkHandle,'StateFusionParamSource','Property');
                set_param(blkHandle,'StateFusionCustom',fusion.simulink.internal.addParamInModelPreLoadCallback...
                    (blkHandle,'StateFusionParameters',obj.StateFusionParameters));
            else
                if ~isempty(obj.StateFusionParameters)
                    blkName = get_param(blkHandle,'Name');
                    warning(message('fusion:simulink:exportToSimulink:FailedToSetParam',...
                        'StateFusionParameters',blkName,class(obj)));
                end
            end
            
            %Preallocate memory for config.
            config = repmat(toStruct(obj.SourceConfigurations{1}),...
                numel(obj.SourceConfigurations),1);
            for i = 1:numel(obj.SourceConfigurations)
                %SourceConfigurations is a cell array of fuserSourceConfiguration.
                config(i) = toStruct(obj.SourceConfigurations{i});
            end
            set_param(blkHandle,'SourceConfigurationExpression',...
                fusion.simulink.internal.addParamInModelPreLoadCallback(blkHandle,...
                'SourceConfigurations',config));
        end
    end
    
    methods(Static, Hidden)    
        function flag = isAllowedInSystemBlock
            flag = false;
        end
    end
end

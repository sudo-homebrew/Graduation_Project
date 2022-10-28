classdef trackFuser< fusion.internal.FuserManager & fusion.internal.ExportToSimulinkInterface
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
%    <a href="matlab:help matlab.System/reset   ">reset</a>               - Resets states of the trackerFuser
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

    methods
        function out=trackFuser
            % Support name-value pair arguments when constructing object
        end

        function out=assign(~) %#ok<STOUT>
            % Go one local system at a time and assign using the assigner
        end

        function out=coastUnassigned(~) %#ok<STOUT>
            % This method adds the coast unassigned information
        end

        function out=coreAlgorithm(~) %#ok<STOUT>
            % Check that the time is not in the past
        end

        function out=deleteTrack(~) %#ok<STOUT>
            %deleteTrack  Delete a track managed by the track fuser
            %    deleted = deleteTrack(obj,trackID) deletes the track
            %    specified by trackID from the track fuser. The deleted
            %    flag returns true if a track with the same trackID existed
            %    and was deleted. If a track with that trackID did not
            %    exist, the deleted flag is false and a warning is issued.
            %
            % Note: the tracker must be updated at least once to be able to
            % delete a track.
        end

        function out=distanceToCentralTrack(~) %#ok<STOUT>
            % thisTrack is a track that already initialized a central track
        end

        function out=ensureTrack(~) %#ok<STOUT>
            % Since we're not preprocessing local tracks anymore, they can
            % be either struct or an objectTrack. This method ensures the
            % output is an objectTrack
        end

        function out=exportToSimulink(~) %#ok<STOUT>
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
        end

        function out=fuseAssigned(~) %#ok<STOUT>
            % Fuse information from the local tracks into the central
            % tracks and update the central tracks
        end

        function out=fuseAttributes(~) %#ok<STOUT>
            % Invokes the AttributesFusion function
        end

        function out=getClassID(~) %#ok<STOUT>
            % Get the new Class ID of the track from the local tracks
        end

        function out=getCoasting(~) %#ok<STOUT>
        end

        function out=getDefaultConfiguration(~) %#ok<STOUT>
        end

        function out=getInitializing(~) %#ok<STOUT>
        end

        function out=getInputTrackTimes(~) %#ok<STOUT>
        end

        function out=getPropertyGroups(~) %#ok<STOUT>
            % Define property section(s) for display in Matlab
        end

        function out=getSelfReporting(~) %#ok<STOUT>
            % A track is considered self-reported if at least one of the
            % local tracks assigned to it comes from a source that is
            % considered internal.
        end

        function out=getToFuse(~) %#ok<STOUT>
            % Choose which local tracks to fuse based on their properties
            % and on the user choices
        end

        function out=initializeCentralTracks(~) %#ok<STOUT>
            % Initialize the central tracks
        end

        function out=initializeTrack(~) %#ok<STOUT>
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
        end

        function out=isInactivePropertyImpl(~) %#ok<STOUT>
            % Return false if property is visible based on object 
            % configuration, for the command line and System block dialog
        end

        function out=isInputComplexityMutableImpl(~) %#ok<STOUT>
        end

        function out=isInputSizeMutableImpl(~) %#ok<STOUT>
            % All inputs except time are varsize
        end

        function out=loadObjectImpl(~) %#ok<STOUT>
            % Set properties in object obj to values in structure s
        end

        function out=predictTracks(~) %#ok<STOUT>
            % This method predicts all the tracks to the update time
        end

        function out=releaseImpl(~) %#ok<STOUT>
            % Release resources, such as file handles
        end

        function out=resetImpl(~) %#ok<STOUT>
            % Initialize / reset discrete-state properties
        end

        function out=saveObjectImpl(~) %#ok<STOUT>
            % Set properties in structure s to values in object obj
        end

        function out=setupImpl(~) %#ok<STOUT>
            % Perform one-time calculations, such as computing constants
        end

        function out=stepImpl(~) %#ok<STOUT>
        end

    end
    properties
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
        Assignment;

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
        AssignmentThreshold;

        %TrackLogic is assumed to be history-based
        %ConfirmationThreshold   Threshold for track confirmation
        %   Specify the threshold for track confirmation  as [M N], where
        %   a track will be confirmed if it receives at least M out of N
        %   updates.
        %
        %   Default: [2 3]
        ConfirmationThreshold;

        %CustomAssignmentFcn Name of 'Custom' assignment function
        %   Specify the function name for the custom assignment. This
        %   function will be used only when Assignment = 'Custom'. The
        %   function must have the following syntax:
        %     [assignment, unCentral, unLocal] = f(cost, costNonAssignment)
        %   <a href="matlab:edit('assignmunkres')">Example of valid assignment function.</a>
        CustomAssignmentFcn;

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
        CustomStateFusionFcn;

        %DeletionThreshold   Threshold for track deletion
        %   Specify the threshold for track deletion as [P R], where a
        %   track will be deleted if in the last R updates, at least P
        %   times it was not assigned to any local track.
        %
        %   Default: [5 5]
        DeletionThreshold;

        %FuseCoasted  Fuse coasted local tracks
        %   Set this value to true if you want to fuse local tracks with
        %   status IsCoasted = true. Set it to false if you want to only
        %   fuse local tracks that are not coasted.
        %
        %   Default: false
        FuseCoasted;

        %FuseConfirmedOnly  Fuse only confirmed local tracks
        %   Set this value to true if you want to fuse only confirmed
        %   local tracks. Set it to false if you want to fuse all local
        %   tracks regardless of their IsConfirmed state.
        %
        %   Default: true
        FuseConfirmedOnly;

        %FuserIndex - Unique index of the track fuser
        %  Specify the track fuser index as a positive integer scalar
        %
        % Default: 1
        FuserIndex;

        %MaxNumCentralTracks   Maximum number of fused (central-level) tracks
        %  Set the maximum number of central-level tracks the fuser can
        %  maintain as a positive real integer.
        %
        %  Default: 100
        MaxNumCentralTracks;

        %MaxNumSourceConfiguration Maximum number of source configurations
        %   Set the maximum number of sources configurations that the fuser
        %   can maintain as a positive real integer.
        %
        %   Default: 20
        MaxNumSources;

        %SourceConfigurations  Configurations of source systems
        %  Specify the configurations of the source systems that report
        %  tracks to the track fuser as fuserSourceConfiguration. You can
        %  specify the SourceConfigurations during construction as a
        %  Name,value pair or set it after construction. 
        %
        %  Default: a 1-by-MaxNumSources cell array of default
        %  fuserSourceConfiguration objects
        SourceConfigurations;

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
        StateFusion;

        %StateFusionParameters Parameters used by state fusion algorithm
        %   Specify optional parameters used by the state fusion algorithm.
        %   By default, this property is empty. You can specify these
        %   parameters in any way, as long as they match the optional
        %   arguments of your state fusion algorithm.
        %
        %   Default: none
        StateFusionParameters;

        pLastTrackID;

    end
end

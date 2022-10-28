classdef (Abstract) FuserManager < matlab.System
    %FuserManager  An abstract class to manage track fuser objects
    %
    % This is an internal function and may be removed or modified in a future
    % release.
    %
    %  fusion.internal.FuserManager is an abstract system object that provides
    %  methods common to track fusers. You can inherit from this class when you
    %  develop a track fuser. Even though it is a system object, the class does
    %  not implement the step method.
    %
    %  fusion.internal.FuserManager properties:
    %
    %  fusion.internal.FuserManager does not expose any public methods.
    %  When developing a fuser that inherits from this class, use the following
    %  system object methods for the purposes listed:
    %    setupImpl      - Set the fuser manager up. Creates all the tracks list
    %    resetImpl      - Initialize and reset the tracker and tracks list
    %    releaseImpl    - Release the fuser
    %    loadObjectImpl - Load the fuser manager state and tracks list from a struct
    %    saveObjectImpl - Save the fuser manager state and tracks list to a struct
    %
    %  In addition, these methods are provided to support track management:
    %    initiateTrack           - Initiate a track
    %    initializeFilter        - Initialize a track filter
    %    recycleTracks           - Recycle tracks to be deleted
    %    findTrackByID           - Find a track by its ID
    %    trackIDs                - Get a list of track IDs by their indices
    %    predictTracksWithCov    - Predict tracks to time with covariance
    %    predictTracksWithoutCov - Predict tracks to time without covariance
    %    predictTrackStruct      - Predict a single track struct
    
    % Copyright 2019-2021 The MathWorks, Inc.
    
    %#codegen
    
    %#function assignmunkres assignjv assignauction
    
    % Public, tunable properties
    properties(Abstract)
        %SourceConfigurations  Configurations of source systems
        %  Specify the configurations of the source systems that report
        %  tracks to the track fuser. as fusingConfigurations. You can
        %  specify the SourceConfigurations during construction as a
        %  Name,value pair or set it after construction. There are no
        %  default values for the SourceConfigurations. You must specify
        %  them before using the fuser.
        SourceConfigurations
        
        %TrackLogic is assumed to be history-based
        %ConfirmationThreshold   Threshold for track confirmation
        ConfirmationThreshold
        
        %DeletionThreshold   Threshold for track deletion
        DeletionThreshold
    end
    
    properties(Abstract, Nontunable)
        %FuserIndex
        FuserIndex (1, 1) {mustBePositive, mustBeInteger}
        
        %MaxNumSourceConfiguration Maximum number of source configurations
        %   Set the maximum number of sources configurations that the fuser
        %   can maintain as a positive real integer.
        MaxNumSources (1, 1) {mustBePositive, mustBeInteger}
        
        %MaxNumCentralTracks   Maximum number of central-level tracks
        %   Set the maximum number of central-level tracks the fuser can
        %   maintain as a positive real integer.
        MaxNumCentralTracks (1, 1) {mustBePositive, mustBeInteger}
    end
    
    % Set-Protected, public get, tunable properties, not used in Simulink
    properties(SetAccess = protected, Dependent = true)
        %NumCentralTracks  Number of central-level tracks
        %   The total number of central-level tracks the currently
        %   maintained by the track fuser.
        %
        %   This value is calculated by the fuser and is read-only
        NumCentralTracks
        
        %NumConfirmedCentralTracks The number of confirmed central-level tracks
        %   The total number of confirmed central-level tracks (IsConfirmed
        %   = true) that the fuser is maintaining.
        %
        %   This value is calculated by the tracker and is read-only
        NumConfirmedCentralTracks
    end
    
    properties(Nontunable)
        %StateTransitionFcn    A function that predicts the track state
        %   Specify the transition of state between times as a function
        %   that calculates an M dimensional state vector at time k given
        %   the state vector at time k-1. The function may take additional
        %   input parameters if needed, e.g., control inputs or the size of
        %   the time step.
        %
        %   If HasAdditiveProcessNoise is true, the function should have
        %   the following signature:
        %         x(k) = StateTransitionFcn(x(k-1), dt)
        %
        %   where:
        %     x(k) - The (estimated) state at time k. It can be a vector
        %            or a matrix. If a matrix, the state is expected to be
        %            arranged along the columns.
        %     dt   - The timestep for the prediction.
        %
        %   If HasAdditiveProcessNoise is false, the function should have
        %   the following signature:
        %         x(k) = StateTransitionFcn(x(k-1), w(k-1), dt)
        %
        %   where:
        %     x(k) - The (estimated) state at time k. It can be a vector
        %            or a matrix. If a matrix, the state is expected to be
        %            arranged along the columns.
        %     w(k) - The process noise at time k.
        %     dt   - The timestep for the prediction.
        %
        %   Default: 'constvel'
        StateTransitionFcn = 'constvel'
        
        %StateTransitionJacobianFcn   Jacobian of StateTransitionFcn
        %   Specify the function that calculates the Jacobian of the
        %   StateTransitionFcn, f. This function must take the same input
        %   arguments as the StateTransitionFcn. If not specified, the
        %   Jacobian will be numerically computed, which may increase
        %   processing time and numerical inaccuracy.
        %
        %   If HasAdditiveProcessNoise is true, the function should have
        %   the following signature:
        %     dfdx(k) = StateTransitionJacobianFcn(x(k), dt)
        %
        %   where:
        %     dfdx(k) - Jacobian of StateTransitionFcn with respect to
        %               states x, df/dx, evaluated at x(k). An M-by-M
        %               matrix where M is the number of states.
        %     x(k)    - The (estimated) state at time k. It can be a vector
        %               or a matrix. If a matrix, the state is expected to
        %               be arranged along the columns.
        %     dt      - The timestep for the prediction.
        %
        %   If HasAdditiveProcessNoise is false, the function should have
        %   the following signature:
        %     [dfdx(k), dfdw(k)] = StateTransitionJacobianFcn(x(k), w(k), dt)
        %
        %   where:
        %     dfdx(k) - Jacobian of StateTransitionFcn with respect to
        %               states x, df/dx, evaluated at x(k), w(k).
        %               An M-by-M matrix where M is the number of states.
        %     dfdw(k) - Jacobian of StateTransitionFcn with respect to
        %               process noise w, df/dw, evaluated at x(k), w(k).
        %               An M-by-W matrix where W is the number of process
        %               noise terms in w.
        %     x(k)    - The (estimated) state at time k. It can be a vector
        %               or a matrix. If a matrix, the state is expected to
        %               be arranged along the columns.
        %     w(k)     - Process noise at time k.
        %     dt       - The timestep for the prediction.
        %
        %   Default: StateTransitionJacobianFcn = ''
        StateTransitionJacobianFcn = ''
    end
    
    properties(Nontunable)
        %HasAdditiveProcessNoise  True if process noise is additive
        %  A Boolean flag that defines whether the noise affecting the
        %  state transition is additive (true) or non-additive (false).
        %
        %  Default: false
        HasAdditiveProcessNoise (1, 1) logical = false
    end
    
    % Tunable properties
    properties
        %ProcessNoise The process noise covariance matrix
        %   If HasAdditiveProcessNoise is true: specify the covariance of
        %   process noise as a scalar or an M-by-M matrix. If you specify
        %   it as a scalar it will be extended to an M-by-M diagonal
        %   matrix, where M is the number of states returned by the source
        %   configuration when converting to a central track.
        %
        %   If HasAdditiveProcessNoise is false: specify the covariance of
        %   process noise as a W-by-W matrix, where W is the number of the
        %   process noise terms. In this case, ProcessNoise must be
        %   specified before the first call to step. After the first
        %   assignment, you can specify it also as a scalar which will be
        %   extended to a W-by-W matrix.
        %
        %   Default: eye(3), representing a 3-D process noise
        ProcessNoise = eye(3)
        
        %StateParameters Parameters about the track state reference frame
        %   Use this property to specify parameters about the fused tracks
        %   that the track fuser outputs. For example, you can use these
        %   parameters to perform coordinate transfer from a vehicle to
        %   another vehicle or a global frame. This property is tunable.
        %
        %   Default: an empty struct
        StateParameters = struct
    end
    
    % Protected properties
    properties (Access = {?fusion.internal.FuserManager, ?matlab.unittest.TestCase})
        % pTracksList The list of all tracks maintained by the fuser
        % This property is publicly accessible to unit tests as well.
        pTracksList
        
        % pTrackLogics - internal track logic objects maintaining the
        % logic status of each central track
        pTrackLogics
    end
    
    % Protected properties
    properties (Access = protected)
        % pNumLiveTracks The number of tracks that are currently tracked by
        % the tracker
        pNumLiveTracks
        
        % pTrackIDs     The list of track IDs current used
        pTrackIDs
        
        % pConfirmedTracks  A logical array. Each element is true if the
        % corresponding track is confirmed
        pConfirmedTracks
        
        % pLogic Holds an instance of the track logic
        pLogic
        
        % pSourceConfigIDs  A list of source configuration IDs
        pSourceConfigIDs
        
        % pUsedConfigIDs  Which source configuration IDs are used in this sstep
        pUsedConfigIDs
        
        % pSourceConfigurations Internal storage of source configurations
        pSourceConfigurations
        
        % pNumUsedConfigs The number of source configurations currently used
        pNumUsedConfigs
        
        % pIsValidSource  A Boolean array indicating if a source has
        % already been validated once
        pIsValidSource
        
        % pLastTimeStamp The last time step the fuser was stepped to.
        pLastTimeStamp
        
        % pStateParameters Parameters about the track state reference frame
        %
        % Default : an empty struct
        pStateParameters = struct;
    end    
    
    % Nontunable, protected propreties
    properties (Access = protected, Nontunable)
        %pHasObjectAttributes  A logical flag. True if the input
        %local tracks have a ObjectAttributes (field or property)
        pHasObjectAttributes = true
        
        %pClassToUse  Class to use for floating point
        pClassToUse
        
        %pStateSize   Size of the state vector
        pStateSize
        
        %pMaxNumBranches  Maximum number of track branches held
        % This number is equal to MaxNumTracks * MaxNumTrackBranches
        % For trackerGNN, it is equal to MaxNumTracks
        pMaxNumBranches
        
        %pTrackLogicStateSize size of the track logic vector
        pTrackLogicStateSize
    end
    
    properties(Access={?fusion.internal.FuserManager, ?matlab.unittest.TestCase}, Nontunable)
        pStateTransitionFcn
        pStateTransitionJacobianFcn
        pCustomStateFusionFcn
    end
    
    properties(Constant, Hidden)
        %MaxNumLocalTracks Maximum number of local tracks
        % Specify the maximum number of input (local) tracks to the fuser.
        % This number is required for memory allocation in code generation.
        %
        % Default: 1000
        MaxNumLocalTracks (1, 1) {mustBePositive, mustBeInteger} = 1000
    end
    
    properties(Constant, Access=protected)
        pConstantTimeTolerance = 1e-5;
    end
    
    methods
        function obj = FuserManager(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
        %------------------------------------------------------------------
        
        function value = get.NumCentralTracks(obj)
            if coder.internal.is_defined(obj.pNumLiveTracks)
                uniqueTrackIDs = unique(obj.pTrackIDs(1:obj.pNumLiveTracks));
                value = numel(uniqueTrackIDs);
            else %Happens before tracker initializes
                value = 0;
            end
        end
        %------------------------------------------------------------------
        
        function value = get.NumConfirmedCentralTracks(obj)
            if coder.internal.is_defined(obj.pConfirmedTracks)
                uniqueTrackIDs = unique(obj.pTrackIDs(obj.pConfirmedTracks));
                value = numel(uniqueTrackIDs);
            else
                value = 0;
            end
        end
        %------------------------------------------------------------------
        
        function set.StateTransitionFcn(obj,func)
            validateattributes(func,{'function_handle','char','string'},{'nonempty'},...
                mfilename,'StateTransitionFcn');
            obj.StateTransitionFcn = func;
        end
        %------------------------------------------------------------------
        
        function val = get.StateParameters(obj)
            val = obj.pStateParameters;
        end
        %------------------------------------------------------------------
        
        function set.StateParameters(obj,value)
           setStateParameters(obj,value);
        end
        %------------------------------------------------------------------
        
        function set.StateTransitionJacobianFcn(obj,func)
            validateattributes(func,{'function_handle','char','string'},{},...
                mfilename,'StateTransitionJacobianFcn');
            obj.StateTransitionJacobianFcn = func;
        end
        %------------------------------------------------------------------
        
        function predictedTracks = predictTracksToTime(obj, track, pTime, varargin)
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
            % Note: the fuser must be updated at least once to be able to
            % predict tracks.
            
            % Has the tracker locked yet?
            coder.internal.assert(isLocked(obj),...
                'fusion:trackFuser:ObjectNotLocked','predictTracksToTime');
            
            % Input validation
            narginchk(3,5);
            validateattributes(pTime, {'numeric'}, ...
                {'real','finite','nonsparse','scalar','>',obj.pLastTimeStamp}, ...
                'predictTracksToTime', 'time');
            
            % Create a list of tracks to predict
            if ischar(track) || isstring(track)
                type = validatestring(track,{'all','confirmed','tentative'},...
                    'predictTracksToTime','category');
                if strcmpi(type,'all')
                    list = true(1,obj.pNumLiveTracks);
                elseif strcmpi(type,'confirmed')
                    list = obj.pConfirmedTracks;
                else
                    list = ~obj.pConfirmedTracks(1:obj.pNumLiveTracks);
                end
            else
                validateattributes(track, {'numeric'}, ...
                    {'real', 'positive', 'scalar', 'integer'}, ...
                    'predictTracksToTime', 'trackID');
                list = false(1,obj.pNumLiveTracks);
                ind = findTrackByID(obj, track);
                if isempty(ind)
                    coder.internal.warning('fusion:trackFuser:TrackIDNotFound', track);
                else
                    list(ind) = true;
                end
            end
            
            % Check flag if vargargin is not empty
            if ~isempty(varargin)
                coder.internal.assert(numel(varargin)==2,'fusion:trackerGNN:PredictToTimeNargin',3,5)
                validatestring(varargin{1},{'WithCovariance'},mfilename);
                validateattributes(varargin{2},{'numeric','logical'},...
                    {'scalar','binary'},mfilename,'tf');
                withCov = varargin{2};
            else
                withCov = false;
            end
            
            %We have a list of tracks we want to predict. Get them using
            %getTracks and then predict them
            predictedTracks = formatOutput(obj, find(list));
            if isempty(predictedTracks)
                return
            end
            
            if withCov
                predictedTracks = predictTracksWithCov(obj,predictedTracks,pTime);
            else
                predictedTracks = predictTracksWithoutCov(obj,predictedTracks,pTime);
            end
        end
        
        function indices = sourceIndices(obj)
            %sourceIndices Return the list of fuser source indices
            % indices = sourceIndices(obj) returns the list of SourceIndex
            % values from the trackFuser list of SourceConfigurations.
            if coder.target('MATLAB')
                indices = cellfun(@(c) c.SourceIndex, obj.SourceConfigurations);
            else
                indices = repmat(obj.SourceConfigurations{1}.SourceIndex, 1, obj.MaxNumSources);
                for i = 2:obj.MaxNumSources
                    indices(i) = obj.SourceConfigurations{i}.SourceIndex;
                end
            end
        end
    end    
    
    methods(Access = protected)
        function setupImpl(obj,tracks)
            % setupImpl Sets the track manager up. Creates all the tracks
            
            % Get one track from the input
            track = parseTrack(obj,tracks(1));
            sourceConfig = getConfigByTrack(obj,track);
            obj.pHasObjectAttributes = ~isempty(track.ObjectAttributes);
            
            % Convert to a central track. Validate that it matches the
            % StateTransitionFcn used
            ct = transformToCentral(sourceConfig,track);
            state = ct.State;
            stateCov = ct.StateCovariance;
            dt = 1;
            
            % This only happens once. If it works, then the state size is
            % locked. This line will also error out if the dimensions are
            % incompatible, because fusion.internal.gaussEKFilter.predict
            % requires the state dimensions to be compatible.
            fusion.internal.gaussEKFilter.predict(...
                state,stateCov,obj.ProcessNoise,...
                obj.pStateTransitionFcn,obj.pStateTransitionJacobianFcn,...
                obj.HasAdditiveProcessNoise,dt);
            
            obj.pClassToUse = class(state);
            obj.pStateSize  = numel(state);
            validateSourceConfig(obj,track)
            
            if obj.pHasObjectAttributes
                s = ct.ObjectAttributes;
            else
                s = struct;
            end
            
            centralTrack = objectTrack(...
                'TrackID', intmax('uint32'), ...
                'BranchID', intmax('uint32'), ...
                'SourceIndex', obj.FuserIndex, ...
                'UpdateTime', zeros(1,1,obj.pClassToUse), ...
                'Age', intmax('uint32'), ...
                'State', zeros(obj.pStateSize,1,obj.pClassToUse), ...
                'StateCovariance', eye(obj.pStateSize,obj.pClassToUse), ...
                'StateParameters', obj.StateParameters, ...
                'ObjectClassID', zeros(1,1,'like',track.ObjectClassID), ...
                'TrackLogic', coder.const("History"), ...
                'TrackLogicState', false(1, max(obj.ConfirmationThreshold(2), obj.DeletionThreshold(2))), ...
                'IsConfirmed', false, ...
                'IsCoasted', false, ...
                'IsSelfReported', false, ...
                'ObjectAttributes', s);
            
            obj.pTracksList = repmat({centralTrack},1,obj.pMaxNumBranches);
            
            % Define obj.pUsedConfigIDs as variable size
            obj.pUsedConfigIDs = false(1,obj.MaxNumSources);
            obj.pUsedConfigIDs = false(2,obj.MaxNumSources);
            obj.pUsedConfigIDs = false(1,obj.MaxNumSources);
        end
        
        function resetImpl(obj)
            % Initialize / reset the fuser
            
            % Reset tracks
            obj.pNumLiveTracks = 0;
            obj.pLastTimeStamp = -eps(obj.pClassToUse);
            obj.pTrackIDs = zeros(1,obj.pMaxNumBranches,'uint32');
            obj.pConfirmedTracks = false(1, obj.pMaxNumBranches);
            obj.pIsValidSource = false(1,obj.MaxNumSources);
        end
        
        function releaseImpl(obj)
            % Release resources, such as file handles
            obj.pNumLiveTracks = 0;
            obj.pTrackIDs = zeros(1,obj.pMaxNumBranches,'uint32');
            obj.pConfirmedTracks = false(1,obj.pMaxNumBranches);
        end
        
        function setStateParameters(obj,value)
            validateattributes(value,{'struct'},{},class(obj),'StateParameters')
            obj.pStateParameters = value;
        end
        
        %------------------------------------------------------------------
        % Methods used during setup, reset, or release
        %------------------------------------------------------------------
        function trackOut = parseTrack(~,trackIn)
            % trackIn can be either an objectTrack or a struct.
            % trackOut is always an objectTrack.
            if isstruct(trackIn)
                trackOut = objectTrack(trackIn);
            else
                trackOut = trackIn;
            end
        end
        
        function config = getConfigByTrack(obj,track)
            configID = track.SourceIndex;
            config = getConfigByID(obj,configID);
        end
        
        function config = getConfigByID(obj,configID)
            inKnownIDs = (configID == obj.pSourceConfigIDs);
            coder.internal.assert(any(inKnownIDs),'fusion:trackFuser:UnknownConfig','SourceIndex','SourceConfigurations');
            if coder.target('MATLAB')
                config = obj.pSourceConfigurations{inKnownIDs};
            else
                for i = 1:numel(inKnownIDs)
                    if inKnownIDs(i)
                        config = obj.pSourceConfigurations{i};
                        return
                    end
                end
                coder.internal.error('fusion:trackFuser:UnknownConfig','SourceIndex','SourceConfigurations');
            end
        end
        
        function setSources(obj,value)
            validateattributes(value,{'cell'},{'nonempty'},...
                mfilename,'SourceConfigurations');
            coder.internal.assert(numel(value)<=obj.MaxNumSources,...
                'fusion:trackFuser:TooManySourceConfigurations',...
                'SourceConfigurations','MaxNumSources')
            if ~coder.internal.is_defined(obj.pSourceConfigurations)
                obj.pSourceConfigurations = cell(1,obj.MaxNumSources);
                for i = coder.unroll(1:obj.MaxNumSources)
                    obj.pSourceConfigurations{i} = clone(value{1});
                end
            end
            
            for i = 1:numel(value)
                validateattributes(value{i},{'fuserSourceConfiguration'},{},...
                    mfilename,'SourceConfigurations');
                obj.pSourceConfigurations{i} = value{i};
            end
            obj.pNumUsedConfigs = numel(value);
            
            % Refresh the list of configurations
            idType = class(obj.pSourceConfigurations{1}.SourceIndex);
            ids = zeros(1,obj.MaxNumSources,idType);
            for i = 1:obj.pNumUsedConfigs
                ids(i) = cast(obj.pSourceConfigurations{i}.SourceIndex,idType);
            end
            usedIDs = ids(1:obj.pNumUsedConfigs);
            coder.internal.assert(numel(unique(usedIDs))==numel(usedIDs),...
                'fusion:trackFuser:ExpectedUniqueConfigIDs','SourceConfigurations',...
                'SourceIndex');
            obj.pSourceConfigIDs = ids;
            obj.pIsValidSource = false(1,obj.MaxNumSources);
        end
        
        function value = getSources(obj)
            value = cell(1,obj.pNumUsedConfigs);
            for i = 1:obj.pNumUsedConfigs
                value{i} = obj.pSourceConfigurations{i};
            end
        end
        
        %------------------------------------------------------------------
        % Methods used during step
        %------------------------------------------------------------------
        function collectSourceIDs(obj,sourceTracks)
            % Keeps track of which source IDs are used in this step
            numSourceTracks = numel(sourceTracks);
            obj.pUsedConfigIDs = false(numSourceTracks,obj.MaxNumSources);
            for i = 1:numSourceTracks
                % Collect source indices
                configID = sourceTracks(i).SourceIndex;
                inKnownIDs = (configID == obj.pSourceConfigIDs);
                coder.internal.assert(any(inKnownIDs),'fusion:trackFuser:UnknownConfig','SourceIndex','SourceConfigurations');
                obj.pUsedConfigIDs(i,inKnownIDs) = true;
                
                % Validate source if first call to it
                if ~obj.pIsValidSource(inKnownIDs)
                    validateSourceConfig(obj,sourceTracks(i));
                    obj.pIsValidSource(inKnownIDs) = true;
                end
            end
        end
        
        function validateSourceConfig(obj,track)
            configID = track.SourceIndex;
            source = getConfigByID(obj,configID);
            cTrack = transformToCentral(source, track);
            coder.internal.assert(numel(cTrack.State)==obj.pStateSize,...
                'fusion:trackFuser:ConfigInconsistentState',...
                configID,numel(cTrack.State),obj.pStateSize)
        end
        
        function costMatrix = distance(obj,localTracks,maxCost)
            % We calculate distance in the source track space. This means
            % that for each source track, we do the following:
            % 1. Predict the central tracks to the time of the source track
            % 2. Convert the central tracks to the source track space
            % 3. Calculate the distance between the tracks and source track
            
            % Allocate memory
            numSourceTracks = numel(localTracks);
            numCentralTracks = obj.pNumLiveTracks;
            costMatrix = inf(numCentralTracks,numSourceTracks,obj.pClassToUse);
            
            if numSourceTracks == 0 || obj.pNumLiveTracks == 0
                return
            end
            
            % Get the central tracks state & covariance
            if coder.target('MATLAB')
                centralTracks = obj.pTracksList(1:obj.pNumLiveTracks);
                trackClasses = cellfun(@(x) uint32(x.ObjectClassID), centralTracks);
            else
                [centralTracks, trackClasses] = getLiveTracks(obj,numCentralTracks);
            end
           
            [centralStates,centralCovars] = collectTrackStates(obj,centralTracks,obj.pStateSize);
            
            % Loop over source tracks
            centralTime = obj.pTracksList{1}.UpdateTime;
            thisInf = inf(1,1,obj.pClassToUse);
            for i = 1:numSourceTracks
                % Predict to source time
                lTrack = localTracks(i);
                sourceTime = lTrack.UpdateTime;
                sourceState = lTrack.State;
                sourceStateSize = numel(sourceState);
                sourceCovar = lTrack.StateCovariance;
                sourceClass = lTrack.ObjectClassID;
                dt = sourceTime - centralTime;
                [xk,P] = fusion.internal.gaussEKFilter.predict(...
                    centralStates,centralCovars,obj.ProcessNoise,...
                    obj.pStateTransitionFcn,obj.pStateTransitionJacobianFcn,...
                    obj.HasAdditiveProcessNoise,dt);
                
                % Transform to source coordinates
                sourceConfig = getConfigByTrack(obj,lTrack);
                
                transformedStates = repmat(sourceState(:),1,numCentralTracks);
                transformedCovars = repmat(lTrack.StateCovariance,1,1,numCentralTracks);
                for j = 1:numCentralTracks
                    track = centralTracks{j};
                    track.State = xk(:,j);
                    track.StateCovariance = P(:,:,j);
                    track.StateParameters = lTrack.StateParameters;
                    track.SourceIndex = cast(lTrack.SourceIndex,'like',track.SourceIndex);
                    transformedTrack = transformToLocal(sourceConfig,track);
                    transformedStates(1:sourceStateSize,j) = transformedTrack.State(1:sourceStateSize);
                    transformedCovars(1:sourceStateSize,1:sourceStateSize,j) = transformedTrack.StateCovariance(1:sourceStateSize,1:sourceStateSize);
                end
                allStates = [sourceState,transformedStates];
                allCovars = cat(3,sourceCovar,transformedCovars);
                oneColumnCost = fusion.internal.gaussNormDiff(allStates,allCovars,true);
                costMatrix(:,i) = oneColumnCost(2:end);
                if sourceClass ~= 0
                    differentClasses = ((trackClasses ~= sourceClass) & (trackClasses ~= 0));
                    costMatrix(differentClasses,i) = thisInf;
                end
            end
            costMatrix(costMatrix > maxCost) = thisInf;
        end
        
        function [liveTracks, trackClasses] = getLiveTracks(obj,numCentralTracks)
            liveTracks = repmat({obj.pTracksList{1}},[1,numCentralTracks]);
            trackClasses = zeros(1,numCentralTracks,'uint32');
            for i = 1:numCentralTracks
                liveTracks{i} = obj.pTracksList{i};
                trackClasses(i) = obj.pTracksList{i}.ObjectClassID;
            end
        end
        
        function [centralStates,centralCovars] = collectTrackStates(obj,tracks,stateSize)
            numTracks = numel(tracks);
            centralStates = zeros(stateSize,numTracks,obj.pClassToUse);
            centralCovars = zeros(stateSize,stateSize,numTracks,obj.pClassToUse);
            for i = 1:numTracks
                centralStates(:,i) = tracks{i}.State;
                centralCovars(:,:,i) = tracks{i}.StateCovariance;
            end
        end
        
        function recycleTracks(obj, toDelete)
            %  Note: For performance purposes, the track is not really removed.
            %  It is moved to the end of the tracks list and its content is
            %  'nullified'. In addition, it does not appear in the list of
            %  live tracks and if it was a confirmed track its confirm
            %  status flag is cleared.
            
            numTracksToDelete = sum(toDelete);
            toKeep = ~toDelete;
            
            % Keep the live tracks in a contiguous list
            if coder.target('MATLAB')
                obj.pTracksList = {obj.pTracksList{toKeep},obj.pTracksList{toDelete}};
                obj.pNumLiveTracks = obj.pNumLiveTracks - numTracksToDelete;
                obj.pConfirmedTracks = [obj.pConfirmedTracks(toKeep), false(1,numTracksToDelete)];
                obj.pTrackIDs = [obj.pTrackIDs(toKeep), zeros(1,numTracksToDelete,'like',obj.pTrackIDs)];
                obj.pTrackLogics = {obj.pTrackLogics{toKeep},obj.pTrackLogics{toDelete}};
            else
                % Codegen cannot index into a cell array of objects. This
                % must be done one by one
                indsToDelete = find(toDelete);
                for i = numTracksToDelete:-1:1
                    currentInd = indsToDelete(i);
                    temp = obj.pTracksList{currentInd};
                    tempLogic = obj.pTrackLogics{currentInd};
                    for j = currentInd+1:obj.pNumLiveTracks
                        obj.pTracksList{j-1} = obj.pTracksList{j};
                        obj.pTrackLogics{j-1} = obj.pTrackLogics{j};
                    end
                    obj.pTracksList{obj.pNumLiveTracks} = temp;
                    obj.pTrackLogics{obj.pNumLiveTracks} = tempLogic;
                    obj.pNumLiveTracks = obj.pNumLiveTracks - 1;
                    obj.pConfirmedTracks(currentInd:end) = [obj.pConfirmedTracks(currentInd+1:end), false];
                    obj.pTrackIDs(currentInd:end) = [obj.pTrackIDs(currentInd+1:end), uint32(0)];
                end
            end
        end
        
        function outTracks = formatOutput(obj,list)
            if numel(list) > 0
                if coder.target('MATLAB')
                    outTracks = [obj.pTracksList{list}]';
                    outTracks = formatTrackWithLogic(obj,outTracks,list);
                else
                    oneStruct = modTrackLogicState(obj,toStruct(obj.pTracksList{list(1)}),list(1));
                    allStructs = repmat(oneStruct,numel(list),1);
                    outTracks = formatTrackWithLogic(obj,allStructs,list);
                end
            else
                if coder.target('MATLAB')
                    outTracks = repmat(obj.pTracksList{1},[0 1]);
                else
                    outTracks = repmat(toStruct(obj.pTracksList{1}),[0 1]);
                end
            end
        end
        
        function tracksOut = formatTrackWithLogic(obj,allTracks,indices)
            % Adds the track logic output to the track output
            
            % In MATLAB everything is ready just modify logic and logic state
            if coder.target('MATLAB')
                tracksOut = allTracks;
                for i = 1:numel(allTracks)
                    tracksOut(i).TrackLogic = 'History';
                    tracksOut(i).TrackLogicState  = output(obj.pTrackLogics{indices(i)});
                end
            else % In codegen, prepare the output from the start
                tracksOut = allTracks;
                for i = 2:numel(allTracks)
                    index = indices(i);
                    tracksOut(i).TrackID          = obj.pTracksList{index}.TrackID;
                    tracksOut(i).BranchID         = obj.pTracksList{index}.BranchID;
                    tracksOut(i).Age              = obj.pTracksList{index}.Age;
                    tracksOut(i).State            = obj.pTracksList{index}.State;
                    tracksOut(i).StateCovariance  = obj.pTracksList{index}.StateCovariance;
                    tracksOut(i).ObjectClassID    = obj.pTracksList{index}.ObjectClassID;
                    tracksOut(i).TrackLogicState  = output(obj.pTrackLogics{index});
                    tracksOut(i).IsConfirmed      = obj.pTracksList{index}.IsConfirmed;
                    tracksOut(i).IsCoasted        = obj.pTracksList{index}.IsCoasted;
                    tracksOut(i).IsSelfReported   = obj.pTracksList{index}.IsSelfReported;
                    tracksOut(i).ObjectAttributes = obj.pTracksList{index}.ObjectAttributes;
                end
            end
        end
        
        function s = modTrackLogicState(obj,st,index)
            fns = fieldnames(st);
            for i = 1:numel(fns)
                if strcmpi(fns{i},'TrackLogicState')
                    s.TrackLogicState = output(obj.pTrackLogics{index});
                elseif strcmpi(fns{i},'TrackLogic')
                    s.TrackLogic = 'History';
                else
                    s.(fns{i}) = st.(fns{i});
                end
            end
        end
        
        function index = findTrackByID(obj, trackID)
            % Returns the index of a track specified by its trackID. If the
            % track does not exist, it returns empty
            index = find(trackID == obj.pTrackIDs, 1, 'first');
        end
        
        function trackIDList = trackIDs(obj, indices)
            % Returns the track IDs of the list of tracks that is indexed
            % by indices. Indices is an array of indices
            trackIDList = zeros(numel(indices),1, 'like', obj.pTracksList{1}.TrackID);
            trackIDList(:) = obj.pTrackIDs(indices);
        end
        
        %------------------------------------------------------------------
        % Implementations of predict tracks to time
        %------------------------------------------------------------------
        function predictedTracks = predictTracksWithoutCov(obj,predictedTracks,pTime)
            currentTime = predictedTracks(1).UpdateTime;
            dt = pTime - currentTime;
            allStates = getAllStates(obj,predictedTracks,obj.pStateSize);
            allStates = obj.pStateTransitionFcn(allStates,dt);
            for i = 1:numel(predictedTracks)
                predictedTracks(i).State      = allStates(:,i);
                predictedTracks(i).UpdateTime = pTime;
            end
        end
        
        function predictedTracks = predictTracksWithCov(obj,predictedTracks,pTime)
            currentTime = predictedTracks(1).UpdateTime;
            dt = pTime - currentTime;
            [allStates, allCovs] = getAllStates(obj,predictedTracks,obj.pStateSize);
            [allStates, allCovs] = fusion.internal.gaussEKFilter.predict(...
                allStates,allCovs,obj.ProcessNoise,...
                obj.pStateTransitionFcn,obj.pStateTransitionJacobianFcn,...
                obj.HasAdditiveProcessNoise,dt);
            for i = 1:numel(predictedTracks)
                predictedTracks(i).State           = allStates(:,i);
                predictedTracks(i).StateCovariance = allCovs(:,:,i);
                predictedTracks(i).UpdateTime      = pTime;
            end
        end
        
        function [allStates,allCovs] = getAllStates(obj,predictedTracks,numStates)
            if coder.target('MATLAB')
                allStates = [predictedTracks.State];
                allCovs = cat(3,predictedTracks.StateCovariance);
            else
                numTracks = numel(predictedTracks);
                allStates = zeros(numStates,numTracks,obj.pClassToUse);
                allCovs = zeros(numStates,numStates,numTracks,obj.pClassToUse);
                for i = 1:numTracks
                    allStates(:,i) = predictedTracks(i).State(:);
                    allCovs(:,:,i) = predictedTracks(i).StateCovariance;
                end
            end
        end
        
        %------------------------------------------------------------------
        % Methods used by the system object
        %------------------------------------------------------------------
        function validatePropertiesImpl(obj)
            % Validate related or interdependent property values            
            validateStateTransitionProperties(obj);
            validateStateFusionProperties(obj);           
        end

        function validateStateTransitionProperties(obj)
            %StateTransitionFcn
            func1 = obj.StateTransitionFcn;
            if isa(func1,'function_handle')
                obj.pStateTransitionFcn = func1;
            else
                obj.pStateTransitionFcn = str2func(func1);
            end

            %StateTransitionJacobianFcn
            func2 = obj.StateTransitionJacobianFcn;
            if isa(func2,'function_handle')
                obj.pStateTransitionJacobianFcn = func2;
            elseif ~isemptystr(func2)
                obj.pStateTransitionJacobianFcn = str2func(func2);
            else
                obj.pStateTransitionJacobianFcn = '';
            end
        end

        function validateStateFusionProperties(obj)
            % State fusion algorithm and properties are closely related.
            if strcmpi(obj.StateFusion,'cross') && ~isempty(obj.StateFusionParameters)
                validateattributes(obj.StateFusionParameters,...
                    {'double','single'},{'real','nonsparse','nonnegative',...
                    'scalar','<',1},class(obj),'StateFusionParameters');
            elseif strcmpi(obj.StateFusion,'intersection') && ~isempty(obj.StateFusionParameters)
                validatestring(obj.StateFusionParameters,...
                    {'det','trace'},class(obj),'StateFusionParameters');
            elseif strcmpi(obj.StateFusion,'custom')
                validateCustomFusionFcn(obj)
            end
        end
        
        function validateCustomFusionFcn(obj)
            % A custom fusion fcn must satisfy the following interface:
            %   [xx,PP] = obj.FusionFcn(allStates,allCovars,[params])
            
            % Validate Custom
            validateattributes(obj.CustomStateFusionFcn, ...
                {'string','char','function_handle'}, {'nonempty'}, ...
                class(obj), 'CustomStateFusionFcn')
            
            % If it is a string or character vector, turn it into a
            % function_handle
            if isstring(obj.CustomStateFusionFcn) || ischar(obj.CustomStateFusionFcn)
                obj.pCustomStateFusionFcn = str2func(obj.CustomStateFusionFcn);
            else
                obj.pCustomStateFusionFcn = obj.CustomStateFusionFcn;
            end
            
            % Check number of inputs is between 2 and 3 or negative.
            % Negative is for anonymous functions or varargin.
            nins = nargin(obj.pCustomStateFusionFcn);
            cond = ~(nins > 0 && (nins < 2 || nins > 3));
            coder.internal.assert(cond, ...
                'fusion:trackFuser:ExpectedNumberOfInputsToCustomFcn', ...
                'CustomStateFusionFcn',2,3);
            
            % If number of inputs is 2, expect StateFusionParameters to be
            % empty. If number of inputs is 3, expect StateFusionParameters
            % to be nonempty
            cond = ((nins==2) && ~isempty(obj.StateFusionParameters) || ...
                (nins==3) && isempty(obj.StateFusionParameters));
            if cond
                if nins==2
                    s = 'empty';
                else
                    s = 'nonempty';
                end
                coder.internal.errorIf(cond, 'fusion:trackFuser:CustomFusionMismatch',...
                    'CustomStateFusionFcn',nins,'StateFusionParameters',s);
            end
            
            % Check number of outputs is exactly 2 or an anonymous function
            nouts = nargout(obj.pCustomStateFusionFcn);
            coder.internal.assert(nouts==2 || nouts < 0,...
                'fusion:trackFuser:ExpectedNumberOfOutputsFromCustomFcn',...
                'CustomStateFusionFcn',2);
        end
        
        function validateInputsImpl(obj,localTracks)
            % Validate inputs to the step method at initialization
            validateattributes(localTracks,{'struct','objectTrack'},...
                {},class(obj),'localTracks')
        end
        
        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s
            
            % Set private and protected properties
            if wasLocked
                for i=1:s.pMaxNumBranches
                    obj.pTracksList{i} = s.pTracksList{i};
                    obj.pTrackLogics{i} = clone(s.pTrackLogics{i});
                end
                obj.pSourceConfigurations = cell(1,s.MaxNumSources);
                for i=1:s.MaxNumSources
                    obj.pSourceConfigurations{i} = s.pSourceConfigurations{i};
                end
            else
                if isfield(s,'pTracksList')
                    obj.pTracksList = s.pTracksList;
                end
                if isfield(s,'pTrackLogics')
                    obj.pTrackLogics = s.pTrackLogics;
                end
                if isfield(s,'pSourceConfigurations')
                    obj.pSourceConfigurations = s.pSourceConfigurations;
                end
                % New field that did not exist before 21a
                if isfield(s,'pTrackLogicStateSize')                                 
                    obj.pTrackLogicStateSize = s.pTrackLogicStateSize;               
                    s = rmfield(s,'pTrackLogicStateSize');                           
                end
                
                if isfield(s,'pStateParameters')                                 
                    obj.pStateParameters = s.pStateParameters;               
                    s = rmfield(s,'pStateParameters');                           
                end
            end
            
            obj.pLastTimeStamp              = s.pLastTimeStamp;
            obj.pSourceConfigIDs            = s.pSourceConfigIDs;
            obj.pUsedConfigIDs              = s.pUsedConfigIDs;
            obj.pNumUsedConfigs             = s.pNumUsedConfigs;
            obj.pIsValidSource              = s.pIsValidSource;
            obj.pNumLiveTracks              = s.pNumLiveTracks;
            obj.pTrackIDs                   = s.pTrackIDs;
            obj.pConfirmedTracks            = s.pConfirmedTracks;
            obj.pLogic                      = s.pLogic;
            obj.pHasObjectAttributes        = s.pHasObjectAttributes;
            obj.pClassToUse                 = s.pClassToUse;
            obj.pStateSize                  = s.pStateSize;
            obj.pMaxNumBranches             = s.pMaxNumBranches;
            obj.pStateTransitionFcn         = s.pStateTransitionFcn;
            obj.pStateTransitionJacobianFcn = s.pStateTransitionJacobianFcn;
            obj.pCustomStateFusionFcn       = s.pCustomStateFusionFcn;
            
            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end
        
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj
            
            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);
            s.MaxNumLocalTracks = obj.MaxNumLocalTracks;
            
            % Set private and protected properties
            % Save the tracks list
            if isLocked(obj)
                for i=1:obj.pMaxNumBranches
                    s.pTracksList{i} = obj.pTracksList{i};
                    s.pTrackLogics{i} = obj.pTrackLogics{i};
                end
                for i=1:obj.MaxNumSources
                    s.pSourceConfigurations{i} = obj.pSourceConfigurations{i};
                end
            else
                s.pTracksList = obj.pTracksList;
                s.pTrackLogics = obj.pTrackLogics;
            end
            
            s.pLastTimeStamp              = obj.pLastTimeStamp;
            s.pSourceConfigIDs            = obj.pSourceConfigIDs;
            s.pUsedConfigIDs              = obj.pUsedConfigIDs;
            s.pSourceConfigurations       = obj.pSourceConfigurations;
            s.pNumUsedConfigs             = obj.pNumUsedConfigs;
            s.pIsValidSource              = obj.pIsValidSource;
            s.pNumLiveTracks              = obj.pNumLiveTracks;
            s.pTrackIDs                   = obj.pTrackIDs;
            s.pConfirmedTracks            = obj.pConfirmedTracks;
            s.pLogic                      = obj.pLogic;
            s.pHasObjectAttributes        = obj.pHasObjectAttributes;
            s.pClassToUse                 = obj.pClassToUse;
            s.pStateSize                  = obj.pStateSize;
            s.pMaxNumBranches             = obj.pMaxNumBranches;
            s.pStateTransitionFcn         = obj.pStateTransitionFcn;
            s.pStateTransitionJacobianFcn = obj.pStateTransitionJacobianFcn;
            s.pCustomStateFusionFcn       = obj.pCustomStateFusionFcn;
            s.pTrackLogicStateSize        = obj.pTrackLogicStateSize;
            s.pStateParameters            = obj.pStateParameters;
        end
    end
end

function tf = isemptystr(st)
% Returns true if st is an empty char array, an empty string, or "" (which
% is a 1x1 string)
tf = isempty(st) || isstring(st) && strcmp(st,"");
end

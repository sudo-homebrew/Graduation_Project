classdef (StrictDefaults, Hidden) trackFuser < trackFuser & ...
        matlabshared.tracking.internal.SimulinkBusPropagation
    % This is the Simulink implementation of trackFuser.
    %
    %   See also: trackFuser
    
    % Copyright 2020-2021 The MathWorks, Inc.
    
    %#codegen
    
    properties(Nontunable)
        %TentativeTracksOutputPort Enable tentative tracks output
        %   Set this property to true if you want to get tentative tracks
        %   as an additional output.
        %
        %   Default: false
        TentativeTracksOutputPort (1, 1) logical = false
        
        %AllTracksOutputPort Enable all tracks output
        %   Set this property to true if you want to get all the tracks
        %   as an additional output.
        %
        %   Default: false
        AllTracksOutputPort (1, 1) logical = false
        
        %InfoOutputPort Enable information output
        %   Set this property to true if you want to get info
        %   as an additional output.
        %
        %   Default: false
        InfoOutputPort (1, 1) logical = false
        
        %IsSourceConfigurationsInput Enable source configurations input
        %   port.
        %   Set this property to true to update source configurations as an
        %   input with each time step.
        %
        %   Default: false
        IsSourceConfigurationsInput (1, 1) logical = false
        
        %IsStateParametersInput Enable state parameters input port.
        %   Set this property to true to update state parameters as an
        %   input with each time step.
        %
        %   Default: false
        IsStateParametersInput (1, 1) logical = false;
    end
    
    properties(Access = protected, Nontunable)
        %pHasTimeInput A flag. True if time is supplied as an input
        pHasTimeInput
    end
    
    properties(Access = protected, Nontunable)
        %pMaxNumSourceTracks A scalar value that defines the maximum number
        %  of tracks in Simulink. This value gets set depending upon the
        %  size of the tracks input bus.
        pMaxNumSourceTracks
        
        %pStateParametersIndex A scalar value that holds the index of state
        %  paramerts when 'State Parameters' input port is enabled.
        pStateParametersIndex
        
        %pSourceConfigurationIndex A scalar value that holds the index of
        %  source configurations when 'Source Configurations' input port is
        %  enabled for it.
        pSourceConfigurationIndex
    end
    
    properties(Constant, Access=protected)
        %pBusPrefix A string that captures the base output bus name
        %   An output bus name are created by the object. It will have the
        %   name given here, appended by the number of tracks.
        %   Additional sub buses are created as well, e.g., for the tracks.
        pBusPrefix = {'BusTrackFuser', 'BusTrackFuserInfo'}
    end
    
    properties(Nontunable)
        %TimeInputSource Prediction time source
        %   Set this property to 'Input port' if you want to pass the
        %   prediction time as an input to step. Set this property
        %   to 'Auto' if you want the prediction time to be inherited from
        %   Simulink. This property only works in Simulink.
        %
        %   Default: 'Input port'
        TimeInputSource = 'Input port'
                
        %BusNameSource Source of output bus name
        %   Set this property to 'Property' if you want to pass the bus
        %   name as an input. Set this property to 'Auto' if you want to
        %   use the default bus name.
        %
        %   Default: 'Auto'
        BusNameSource = 'Auto';

        %BusName Specify an output bus name
        BusName = char.empty(1,0)

        %BusName2Source Source of output info bus name
        %   Set this property to 'Property' if you want to pass the bus
        %   name as an input. Set this property to 'Auto' if you want to
        %   use the default bus name in Simulink. This property only works
        %   in Simulink.
        %
        %   Default: 'Auto'
        BusName2Source = 'Auto'
        
        %BusName2 Specify an output info bus name
        BusName2 = char.empty(1,0)
    end
    
    properties(Nontunable)
        %StateFusionCross A scalar value cross covariance factor for
        %  the effective correlation coefficient when computing the cross
        %  covariance.
        %
        % Default : 0.4
        StateFusionCross = 0.4
        
        %StateFusionIntersection An estimation factor for covariance
        %  intersection fusion.Valid values are 'det' or 'trace'.
        %
        % Default : 'det'
        StateFusionIntersection = 'det'
        
        %StateFusionCustom An estimation factor for custom fusion of tracks.
        %
        % Default : ''
        StateFusionCustom =''
        
        %SourceConfigurationExpression A struct value that defines the
        %  source configurations in the block.
        %
        % Default: struct('SourceIndex',1)
        SourceConfigurationExpression = struct('SourceIndex',1);
        
        %StateParametersSimulink A struct values that defines the state
        %  parameters in the block.
        %
        % Default : struct
        StateParametersSimulink = struct;
        
        %CentralTrackStateSize A numeric value that locks the central track
        %  state size in simulink.
        %
        % Default : 6
        CentralTrackStateSize = 6;
        
        %CentralObjectAttributes A struct that locks the central track
        %  object attributes parameter in simulink.
        %
        % Default : struct
        CentralObjectAttributes = struct;
        
        %StateFusionParamSource Allows the additional parameters to take
        %  default values defined inside the track fusion algorithms.
        %
        % Default: 'Auto'
        StateFusionParamSource = 'Auto'
    end
    
    properties(Hidden,Constant)
        TimeInputSourceSet = matlab.system.StringSet({'Input port','Auto'});
        BusNameSourceSet = matlab.system.StringSet({'Auto','Property'});
        BusName2SourceSet = matlab.system.StringSet({'Auto','Property'});
        StateFusionParamSourceSet = matlab.system.StringSet({'Auto','Property'});
    end
    %Public
    methods
        %% Common functions
        function obj = trackFuser(varargin)
            setProperties(obj,nargin,varargin{:});
            obj.pLastTimeStamp = -eps;
        end
        
        % Bus name validation for buses
        function val = get.BusName(obj)
            val = obj.BusName;
            val = getBusName(obj,val);
        end

        function set.BusName(obj,val)
            validateBusName(obj,val,'BusName')
            obj.BusName = val;
        end

        function val = get.BusName2(obj)
            val = obj.BusName2;
            val = getBusName(obj,val,2);
        end
        
        function set.BusName2(obj,val)
            validateBusName(obj,val,'BusName2')
            obj.BusName2 = val;
        end
        
        function set.SourceConfigurationExpression(obj,value)
            validateattributes(value,{'struct'},{},class(obj),'Source Configurations')
            validFldNames = {'SourceIndex','IsInternalSource','IsInitializingCentralTracks', ...
                'LocalToCentralTransformFcn','CentralToLocalTransformFcn'};
            InpFldNames = fieldnames(value);
            for id = 1:numel(InpFldNames)
                coder.internal.assert(any(strcmp(validFldNames,InpFldNames{id})),...
                    'fusion:simulink:trackFuser:InvalidConfig',InpFldNames{id});
            end
            obj.SourceConfigurationExpression = value;
        end
        
        function set.StateParametersSimulink(obj,value)
            validateattributes(value,{'struct'},{},class(obj),'StateParametersSimulink')
            obj.StateParametersSimulink = value;
        end
    end
    %% Protected
    methods(Access=protected)
        
        function setupImpl(obj,localTracks,varargin)
            % Setup properties that will not be modified later on
            % Perform one-time calculations, such as computing constants
            
            % Extract tracks from a bus
            trs = localTracks.Tracks;
            
            % Set the value for maximum number of tracks that can be
            % passed through the input tracks bus
            obj.pMaxNumSourceTracks = cast(numel(localTracks.Tracks),'uint32');
            
            % Set the state parameters in the block.
            setStateParameters(obj, obj.StateParametersSimulink);
            
            % Set the source configurations in the block.
            config = getConfigurationsFromStruct(obj.SourceConfigurationExpression);
            setSources(obj,config);
           
            if strcmpi(obj.StateFusionParamSource, 'Property')
                if strcmpi(obj.StateFusion,'Cross')
                    obj.StateFusionParameters = obj.StateFusionCross;
                elseif strcmpi(obj.StateFusion,'Intersection')
                    obj.StateFusionParameters = obj.StateFusionIntersection;
                else
                    obj.StateFusionParameters = obj.StateFusionCustom;
                end
            end
            %validate State fusion properties.
            validateStateFusionProperties(obj);
            
            % setupImpl Sets the track manager up. Creates all the tracks
            obj.pTrackLogicStateSize = max(obj.ConfirmationThreshold(2), obj.DeletionThreshold(2));
            obj.pClassToUse = class(trs(1).State);
            obj.pStateSize  = obj.CentralTrackStateSize;
            % Create a default track to initialize the defult tracks list.
            s = obj.CentralObjectAttributes;
            centralTrack = objectTrack(...
                'TrackID', intmax('uint32'), ...
                'BranchID', intmax('uint32'), ...
                'SourceIndex', obj.FuserIndex, ...
                'UpdateTime', zeros(1,1,obj.pClassToUse), ...
                'Age', intmax('uint32'), ...
                'State', zeros(obj.pStateSize,1,obj.pClassToUse), ...
                'StateCovariance', eye(obj.pStateSize,obj.pClassToUse), ...
                'StateParameters', obj.StateParameters, ...
                'ObjectClassID', zeros(1,1,'like',trs(1).ObjectClassID), ...
                'TrackLogic', coder.const("History"), ...
                'TrackLogicState', false(1, max(obj.ConfirmationThreshold(2), obj.DeletionThreshold(2))), ...
                'IsConfirmed', false, ...
                'IsCoasted', false, ...
                'IsSelfReported', false, ...
                'ObjectAttributes', s);
            
            setupImpl@trackFuser(obj,trs,[],varargin{:});
            
            obj.pTracksList = repmat({centralTrack},1,obj.pMaxNumBranches);
            
            % Define obj.pUsedConfigIDs as fixed size
            obj.pUsedConfigIDs = false(1,obj.MaxNumSources);
            obj.pUsedConfigIDs = false(2,obj.MaxNumSources);
            obj.pUsedConfigIDs = false(1,obj.MaxNumSources);
        end
        
        function resetImpl(obj)
            % Returns the tracker to its initial state
            resetImpl@trackFuser(obj);
            obj.pLastTimeStamp = cast(-eps,'like',obj.pLastTimeStamp); %Has to be negative to run tracker from t=0
        end
        
        function releaseImpl(obj)
            % Release resources
            releaseImpl@trackFuser(obj);
        end
        
        %% Save / Load / Clone Impl
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj
            
            % Save the base class information
            s = saveObjectImpl@trackFuser(obj);
            
            if isLocked(obj)
                s.pMaxNumSourceTracks           = obj.pMaxNumSourceTracks;
                s.pHasTimeInput                 = obj.pHasTimeInput;
                s.pStateParametersIndex         = obj.pStateParametersIndex;
                s.pSourceConfigurationIndex     = obj.pSourceConfigurationIndex;
                s.pBusPrefix                    = obj.pBusPrefix;
            end
        end
        
        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s
            if wasLocked
                if isfield(s,'pMaxNumSourceTracks')
                    obj.pMaxNumSourceTracks = s.pMaxNumSourceTracks;
                    s = rmfield(s,'pMaxNumSourceTracks');
                end
                if isfield(s,'pHasTimeInput')
                    obj.pHasTimeInput = s.pHasTimeInput;
                    s = rmfield(s,'pHasTimeInput');
                end
                if isfield(s,'pStateParametersIndex')
                    obj.pStateParametersIndex = s.pStateParametersIndex;
                    s = rmfield(s,'pStateParametersIndex');
                end
                if isfield(s,'pSourceConfigurationIndex')
                    obj.pSourceConfigurationIndex   =   s.pSourceConfigurationIndex;
                    s = rmfield(s,'pSourceConfigurationIndex');
                end
                if isfield(s,'pBusPrefix')
                    obj.pBusPrefix = s.pBusPrefix;
                    s = rmfield(s,'pBusPrefix');
                end
            end
            loadObjectImpl@trackFuser(obj,s,wasLocked);
        end
        
        function newTracker = cloneImpl(obj)
            %clone Creates a copy of the simulink trackFuser
            %   newTracker = clone(tracker) returns a copy of the tracker
            %   object.
            
            % Copy superclass properties
            newTracker = cloneImpl@trackFuser(obj);
            
            if ~coder.internal.is_defined(obj.pTracksList) %Only happens after setup
                newTracker.pMaxNumSourceTracks = obj.pMaxNumSourceTracks;
                newTracker.pHasTimeInput = obj.pHasTimeInput;
                newTracker.pStateParametersIndex = obj.pStateParametersIndex;
                newTracker.pSourceConfigurationIndex   =   obj.pSourceConfigurationIndex;
            end
            
            newTracker.TentativeTracksOutputPort  = obj.TentativeTracksOutputPort;
            newTracker.AllTracksOutputPort        = obj.AllTracksOutputPort;
            newTracker.InfoOutputPort             = obj.InfoOutputPort;
            newTracker.BusName2Source             = obj.BusName2Source;
            newTracker.BusName2                   = obj.BusName2;
        end
        
        function varargout = stepImpl(obj, localTracks, varargin)
            localTrs = getLocalTracks(localTracks);
            tFusion = processInputs(obj,varargin{:});
            info = coreAlgorithm(obj,localTrs,tFusion);
            
            % Process Output
            [varargout{1:nargout}] = getTrackerOutputs(obj,info);
            
            % Update the last update time stamp
            obj.pLastTimeStamp = tFusion;
        end
        
        function out = ensureTrack(obj, in)
            % This method ensures the output is an objectTrack
            if isa(in,'objectTrack')
                stIn = toStruct(in);
            else
                stIn = in;
            end
            f = fieldnames(stIn);
            st = struct;
            for i = 1:numel(f)
                if ~(strcmpi(f{i},'State') || strcmpi(f{i},'StateCovariance'))
                    st.(f{i}) = stIn.(f{i});
                end
            end
            st.State = stIn.State(1:obj.pStateSize);
            st.StateCovariance = stIn.StateCovariance(1:obj.pStateSize,1:obj.pStateSize);
            out = objectTrack(st);
        end
        
        function tFusion = processInputs(obj,varargin)
            % Work with and without time input
            if obj.pHasTimeInput
                tFusion = varargin{1};
                % Check that the time is not in the past
                coder.internal.assert(tFusion > obj.pLastTimeStamp(1), ...
                    'fusion:trackFuser:TimeMustIncrease',mfilename);
            else
                tFusion = getCurrentTime(obj);
            end
            % Extract source configurations and state parameters when source
            % configurations and state parameters are feeded to the block
            % as an input.
            if obj.IsSourceConfigurationsInput
                sourceConfig = varargin{obj.pSourceConfigurationIndex};
                inputConfigurations = sourceConfig.Configurations(1:sourceConfig.NumConfigurations);
                for numSources = 1:numel(obj.pSourceConfigurations)
                    sync(obj.pSourceConfigurations{numSources},inputConfigurations(numSources));
                end
                setSources(obj,obj.pSourceConfigurations);
            end
            
            if obj.IsStateParametersInput
                stParam = varargin{obj.pStateParametersIndex};
                setStateParameters(obj,stParam);
            end
        end
        
        function varargout =  getTrackerOutputs(obj, info)
            % This method extracts the output tracks.
            % Output
            track = getTrackStruct(obj, true);
            allTracks = getTrackStruct(obj, true(1,obj.pNumLiveTracks));
            confTracks = allTracks(obj.pConfirmedTracks);
            tentTracks = allTracks(~obj.pConfirmedTracks(1:obj.pNumLiveTracks));
            
            vargs = cell(1, nargout);
            if obj.InfoOutputPort
                numOut = nargout-2;
                vargs{nargout} = getInfo(obj, info);
            else
                numOut = nargout-1;
            end
            % First output will always be confirmed tracks
            vargs{1} = sendToBus(obj, confTracks, 1, track);
            if numOut>0
                switch(numOut)
                    case 1
                        if obj.TentativeTracksOutputPort
                            vargs{2} = sendToBus(obj, tentTracks, 1, track);
                        else
                            vargs{2} = sendToBus(obj, allTracks, 1, track);
                        end
                    case 2
                        vargs{2} = sendToBus(obj, tentTracks, 1, track);
                        vargs{3} = sendToBus(obj, allTracks, 1, track);
                    otherwise
                        % This is to satisfy coder and is not a branch that
                        % should ever be hit.
                        assert(false)
                end
            end
            varargout = vargs;
        end
        
        function trackStruct = getTrackStruct(obj, list)
            numTracks = sum(list);
            indsList = find(list);
            thisTrack = obj.pTracksList{1};
            trackStruct = repmat(track2struct(obj, thisTrack), numTracks,1);
            for i = 1:numTracks
                thisTrack = obj.pTracksList{indsList(i)};
                trackStruct(i) = track2struct(obj, thisTrack, indsList(i));
            end
        end
        
        function trackStruct = track2struct(obj,thisTrack,index)
            if nargin < 3
                index = 1;
            end
            tls = output(obj.pTrackLogics{index});
            thisTrack.TrackLogicState = tls;
            st = protectedToStruct(thisTrack,obj.pTrackLogicStateSize);
            trackStruct = matlabshared.tracking.internal.fusion.removeOptionalField(st,{'StateParameters','ObjectAttributes'});
        end
        
        function trackOut = parseTrack(obj,trackIn)
            % trackIn can be either an objectTrack or a struct.
            % trackOut is always an objectTrack.
            if isstruct(trackIn)
                s = struct;
                fields = fieldnames(trackIn);
                for i = 1:numel(fields)
                    if ~strncmpi(fields{i}, 'TrackLogic', 10) % Both TrackLogic and TrackLogicState
                        s.(fields{i}) = trackIn.(fields{i});
                    elseif strcmpi(fields{i},'TrackLogic')
                        s.TrackLogic = coder.const("History"); % Track fuser supports only history
                    else
                        s.TrackLogicState = coder.const(false(1,obj.pTrackLogicStateSize));
                    end
                end
                trackOut = objectTrack(s);
            else
                trackOut = trackIn;
            end
        end
        
        function validatePropertiesImpl(obj)
            if strcmpi(obj.TimeInputSource, 'Input port')
                obj.pHasTimeInput = true;
            else
                obj.pHasTimeInput = false;
            end
            
            obj.pSourceConfigurationIndex = obj.IsSourceConfigurationsInput + obj.pHasTimeInput;
            obj.pStateParametersIndex  = obj.pSourceConfigurationIndex + obj.IsStateParametersInput;
            
            validateStateTransitionProperties(obj);
        end
        
        function validateInputsImpl(obj,~,varargin)
            % Validate inputs to the step method at initialization
            if strcmpi(obj.TimeInputSource, 'Input port')
                % Validate time input
                if numel(varargin) > 0
                    validateattributes(varargin{1},{'single','double'}, ...
                        {'real','finite','nonsparse','scalar','nonnegative'}, ...
                        mfilename, 'time')
                end
            end
            
            if obj.IsStateParametersInput
                if numel(varargin) > 1
                    validateattributes(varargin{obj.pStateParametersIndex},{'struct'},{},class(obj),'StateParameters');
                end
            end
            
            if obj.IsSourceConfigurationsInput
                if numel(varargin) > 1
                    validateattributes(varargin{obj.pSourceConfigurationIndex},{'struct'},{},class(obj),'SourceConfigurations');
                end
            end
        end
        
        function info = getInfo(obj, information)
            sampleInfo = defaultInfo(obj);
            infoTrs = padInfoData(obj, information);
            info = sendToBus(obj, sampleInfo, 2, infoTrs);
        end
        
        function flag = isInactivePropertyImpl(obj, prop)
            % Return false if property is visible based on object
            % configuration, for the command line and System block dialog
            flag = isInactiveBusProperty(obj,prop);
            flag = flag || strcmp(obj.BusName2Source,'Auto') && strcmp(prop,'BusName2');
            
            invisibleInSimulink = {'NumCentralTracks', 'NumConfirmedCentralTracks'};
            invisFlags = strcmp(prop, invisibleInSimulink);
            if any(invisFlags)
                flag = true;
            end
            
            if ~obj.InfoOutputPort && startsWith(prop,'BusName2')
                flag = true;
            end
            flag = flag | (strcmp(prop, 'CustomAssignmentFcn') && ~strcmp(obj.Assignment,'Custom'));
            flag = flag | (strcmp(prop, 'CustomStateFusionFcn') && ~strcmp(obj.StateFusion,'Custom'));
            flag = flag | (strcmp(obj.StateFusionParamSource, 'Auto') && strcmp(prop, 'StateFusionCross'))...
                | (strcmp(obj.StateFusionParamSource, 'Auto') && strcmp(prop,'StateFusionIntersection'))...
                | (strcmp(obj.StateFusionParamSource, 'Auto') && strcmp(prop,'StateFusionCustom'));
            
            flag = flag | (strcmp(obj.StateFusion, 'Cross') && strcmp(prop, 'StateFusionIntersection'))...
                |(strcmp(obj.StateFusion, 'Cross') && strcmp(prop, 'StateFusionCustom'));
            
            flag = flag | (strcmp(obj.StateFusion, 'Intersection') && strcmp(prop, 'StateFusionCross'))...
                |(strcmp(obj.StateFusion, 'Intersection') && strcmp(prop, 'StateFusionCustom'));
            
            flag = flag | (strcmp(obj.StateFusion, 'Custom') && strcmp(prop, 'StateFusionCross'))...
                |(strcmp(obj.StateFusion, 'Custom') && strcmp(prop, 'StateFusionIntersection'));
            
        end
        
        
        function num = getNumInputsImpl(obj)
            % Define total number of inputs for system with optional inputs
            num = 1;
            
            if strcmpi(obj.TimeInputSource, 'Input port')
                num = num + 1;
            end
            if obj.IsStateParametersInput
                num = num + 1;
            end
            if obj.IsSourceConfigurationsInput
                num = num + 1;
            end
        end
        
        function num = getNumOutputsImpl(obj)
            % Define total number of outputs for system
            num = 1 + obj.TentativeTracksOutputPort + obj.AllTracksOutputPort ...
                + obj.InfoOutputPort;
        end
        
        function icon = getIconImpl(~)
            % Define icon for System block
            icon = getString(message('fusion:simulink:trackFuser:trackFuserIcon'));
        end
        
        function [name,varargout] = getInputNamesImpl(obj)
            % Return input port names for System block
            name = getString(message('fusion:simulink:trackFuser:Tracks'));
            varargout = {};
            
            if strcmpi(obj.TimeInputSource, 'Input port')
                varargout = [varargout(:)' getString(message('fusion:simulink:trackFuser:Prediction'))];
            end
            
            if obj.IsStateParametersInput
                varargout = [varargout(:)' getString(message('fusion:simulink:trackFuser:InputStateParam'))];
            end
            
            if obj.IsSourceConfigurationsInput
                varargout = [varargout(:)' getString(message('fusion:simulink:trackFuser:InputConfigurations'))];
            end
        end
        
        function [name1, varargout] = getOutputNamesImpl(obj)
            % Return output port names for System block
            name1 = getString(message('fusion:simulink:trackFuser:ConfirmedTrs'));
            varargout = {};
            if obj.TentativeTracksOutputPort
                varargout = [varargout(:)' getString(message('fusion:simulink:trackFuser:TentativeTrs'))];
            end
            if obj.AllTracksOutputPort
                varargout = [varargout(:)' getString(message('fusion:simulink:trackFuser:AllTrs'))];
            end
            if obj.InfoOutputPort
                varargout = [varargout(:)' getString(message('fusion:simulink:trackFuser:Info'))];
            end
        end
        
        function [sz1,sz2,sz3,sz4] = getOutputSizeImpl(~)
            sz1 = [1 1];
            sz2 = [1 1];
            sz3 = [1 1];
            sz4 = [1 1];
        end
        
        function  varargout = getOutputDataTypeImpl(obj)
            if obj.InfoOutputPort
                [dtTracks, dtInfo] = getBusDataTypes(obj);
            else
                dtTracks = getBusDataTypes(obj);
            end
            varargout = {};
            varargout{1} = dtTracks;
            if obj.TentativeTracksOutputPort
                varargout = [varargout(:)' {dtTracks}];
            end
            if obj.AllTracksOutputPort
                varargout = [varargout(:)' {dtTracks}];
            end
            if obj.InfoOutputPort
                varargout = [varargout(:)' {dtInfo}];
            end
        end
        
        function [cp1,cp2,cp3,cp4] = isOutputComplexImpl(~)
            cp1 = false;
            cp2 = false;
            cp3 = false;
            cp4 = false;
        end
        
        function [out1,out2,out3,out4] = isOutputFixedSizeImpl(~)
            out1 = true;
            out2 = true;
            out3 = true;
            out4 = true;
        end
        
    end
    
    methods(Access=protected)
        %---------------------------------
        % Simulink bus propagation methods
        %---------------------------------
        
        function [out, argsToBus] = defaultOutput(obj,busIdx)
            
            if nargin==1
                busIdx = 1;
            end
            out = struct.empty();
            argsToBus = {};
            trsBusType = propagatedInputBus(obj, 1);
            
            switch busIdx
                case 1
                    if isempty(trsBusType)
                        return;
                    end
                    trk = defaultTrack(obj, trsBusType);
                    out = repmat(trk, [obj.MaxNumCentralTracks, 1]);
                    sampleTrack = trk;
                    argsToBus = {sampleTrack};
                case 2
                    if isempty(trsBusType)
                        return;
                    end
                    [maxTracks, class] = getMaxNumSourceTracks(obj, trsBusType);
                    sampleInfo = defaultInfo(obj,maxTracks,class);
                    out = sampleInfo;
                    argsToBus = {sampleInfo};
            end
        end
        
        function [maxNumTracks, classToUse] = getMaxNumSourceTracks(obj, trsBusType)
            trsBus = evalinGlobalScope(bdroot, trsBusType);
            for i = 1:numel(trsBus.Elements)
                isBus = isBusDataType(trsBus.Elements(i).DataType);
                if isBus
                    maxNumTracks = max(trsBus.Elements(i).Dimensions);
                    trs = getTracksFromBus(obj, trsBusType);
                    classToUse = class(trs.State);
                end
            end
        end
        
        function sampleTrack = getTracksFromBus(~, busName)
            allTrs = matlabshared.tracking.internal.SimulinkBusUtilities.bus2struct(busName);
            sampleTrack = allTrs.Tracks(1);
        end
        
        function trackStruct = defaultTrack(obj, tracksDataType)
            % Setting up the default bus for central tracks
            allTrs = matlabshared.tracking.internal.SimulinkBusUtilities.bus2struct(tracksDataType);
            
            % Remove StateSize and LogicSize from the input bus if it is
            % available.
            unitTrack = matlabshared.tracking.internal.fusion.removeStructFields(...
                allTrs.Tracks(1),{'StateSize','LogicSize'});

            % Get one track from the input
            inpTrack = parseTrack(obj,unitTrack);
            
            % Setting up the default bus for central tracks
            obj.pTrackLogicStateSize = max(obj.ConfirmationThreshold(2), obj.DeletionThreshold(2));
            
            % generate the track structure
            trackSt = protectedToStruct(inpTrack,0);
            trackSt.State = zeros(obj.CentralTrackStateSize,1,'like',allTrs.Tracks(1).State);
            trackSt.StateCovariance = eye(obj.CentralTrackStateSize,'like',allTrs.Tracks(1).State);
            
            % set track property values from the user input
            trackSt.ObjectAttributes = obj.CentralObjectAttributes;
            trackSt.TrackLogicState = false(1,obj.pTrackLogicStateSize);
            trackSt.StateParameters = obj.StateParametersSimulink;
            
            % remove optional fields
            trk = matlabshared.tracking.internal.fusion.removeOptionalField(trackSt,{'StateParameters','ObjectAttributes'});
            trackStruct = trk;
        end
        
        function tracksOutput = sendToBus(obj, st, busIdx, varargin)
            % sampleData can have the information of sample tracks or the
            % tracker info depending upon the busIdx.
            sampleData = varargin{1};
            switch busIdx
                case 1
                    fullTracksList = repmat(sampleData, [obj.MaxNumCentralTracks, 1]);
                    listLength = numel(st);
                    fullTracksList(1:listLength) = st;
                    tracksOutput = struct('NumTracks', listLength, 'Tracks', fullTracksList);
                case 2
                    tracksOutput = struct('TrackerInfo', sampleData);
            end
        end
    end
        
    methods(Hidden)
        function infoStruct = defaultInfo(obj, maxTrs, classToUse)
            num = obj.MaxNumCentralTracks;
            if nargin == 1
                maxTrs = obj.pMaxNumSourceTracks;
                classToUse = obj.pClassToUse;
            end
            
            infoStruct = struct(...
                'TrackIDsAtStepBeginning', zeros(1,num,'uint32'), ...
                'CostMatrix', zeros(num,maxTrs,classToUse), ...
                'Assignments', zeros(num,2,'uint32'), ...
                'UnassignedCentralTracks', zeros(num,1,'uint32'), ...
                'UnassignedLocalTracks', zeros(num,1,'uint32'),...
                'NonInitializingLocalTracks', zeros(1,num,'uint32'),...
                'InitializedCentralTrackIDs', zeros(1,num,'uint32'), ...
                'UpdatedCentralTrackIDs', zeros(1,num, 'uint32'), ...
                'DeletedTrackIDs', zeros(1,num,'uint32'), ...
                'TrackIDsAtStepEnd', zeros(1,num,'uint32'));
        end
        
        function info = padInfoData(obj,infoTrack)
            % Variable size data can not be logged in simulink hence
            % padding the data to make it fixed size.
            num = obj.MaxNumCentralTracks;
            trIDs = zeros(1,num, 'like', infoTrack.TrackIDsAtStepBeginning);
            if ~isempty(infoTrack.TrackIDsAtStepBeginning)
                numTrsInitial = numel(infoTrack.TrackIDsAtStepBeginning);
                trIDs(1:numTrsInitial) = infoTrack.TrackIDsAtStepBeginning;
            end
            
            costMat = zeros(num,obj.pMaxNumSourceTracks,obj.pClassToUse);
            if ~isempty(infoTrack.CostMatrix)
                numTracks = size(infoTrack.CostMatrix,1);
                numDets   = size(infoTrack.CostMatrix,2);
                costMat(1:numTracks,1:numDets)= infoTrack.CostMatrix(1:numTracks,1:numDets);
            end
            
            assgn = zeros(num,2,'like',infoTrack.Assignments);
            if ~isempty(infoTrack.Assignments)
                numAssignments = size(infoTrack.Assignments,1);
                assgn(1:numAssignments,:) = infoTrack.Assignments;
            end
            
            unassgnCentralTrs = zeros(num,1,'like',infoTrack.UnassignedCentralTracks);
            if ~isempty(infoTrack.UnassignedCentralTracks)
                numUnassignTrs = numel(infoTrack.UnassignedCentralTracks);
                unassgnCentralTrs(1:numUnassignTrs) = infoTrack.UnassignedCentralTracks;
            end
            
            unassignSourceTracks = zeros(num,1,'like',infoTrack.UnassignedLocalTracks);
            if ~isempty(infoTrack.UnassignedLocalTracks)
                numUnassignSourceTrs = numel(infoTrack.UnassignedLocalTracks);
                unassignSourceTracks(1:numUnassignSourceTrs) = infoTrack.UnassignedLocalTracks;
            end
            
            NonInitLocalTracks = zeros(1,num,'like',infoTrack.NonInitializingLocalTracks);
            if ~isempty(infoTrack.NonInitializingLocalTracks)
                numNonInitSourceTrs = numel(infoTrack.NonInitializingLocalTracks);
                NonInitLocalTracks(1:numNonInitSourceTrs) = infoTrack.NonInitializingLocalTracks;
            end
            
            initCentralTrIDs = zeros(1,num,'like',infoTrack.InitializedCentralTrackIDs);
            if ~isempty(infoTrack.InitializedCentralTrackIDs)
                numCentralTrsIDs = numel(infoTrack.InitializedCentralTrackIDs);
                initCentralTrIDs(1:numCentralTrsIDs) = infoTrack.InitializedCentralTrackIDs;
            end
            
            updateCentralTrIDs = zeros(1,num,'like',infoTrack.UpdatedCentralTrackIDs);
            if ~isempty(infoTrack.UpdatedCentralTrackIDs)
                numUpdateCentralTrsIDs = numel(infoTrack.UpdatedCentralTrackIDs);
                updateCentralTrIDs(1:numUpdateCentralTrsIDs) = infoTrack.UpdatedCentralTrackIDs;
            end
            
            delTrIDs = zeros(1,num,'like',infoTrack.DeletedTrackIDs);
            if ~isempty(infoTrack.DeletedTrackIDs)
                numDelTrsIDs = numel(infoTrack.DeletedTrackIDs);
                delTrIDs(1:numDelTrsIDs) = infoTrack.DeletedTrackIDs;
            end
            
            trsIDEnd = zeros(1,num,'like',infoTrack.TrackIDsAtStepEnd);
            if ~isempty(infoTrack.TrackIDsAtStepEnd)
                numTrsStepEnd = numel(infoTrack.TrackIDsAtStepEnd);
                trsIDEnd(1:numTrsStepEnd) = infoTrack.TrackIDsAtStepEnd;
            end
            
            info = struct(...
                'TrackIDsAtStepBeginning', trIDs, ...
                'CostMatrix', costMat, ...
                'Assignments', assgn, ...
                'UnassignedCentralTracks', unassgnCentralTrs, ...
                'UnassignedLocalTracks', unassignSourceTracks,...
                'NonInitializingLocalTracks', NonInitLocalTracks,...
                'InitializedCentralTrackIDs', initCentralTrIDs, ...
                'UpdatedCentralTrackIDs', updateCentralTrIDs, ...
                'DeletedTrackIDs', delTrIDs, ...
                'TrackIDsAtStepEnd', trsIDEnd);
        end
    end
    
    %  --------------------------------------------------------------------
    methods(Static, Access = protected)
        %% Simulink customization functions
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(...
                'Title', 'fusion:block:t2tfTrackerTitle', ...
                'Text',	 'fusion:block:t2tfTrackerDesc');
        end
        
        function groups = getPropertyGroupsImpl
            % Define property section(s) for System block dialog, only in
            % Simulink
            
            absTrkrSection = matlabshared.tracking.internal.SimulinkBusPropagation.getBusPropertyGroups;
            
            propFuserList = {'FuserIndex', 'Assignment', 'CustomAssignmentFcn', ...
                'AssignmentThreshold', 'MaxNumCentralTracks','MaxNumSources',...
                'SourceConfigurationExpression','IsSourceConfigurationsInput' ...
                'StateParametersSimulink','IsStateParametersInput'};
            fuserSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackFuser','',propFuserList);
            fuserGroup = matlab.system.display.SectionGroup( ...
                'Title', getString(message('fusion:simulink:trackFuser:GroupFuserSection')), ...
                'Sections', fuserSection);
            
            propPredictionList = {'CentralTrackStateSize', 'CentralObjectAttributes', ...
                'StateTransitionFcn', 'StateTransitionJacobianFcn', ...
                'ProcessNoise', 'HasAdditiveProcessNoise'};
            predictionSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackFuser','CTParam',propPredictionList);
            
            propLogicList = {'ConfirmationThreshold', 'DeletionThreshold'};
            logicSection  = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackFuser','Logic',propLogicList);
            logicGroup    = matlab.system.display.SectionGroup( ...
                'Title', getString(message('fusion:simulink:trackFuser:GroupLogicSection')), ...
                'Sections', [logicSection, predictionSection]);
            
            propFusionLogicList = {'StateFusion', 'CustomStateFusionFcn', ...
                'StateFusionParamSource', 'StateFusionCross', 'StateFusionIntersection' ...
                'StateFusionCustom', 'FuseConfirmedOnly', 'FuseCoasted'};
            fusionSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackFuser','',propFusionLogicList);
            fusionGroup   = matlab.system.display.SectionGroup( ...
                'Title', getString(message('fusion:simulink:trackFuser:GroupFusionSection')), ...
                'Sections', fusionSection);
            
            busList      = absTrkrSection.PropertyList;
            portsList    = {busList{:},'BusName2Source','BusName2'};
            portsSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackFuser','OutputPortSettings',portsList);
            propIOList = {'TimeInputSource','TentativeTracksOutputPort', ...
                'AllTracksOutputPort','InfoOutputPort'};
            ioSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackFuser','InputsOutputs',propIOList);
            ioGroup    = matlab.system.display.SectionGroup( ...
                'Title', getString(message('fusion:simulink:trackFuser:GroupIOSection')), ...
                'Sections', [ioSection,portsSection]);
            
            groups = [fuserGroup, logicGroup, fusionGroup, ioGroup];
        end
    end
    
    methods(Static, Hidden)
        function flag = isAllowedInSystemBlock
            flag = true;
        end
    end
end
%--------------------------------------------------------------------------
%      HELPER FUNCTIONS
%--------------------------------------------------------------------------
function flag = isBusDataType(dt)
flag = false;
dt = strtrim(strrep(dt,'Bus:',''));
if localevalin(['exist(''' dt ''', ''var'');'])
    tmp = localevalin(dt);
    flag = isa(tmp,'Simulink.Bus');
end
end

function out = localevalin(evalstr)
if nargout>0
    out = evalinGlobalScope(bdroot,evalstr);
else
    evalinGlobalScope(bdroot,evalstr);
end
end

function Trs = getLocalTracks(localTracks)
% This methods extracts the actual sized element for local tracks structures.
if ~isequal(localTracks.NumTracks,0)
    tr  = localTracks.Tracks(1:localTracks.NumTracks);
    trk = getSingleTrackStruct(tr(1));
    Trs = repmat(trk,size(tr));
    for m = 1:numel(tr)
        Trs(m) = getSingleTrackStruct(tr(m));
    end
else
    tr = localTracks.Tracks;
    trk = getSingleTrackStruct(tr(1));
    Trs = repmat(trk,0,1);
end
end

function Trs = getSingleTrackStruct(tr)
% This method creates a new track structure by making sure that the new
% track structure has all the elements with their expected sizes all the
% padding done during the track concatenation is removed with the
% additional fields StateSize and LogicSize that were added during concatenation.
Trs = struct;
fields = fieldnames(tr);
for i = 1:numel(fields)
    if ~(strcmpi(fields{i},'StateSize')||strcmpi(fields{i},'LogicSize'))
        if strcmpi(fields{i}, 'State')
            st = tr.(fields{i});
            if strcmpi(fields{end-1}, 'StateSize')
                stsz = tr.(fields{end-1});
                Trs.(fields{i}) = st(1:stsz);
            else
                Trs.(fields{i}) = st;
            end
        elseif strcmpi(fields{i}, 'StateCovariance')
            stCov = tr.(fields{i});
            if strcmpi(fields{end-1}, 'StateSize')
                stsz = tr.(fields{end-1});
                Trs.(fields{i}) = stCov(1:stsz,1:stsz);
            else
                Trs.(fields{i}) = stCov;
            end
        elseif strcmpi(fields{i}, 'TrackLogicState')
            logicSt = tr.(fields{i});
            if strcmpi(fields{end}, 'LogicSize')
                lgsz = tr.(fields{end});
                Trs.(fields{i}) = logicSt(1:lgsz);
            else
                Trs.(fields{i}) = logicSt;
            end
        elseif strcmpi(fields{i}, 'TrackLogic')
            Trs.(fields{i}) = 'History';
        else
            Trs.(fields{i}) = tr.(fields{i});
        end
    end
end
if ~isfield(tr,'StateParameters')
    Trs.StateParameters = struct;
end
end

function config = getConfigurationsFromStruct(configStruct)
num = numel(configStruct);
config = cell(1,num);
for i = 1:num
    fnNames = fieldnames(configStruct(i));
    fnValues = struct2cell(configStruct(i));
    config{i} = generateConfigurations(fnNames,fnValues);
end
end
        
function config = generateConfigurations(fnNames,fnValues)
fn = cell(numel(fnNames),2);
for fnNamesIdx = 1:numel(fnNames)
    fn{fnNamesIdx,1} = fnNames{fnNamesIdx};
    fn{fnNamesIdx,2} = fnValues{fnNamesIdx};
end
source = cell(1,numel(fn));
j = 1;
for k = 1:size(fn,1)
    for l = 1:2
        source{j} = fn{k,l};
        j = j+1;
    end
end
config = fuserSourceConfiguration(source{:});
end

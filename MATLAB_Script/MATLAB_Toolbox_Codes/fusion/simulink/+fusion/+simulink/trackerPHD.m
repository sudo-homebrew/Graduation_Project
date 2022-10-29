classdef (StrictDefaults, Hidden) trackerPHD < trackerPHD & ...
        matlabshared.tracking.internal.fusion.AbstractSimulinkTracker

    % This is the Simulink implementation of trackerPHD.
    %
    %   See also: trackerPHD
    
    % Copyright 2020-2021 The MathWorks, Inc.
    
    %#codegen
    properties(Nontunable)
        %TentativeTracksOutputPort  Enable tentative tracks output
        %   Set this property to true if you want to get tentative tracks
        %   as an additional output.
        %
        %   Default: false
        TentativeTracksOutputPort = false
        
        %AllTracksOutputPort  Enable all tracks output
        %   Set this property to true if you want to get all the tracks
        %   as an additional output.
        %
        %   Default: false
        AllTracksOutputPort = false
        
        %InfoOutputPort  Enable info output
        %   Set this property to true if you want to get info
        %   as an additional output.
        %
        %   Default: false
        InfoOutputPort (1, 1) logical = false
        
        %IsSensorConfigurationsInput Enable sensor configurations input
        %   port.
        %   Set this property to true to update sensor configurations as an
        %   input with each time step.
        %
        %   Default: false
        IsSensorConfigurationsInput (1, 1) logical = false
        
        %IsStateParametersInput Enable state parameters input port.
        %   Set this property to true to update state parameters as an
        %   input with each time step.
        %
        %   Default: false
        IsStateParametersInput (1, 1) logical = false;
    end
    
    properties (Nontunable)
        % SensorConfigurationExpression MATLAB Expression for SensorConfigurations
        SensorConfigurationExpression = struct('SensorIndex',1,...
            'IsValidTime',true);
        
        % TimeInputSource   Prediction time source
        %   Set this property to 'Input port' if you want to pass the
        %   prediction time as an input to step. Set this property
        %   to 'Auto' if you want the prediction time to be inherited from
        %   Simulink.
        %
        %   Default: 'Input port'
        TimeInputSource = 'Input port';
        
        %TrackerIndexSimulink Unique identifier of the tracker
        %   Specify the unique index associated with this tracker in a
        %   decentralized tracking architecture. This index is used as the
        %   SourceIndex in the tracks output, and serves in track-to-track
        %   fusion. You must define this property to a positive value to
        %   use the track outputs as inputs to a track fuser.
        TrackerIndexSimulink = 0
    end
    
    properties(Access = protected, Nontunable)
        %pMaxNumDetections A scalar value that defines the maximum number
        %of detections in Simulink. This value gets set depending upon the
        %size of the detection input bus.
        pMaxNumDetections
               
        %pStateParametersIndex A scalar value that holds the index of state
        %  parameters when 'IsStateParametersInput' property is set to
        %  'true'.
        pStateParametersIndex
        
        %pSourceConfigurationIndex A scalar value that holds the index of
        %  source configurations when 'IsSensorConfigurationsInput' property
        %  is set to 'true'.
        pSensorConfigurationsIndex
    end
    
    properties(Access = protected)
        %pSampleTrack is the default sample track struct for trackerPHD.
        %   This property onlyworks in Simulink.
        pSampleTrack
    end
    
    properties(Hidden,Constant)
        TimeInputSourceSet = matlab.system.StringSet({'Input port','Auto'});
        BusNameSourceSet = matlab.system.StringSet({'Auto','Property'});
        BusName2SourceSet = matlab.system.StringSet({'Auto','Property'});
    end

    properties (Constant, Access = protected)
        pBusPrefix = {'BusTrackerPHD','BusTrackerInfo'};
    end
    
    properties(Nontunable)
        %BusNameSource Source of output bus name
        %   Set this property to 'Property' if you want to pass the bus
        %   name as an input. Set this property to 'Auto' if you want to
        %   use the default bus name.
        %
        %   Default: 'Auto'
        BusNameSource = 'Auto';

        %BusName Specify an output bus name
        BusName = char.empty(1,0)

        %BusName2Source Source of output Information bus name
        %   Set this property to 'Property' if you want to pass the bus
        %   name as an input. Set this property to 'Auto' if you want to
        %   use the default bus name in Simulink. This property only works
        %   in Simulink.
        %
        %   Default: 'Auto'
        BusName2Source = 'Auto'
        
        %BusName2 Specify an output Information bus name
        BusName2 = char.empty(1,0)
    end
    
    properties(Nontunable)
        %StateParametersSimulink  Nontunable StateParameters
        %
        StateParametersSimulink = struct;
    end
    
    properties(Constant, Access=protected)
        %pConstantTimeTol  A tolerance value for time interval check
        %   Defines a time tolerance that is used when comparing the
        %   detection timestamps with the time interval expected by the
        %   obj. Only applies if TimeInputSource is 'Input Port'
        %
        %   NOTES: the value of the tolerance should be:
        %     1. Large enough to avoid issues in floating point comparison
        %        (e.g., 1.00000001 > 1).
        %     2. Small enough so that no sensor detection in the next
        %        time frame can have a timestamp less than this tolerance
        %        value after the end of the current time frame.
        pConstantTimeTol = 1e-5
    end
    
    %Public
    methods
        %% Common functions
        function obj = trackerPHD(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
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
        
        function set.SensorConfigurationExpression(obj,value)
            validateattributes(value,{'struct'},{},class(obj),'Sensor Configurations')
            validFldNames = {'SensorIndex','IsValidTime','SensorLimits','SensorResolution', ...
                'SensorTransformFcn','SensorTransformParameters','FilterInitializationFcn',...
                'MaxNumDetsPerObject','ClutterDensity','DetectionProbability','MinDetectionProbability'};
            InpFldNames = fieldnames(value);
            for id = 1:numel(InpFldNames)
                coder.internal.assert(any(strcmp(validFldNames,InpFldNames{id})),...
                    'fusion:simulink:trackerPHD:InvalidConfig',InpFldNames{id});
            end
            obj.SensorConfigurationExpression = value;
        end
        
        function set.StateParametersSimulink(obj,value)
            validateattributes(value,{'struct'},{},class(obj),'StateParametersSimulink')
            obj.StateParametersSimulink = value;
        end
        
        function set.TrackerIndexSimulink(obj,value)
            validateattributes(value,{'numeric'},{'real','finite','nonsparse',...
                'nonnegative','integer','scalar'},class(obj),'TrackerIndex')
            obj.TrackerIndexSimulink = value;
            obj.TrackerIndex = value;
        end
    end
    %% Protected
    methods(Access=protected)
        
        function setupImpl(obj, detections, varargin)
            % Setup properties that will not be modified later on
            % Perform one-time calculations, such as computing constants
            
            % Extract detections from a bus
            dets = detections.Detections;
            detectionCells = makeDetectionCells(dets(1));
            
            % Set the value for maximum number of detections that can be
            % passed through the input detection bus
            obj.pMaxNumDetections = cast(numel(dets),'uint32');
            
            if obj.IsSensorConfigurationsInput
                obj.HasSensorConfigurationsInput = true;
            else
                obj.HasSensorConfigurationsInput = false;
            end
            
            % set State parameters
            setStateParameters(obj, obj.StateParametersSimulink);
            
            % Set Sensor Configurations
            setSensorConfigurations(obj);
            
            setupImpl@trackerPHD(obj, detectionCells, varargin{:});
            
            % Keep information about a SampleTrack in Simulink
            obj.pSampleTrack = obj.pLabelManager.defaultOutput(obj.pDensity,obj.StateParameters);
        end
        
        %% Save / Load / Clone Impl
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj
            
            % Save the base class information
            s = saveObjectImpl@trackerPHD(obj);
            if isLocked(obj)
                s.pMaxNumDetections          = obj.pMaxNumDetections;
                s.pStateParametersIndex        = obj.pStateParametersIndex;
                s.pSensorConfigurationsIndex   = obj.pSensorConfigurationsIndex;
                s.pSampleTrack                 = obj.pSampleTrack;
            end
            s = saveSimulinkProps(obj,s);
        end
        
        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s
            if wasLocked
                if isfield(s,'pMaxNumDetections')
                    obj.pMaxNumDetections = s.pMaxNumDetections;
                    s = rmfield(s, 'pMaxNumDetections');
                end
                if isfield(s,'pStateParametersIndex')
                    obj.pStateParametersIndex         = s.pStateParametersIndex;
                    s = rmfield(s, 'pStateParametersIndex');
                end
                if isfield(s,'pSensorConfigurationsIndex')
                    obj.pSensorConfigurationsIndex    = s.pSensorConfigurationsIndex;
                    s = rmfield(s, 'pSensorConfigurationsIndex');
                end
                if isfield(s,'pSampleTrack')
                    obj.pSampleTrack                  = s.pSampleTrack;
                    s = rmfield(s, 'pSampleTrack');
                end
            end
            s = loadSimulinkProps(obj,s,wasLocked);
            loadObjectImpl@trackerPHD(obj,s,wasLocked);
        end
        
        function newTracker = cloneImpl(obj)
            %clone Creates a copy of the simulink trackerPHD
            %   newTracker = clone(tracker) returns a copy of the tracker
            %   object.
            
            % Copy public properties
            newTracker = cloneImpl@trackerPHD(obj);
            if coder.internal.is_defined(obj.pDensity)
                newTracker.pMaxNumDetections          = obj.pMaxNumDetections;
                newTracker.pHasTimeInput              = obj.pHasTimeInput;
                newTracker.pStateParametersIndex      = obj.pStateParametersIndex;
                newTracker.pSensorConfigurationsIndex = obj.pSensorConfigurationsIndex;
                newTracker.pSampleTrack               = obj.pSampleTrack;
            end
        end
        
        
        function varargout = stepImpl(obj, detections, varargin)
            if obj.pHasTimeInput
                time = varargin{1};
            else
                time = getCurrentTime(obj);
            end
            % Extract sensor configurations and state parameters when
            % sensor configurations and state parameters is set to Input
            % port.
            detsCell = makeDetectionCells(detections.Detections(1:detections.NumDetections));
            
            if obj.IsStateParametersInput
                stParam = varargin{obj.pStateParametersIndex};
                setStateParameters(obj,stParam);
            end
            
            if obj.HasSensorConfigurationsInput
                sensorConfig = varargin{obj.pSensorConfigurationsIndex};
                inputConfigurations = sensorConfig.Configurations(1:sensorConfig.NumConfigurations);
                [confTracks, tentTracks, aTracks,info] = stepImpl@trackerPHD(obj, ...
                    detsCell, inputConfigurations, time);
            else
                [confTracks, tentTracks, aTracks,info] = stepImpl@trackerPHD(obj, ...
                    detsCell, time);
            end
            % Process Output
            [varargout{1:nargout}] = getTrackerOutputs(obj, confTracks, ...
                tentTracks, aTracks,info);
        end
        
        function varargout = getTrackerOutputs(obj, varargin)
            st = obj.pSampleTrack;
            if obj.InfoOutputPort
                sInfo =  varargin{end};
            end
            varargout{1} = sendToBus(obj, varargin{1}, 1, st);
            
            if obj.TentativeTracksOutputPort
                varargout{1+obj.TentativeTracksOutputPort} = sendToBus(obj, varargin{2}, 1, st);
            end
            
            if obj.AllTracksOutputPort
                varargout{1+obj.TentativeTracksOutputPort+ ...
                    obj.AllTracksOutputPort} = sendToBus(obj, varargin{3}, 1, st);
            end
            if obj.InfoOutputPort
                varargout{1+obj.TentativeTracksOutputPort+ ...
                    obj.AllTracksOutputPort+obj.InfoOutputPort} = getInfo(obj, sInfo);
            end
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
        end
        
        function validatePropertiesImpl(obj)
            if strcmpi(obj.TimeInputSource, 'Input port')
                obj.pHasTimeInput = true;
            else
                obj.pHasTimeInput = false;
            end
            
            if obj.IsSensorConfigurationsInput
                obj.pSensorConfigurationsIndex = 1 + obj.pHasTimeInput;
            else
                obj.pSensorConfigurationsIndex = obj.pHasTimeInput;
            end
            
            obj.pStateParametersIndex  = obj.pSensorConfigurationsIndex + obj.IsStateParametersInput;
            validatePropertiesImpl@trackerPHD(obj);
        end
        
        function info = getInfo(obj, infomation)
            sampleInfo = defaultInfo(obj);
            infoTrs = padInfoData(obj, infomation);
            info = sendToBus(obj, sampleInfo, 2, infoTrs);
        end
        
        function flag = isInactivePropertyImpl(obj, prop)
            % Return false if property is visible based on object
            % configuration, for the command line and System block dialog
            flag = isInactivePropertyImpl@matlabshared.tracking.internal.fusion.AbstractTracker(obj,prop);
            flag = flag || isPropInactiveInSimulink(obj,prop);
        end
        
        function flag = isInputSizeMutableImpl(obj, index)
            % Return true if input size is not allowed to change while
            % system is running
            flag = false; % All inputs except time are varsize
            if strcmpi(obj.TimeInputSource, 'Input port') && index == 2 % time is an input
                flag = true;
            end
        end
        
        function num = getNumInputsImpl(obj)
            % Define total number of inputs for system with optional inputs
            num = 1;
            if strcmpi(obj.TimeInputSource, 'Input port')
                num = num + 1;
            end
            
            if obj.IsSensorConfigurationsInput
                num = num + 1;
            end
            
            if obj.IsStateParametersInput
                num = num + 1;
            end
        end
        
        function num = getNumOutputsImpl(obj)
            % Define total number of outputs for system
            num = 1;
            if obj.TentativeTracksOutputPort
                num = num + 1;
            end
            if obj.AllTracksOutputPort
                num = num + 1;
            end
            if obj.InfoOutputPort
                num = num + 1;
            end
        end
        
        function icon = getIconImpl(~)
            % Define icon for System block
            icon = getString(message('fusion:simulink:trackerPHD:PHDICON'));
        end
        
        function [name,varargout] = getInputNamesImpl(obj)
            % Return input port names for System block
            name = getString(message('fusion:simulink:trackerPHD:Detection'));
            varargout = {};
            if strcmpi(obj.TimeInputSource, 'Input port')
                varargout = [varargout(:)' getString(message('fusion:simulink:trackerPHD:Prediction'))];
            end
            if obj.IsStateParametersInput
                varargout = [varargout(:)' getString(message('fusion:simulink:trackerPHD:InputStateParam'))];
            end
            if obj.IsSensorConfigurationsInput
                varargout = [varargout(:)' getString(message('fusion:simulink:trackerPHD:InputConfigurations'))];
            end
            
        end
        
        function [name, varargout] = getOutputNamesImpl(obj)
            % Return output port names for System block
            name = getString(message('fusion:simulink:trackerPHD:ConfirmedTrs'));
            varargout = {};
            if obj.TentativeTracksOutputPort
                varargout = [varargout(:)' getString(message('fusion:simulink:trackerPHD:TentativeTrs'))];
            end
            if obj.AllTracksOutputPort
                varargout = [varargout(:)' getString(message('fusion:simulink:trackerPHD:AllTrs'))];
            end
            if obj.InfoOutputPort
                varargout = [varargout(:)' getString(message('fusion:simulink:trackerPHD:Info'))];
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
    
    methods (Access = {?trackerPHD,?matlab.unittest.TestCase})
        function validateDetectionTimeStamps(obj, detectionTimes, time)
            coder.internal.assert(all(time+obj.pConstantTimeTol >= detectionTimes) &...
                all(obj.pLastTime < detectionTimes),'fusion:trackerPHD:DetectionTimeMismatch','step');
        end
    end
    
    methods(Access=protected)
        function setSensorConfigurations(obj,configStruct)
            if nargin==1
                configs = getConfigurationsFromStruct(obj.SensorConfigurationExpression);
            else
                num = numel(configStruct);
                coder.internal.assert(num<=obj.MaxNumSensors,'fusion:trackerPHD:SensorsGreaterThanMax');
                configs = getConfigurationsFromStruct(configStruct);
            end
            n = numel(configs);
            sensorIndex = zeros(n,1,'uint32');
            for i = 1:numel(configs)
                sensorIndex(i) = configs{i}.SensorIndex;
            end
            coder.internal.assert(numel(unique(sensorIndex)) == n,'fusion:trackerPHD:ExpectedUniqueSensors');
            obj.pSensorIndices = sensorIndex;
            obj.pSensorConfigurations = configs;
        end
        
        function [out, argsToBus] = defaultOutput(obj, busIdx)
            if nargin==1
                busIdx = 1;
            end
            out = struct.empty();
            argsToBus = {};
            switch busIdx
                case 1
                    detsBusType = propagatedInputBus(obj, 1);
                    if isempty(detsBusType)
                        return;
                    end
                    trk = defaultTrack(obj);
                    out = repmat(trk, [obj.MaxNumTracks, 1]);
                    sampleTrack = trk;
                    argsToBus = {sampleTrack};
                case 2
                    sampleInfo = defaultInfo(obj);
                    out = sampleInfo;
                    argsToBus = {sampleInfo};
            end
        end
        
        function trs = defaultTrack(obj)
            % Grab the sensor configuration
            sensorConfig = getConfigurationsFromStruct(obj.SensorConfigurationExpression);
            
            % Initialize the PHD filter
            phd = initialize(sensorConfig{1});
            
            % Grab state parameters
            StateParameters = obj.StateParametersSimulink;
            
            % Ask label manager to create default track
            sampleTrack = fusion.internal.LabelManager.defaultOutput(phd, StateParameters);
            
            % Remove optional State parameter from default track
            trs = matlabshared.tracking.internal.fusion.removeOptionalField(sampleTrack,'StateParameters');
        end
        
        function tracksOutput = sendToBus(obj, tracks, varargin)
            % sampleData can have the information of sample tracks or the
            % tracker info depending upon the busIdx.
            
            if isnumeric(varargin{1})
                busIdx = varargin{1};
                if numel(varargin)>1
                    st = varargin{2};
                    sampleTrack = matlabshared.tracking.internal.fusion.removeOptionalField(st,'StateParameters');
                end
            else
                busIdx = 1; % Single output port case
                st = varargin{1};
                sampleTrack = matlabshared.tracking.internal.fusion.removeOptionalField(st,'StateParameters');
            end
            
            switch busIdx
                case 1
                    fullTracksList = repmat(sampleTrack, [obj.MaxNumTracks, 1]);
                    listLength = numel(tracks);
                    for iTrack = 1:listLength
                        lTrack = tracks(iTrack);
                        trs = matlabshared.tracking.internal.fusion.removeOptionalField(lTrack,'StateParameters');
                        fullTracksList(iTrack) = trs;
                    end
                    tracksOutput = struct('NumTracks', listLength, 'Tracks', fullTracksList);
                case 2
                    tracksOutput = sampleTrack;
            end
        end
    end
    
    methods(Hidden)
        function infoStruct = defaultInfo(obj)
            num = obj.MaxNumTracks;
            % Grab the sensor configuration
            sensorConfig = getConfigurationsFromStruct(obj.SensorConfigurationExpression);
            % Initialize the PHD filter
            phd = initialize(sensorConfig{1});
            maxNumSize = phd.MaxNumComponents;
            sensorAnalysis = struct('SensorIndex',0,...
                'DetectionCells', false(num,num),...
                'DetectionLikelihoods', zeros(num,num),...
                'IsBirthCell',false(1,num),...
                'NumPartitions' ,0,...
                'DetectionProbability',zeros(num,1), ...
                'LabelsBeforeCorrection',zeros(1,maxNumSize,'uint32'), ...
                'LabelsAfterCorrection', zeros(1,maxNumSize,'uint32'),...
                'WeightsBeforeCorrection' ,zeros(1,maxNumSize) , ...
                'WeightsAfterCorrection',zeros(1,maxNumSize));
            infoStruct = struct(...
                'CorrectionOrder', zeros(obj.MaxNumSensors,1),...
                'TrackIDsAtStepBeginning', zeros(num,1,'uint32'),...
                'DeletedTrackIDs', zeros(num,1,'uint32'),...
                'TrackIDsAtStepEnd', zeros(num,1,'uint32'),...
                'SensorAnalysisInfo', repmat(sensorAnalysis,obj.MaxNumSensors,1) ...
                );
        end
        
        function info = padInfoData(obj,infoTrack)
            % Variable size data can not be logged in simulink hence
            % padding the data to make it fixed size.
            
            num = obj.MaxNumTracks;
            phd = initialize(obj.SensorConfigurations{1});
            maxNumSize = phd.MaxNumComponents;
            crOrder = zeros(obj.MaxNumSensors,1,'like',infoTrack.CorrectionOrder);
            if ~isempty(infoTrack.CorrectionOrder)
                d1 = numel(infoTrack.CorrectionOrder);
                crOrder(1:d1) = infoTrack.CorrectionOrder;
            end
            
            trIDs = zeros(num,1,'like',infoTrack.TrackIDsAtStepBeginning);
            if ~isempty(infoTrack.TrackIDsAtStepBeginning)
                d1 = numel(infoTrack.TrackIDsAtStepBeginning);
                trIDs(1:d1)= infoTrack.TrackIDsAtStepBeginning;
            end
            
            deletedTrack = zeros(num, 1, 'like', infoTrack.DeletedTrackIDs);
            if ~isempty(infoTrack.DeletedTrackIDs)
                d1 = numel(infoTrack.DeletedTrackIDs);
                deletedTrack(1:d1) = infoTrack.DeletedTrackIDs;
            end
            
            trackIDsEnd = zeros(num, 1, 'like', infoTrack.TrackIDsAtStepEnd);
            if ~isempty(infoTrack.TrackIDsAtStepEnd)
                d1 = numel(infoTrack.TrackIDsAtStepEnd);
                trackIDsEnd(1:d1) = infoTrack.TrackIDsAtStepEnd;
            end
            
            infoStruct = defaultInfo(obj);
            sensorAnalysis = infoStruct.SensorAnalysisInfo;
            
            if ~isempty(infoTrack.SensorAnalysisInfo)
                for sensorIdx = 1:numel(infoTrack.SensorAnalysisInfo)
                    sIndex = cast(0,'like',infoTrack.SensorAnalysisInfo{sensorIdx}.SensorIndex);
                    if ~isempty(infoTrack.SensorAnalysisInfo{sensorIdx}.SensorIndex)
                        sIndex = infoTrack.SensorAnalysisInfo{sensorIdx}.SensorIndex;
                    end
                    
                    detsCell = false(num,num,'like',infoTrack.SensorAnalysisInfo{sensorIdx}.DetectionCells);
                    if ~isempty(infoTrack.SensorAnalysisInfo{sensorIdx}.DetectionCells)
                        d1 = size(infoTrack.SensorAnalysisInfo{sensorIdx}.DetectionCells,1);
                        d2   = size(infoTrack.SensorAnalysisInfo{sensorIdx}.DetectionCells,2);
                        detsCell(1:d1,1:d2)= infoTrack.SensorAnalysisInfo{sensorIdx}.DetectionCells(1:d1,1:d2);
                    end
                    
                    detslhood = zeros(num,num,'like',infoTrack.SensorAnalysisInfo{sensorIdx}.DetectionLikelihoods);
                    if ~isempty(infoTrack.SensorAnalysisInfo{sensorIdx}.DetectionLikelihoods)
                        d1 = size(infoTrack.SensorAnalysisInfo{sensorIdx}.DetectionLikelihoods,1);
                        d2   = size(infoTrack.SensorAnalysisInfo{sensorIdx}.DetectionLikelihoods,2);
                        detslhood(1:d1,1:d2)= infoTrack.SensorAnalysisInfo{sensorIdx}.DetectionLikelihoods(1:d1,1:d2);
                    end
                    
                    birthCell = false(1,num,'like',infoTrack.SensorAnalysisInfo{sensorIdx}.IsBirthCell);
                    if ~isempty(infoTrack.SensorAnalysisInfo{sensorIdx}.IsBirthCell)
                        d1 = numel(infoTrack.SensorAnalysisInfo{sensorIdx}.IsBirthCell);
                        birthCell(1:d1)= infoTrack.SensorAnalysisInfo{sensorIdx}.IsBirthCell;
                    end
                    
                    numPart = cast(0,'like',infoTrack.SensorAnalysisInfo{sensorIdx}.NumPartitions);
                    if ~isempty(infoTrack.SensorAnalysisInfo{sensorIdx}.NumPartitions)
                        numPart = infoTrack.SensorAnalysisInfo{sensorIdx}.NumPartitions;
                    end
                    
                    detProbs = zeros(num,1,'like',infoTrack.SensorAnalysisInfo{sensorIdx}.DetectionProbability);
                    if ~isempty(infoTrack.SensorAnalysisInfo{sensorIdx}.DetectionProbability)
                        d1 = numel(infoTrack.SensorAnalysisInfo{sensorIdx}.DetectionProbability);
                        detProbs(1:d1)= infoTrack.SensorAnalysisInfo{sensorIdx}.DetectionProbability;
                    end
                    
                    lBeforeCorr = zeros(1,maxNumSize,'like',infoTrack.SensorAnalysisInfo{sensorIdx}.LabelsBeforeCorrection);
                    if ~isempty(infoTrack.SensorAnalysisInfo{sensorIdx}.LabelsBeforeCorrection)
                        d1 = numel(infoTrack.SensorAnalysisInfo{sensorIdx}.LabelsBeforeCorrection);
                        lBeforeCorr(1:d1)= infoTrack.SensorAnalysisInfo{sensorIdx}.LabelsBeforeCorrection;
                    end
                    
                    lAfterCorr = zeros(1,maxNumSize,'like',infoTrack.SensorAnalysisInfo{sensorIdx}.LabelsAfterCorrection);
                    if ~isempty(infoTrack.SensorAnalysisInfo{sensorIdx}.LabelsAfterCorrection)
                        d1 = numel(infoTrack.SensorAnalysisInfo{sensorIdx}.LabelsAfterCorrection);
                        lAfterCorr(1:d1)= infoTrack.SensorAnalysisInfo{sensorIdx}.LabelsAfterCorrection;
                    end
                    
                    wBeforeCorr = zeros(1,maxNumSize,'like',infoTrack.SensorAnalysisInfo{sensorIdx}.WeightsBeforeCorrection);
                    if ~isempty(infoTrack.SensorAnalysisInfo{sensorIdx}.WeightsBeforeCorrection)
                        d1 = numel(infoTrack.SensorAnalysisInfo{sensorIdx}.WeightsBeforeCorrection);
                        wBeforeCorr(1:d1)= infoTrack.SensorAnalysisInfo{sensorIdx}.WeightsBeforeCorrection;
                    end
                    
                    wAfterCorr = zeros(1,maxNumSize,'like',infoTrack.SensorAnalysisInfo{sensorIdx}.WeightsAfterCorrection);
                    if ~isempty(infoTrack.SensorAnalysisInfo{sensorIdx}.WeightsAfterCorrection)
                        d1 = numel(infoTrack.SensorAnalysisInfo{sensorIdx}.WeightsAfterCorrection);
                        wAfterCorr(1:d1)= infoTrack.SensorAnalysisInfo{sensorIdx}.WeightsAfterCorrection;
                    end
                    
                    sensorAnalysis(sensorIdx).SensorIndex            = sIndex;
                    sensorAnalysis(sensorIdx).DetectionCells         = detsCell;
                    sensorAnalysis(sensorIdx).DetectionLikelihoods   = detslhood;
                    sensorAnalysis(sensorIdx).IsBirthCell            = birthCell;
                    sensorAnalysis(sensorIdx).NumPartitions          = numPart;
                    sensorAnalysis(sensorIdx).DetectionProbability   = detProbs;
                    sensorAnalysis(sensorIdx).LabelsBeforeCorrection = lBeforeCorr;
                    sensorAnalysis(sensorIdx).LabelsAfterCorrection  = lAfterCorr;
                    sensorAnalysis(sensorIdx).WeightsBeforeCorrection= wBeforeCorr ;
                    sensorAnalysis(sensorIdx).WeightsAfterCorrection = wAfterCorr;
                end
            end
            
            info = struct(...
                'CorrectionOrder', crOrder,...
                'TrackIDsAtStepBeginning', trIDs,...
                'DeletedTrackIDs', deletedTrack,...
                'TrackIDsAtStepEnd', trackIDsEnd,...
                'SensorAnalysisInfo', sensorAnalysis ...
                );
        end
    end
    
    methods(Static, Access = protected)
        %% Simulink customization functions
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(...
                'Title', 'fusion:block:phdTrackerTitle', ...
                'Text',	 'fusion:block:phdTrackerDesc');
        end
        
        function groups = getPropertyGroupsImpl
            % Define property section(s) for System block dialog, only in
            % Simulink
            
            absTrkrSection = matlabshared.tracking.internal.fusion.AbstractSimulinkTracker.getBusPropertyGroups;
            
            trkManagementProps = {'TrackerIndexSimulink', 'PartitioningFcn', ...
                'AssignmentThreshold','MaxNumSensors', 'MaxNumTracks', ...
                'SensorConfigurationExpression','IsSensorConfigurationsInput' ...
                'StateParametersSimulink','IsStateParametersInput'};
            
            trsManagementProps = {'BirthRate', 'DeathRate', ...
                'ExtractionThreshold', 'ConfirmationThreshold', ...
                'DeletionThreshold','MergingThreshold', 'LabelingThresholds'};
            
            trkManagementSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackerPHD','',trkManagementProps);
            trsManagementSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackerPHD','',trsManagementProps);
            
            trkGroup = matlab.system.display.SectionGroup( ...
                'Title', getString(message('fusion:simulink:trackerPHD:GrouptrkManagementSection')), ...
                'Sections', trkManagementSection);
            
            trsGroup = matlab.system.display.SectionGroup( ...
                'Title', getString(message('fusion:simulink:trackerPHD:GrouptrsManagementSection')), ...
                'Sections', trsManagementSection);
            
            busList = absTrkrSection.PropertyList;
            redirectList = {1 , 'SimulinkBusPropagation'; 2, 'SimulinkBusPropagation'};
            portsList = [busList(:)',{'BusName2Source'},{'BusName2'}];
            portsSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackerPHD','OutputPortSettings', portsList, redirectList);
            
            propIOList = {'TimeInputSource', 'TentativeTracksOutputPort', ...
                'AllTracksOutputPort', 'InfoOutputPort'};
            ioSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackerPHD','InputsOutputs', propIOList);
            ioGroup = matlab.system.display.SectionGroup( ...
                'Title', getString(message('fusion:simulink:trackerPHD:GroupIO')), ...
                'Sections', [ioSection,portsSection]);
            
            groups = [trkGroup, trsGroup, ioGroup];
        end
    end
    
    methods(Static, Hidden)
        function flag = isAllowedInSystemBlock
            flag = true;
        end
    end
end

function detCells = makeDetectionCells(detsArray)
% detsArray is a struct array, which needs to be converted to a
% cell array of objectDetections
detCells = cell(numel(detsArray),1);

hasAttribs = isfield(detsArray,'ObjectAttributes');
hasMeasParams = isfield(detsArray,'MeasurementParameters');

for i = 1:numel(detCells)
    time = detsArray(i).Time;
    meas = detsArray(i).Measurement;
    measNoise = detsArray(i).MeasurementNoise;
    objClass = detsArray(i).ObjectClassID;
    if hasAttribs
        objAttribs = detsArray(i).ObjectAttributes;
    else
        objAttribs = struct;
    end
    if hasMeasParams
        measParams = detsArray(i).MeasurementParameters;
    else
        measParams = struct;
    end
    senIdx = detsArray(i).SensorIndex;
    
    detection = objectDetection(time,meas,'MeasurementNoise',measNoise,...
        'MeasurementParameters',measParams,'ObjectAttributes',objAttribs,...
        'ObjectClassID',objClass);
    if senIdx > 0
        detection.SensorIndex = senIdx;
    end
    detCells{i} = detection;
end
end

function config = getConfigurationsFromStruct(configStruct)
num = numel(configStruct);
config = cell(1,num);
for i = 1:num
    fnNames = fieldnames(configStruct(i));
    fnValues = struct2cell(configStruct(i));
    config{i} = generateConfigurations(fnNames, fnValues);
end
end

function config = generateConfigurations(fnNames, fnValues)
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
config = trackingSensorConfiguration(source{:});
end
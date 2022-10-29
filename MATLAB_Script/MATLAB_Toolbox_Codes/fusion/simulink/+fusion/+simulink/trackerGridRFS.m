classdef (StrictDefaults, Hidden) trackerGridRFS < trackerGridRFS & ...
        matlabshared.tracking.internal.fusion.AbstractSimulinkTracker

    % This is the Simulink implementation of trackerGridRFS.
    %
    %   See also: trackerGridRFS

    % Copyright 2021 The MathWorks, Inc.

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

        %HasSensorConfigurations Enable sensor configurations input
        %   port.
        %   Set this property to true to update sensor configurations as an
        %   input with each time step.
        %
        %   Default: false
        HasSensorConfigurations (1, 1) logical = false

        %HasStateParametersInput Enable state parameters input port.
        %   Set this property to true to update state parameters as an
        %   input with each time step.
        %
        %   Default: false
        HasStateParametersInput (1, 1) logical = false;

        %HasVisualization Enable the dynamic grid map visualization.
        %   Plots the dynamic grid map in local coordinates for
        %   each time step.
        %
        %   Default: false
        HasVisualization (1, 1) logical = false;

        %HasPlotVelocity Enable flag to control if velocity vectors must
        %   be plotted in the visualization.
        %
        %   Default: false
        HasPlotVelocity (1, 1) logical = false;

        %HasFastUpdate Enable to perform a lightweight update
        %   to the map in the figure.
        %
        %   Default: true
        HasFastUpdate (1, 1) logical = true;

        %HasFastUpdate Enable to invert the colors in the map. In the
        %   default settings, empty spaces are white and occupied spaces are
        %   black.
        %
        %   Default: false
        HasInvertColors (1, 1) logical = false;
    end

    properties (Nontunable)
        % SensorConfigurationExpression MATLAB Expression for SensorConfigurations
        SensorConfigurationExpression = struct('SensorIndex',1,...
            'IsValidTime',true,'SensorLimits',[-180 180; 0 100]);

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
        %
        %   Default: 0
        TrackerIndexSimulink = 0
    end

    properties(Access = protected, Nontunable)
        %pStateParametersIndex A scalar value that holds the index of state
        %  parameters when 'HasStateParametersInput' property is set to
        %  'true'.
        pStateParametersIndex

        %pSourceConfigurationIndex A scalar value that holds the index of
        %  source configurations when 'HasSensorConfigurationsInput' property
        %  is set to 'true'.
        pSensorConfigurationsIndex

        %pSampleTrack is the default sample track struct for trackerGridRFS.
        %   This property only works in Simulink.
        pSampleTrack
        
        %pRandSettings stores the sysObj for random number setting
        pRandSettings
    end

    properties (Constant, Access = protected)
        pBusPrefix = 'BusTrackerGridRFS';
    end

    properties(Hidden,Constant)
        TimeInputSourceSet = matlab.system.StringSet({'Input port','Auto'});
        BusNameSourceSet = matlab.system.StringSet({'Auto','Property'});
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
    end

    properties(Nontunable)
        %StateParametersSimulink  Nontunable StateParameters
        %
        %  Default: struct
        StateParametersSimulink = struct;

        %GridLengthSimulink Dimension of the grid in x-direction of the local
        %  coordinate.
        %  Specify the length as a positive scalar value describing
        %  the length of the 2-D grid.
        %
        %  Default: 100
        GridLengthSimulink = 100;

        %GridWidthSimulink Dimension of the grid in y-direction of the local
        %  coordinate.
        %  Specify the width as a positive scalar value describing
        %  the width of the 2-D grid.
        %
        %  Default: 100
        GridWidthSimulink = 100;

        %GridResolutionSimulink Resolution of the grid.
        %  Specify the resolution of the grid as a positive scalar
        %  describing number of cells per meter of the grid in both x and y
        %  direction.
        %
        %  Default: 1
        GridResolutionSimulink = 1;

        %GridOriginInLocalSimulink Location of the grid in local coordinates
        %  A vector defining the [X Y] location of the bottom-left
        %  corner of the grid, relative to the local frame.
        %
        %   Default: [-50 50]
        GridOriginInLocalSimulink = [-50 50];
        
        %RandNumberGenerator 
        RandNumberGenerator = toStruct(fusion.simulink.internal.RandomNumberGenerator);

        %MaxNumSensorsSimulink  Maximum number of sensors that are attached to the
        %  autonomous system.
        %
        %  Default: 20
        MaxNumSensorsSimulink (1, 1) {mustBePositive, mustBeInteger} = 20;
    end

    %Public
    methods
        %% Common functions
        function obj = trackerGridRFS(varargin)
            % Constructor call to MATLAB class
            obj@trackerGridRFS(varargin{:});
        end

        function set.TrackerIndexSimulink(obj,value)
            validateattributes(value,{'numeric'},{'real','finite','nonsparse',...
                'nonnegative','integer','scalar'},class(obj),'TrackerIndex')
            obj.TrackerIndexSimulink = value;
            obj.TrackerIndex = value;
        end

        function set.MaxNumSensorsSimulink(obj,value)
            validateattributes(value,{'numeric'},{'scalar','positive','real','finite','nonsparse'},mfilename,'MaxNumSensorsSimulink');
            obj.MaxNumSensorsSimulink = value;
            obj.MaxNumSensors = value;
        end

        function set.GridLengthSimulink(obj,value)
            validateattributes(value,{'numeric'},{'scalar','positive','real','finite','nonsparse'},mfilename,'GridLengthSimulink');
            obj.GridLengthSimulink = value;
            obj.GridLength = value;
        end

        function set.GridWidthSimulink(obj,value)
            validateattributes(value,{'numeric'},{'scalar','positive','real','finite','nonsparse'},mfilename,'GridWidthSimulink');
            obj.GridWidthSimulink = value;
            obj.GridWidth = value;
        end

        function set.GridResolutionSimulink(obj,value)
            validateattributes(value,{'numeric'},{'scalar','positive','real','finite','nonsparse'},mfilename,'GridResolutionSimulink');
            obj.GridResolutionSimulink = value;
            obj.GridResolution = value;
        end

        function set.GridOriginInLocalSimulink(obj,value)
            validateattributes(value,{'numeric'},{'real','finite','nonsparse','numel',2},mfilename,'GridLength');
            obj.GridOriginInLocalSimulink = value;
            obj.GridOriginInLocal = value;
        end

        function set.SensorConfigurationExpression(obj,value)
            obj.SensorConfigurationExpression = value;
        end

        function set.HasSensorConfigurations(obj,value)
            obj.HasSensorConfigurations = value;
            if obj.HasSensorConfigurations
                obj.HasSensorConfigurationsInput = true;
            else
                obj.HasSensorConfigurationsInput = false;
            end
        end
        
        function set.RandNumberGenerator(obj,val)
            sysObj = fusion.simulink.internal.RandomNumberGenerator.toRandfromStruct(val);
            % Save internally as struct
            obj.RandNumberGenerator = toStruct(sysObj);
        end

        function val = get.RandNumberGenerator(obj)
            val = fusion.simulink.internal.RandomNumberGenerator.toRandfromStruct(obj.RandNumberGenerator);
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
    end
    %% Protected
    methods(Access=protected)

        function setupImpl(obj, sensorData, varargin)
            % get sensor data
            sData = sensorData.SensorData;
            % get sensor configurations
            sensorConfig = getConfigurationsFromStruct(obj.SensorConfigurationExpression);
            % set sensor configurations
            obj.pMeasurementProjector.SensorConfigurations = sensorConfig;
            % set state parameters
            obj.pStateParameters = obj.StateParametersSimulink;
            
            % get time input
            if obj.pHasTimeInput
                time = varargin{1};
            else
                time = getCurrentTime(obj);
            end
            obj.pRandSettings = fusion.simulink.internal.RandomNumberGenerator.toRandfromStruct(obj.RandNumberGenerator);
            
            if any(strcmpi(obj.pRandSettings.InitialSeedSource,{'Repeatable','Not repeatable'}))
                if obj.UseGPU
                    sc = RandStream('Threefry','NormalTransform','Inversion', ...
                        'Seed',obj.pRandSettings.UsedSeed);
                    RandStream.setGlobalStream(sc)
                    sg = parallel.gpu.RandStream('Threefry','NormalTransform', ...
                        'Inversion','Seed',obj.pRandSettings.UsedSeed);
                    parallel.gpu.RandStream.setGlobalStream(sg);
                else
                    rng(obj.pRandSettings.UsedSeed,'twister');
                end
            else
                if obj.UseGPU
                    sc = RandStream('Threefry','NormalTransform','Inversion', ...
                        'Seed',obj.pRandSettings.InitialSeed);
                    RandStream.setGlobalStream(sc)
                    sg = parallel.gpu.RandStream('Threefry','NormalTransform', ...
                        'Inversion','Seed',obj.pRandSettings.InitialSeed);
                    parallel.gpu.RandStream.setGlobalStream(sg);
                else
                    rng(obj.pRandSettings.InitialSeed,'twister');
                end
            end

            if obj.HasSensorConfigurations
                setupImpl@trackerGridRFS(obj,sData,obj.SensorConfigurationExpression,time);
            else
                setupImpl@trackerGridRFS(obj,sData,time);
            end
            % get sample track structure
            sampleTrack = getSampleTrackStruct(obj);
            obj.pSampleTrack = sampleTrack;
        end

        function varargout = stepImpl(obj,data,varargin)
            % get sensor input data
            sensorData = getSensorData(data);
            % get the time input
            if obj.pHasTimeInput
                time = varargin{1};
            else
                time = getCurrentTime(obj);
            end
            if obj.HasStateParametersInput
                stParam = varargin{obj.pStateParametersIndex};
                setStateParameters(obj,stParam);
            end

            if obj.HasSensorConfigurations
                sensorConfig = varargin{obj.pSensorConfigurationsIndex};
                inputConfigurations = sensorConfig.Configurations(1:sensorConfig.NumConfigurations);
                [confTracks, tentTracks, aTracks] = stepImpl@trackerGridRFS(obj, ...
                    sensorData, inputConfigurations, time);
            else
                [confTracks, tentTracks, aTracks] = stepImpl@trackerGridRFS(obj, ...
                    sensorData, time);
            end

            % Process Output
            [varargout{1:nargout}] = getTrackerOutputs(obj, confTracks, ...
                tentTracks, aTracks);

            % Visualize
            if obj.HasVisualization
                showDynamicMap(obj,'FastUpdate',obj.HasFastUpdate, ...
                    'PlotVelocity',obj.HasPlotVelocity, 'InvertColors',obj.HasInvertColors);
            end
        end
        
        function varargout = getTrackerOutputs(obj, varargin)
            st = obj.pSampleTrack;
            varargout{1} = sendToBus(obj, varargin{1}, st);

            if obj.TentativeTracksOutputPort
                varargout{1+obj.TentativeTracksOutputPort} = sendToBus(obj, varargin{2}, st);
            end

            if obj.AllTracksOutputPort
                varargout{1+obj.TentativeTracksOutputPort+ ...
                    obj.AllTracksOutputPort} = sendToBus(obj, varargin{3}, st);
            end

        end

        function tracksOutput = sendToBus(obj, tracks, varargin)
            % sampleData can have the information of sample tracks or the
            % tracker info depending upon the busIdx.

            st = varargin{1};
            sampleTrack = matlabshared.tracking.internal.fusion.removeOptionalField(st,'StateParameters');

            fullTracksList = repmat(sampleTrack, [obj.MaxNumTracks, 1]);
            listLength = numel(tracks);
            for iTrack = 1:listLength
                lTrack = tracks(iTrack);
                if isa(lTrack,'objectTrack')
                    uTrack = protectedToStruct(lTrack,numel(lTrack.TrackLogicState));
                else
                    histLength = max(obj.ConfirmationThreshold(2),obj.DeletionThreshold(2));
                    uTrack = protectedToStruct(objectTrack(lTrack),histLength);
                end
                uTrk = matlabshared.tracking.internal.fusion.removeOptionalField(uTrack,{'StateParameters','ObjectAttributes'});
                fullTracksList(iTrack) = uTrk;
            end
            tracksOutput = struct('NumTracks', listLength, 'Tracks', fullTracksList);
        end

        function [out, argsToBus] = defaultOutput(obj)
            out = struct.empty();
            argsToBus = {};
            sensorBusType = propagatedInputBus(obj, 1);
            if isempty(sensorBusType)
                return;
            end
            sampleSensorData = getSensorDataFromBus(obj, sensorBusType);
            trk = defaultTrack(obj,sampleSensorData);
            out = repmat(trk, [obj.MaxNumTracks, 1]);
            sampleTrack = trk;
            argsToBus = {sampleTrack};
        end

        function sampleTrack = defaultTrack(obj,sensorData)
            % Grab the sensor configuration
            % Setup projector
            % Define class to use
            if ~coder.internal.is_defined(obj.pClassToUse)
                coder.internal.assert(numel(sensorData) > 0,'fusion:GridTracker:UndefinedSampleData');
                obj.pClassToUse = class(sensorData(1).Measurement);
            end
            sampleTrack = getSampleTrackStruct(obj);
        end

        function sTrack = getSampleTrackStruct(obj)
            switch obj.MotionModel
                case 'constant-velocity'
                    numVar = 4;
                case 'constant-acceleration'
                    numVar = 6;
                case 'constant-turnrate'
                    numVar = 5;
            end
            SampleCellInformation = dynamicEvidentialGridMap.getSampleCellSimulink(numVar,obj.pClassToUse);

            % Create track using user-provided TrackInitializationFcn
            if isstring(obj.TrackInitializationFcn)||ischar(obj.TrackInitializationFcn)
                func = str2func(obj.TrackInitializationFcn);
                initTrack = func(SampleCellInformation);
            else
                initTrack = obj.TrackInitializationFcn(SampleCellInformation);
            end

            % SampleTrack using information from extractor as well as the
            % track.
            trs = createSampleTrack(obj, initTrack);
            if ~isempty(obj.StateParametersSimulink)
                trs.StateParameters = obj.StateParametersSimulink;
            end
            % Remove optional State parameter from default track
            sTrack = matlabshared.tracking.internal.fusion.removeOptionalField(trs,{'StateParameters','ObjectAttributes'});
        end

        function sampleTrack = createSampleTrack(obj, initTrack)
            % Create sample track from information captured in the object
            % as well as the track supplied by the initialization function.

            state = initTrack.State;
            n = numel(state);
            classToUse = class(state);
            if ~coder.internal.is_defined(obj.pClassToUse)
                obj.pClassToUse = classToUse;
            end
            objAttributes = initTrack.ObjectAttributes;
            histLength = max(obj.ConfirmationThreshold(2),obj.DeletionThreshold(2));
            sampleObjectTrack = objectTrack('TrackID',1,...
                'State',zeros(n,1,classToUse),...
                'StateCovariance',eye(n,classToUse),...
                'SourceIndex',uint32(1),...
                'Age',uint32(0),...
                'StateParameters',obj.StateParameters,....
                'ObjectClassID',0,...
                'TrackLogic','History',...
                'TrackLogicState',false(1,histLength),...
                'ObjectAttributes',objAttributes,...
                'IsConfirmed',false,...
                'IsCoasted',false...
                );

            % In Simulink create an array of structs
            sampleTrack = protectedToStruct(sampleObjectTrack,histLength);
        end

        function sampleSensorData = getSensorDataFromBus(obj, busName)
            allData = matlabshared.tracking.internal.SimulinkBusUtilities.bus2struct(busName);
            checkValidInputBus(obj,allData);
            if isstruct(allData.SensorData)
                sampleSensorData = allData.SensorData(1);
            else
                sampleSensorData = [];
            end
        end

        %% Save / Load / Clone Impl
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj

            % Save the base class information
            s = saveObjectImpl@trackerGridRFS(obj);
            if isLocked(obj)
                s.pSampleTrack                = obj.pSampleTrack;
                s.pStateParametersIndex       = obj.pStateParametersIndex;
                s.pSensorConfigurationsIndex  = obj.pSensorConfigurationsIndex;
            end
           s = saveSimulinkProps(obj,s);
        end

        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s
            if wasLocked
                obj.pSampleTrack = s.pSampleTrack;
                s = rmfield(s, 'pSampleTrack');
                obj.pStateParametersIndex         = s.pStateParametersIndex;
                s = rmfield(s, 'pStateParametersIndex');
                obj.pSensorConfigurationsIndex    = s.pSensorConfigurationsIndex;
                s = rmfield(s, 'pSensorConfigurationsIndex');
            end
            s = loadSimulinkProps(obj,s,wasLocked);
            loadObjectImpl@trackerGridRFS(obj,s,wasLocked);
        end

        function validateInputsImpl(obj, ~, varargin)
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
        
        function releaseImpl(obj)
            % Release resources
            if any(strcmpi(obj.pRandSettings.InitialSeedSource,{'Repeatable','Not repeatable'}))
                rng(obj.pRandSettings.UsedSeed);
            else
                rng(obj.pRandSettings.InitialSeed);
            end
            releaseImpl@trackerGridRFS(obj);
        end

        function flag = isInactivePropertyImpl(obj, prop)
            % Return false if property is visible based on object
            % configuration, for the command line and System block dialog
            flag = isInactivePropertyImpl@matlabshared.tracking.internal.fusion.AbstractTracker(obj,prop);
            flag = flag || isInactiveBusProperty(obj,prop);

            flag = flag || isInactivePropertyImpl@trackerGridRFS(obj,prop);

            flag = flag || (~obj.HasVisualization && any(strcmpi(prop,{'HasPlotVelocity', ...
                'HasFastUpdate','HasInvertColors'})));
            invisibleInSimulink = {'NumTracks', 'NumConfirmedTracks','StateParameters'};
            invisFlags = strcmp(prop, invisibleInSimulink);
            if any(invisFlags)
                flag = true;
            end
        end

        function num = getNumInputsImpl(obj)
            % Define total number of inputs for system with optional inputs
            num = 1;
            if strcmpi(obj.TimeInputSource, 'Input port')
                num = num + 1;
            end
            if obj.HasSensorConfigurations
                num = num + 1;
            end
            if obj.HasStateParametersInput
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
        end

        function icon = getIconImpl(~)
            % Define icon for System block
            icon = getString(message('fusion:simulink:trackerGridRFS:GRIDICON'));
        end

        function [name,varargout] = getInputNamesImpl(obj)
            % Return input port names for System block
            name = getString(message('fusion:simulink:trackerGridRFS:SensorData'));
            varargout = {};
            if strcmpi(obj.TimeInputSource, 'Input port')
                varargout = [varargout(:)' getString(message('fusion:simulink:trackerGridRFS:Prediction'))];
            end
            if obj.HasStateParametersInput
                varargout = [varargout(:)' getString(message('fusion:simulink:trackerGridRFS:InputStateParam'))];
            end
            if obj.HasSensorConfigurations
                varargout = [varargout(:)' getString(message('fusion:simulink:trackerGridRFS:InputConfigurations'))];
            end
        end

        function [name, varargout] = getOutputNamesImpl(obj)
            % Return output port names for System block
            name = getString(message('fusion:simulink:trackerGridRFS:ConfirmedTrs'));
            varargout = {};
            if obj.TentativeTracksOutputPort
                varargout = [varargout(:)' getString(message('fusion:simulink:trackerGridRFS:TentativeTrs'))];
            end
            if obj.AllTracksOutputPort
                varargout = [varargout(:)' getString(message('fusion:simulink:trackerGridRFS:AllTrs'))];
            end
        end

        function [sz1,sz2,sz3] = getOutputSizeImpl(~)
            sz1 = [1 1];
            sz2 = [1 1];
            sz3 = [1 1];
        end

        function  varargout = getOutputDataTypeImpl(obj)
            dtTracks = getBusDataTypes(obj);
            varargout = {};
            varargout{1} = dtTracks;
            if obj.TentativeTracksOutputPort
                varargout = [varargout(:)' {dtTracks}];
            end
            if obj.AllTracksOutputPort
                varargout = [varargout(:)' {dtTracks}];
            end
        end

        function [cp1,cp2,cp3] = isOutputComplexImpl(~)
            cp1 = false;
            cp2 = false;
            cp3 = false;
        end

        function [out1,out2,out3] = isOutputFixedSizeImpl(~)
            out1 = true;
            out2 = true;
            out3 = true;
        end

        function validatePropertiesImpl(obj)
            if strcmpi(obj.TimeInputSource, 'Input port')
                obj.pHasTimeInput = true;
            else
                obj.pHasTimeInput = false;
            end
            if obj.HasSensorConfigurations
                obj.pSensorConfigurationsIndex = 1 + obj.pHasTimeInput;
            else
                obj.pSensorConfigurationsIndex = obj.pHasTimeInput;
            end
            obj.pStateParametersIndex  = obj.pSensorConfigurationsIndex + obj.HasStateParametersInput;
        end
    end

    methods(Static, Access = protected)
        %% Simulink customization functions
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(...
                'Title', 'fusion:block:gridTrackerTitle', ...
                'Text',	 'fusion:block:gridTrackerDesc');
        end

        function groups = getPropertyGroupsImpl
            % Define property section(s) for System block dialog, only in
            % Simulink

            absTrkrSection = matlabshared.tracking.internal.fusion.AbstractSimulinkTracker.getBusPropertyGroups;

            trkConfigurationProps = {'TrackerIndexSimulink', 'MaxNumSensorsSimulink', ...
                'MaxNumTracks', 'SensorConfigurationExpression','HasSensorConfigurations',...
                'StateParametersSimulink','HasStateParametersInput','UseGPU'};

            trkManagementSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackerGridRFS','',trkConfigurationProps);
            gridDefinationProps = {'GridLengthSimulink','GridWidthSimulink','GridResolutionSimulink', ...
                'GridOriginInLocalSimulink'};
            gridDefinitionSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackerGridRFS','DefineGrid',gridDefinationProps);
            trkGroup = matlab.system.display.SectionGroup( ...
                'Title', getString(message('fusion:simulink:trackerGridRFS:GrouptrkManagementSection')), ...
                'Sections', [trkManagementSection,gridDefinitionSection]);

            particlefilteringProps = {'MotionModel', 'VelocityLimits', 'HasAdditiveProcessNoise', ...
                'ProcessNoise', 'NumParticles', 'NumBirthParticles', ...
                'BirthProbability', 'DeathRate', 'FreeSpaceDiscountFactor'};
            particlefilteringSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackerGridRFS','',particlefilteringProps);
            classSet = matlab.system.display.internal.ClassStringSet(...
                {'fusion.simulink.internal.RandomNumberGenerator'},...
                'PropertiesTitle', '', ...
                'NestDisplay', false);
            pRandom = matlab.system.display.internal.Property('RandNumberGenerator', ...
                'ClassStringSet', classSet);
            randomList = {pRandom};
            randSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:RandomNumberGenerator','RandNumber',randomList);
            particleGroup = matlab.system.display.SectionGroup( ...
                'Title', getString(message('fusion:simulink:trackerGridRFS:GroupparticlefltSection')), ...
                'Sections', [particlefilteringSection, randSection]);

            busList = absTrkrSection.PropertyList;
            redirectList = {1 , 'SimulinkBusPropagation'; 2, 'SimulinkBusPropagation'};
            portsList = busList(:)';
            portsSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackerGridRFS','OutputPortSettings', portsList, redirectList);

            propIOList = {'TimeInputSource', 'TentativeTracksOutputPort', ...
                'AllTracksOutputPort'};
            ioSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackerGridRFS','InputsOutputs', propIOList);
            ioGroup = matlab.system.display.SectionGroup( ...
                'Title', getString(message('fusion:simulink:trackerGridRFS:GroupIO')), ...
                'Sections', [ioSection,portsSection]);
            visualizationProps = {'HasVisualization', 'HasPlotVelocity', ...
                'HasFastUpdate','HasInvertColors'};
            visualizationSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackerGridRFS','Visual',visualizationProps);
            visualGroup = matlab.system.display.SectionGroup( ...
                'Title', getString(message('fusion:simulink:trackerGridRFS:GroupvisualizationSection')), ...
                'Sections', visualizationSection);

            trackInitializationList = {'Clustering','CustomClusteringFcn','ClusteringThreshold', ...
                'MinNumCellsPerCluster','TrackInitializationFcn'};
            trackInitSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackerGridRFS','TrackInit',trackInitializationList);

            trackManagementProps = {'TrackUpdateFcn','AssignmentThreshold', ...
                'ConfirmationThreshold','DeletionThreshold'};
            trackManagementSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackerGridRFS','',trackManagementProps);
            trackManagementGroup = matlab.system.display.SectionGroup( ...
                'Title', getString(message('fusion:simulink:trackerGridRFS:GroupTrk')), ...
                'Sections', [trackInitSection, trackManagementSection]);

            groups = [trkGroup,particleGroup,trackManagementGroup,visualGroup,ioGroup];
        end
    end
    
    methods(Access=private)
        function checkValidInputBus(~, inp)
            % Validate inputs to the step method at initialization
            validateattributes(inp,{'struct'},{'scalar'},...
                'trackerGridRFS','input bus SensorData');
            % Check for invalid fields
            validFlds = {'NumSensors', 'SensorData'};
            flds = fieldnames(inp);
            for m = 1:numel(flds)
                thisFld = flds{m};
                coder.internal.assert(any(strcmp(thisFld,validFlds)),'fusion:simulink:trackerGridRFS:invalidField',['Input' '.' thisFld]);
            end
            % Make sure that Sensor data is not empty
            validateattributes(inp.SensorData,{'struct'},{'nonempty'},...
                'trackerGridRFS',['Input' '.' 'SensorData']);
            coder.internal.assert(inp.NumSensors <= numel(inp.SensorData),'fusion:simulink:trackerGridRFS:tooManyData',['Input' '.' 'SensorData(:)'],numel(inp.SensorData),inp.NumSensors);

            % NumTracks is a nonnegative integer scalar
            validateattributes(inp.NumSensors ,{'numeric'},{'scalar','nonnegative','integer','real','finite'},...
                'trackerGridRFS',['Input' '.NumSensors']);

            % Has valid Sensor Data fields
            validFlds = {'Time','SensorIndex','Measurement', ...
                'MeasurementParameters','NumMeasurements'};
            flds = fieldnames(inp.SensorData);
            for m = 1:numel(flds)
                thisFld = flds{m};
                coder.internal.assert(any(strcmp(thisFld,validFlds)),'fusion:simulink:trackerGridRFS:invalidField',['Input' '.SensorData(:).' thisFld]);
            end
        end
    end
        
    methods(Static, Hidden)
        function flag = isAllowedInSystemBlock
            flag = true;
        end
    end
end

function config = getConfigurationsFromStruct(configStruct)
num = numel(configStruct);
config = cell(1,num);
for i = 1:num
    if isstruct(configStruct(i))
        fnNames = fieldnames(configStruct(i));
        fnValues = struct2cell(configStruct(i));
        config{i} = generateConfigurations(fnNames, fnValues);
    end
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

function sensorData = getSensorData(InpData)
% This methods extracts the actual sized element for local tracks structures.
if InpData.NumSensors>0
    data  = InpData.SensorData(1:InpData.NumSensors);
    sdata = getSingleSensorDataStruct(data(1));
    sensorData = repmat(sdata,size(data));
    for m = 1:numel(data)
        sensorData(m) = getSingleSensorDataStruct(data(m));
    end
else
    data = InpData.SensorData;
    sdata = getSingleSensorDataStruct(data(1));
    sensorData = repmat(sdata,0,1);
end
end

function sData = getSingleSensorDataStruct(inpData)
% This method creates a new sensor data structure by making sure that the
% new sensor data structure has all the elements with their expected sizes
% all the padding done during the sensor concatenation is removed with the
% additional fields NumMeasurements that was added during concatenation.
if coder.target('MATLAB')
    sData = inpData;
    sData.Measurement = inpData.Measurement(:,1:inpData.NumMeasurements);
    sData = rmfield(sData,'NumMeasurements');
else
    sData = struct;
    fieldsToCopy = {'Time','SensorIndex','MeasurementParameters'};
    for i = 1:numel(fieldsToCopy)
        sData.(fieldsToCopy{i}) = inpData.(fieldsToCopy{i});
    end
    sData.Measurement = inpData.Measurement(:,1:inpData.NumMeasurements);
end
end
% trackingSensorConfiguration Represent a sensor configuration for tracking
% config = trackingSensorConfiguration(sensorIndex) creates a sensor
% configuration for a sensor to be used with the trackerPHD and
% trackerGridRFS. The config allows you to specify the sensor parameters
% like clutter density, sensor limits and resolution for each sensor used
% with the tracker. It also allows you to specify how the sensor perceives
% the state of a track to detect it using properties like
% SensorTransformFcn, SensorTransformParameters and
% FilterInitializationFcn.
%
% When used with trackerPHD, the configuration enables the tracker to
% perform four main routine operations:
%
% 1. Evaluate the probability of detection at points in state-space.
% 2. Compute expected number of detections from a target.
% 3. Initiate components in the probability hypothesis density.
% 4. Obtain the clutter density of the sensor.
%
% When used with trackerGridRFS, the configuration assists the tracker to
% project sensor data on the 2-D grid. The tracker uses the
% SensorTransformParameters to calculate the ego vehicle's and sensor's
% location and orientation in the tracking coordinate frame. The tracker
% uses the SensorLimits to calculate the field of view and maximum range of
% the sensor. The SensorTransformFcn and FilterInitializationFcn properties
% of trackingSensorConfiguration are not relevant for trackerGridRFS.
% 
% config = trackingSensorConfiguration(sensorIndex) creates a
% trackingSensorConfiguration with SensorIndex set to the sensorIndex. The
% config created by this syntax is representative of radar sensor with
% field of view 20 and 5 in azimuth and elevation respectively.It can
% detect targets up to a maximum range of 1000 meters. The resolutions are
% set to 4 deg, 2 deg and 10 meters in azimuth, elevation and range
% respectively. In addition to the properties above, the config proposes
% components in the density using a constant velocity model. It specifies
% the probability of detection to be 0.9 and assumes a clutter density of
% 1e-3 false alarms per unit volume.
%
% config = trackingSensorConfiguration(sensorIndex,'Name',value)
% allows specifying properties of the object during construction.
%
% sensorConfiguration properties:
% SensorIndex               -   Unique identifier for the sensor
% IsValidTime               -   A flag indicating if this sensor should be
%                               used for updating the tracker
% SensorLimits              -   Limits on sensor's ability to detect a
%                               target
% SensorResolution          -   Resolution of each parameter provided in
%                               SensorLimits
% SensorTransformFcn        -   A function handle to transform a track
%                               state to sensor's detection space
% SensorTransformParameters -   A struct or array of struct which contains
%                               information about transformation of track
%                               state to sensor's detection space
% DetectionProbability      -   Probability of detecting a target inside
%                               SensorLimits
% ClutterDensity            -   Expected number of false alarms per unit
%                               volume of the sensor
% FilterInitializationFcn   -   A function_handle to specify a filter
%                               initialization function
% MaxNumDetsPerObject       -   Maximum number of detections that the
%                               sensor can generate per object
%
% % Example: Represent a radar model using trackingSensorConfiguration
% % -------------------------------------------------------------------
% % Consider a radar with the ability to detect targets inside its
% % angular field of view with maximum range of 500 meters and maximum
% % range-rate of 50 m/s
% azLimits = [-10 10];
% elLimits = [-2.5 2.5];
% rangeLimits = [0 500];
% rangeRateLimits = [-50 50];
% sensorLimits = [azLimits;elLimits;rangeLimits;rangeRateLimits];
% sensorResolution = [5 2 10 3]; % [az el r rr];
% 
% % Specifying the function handle to transform the state into
% % [az;el;range;rr]. The signature of this function is similar to the
% % cvmeas measurement model. Hence, you can use the function cvmeas as
% % SensorTransformFcn
% transformFcn = @cvmeas;
% 
% % To specify the parameters required for cvmeas, you can use the
% % SensorTransformParameters property of the config. Here, we represent
% % that the sensor is mounted at the center of the platform and the
% % platform is located at [100;30;20] and is moving with a velocity of
% % [-5;4;2] units in the scenario.
% 
% % This structure defines the sensor mounting on the platform.
% params(1) = struct('Frame','Spherical','OriginPosition',[0;0;0],...
%                  'OriginVelocity',[0;0;0],'HasRange',true,...
%                  'HasVelocity',true);
% % This structure defines the platform location, velocity, and orientation
% % in the scenario.
% params(2) = struct('Frame','Rectangular','OriginPosition',[100;30;20],...
%                   'OriginVelocity',[-5;4;2],'HasRange',true,...
%                   'HasVelocity',true);
% 
% % Creating the configuration
% config = trackingSensorConfiguration('SensorIndex',3,'SensorLimits',sensorLimits,...
%                                      'SensorResolution',sensorResolution,...
%                                      'SensorTransformParameters',params,...
%                                      'SensorTransformFcn',@cvmeas);
%
%
% % Example2: Represent a radar model using trackingSensorConfiguration
% % using fusionRadarSensor model.
% % -------------------------------------------------------------------
% % Consider a radar with the ability to detect targets inside its angular
% % field of view with maximum range of 500 meters and maximum range-rate of
% % 50 m/s.
% 
% sensor = fusionRadarSensor(1, 'FieldOfView',[20 5],'RangeLimits',[0 500], ...
% 'HasRangeRate',true,'HasElevation',true,'RangeRateLimits',[-50 50], ...
% 'AzimuthResolution',5,'RangeResolution',10,'ElevationResolution',2, ...
% 'RangeRateResolution',3);
% 
% % Specifying the function handle to transform the state into
% % [az;el;range;rr]. The signature of this function is similar to the
% % cvmeas measurement model. Hence, you can use the function cvmeas as
% % SensorTransformFcn
% transformFcn = @cvmeas;
%  
% config = trackingSensorConfiguration(sensor,'SensorTransformFcn',transformFcn);
% 
% % This structure defines the platform location and velocity in the 
% % scenario.
% config.SensorTransformParameters(2).OriginPosition = [100;30;20];
% config.SensorTransformParameters(2).OriginVelocity = [-5;4;2];
%
% See also: partitionDetections, cvmeas, initcvggiwphd, initcaggiwphd,
% initctggiwphd.

% Copyright 2018-2021 The MathWorks, Inc.

%#codegen
%#function initcvggiwphd initcaggiwphd initctggiwphd
%#function cvmeas cameas ctmeas cvmeasjac ctmeasjac cameasjac

classdef trackingSensorConfiguration < fusion.internal.AbstractTrackingSensorConfiguration ...
        &  matlabshared.tracking.internal.fusion.CustomDisplay
    properties
        % SensorIndex Unique identifier for the sensor
        SensorIndex
        % IsValidTime A flag indicating if the tracker must be updated
        % with this sensor.
        % Default: false.
        IsValidTime = false;
    end
    
    properties (Dependent)
        % SensorLimits Upper and lower limits of the sensor's ability to
        % detect a target. For example, [minAz maxAz;minEl maxEl;minR maxR]
        % for a sensor which can detect all targets inside its angular
        % field of view and within a certain range. Az, El and R refers to
        % azimuth, elevation and range respectively.
        SensorLimits
        
        % SensorResolution Resolution of the sensor for each parameter
        % specified in the SensorLimits. For example,
        % [azResolution;elResolution;rangeResolution] for a sensor which
        % can detect all targets inside its angular field of view and
        % within a certain range. Resolution must be a N-by-1 vector, where
        % N is the number of rows in the Limits. For a sensor which does
        % not have a resolution in one of the parameters, you can specify
        % its resolution as equal to the maximum minus minimum limits of
        % the parameter, which is inferred as one resolution cell per
        % parameter.
        SensorResolution
        
        % SensorTransformFcn A function to transform a track state to the
        % sensor detection state. For example, [x;vx;y;vy;z;vz] in
        % scenario to [az;el;range] in sensor frame. It must support the
        % following signature function
        % 	detStates = SensorTransformFcn(trackStates,params)
        % where params are the values stored in SensorTransformParameters
        % The signature of the function is similar to measurement models,
        % therefore you can use a measurement function cvmeas as the
        % SensorTransformFcn.
        %
        % When used with gmphd for non-extended targets or with ggiwphd:
        %
        % detStates is a N-by-M matrix, where N is the number of rows in
        % the SensorLimits and M is the number of input states.
        %
        % When used with gmphd for extended targets:
        %
        % The SensorTransformFcn allows you to specify multiple detStates
        % per trackState. In this case, detStates can be a N-by-M-by-S
        % matrix, where S are the number of sources on the state. For
        % example, if the target is described by a rectangular state, the
        % sources can be the corners of the rectangle. If any of the source
        % falls inside the SensorLimits, the target is declared detectable.
        % To calculate expected number of detections for extended targets,
        % the trackingSensorConfiguration uses the spread (max - min) of
        % each detStates of the source and uses the ratio of spreads and
        % resolution on each limit to calculate the expected number of
        % detections. You can override this by providing an optional output
        % from the SensorTransformFcn:
        %
        % [..., Nexp] = SensorTransformFcn(trackStates, params)
        %
        % where Nexp are the expected number of detections from
        % each state.
        %
        SensorTransformFcn
        
        % SensorTransformParameters Additional parameters passed as input to
        % the TransformFcn. For example, to transform a state
        % [x;vx;y;vy;z;vz] in scenario to [az;el;range] in sensor's frame,
        % we need the information about sensor and its mounting platform.
        % You can define these using an array of struct in the
        % following manner:
        %
        % First define the state of sensor with respect to the platform
        % origin.
        % senToPlatStruct =
        % struct('Frame','Spherical','OriginPosition',[0;0;0],...
        %         'OriginVelocity',[0;0;0],'Orientation',eye(3),...
        %         'HasRange',true,'HasVelocity',false,'HasAzimuth',true,...
        %         'HasElevation',true);
        %
        % Then, define the state of the platform with respect to the
        % scenario origin.
        %
        % platToScenarioStruct =
        % struct('Frame','Rectangular','OriginPosition',[100;50;0],...
        %         'OriginVelocity',[10;5;2],'Orientation',eye(3),...
        %         'HasRange',true,'HasVelocity',true,'HasAzimuth',true,...
        %         'HasElevation',true);
        %
        % The cvmeas function returns the azimuth, elevation and range
        % using the following transformParameters.
        % transformParameters = [senToPlatStruct;platToScenarioStruct];
        SensorTransformParameters
    end
    
    properties (Dependent)
        % DetectionProbability Probability of detecting a target inside the
        % SensorLimits
        DetectionProbability
        
        % ClutterDensity Expected number of false alarms per unit volume
        % from this sensor.
        ClutterDensity
        
        % MinDetectionProbability Probability of detecting a target
        % estimated to be outside the SensorLimits.
        MinDetectionProbability
    end
    
    properties (Dependent)
        %FilterInitializationFcn  Filter initialization function name
        %   Specify the function for initializing the PHD filter used
        %   by tracker. The function must support the following signatures:
        %       filter = FilterInitializationFcn()
        %       filter = FilterInitializationFcn(detections)
        %
        %   filter     - a valid PHD filter that has the components of
        %                density initialized.
        %   detections - a cell array of objectDetection that initiates
        %                components in the density
        %   The no-input argument function signature allows you to specify
        %   the birth density in PHD filter without using detections
        FilterInitializationFcn
    end
    
    properties (Dependent)
        % MaxNumDetsPerObject Maximum number of detections the sensor can
        % report for a given object. For sensors reporting one detection
        % per object, this number should be set to 1.
        MaxNumDetsPerObject;
    end
    
    properties (Access = {?trackingSensorConfiguration,?matlab.unittest.TestCase})
        pMaxNumDetsPerObject
        pDetectionProbability
        pClutterDensity;
        pMinDetectionProbability;
        pSensorLimits;
        pSensorResolution;
        pTransformFcn
        pTransformParameters;
        pNumParameters;
        pInitializationFcn;
        pDataType;
        pIsTransformValid = false;
        pHasExpectedDetsOutput
        pHasSourcesOutput
    end
    
    properties (Access = {?trackingSensorConfiguration,?matlab.unittest.TestCase})
        constProbabilityDetection = 0.9;
        constClutterDensity = 1e-3;
        constMinProbDetection = 0.05;
        constMaxNumDetsPerObject = inf;
    end
    
    methods
        function obj = trackingSensorConfiguration(varargin)
            % obj = trackingSensorConfiguration(1);
            % obj = trackingSensorConfiguration('SensorIndex',1);
            % Both signatures above create default values.
            if nargin >= 1 && isa(varargin{1},'matlabshared.tracking.internal.fusion.BaseRadarDataGenerator')
                sensor = varargin{1};
                createFromSensor(obj, sensor, varargin{2:end});
            else
                allocateMemory(obj,varargin{:});
                numArguments = numel(varargin);
                if numArguments == 1 || (numArguments == 2 && strcmpi(varargin{1},'SensorIndex'))
                    args = getDefaultArgs(obj);
                elseif mod(numArguments,2) == 1
                    args = {varargin{2:end}};
                else
                    args = varargin;
                end
                matlabshared.fusionutils.internal.setProperties(obj,numel(args),args{:});
            end
        end
    end
    
    methods (Hidden)
        function filter = initialize(obj,varargin)
            % filter = initialize(obj,detections) creates a filter from
            % detections.
            % filter = initialize(obj) creates the predictive birth
            % filter
            isInitFcnDefined = coder.internal.is_defined(obj.pInitializationFcn);
            isTransformFcnDefined = coder.internal.is_defined(obj.pTransformFcn);

            if ~isInitFcnDefined && ~isTransformFcnDefined
                obj.pInitializationFcn = @initcvggiwphd;
                obj.pTransformFcn = @cvmeas;
            elseif ~isInitFcnDefined && isTransformFcnDefined
                coder.internal.assert(false,'fusion:trackingSensorConfiguration:UndefinedFcn','FilterInitializationFcn');
            end
            filter = obj.FilterInitializationFcn(varargin{:});
        end

        function Pd = probDetection(obj,states,varargin)
            % Pd = probDetection(obj, meanStates) calculates the
            % probability of detection at each state provided in the
            % meanState.
            % meanState is a N-by-P matrix, where N is the size of state
            % and P are the number of states.
            % Pd is a 1-by-P vector defining the probability of detection
            % of each state.
            % Pd = probDetection(obj,meanStates,stateCovariances) allows
            % specifying an uncertainty matrix for calculating the
            % probability of detection.
            % stateCovariances is a N-by-N-by-P matrix defining the
            % covariance of each state.
            % The function calculates the samples of state around the
            % Pd = probDetection(obj, stateSamples) calculates the
            % probability of detection at the provided stateSamples.
            % stateSamples is a n-by-p-by-q matrix, where n is the size of
            % the state, p are number of states and q are the number of
            % samples of this state. If any of the sample is within
            % detectable limits, the probability of detection is
            % obj.DetectionProbability, otherwise the probability of
            % detection is minimum detection probability.
            % When q is 1, i.e. stateSamples is a 2-D
            % matrix, only one sample is used for the calculating the
            % detectability.
            narginchk(2,3)

            stateSize = size(states,1);
            numStates = size(states,2);
            if numStates == 0
                Pd = zeros(1,0,class(states));
                return;
            end

            [stateSamples, numSamples] = sampleState(obj, states, varargin{:});
            flatStateSamples = reshape(stateSamples,stateSize,[]);
            limits = obj.SensorLimits;
            numParams = obj.pNumParameters;

            % Initialize isDetectable
            isDetectable = false(1,numStates);

            % Validate transform function.
            validateSensorTransformFcn(obj,flatStateSamples(:,1));

            % Compute detection params and check which one are within
            % limits.
            detParams = obj.SensorTransformFcn(flatStateSamples,obj.SensorTransformParameters);
            detParamsWithinLimits = bsxfun(@lt, detParams, limits(:,2)) & bsxfun(@gt, detParams, limits(:,1));
            % Reshaping to perform sample/source per state at third dimension
            unflatDetParams = reshapeParamsOnThird(obj, detParamsWithinLimits, numParams, numStates, numSamples);

            % Check detectability using the span of params.
            isDetectable = isDetectable | any(all(unflatDetParams,1),3);

            % Detectables are Pd. Non detectables are minimum Pd.
            Pd = isDetectable(:)*obj.DetectionProbability;
            Pd = max(obj.MinDetectionProbability,Pd);
        end

        function N = expectedNumDets(obj, states, varargin)
            % expectedNumDets(obj, states, covariances) calculates the
            % expected number of detections by using a mean
            % expectedNumDets(obj, stateSamples) calculates the expected
            % number of detections, given samples from a state. This can
            % correspond to sigma-points generated on the state.

            stateSize = size(states,1);
            numStates = size(states,2);
            if numStates == 0
                % nothing to do, no state provided.
                N = zeros(1,0,class(states));
                return;
            end

            [stateSamples, numSamples] = sampleState(obj, states, varargin{:});
            flatStateSamples = reshape(stateSamples,stateSize,[]);
            numParams = obj.pNumParameters;

            % Validate transform function
            validateSensorTransformFcn(obj,stateSamples(:,1));

            % if N is provided by the transform function
            if obj.pHasExpectedDetsOutput
                [~,Nsamples] = obj.SensorTransformFcn(flatStateSamples,obj.SensorTransformParameters);
                Nreshaped = reshape(Nsamples,numStates,1,[]);
                N = mean(Nreshaped,3);
            else
                % N is calculated using the span of covariance and sources.
                transformedStates = obj.SensorTransformFcn(flatStateSamples,obj.SensorTransformParameters);
                unflatTransformedStates = reshapeParamsOnThird(obj, transformedStates, numParams, numStates, numSamples);
                N = calcNumDetsFromSpan(obj, unflatTransformedStates);
            end
            N = min(obj.MaxNumDetsPerObject,N);
        end

        function sync(obj,miniObject)
            % sync(obj,miniObj);
            %
            % Sync configuration with a mini configuration which can be a
            % struct or a trackingSensorConfiguration.

            validateattributes(miniObject,{'struct','trackingSensorConfiguration'},{'scalar'},'trackingSensorConfiguration');
            if isstruct(miniObject)
                fieldsSpecified = fieldnames(miniObject);
                if coder.target('MATLAB')
                    isFieldAMember = ismember(fieldsSpecified,{'SensorLimits', ...
                        'SensorResolution', 'SensorTransformParameters', ...
                        'SensorIndex', 'IsValidTime', 'MeasurementParameters', ...
                        'FieldOfView', 'IsScanDone', 'RangeLimits', 'RangeRateLimits'});
                else
                    isFieldAMember = false(numel(fieldsSpecified),1);
                    for i = 1:numel(fieldsSpecified)
                        isFieldAMember(i) = any(strcmpi(fieldsSpecified{i},{'SensorLimits', ...
                            'SensorResolution','SensorTransformParameters', ...
                            'SensorIndex','IsValidTime', 'MeasurementParameters', ...
                            'FieldOfView', 'IsScanDone', 'RangeLimits', 'RangeRateLimits'}));
                    end
                end
                if ~all(isFieldAMember)
                    firstInvalidProp = {fieldsSpecified{~isFieldAMember}};
                    coder.internal.assert(false,'fusion:trackingSensorConfiguration:invalidProp',firstInvalidProp{1});
                end
                for i = 1:numel(fieldsSpecified)
                    if strcmpi(fieldsSpecified{i},'MeasurementParameters')
                        syncMeasurementParameters(obj,miniObject);
                    elseif strcmpi(fieldsSpecified{i},'FieldOfView') && isfield(miniObject,'MeasurementParameters')
                        syncSensorLimits(obj,miniObject);
                    elseif any(strcmpi(fieldsSpecified{i},{'IsScanDone', 'RangeLimits', 'RangeRateLimits'}))
                        continue;
                    else
                        obj.(fieldsSpecified{i}) = miniObject.(fieldsSpecified{i});
                    end
                end
            else
                % Only sync tunable properties
                obj.SensorLimits = miniObject.SensorLimits;
                obj.SensorResolution = miniObject.SensorResolution;
                obj.SensorTransformParameters = miniObject.SensorTransformParameters;
                obj.IsValidTime = miniObject.IsValidTime;
            end
        end

        function outStruct = toStruct(obj)
            % This method converts trackingSensorConfiguration object to
            % structure.

            outStruct = struct('SensorIndex',obj.SensorIndex, ...
                'IsValidTime', obj.IsValidTime, ...
                'SensorLimits', obj.SensorLimits, ...
                'SensorResolution', obj.SensorResolution, ...
                'SensorTransformParameters', obj.SensorTransformParameters, ...
                'MaxNumDetsPerObject', obj.MaxNumDetsPerObject, ...
                'ClutterDensity', obj.ClutterDensity, ...
                'DetectionProbability', obj.DetectionProbability, ...
                'MinDetectionProbability', obj.MinDetectionProbability ...
                );

            if ~isempty(obj.SensorTransformFcn)
                outStruct.SensorTransformFcn = func2str(obj.SensorTransformFcn);
            end

            if ~isempty(obj.FilterInitializationFcn)
                outStruct.FilterInitializationFcn = func2str(obj.FilterInitializationFcn);
            end
        end

        function obj2 = clone(obj)
            % Create a clone of the object, obj
            % Create object with memory controlling inputs
            obj2 = trackingSensorConfiguration('SensorIndex',obj.SensorIndex,...
                'SensorLimits',obj.SensorLimits,...
                'SensorTransformParameters',obj.SensorTransformParameters);
            ppProperties = {'IsValidTime',...
                'pMaxNumDetsPerObject',...
                'pDetectionProbability',...
                'pClutterDensity',...
                'pMinDetectionProbability',...
                'pSensorResolution',...
                'pTransformFcn',...
                'pTransformParameters',...
                'pInitializationFcn',...
                'pIsTransformValid',...
                'pHasSourcesOutput',...
                'pHasExpectedDetsOutput'...
                };
            for i = 1:numel(ppProperties)
                if coder.internal.is_defined(obj.(ppProperties{i}))
                    obj2.(ppProperties{i}) = obj.(ppProperties{i});
                end
            end
        end

        function createFromSensor(obj, sensor, varargin)
            sensorArgs = getSensorConfigurationArgs(sensor);
            obj.SensorIndex = sensorArgs.SensorIndex;
            obj.pNumParameters = size(sensorArgs.SensorLimits,1);
            obj.pDataType = class(sensorArgs.SensorLimits);
            obj.pSensorLimits = sensorArgs.SensorLimits;
            obj.pSensorResolution = sensorArgs.SensorResolution;
            obj.pTransformParameters = sensorArgs.SensorTransformParameters;
            obj.ClutterDensity = sensorArgs.ClutterDensity;
            obj.MaxNumDetsPerObject = sensorArgs.MaxNumDetsPerObject;
            obj.DetectionProbability = sensorArgs.DetectionProbability;

            % Parse varargin for FilterInitializationFcn,
            % SensorTransformFcn and IsValidTime.
            filtFcnIdx = fusion.internal.findProp('FilterInitializationFcn',varargin{:});
            if filtFcnIdx <= numel(varargin) - 1
                filtFcn = varargin{filtFcnIdx+1};
                obj.FilterInitializationFcn = filtFcn;
            end
            transformFcnIdx = fusion.internal.findProp('SensorTransformFcn',varargin{:});
            if transformFcnIdx <= numel(varargin) - 1
                transformFcn = varargin{transformFcnIdx+1};
                obj.SensorTransformFcn = transformFcn;
            end
            isValidTimeIdx = fusion.internal.findProp('IsValidTime',varargin{:});
            if isValidTimeIdx <= numel(varargin) - 1
                isValidTime = varargin{isValidTimeIdx+1};
                obj.IsValidTime = isValidTime;
            end
        end
    end

    methods (Access = {?trackingSensorConfiguration,?matlab.unittest.TestCase})
        function [stateSamples, numSamples] = sampleState(~, states, varargin)
            if nargin == 2
                stateSamples = states;
                numSamples = size(stateSamples,3);
            else
                dataType = class(states);
                stateSize = size(states,1);
                numStates = size(states,2);
                % 3 standard deviations.
                scale = cast(3,dataType);
                covariances = varargin{1};
                numSamples = 2*stateSize + 1;
                stateSamples = zeros(stateSize,numStates,numSamples,dataType);
                stateSamples(:,:,1) = states;
                for i = 1:numStates
                    [~,stateSamples(:,i,2:end)] = matlabshared.tracking.internal.calcSigmaPoints(covariances(:,:,i),states(:,i),scale);
                end
            end
        end
        
        function N = calcNumDetsFromSpan(obj, transformedStates)
            maxValue = bsxfun(@min,obj.SensorLimits(:,2),max(transformedStates,[],3));
            minValue = bsxfun(@max,obj.SensorLimits(:,1),min(transformedStates,[],3));
            span = maxValue - minValue;
            % Outside FOV
            negativeSpan = any(span < 0);
            span(:,negativeSpan) = 0;
            participatingCells = isfinite(obj.SensorResolution) & (obj.SensorLimits(:,2) - obj.SensorLimits(:,1))./obj.SensorResolution > 1;
            perResolutionNumber = bsxfun(@rdivide,span(participatingCells,:),obj.SensorResolution(participatingCells));
            perResolutionNumber = max(1, perResolutionNumber);
            N = prod(perResolutionNumber,1)';
            N(negativeSpan) = 0;
        end
        
        function unflatDetParams = reshapeParamsOnThird(obj, detParams, numParams, numStates, numSamples)
            if obj.pHasSourcesOutput
                % With source output, there are sources per sample.
                numSources = size(detParams,3);
                unflatDetParams = zeros(numParams,numStates,numSamples*numSources,class(detParams));
                for i = 1:numStates
                    thisStateSamples = detParams(:,i:numStates:end,:);
                    unflatDetParams(:,i,:) = reshape(thisStateSamples,numParams,1,numSamples*numSources);
                end
            else
                unflatDetParams = reshape(detParams,numParams,numStates,numSamples);
            end
        end
    end
    
    % Setters, getters
    methods
        function set.SensorIndex(obj,val)
            validateattributes(val,{'numeric'},{'scalar','integer','positive'},'trackingSensorConfiguration','SensorIndex');
            obj.SensorIndex = val;
        end
        
        function set.IsValidTime(obj,val)
            validateattributes(val,{'numeric','logical'},{'binary','scalar'},'trackingSensorConfiguration','IsValidTime');
            obj.IsValidTime = cast(val,'logical');
        end
        
        function set.SensorLimits(obj,val)
            % Test data type, cannot call transpose on invalid data types.
            validateattributes(val,{'single','double'},{'real','nonsparse','nonnan','ncols',2},'trackingSensorConfiguration','SensorLimits');
            validateattributes(val',{'single','double'},{'increasing'},'trackingSensorConfiguration','SensorLimits');
            obj.pSensorLimits = cast(val,obj.pDataType);
        end
        
        function val = get.SensorLimits(obj)
            val = obj.pSensorLimits;
        end
        
        function set.SensorResolution(obj,val)
            % Allow inf as Resolution;
            validateattributes(val,{'single','double'},{'real','nonsparse','nonnan','vector'},'trackingSensorConfiguration','SensorResolution');
            obj.pSensorResolution = cast(val(:),obj.pDataType);
        end
        
        function val = get.SensorResolution(obj)
            val = obj.pSensorResolution;
        end
        
        function set.SensorTransformParameters(obj,val)
            validateattributes(val,{'struct'},{'2d'},'trackingSensorConfiguration','SensorTransformParameters');
            obj.pTransformParameters = val;
        end
        
        function val = get.SensorTransformParameters(obj)
            val = obj.pTransformParameters;
        end
        
        function set.FilterInitializationFcn(obj,func)
            validateattributes(func,{'function_handle','char','string'},{'nonempty'},'trackingSensorConfiguration','FilterInitializationFcn');
            if isa(func,'function_handle')
                obj.pInitializationFcn = func;
            else
                obj.pInitializationFcn = str2func(func);
            end
        end
        
        function val = get.FilterInitializationFcn(obj)
            val = obj.pInitializationFcn;
        end
        
        function set.SensorTransformFcn(obj,func)
            validateattributes(func,{'function_handle','char','string'},{'nonempty'},'trackingSensorConfiguration','SensorTransformFcn');
            if isa(func,'function_handle')
                obj.pTransformFcn = func;
            else
                obj.pTransformFcn = str2func(func);
            end
        end
        
        function val = get.SensorTransformFcn(obj)
            val = obj.pTransformFcn;
        end
        
        function set.DetectionProbability(obj,val)
            if coder.internal.is_defined(obj.pMinDetectionProbability)
                validateattributes(val,{'double','single'},...
                    {'real','finite','nonsparse','scalar','<',1,'>',obj.MinDetectionProbability},...
                    'trackingSensorConfiguration','DetectionProbability');
            else
                validateattributes(val,{'double','single'},...
                    {'real','finite','nonsparse','scalar','positive','<',1},...
                    'trackingSensorConfiguration','DetectionProbability');
            end
            obj.pDetectionProbability = cast(val,obj.pDataType);
        end
        
        function val = get.DetectionProbability(obj)
            if ~coder.internal.is_defined(obj.pDetectionProbability)
                % Default
                obj.pDetectionProbability = cast(obj.constProbabilityDetection,obj.pDataType);
            end
            val = obj.pDetectionProbability;
        end
        
        function set.ClutterDensity(obj,val)
            validateattributes(val,{'double','single'},...
                {'real','finite','nonsparse','scalar','positive'},...
                'trackingSensorConfiguration','ClutterDensity');
            obj.pClutterDensity = cast(val,obj.pDataType);
        end
        
        function val = get.ClutterDensity(obj)
            if ~coder.internal.is_defined(obj.pClutterDensity)
                % Default
                obj.pClutterDensity = cast(obj.constClutterDensity,obj.pDataType);
            end
            val = obj.pClutterDensity;
        end
        
        function set.MinDetectionProbability(obj,val)
            if coder.internal.is_defined(obj.pDetectionProbability)
                validateattributes(val,{'double','single'},...
                    {'real','finite','nonsparse','scalar','positive','<',obj.DetectionProbability},...
                    'trackingSensorConfiguration','MinDetectionProbability');
            else
                validateattributes(val,{'double','single'},...
                    {'real','finite','nonsparse','scalar','<',1,'positive'},...
                    'trackingSensorConfiguration','MinDetectionProbability');
            end
            obj.pMinDetectionProbability = cast(val,obj.pDataType);
        end
        
        function val = get.MinDetectionProbability(obj)
            if ~coder.internal.is_defined(obj.pMinDetectionProbability)
                % Default
                obj.pMinDetectionProbability = cast(obj.constMinProbDetection,obj.pDataType);
            end
            val = obj.pMinDetectionProbability;
        end
        
        function set.MaxNumDetsPerObject(obj,val)
            validateattributes(val,{'numeric'},{'nonsparse','scalar','positive'},...
                'trackingSensorConfiguration','MaxNumDetsPerObject');
            obj.pMaxNumDetsPerObject = cast(val,obj.pDataType);
        end
        
        function val = get.MaxNumDetsPerObject(obj)
            if ~coder.internal.is_defined(obj.pMaxNumDetsPerObject)
                obj.pMaxNumDetsPerObject = cast(inf,obj.pDataType);
            end
            val = obj.pMaxNumDetsPerObject;
        end
    end
    
    methods (Access = {?trackingSensorConfiguration,?matlab.unittest.TestCase})
        function validateSensorTransformFcn(obj,sampleState)
            % Set valid SensorTransformFcn if not provided yet.
            if ~coder.internal.is_defined(obj.pTransformFcn) && coder.internal.is_defined(obj.pInitializationFcn)
                f = obj.pInitializationFcn();
                if isprop(f,'MeasurementFcn')
                    obj.pTransformFcn = f.MeasurementFcn;
                end
            end

            if ~obj.pIsTransformValid || ~coder.internal.is_defined(obj.pHasSourcesOutput)
                % Call with three states to make sure the function is
                % vectorized.
                if nargout(obj.pTransformFcn) == 2
                    [detSample,expDetSample] = obj.pTransformFcn([sampleState sampleState sampleState],obj.pTransformParameters);
                    validateattributes(expDetSample, {'numeric'},{'real','nonsparse'},'SensorTransformFcn','Nexp');
                    obj.pHasExpectedDetsOutput = ~(size(expDetSample,2) == 2);
                    if obj.pHasExpectedDetsOutput
                        validateattributes(expDetSample, {'numeric'},{'finite','numel',3},'SensorTransformFcn','Nexp');
                    end
                else
                    detSample = obj.pTransformFcn([sampleState sampleState sampleState],obj.pTransformParameters);
                    obj.pHasExpectedDetsOutput = false;
                end
                validateattributes(detSample,{'single','double'},{'3d','real','finite','nonsparse'},'SensorTransformFcn','detStates');
                sizeDetSample = size(detSample);
                coder.internal.assert(all(sizeDetSample(1:2) == [obj.pNumParameters 3]),'fusion:trackingSensorConfiguration:InvalidTransformFcnSize');
                obj.pHasSourcesOutput = numel(sizeDetSample) == 3;
                if obj.pHasSourcesOutput
                    coder.internal.assert(sizeDetSample(3) > 1, 'fusion:trackingSensorConfiguration:invalidSourceSize');
                end
                obj.pIsTransformValid = true;
            end
        end

        function checkValidInput(obj,varargin)
            InputArgsNames = {varargin{2:2:numel(varargin)}};
            if coder.target('MATLAB')
                isFieldAMember = ismember(InputArgsNames,{'IsValidTime', ...
                    'SensorTransformFcn','FilterInitializationFcn',});
            else
                isFieldAMember = false(numel(InputArgsNames),1);
                for i = 1:numel(InputArgsNames)
                    isFieldAMember(i) = any(strcmpi(InputArgsNames{i},{'IsValidTime', ...
                        'SensorTransformFcn','FilterInitializationFcn'}));
                end
            end
            if ~all(isFieldAMember)
                firstInvalidProp = {InputArgsNames{~isFieldAMember}};
                coder.internal.assert(false,'fusion:trackingSensorConfiguration:UnsetProp',firstInvalidProp{1});
            end
        end

        function syncMeasurementParameters(obj,miniObject)
            if isscalar(miniObject.MeasurementParameters)
                obj.SensorTransformParameters(1) = miniObject.MeasurementParameters;
            else
                obj.SensorTransformParameters =  miniObject.MeasurementParameters;
            end
            % The radar sensor model may output the Frame in the
            % configuration as Rectangular. The trackingSensorConfiguration
            % needs the Frame as Spherical to convert the track state to
            % sensor spherical coordinates. Therefore, change Frame to
            % Spherical.
            if isfield(obj.SensorTransformParameters,'Frame')
                % The data type of the frame must be preserved for code
                % generation. The radar sensor models can output one of
                % these coordinate frames. The Enums are used in Simulink.
                % The character arrays are used in MATLAB.
                frame = obj.SensorTransformParameters(1).Frame;
                if isa(frame,'drivingCoordinateFrameType')
                    frame = drivingCoordinateFrameType.Spherical;
                elseif isa(frame,'fusionCoordinateFrameType') 
                    frame = fusionCoordinateFrameType.Spherical;
                elseif ischar(frame)
                    frame = 'Spherical';
                elseif isstring(frame)
                    frame = "Spherical";
                end
                obj.SensorTransformParameters(1).Frame = frame;
            end
        end

        function syncSensorLimits(obj,miniObject)
            % get valid sensor limits parameters
            [hasAz,hasEl,hasR,hasRR] = getValidParameters(obj,miniObject);

            % Compute index
            azIdx = 1;
            elIdx = azIdx + hasEl;
            rIdx  = elIdx + hasR;
            rrIdx = rIdx + hasRR;
            
            sensorLimits = cast(repmat([0 inf],sum([hasAz,hasEl,hasR,hasRR]),1),class(obj.SensorLimits));
            fov = miniObject.FieldOfView(~isnan(miniObject.FieldOfView));
            if isvector(fov)
                fovLimits = [-1/2 1/2].*fov(:);
            else
                fovLimits = fov;
            end

            if hasAz
                sensorLimits(azIdx,:) = fovLimits(1,:);
            end

            if hasEl
                sensorLimits(elIdx,:) = fovLimits(2,:);
            end

            if hasR
                if isfield(miniObject,'RangeLimits')
                    sensorLimits(rIdx,:)  = miniObject.RangeLimits;
                end
            end

            if hasRR
                if isfield(miniObject,'RangeRateLimits')
                    sensorLimits(rrIdx,:) = miniObject.RangeRateLimits;
                end
            end
            
            obj.SensorLimits = sensorLimits(1:obj.pNumParameters,:);
        end

        function [hasAz,hasEl,hasR,hasRR] = getValidParameters(obj,miniObject)
            % numValidParams will contain the logical flags for HasAzimuth,
            % HasElevation, HasRange and HasRangeRate in this order.
            hasAz = false;
            hasEl = false;
            hasR = false;
            hasRR = false;
            if isfield(miniObject.MeasurementParameters,'HasAzimuth')
                hasAz = miniObject.MeasurementParameters(1).HasAzimuth;
            else
                hasAz = true;
            end

            if isfield(miniObject.MeasurementParameters,'HasElevation')
                hasEl = miniObject.MeasurementParameters(1).HasElevation;
            else
                hasEl = true;
            end

            if isfield(miniObject.MeasurementParameters,'HasRange')
                hasR = miniObject.MeasurementParameters(1).HasRange;
            else
                hasR = true;
            end

            if isfield(miniObject.MeasurementParameters,'HasRange')&& ...
                    isfield(miniObject.MeasurementParameters,'HasVelocity')
                hasRR = (miniObject.MeasurementParameters(1).HasRange && ...
                    miniObject.MeasurementParameters(1).HasVelocity);
            else
                if ~isfield(miniObject.MeasurementParameters,'HasRange')&& ...
                        ~isfield(miniObject.MeasurementParameters,'HasVelocity')
                    hasRR = true;
                end
            end
        end
    end
    
    methods (Access = protected)
        function allocateMemory(obj,varargin)
            % Is SensorIndex specified as property or first variable
            senIndexID = fusion.internal.findProp('SensorIndex',varargin{:});
            if ~(senIndexID <= numel(varargin) - 1)
                coder.internal.assert(mod(nargin,2) == 0,...
                    'fusion:trackingSensorConfiguration:PropertyNeededOnConstruction','SensorIndex');
                sensorIndex = varargin{1};
            else
                sensorIndex = varargin{senIndexID+1};
            end
            validateattributes(sensorIndex,{'numeric'},...
                {'scalar','nonsparse','integer','positive'},...
                'trackingSensorConfiguration','SensorIndex',1);

            obj.SensorIndex = sensorIndex;

            % Two main things to allocate memory for
            % 1. Limits and Resolution
            % 2. Transform Parameters;
           
            % Is SensorLimits or SensorResolution specified as input
            senLimitsID = fusion.internal.findProp('SensorLimits',varargin{:});
            senResID = fusion.internal.findProp('SensorResolution',varargin{:});

            if senLimitsID <= numel(varargin) - 1
                limits = varargin{senLimitsID + 1};
                validateattributes(limits,{'single','double'},...
                    {'real','nonsparse','nonnan','2d','ncols',2},...
                    'trackingSensorConfiguration','SensorLimits');
                numParams = size(limits,1);
                dataType = class(limits);
            elseif senResID <= numel(varargin) - 1
                res = varargin{senResID + 1};
                validateattributes(res,{'single','double'},...
                    {'real','nonsparse','vector'},...
                    'trackingSensorConfiguration','SensorResolution');
                numParams = numel(res);
                dataType = class(res);
            else % Defaults
                numParams = 3;
                dataType = 'double';
            end
            obj.pNumParameters = numParams;
            obj.pDataType = dataType;
            obj.pSensorResolution = 2*ones(numParams,1,dataType);
            obj.pSensorLimits = bsxfun(@times,[-1 1],ones(numParams,2,dataType));

            % SensorTransformParameters specified as input
            paramsID = fusion.internal.findProp('SensorTransformParameters',varargin{:});
            if paramsID <= numel(varargin) - 1
                params = varargin{paramsID + 1};
            else
                params = getDefaultParams(obj,dataType);
            end
            obj.pTransformParameters = params;
        end

        function propGroup = getPropertyGroups(~)
            indexList = {'SensorIndex','IsValidTime'};

            propGroup = matlab.mixin.util.PropertyGroup(indexList);

            capabilities = {'SensorLimits','SensorResolution','SensorTransformFcn','SensorTransformParameters'};
            propGroup(2) = matlab.mixin.util.PropertyGroup(capabilities);

            initFcn = {'FilterInitializationFcn','MaxNumDetsPerObject'};
            propGroup(3) = matlab.mixin.util.PropertyGroup(initFcn);

            customGroup = {'ClutterDensity','DetectionProbability','MinDetectionProbability'};
            propGroup(4) = matlab.mixin.util.PropertyGroup(customGroup);
        end
    end
    methods (Access = {?trackingSensorConfiguration,?matlab.unittest.TestCase})
        function args = getDefaultArgs(~)
            args = {'SensorLimits',[-10 10;-2.5 2.5;0 1000],...
                'SensorResolution',[4;2;10],...
                'FilterInitializationFcn',@initcvggiwphd,...
                'SensorTransformFcn',@cvmeas};
        end

        function measParam = getDefaultParams(~,dataType)
            measParam = struct(...
                'Frame',{'Spherical';'Rectangular'},...
                'OriginPosition',{zeros(3,1,dataType);zeros(3,1,dataType)},...
                'OriginVelocity',{zeros(3,1,dataType);zeros(3,1,dataType)},...
                'Orientation',{eye(3,dataType);eye(3,dataType)},...
                'IsParentToChild',{false;false},...
                'HasAzimuth',{true;true},...
                'HasElevation',{true;true},...
                'HasRange',{true;true},...
                'HasVelocity',{false;true}...
                );
        end
    end
    
    methods(Static,Hidden)
        function props = matlabCodegenNontunableProperties(~)
            % Let the coder know about non-tunable parameters so that
            % it can generate more efficient code.
            props = {'pNumParameters','pDataType','pInitializationFcn','pTransformFcn','pHasSourcesOutput','pHasExpectedDetsOutput'};
        end
    end
    
end
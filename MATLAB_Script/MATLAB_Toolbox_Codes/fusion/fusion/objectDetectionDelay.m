classdef objectDetectionDelay < matlab.System
    %objectDetectionDelay  Simulate out-of-sequence object detections
    % HDELAY = objectDetectionDelay() creates an object that allows you to
    % add a delay before detections are passed from the sensor to the
    % tracker. Use the objectDetectionDelay object to simulate
    % out-of-sequence measurements, which are measurements from a sensor
    % that are delayed due to the sensor processing time or due to random
    % network lags.
    %
    % HDELAY = objectDetectionDelay('Name', value) creates an
    % objectDetectionDelay object by specifying its properties as
    % name-value pair arguments. Unspecified properties have default
    % values. See the list of properties below.
    %
    % Step method syntax:
    %   DELAYEDDETS = step(HDELAY, DETS, CURRENTTIME) stores the current
    %   detections, DETS, reported by the sensor and returns the delayed
    %   detections, DELAYEDDETS. Use this syntax when DelaySource is set to
    %   'Property'.
    %
    %   DELAYEDDETS = step(HDELAY, DETS, CURRENTTIME, DELAY), allows you to
    %   specify the time delay, DELAY, by which the detections should be
    %   delayed. Use this syntax when DelaySource is set to 'Input'.
    %
    %   The delayed detections, DELAYEDDETS, are detections that satisfy
    %   the following inequality:
    %
    %      detection.Time + delay <= currentTime
    %
    %   DETS can be a cell array or an array of objectDetection objects or
    %   the corresponding array of structure.
    %   CURRENTTIME is a real scalar corresponding to the current time of
    %   the system, for example, the SimulationTime property of the
    %   scenario.
    %   If specified, DELAY is a real nonnegative scalar or N-element
    %   array, where N is the number of elements in DETS.
    %   DELAYEDDETS is an array of the same type as DETS.
    %
    %   [..., USEDCAPACITY, INFO] = step(...), additionally, allows you
    %   to get USEDCAPACITY, the number of objects currently stored by the
    %   object, and INFO, a detailed information about the stored
    %   detections. INFO is a struct that contains the following fields:
    %     DetectionTime - The timestamps of stored detections.
    %     Delay         - The time delay for each stored detection.
    %     DeliveryTime  - The time each detection will be delivered.
    %
    %   System objects may be called directly like a function instead of
    %   using the step method. For example, y = step(obj) and y = obj() are
    %   equivalent.
    %
    % objectDetectionDelay properties:
    %   SensorIndices     - A list of sensor indices to apply the time delay
    %   Capacity          - Maximum number of detections the object stores
    %   DelaySource       - Choose the source of delay
    %   DelayDistribution - Choose the type of delay to apply
    %   DelayParameters   - Parameters controlling the delay
    %
    % objectDetectionDelay methods:
    %   step     - Add time delay to detections
    %   clone    - Create a copy of the objectDetectionDelay object
    %   release  - Allow property value and input characteristics changes
    %   reset    - Reset the objectDetectionDelay buffer
    %   isLocked - Return the locked status (logical)
    %
    % Example:
    % % Create an objectDetectionDelay that delays detections from sensor 2.
    % delay = objectDetectionDelay("SensorIndices", 2);
    %
    % % Create two detections, one from each sensor.
    % det1 = objectDetection(0, [1;1;1], "SensorIndex", 1);
    % det2 = objectDetection(0, [2;2;2], "SensorIndex", 2);
    % dets = {det1; det2};
    % 
    % % Pass the detections to the delay object to add a 1 second delay
    % % to det2.
    % delayedDets = delay(dets, 0); 
    % disp(delayedDets{:})
    %
    % % Step the delay object again to get the delayed detection after 1
    % % second delay
    % delayedDets = delay({}, 1); 
    % disp(delayedDets{:});
    %
    % See also objectDetection

    % Copyright 2021 The MathWorks, Inc.
    %#codegen
    
    properties(Nontunable)
        %SensorIndices  A list of sensor indices to apply the time delay
        % Specify the sensor indices to which time delays apply as a vector
        % of positive integers. Time delay is applied to any detection with
        % SensorIndex that are listed in this property. To delay all the
        % detections, use the value "All".
        %
        % Default: "All"
        SensorIndices = "All"
    end

    properties (Nontunable)
        %Capacity  Specify the number of data entities to be delayed
        % Specify the number of data entities to be delayed as a positive
        % value. You can specify it as Inf, indicating variable-sized array
        % of data entities or provide a finite positive integer scalar. If
        % finite, the object can support non-dynamic memory allocation in
        % code generation, but will error out when the capacity is
        % exceeded.
        %
        % Default: Inf
        Capacity (1,1) {mustBeNumeric, mustBeReal, mustBeNonNan} = Inf
    end

    properties(Hidden,Constant)
        DelaySourceSet = matlab.system.StringSet({'Property','Input'});
        DelayDistributionSet = matlab.system.StringSet({'Constant','Uniform','Normal'});
    end
    
    properties(Nontunable)
        %DelaySource  Choose the source of delay
        % Choose the source of delay from {['Property'],'Input'}. If you
        % set the DelaySource to 'Property', you can set the
        % DelayDistribution and DelayParameters properties to control the
        % time delay applied to each detection.
        %
        % If you set the DelaySource to 'Input', you can provide the time
        % delay as an input to the step method.
        DelaySource = 'Property';

        %DelayDistribution Choose the type of delay to apply
        % Choose the type of delay to apply from
        % [{'Constant'},'Uniform','Normal']. Use the DelayParameters
        % property to define the parameters according to the choice:
        %   Constant - The time delay is constant.
        %   Uniform  - The time delay is uniformly distributed.
        %   Normal   - The time delay is normally distributed.
        %
        % Default = 'Constant'
        DelayDistribution = 'Constant'
    end

    properties(Dependent)
        %DelayParameters Define the parameters of the time delay
        % Define the parameters of the time delay based on your choice of
        % delay type:
        %   Constant - Define a nonnegative scalar.
        %   Uniform  - Define as a 2-element array, [minimum maximum]. Both
        %              values must be nonnegative and real.
        %   Normal   - Define as a 2-element array, [mean, sigma].Both
        %              values must be nonnegative and real.
        %
        % Default values depend on the choice of delay type:
        %   Constant - 1 second.
        %   Uniform  - [0 1] seconds.
        %   Normal   - [1 0.1] seconds.
        DelayParameters
    end

    properties(Hidden)
        DelayParametersConstant (1,1) {mustBeNumeric,mustBeReal,mustBeNonnegative} = 1
        DelayParametersUniform (1,2) {mustBeNumeric,mustBeReal,mustBeFinite,mustBeNonnegative} = [0 1]
        DelayParametersNormal (1,2) {mustBeNumeric,mustBeReal,mustBeFinite,mustBeNonnegative} = [1 0.1]
    end
    
    properties (Access = {?objectDetectionDelay, ?matlab.unittest.TestCase})
        cDataDelay
    end

    methods
        function obj = objectDetectionDelay(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
        
        function set.SensorIndices(obj, value)
            validateattributes(value,{'char','string','numeric'},{},mfilename,'SensorIndices');
            if ischar(value) || isstring(value)
                obj.SensorIndices = validatestring(value,{'all'},mfilename,'SensorIndices');
            else
                validateattributes(value,{'numeric'},{'real','positive','integer','vector'},...
                    mfilename,'SensorIndices');
                obj.SensorIndices = value(:)';
            end
        end

        function set.Capacity(obj, value)
            if isfinite(value)
                validateattributes(value,{'numeric'},{'positive','integer'},...
                    mfilename,'Capacity');
            end
            obj.Capacity = value;
        end

        function set.DelayParameters(obj, value)
            switch obj.DelayDistribution
                case 'Constant'
                    obj.DelayParametersConstant = value;
                case 'Uniform'
                    obj.DelayParametersUniform = value;
                case 'Normal'
                    obj.DelayParametersNormal = value;
                otherwise
                    % Do nothing, property is inactive
                    assert(false)
            end

            % Update the data delay object
            if coder.target('MATLAB')
                setDelayParameters(obj);
            else
                coder.internal.defer_inference('setDelayParameters',obj);
            end
            
        end

        function setDelayParameters(obj)
            if isLocked(obj) && coder.internal.is_defined(obj.cDataDelay)
                obj.cDataDelay.DelayParameters = obj.DelayParameters;
            end
        end

        function value = get.DelayParameters(obj)
            switch obj.DelayDistribution
                case 'Constant'
                    value = obj.DelayParametersConstant;
                case 'Uniform'
                    value = obj.DelayParametersUniform;
                case 'Normal'
                    value = obj.DelayParametersNormal;
                otherwise
                    % This branch will never be hit
                    assert(false)
            end
        end
    end

    methods(Access = protected)
        function varargout = stepImpl(obj, newData, time, varargin)
            [varargout{1:nargout}] = step(obj.cDataDelay, newData, time, varargin{:});
        end

        function setupImpl(obj, newData, ~, varargin)
            % Perform one-time calculations, such as computing constants

            % Error out if newData is empty on first call to step
            coder.internal.errorIf(isempty(newData),'fusion:objectDetectionDelay:emptyOnSetup');

            obj.cDataDelay = fusion.internal.DataDelay(...
                'DelayAll', ~isnumeric(obj.SensorIndices), ...
                'SensorIndices', obj.SensorIndices, ...
                'Capacity', obj.Capacity, ...
                'DelaySource', obj.DelaySource, ...
                'DelayDistribution', obj.DelayDistribution, ...
                'DelayParameters', obj.DelayParameters...
                );
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            reset(obj.cDataDelay);
        end

        function releaseImpl(obj)
            release(obj.cDataDelay);
        end

        function flag = isInactivePropertyImpl(obj,prop)
            % Return false if property is visible based on object 
            % configuration, for the command line and System block dialog
            if strcmpi(obj.DelaySource, 'Input')
                flag = (strcmpi(prop,'DelayParameters') || strcmpi(prop,'DelayDistribution'));
            else
                flag = false;
            end
        end

        function validateInputsImpl(obj,newData,time,varargin)
            % Validate inputs to the step method at initialization
            
            % Validate time
            validateattributes(time,{'single','double'},...
                {'real','finite','nonnegative','scalar'},...
                mfilename,'currentTime');

            % Validate time delay input
            if strcmpi(obj.DelaySource, 'Input')
                validateattributes(varargin{1}, {'single','double'},...
                    {'real','finite','nonnegative'},...
                    mfilename, 'timeDelay');
                if ~isscalar(varargin{1}) && ~isempty(newData)
                    coder.internal.assert(numel(newData) == numel(varargin{1}),...
                        'fusion:objectDetectionDelay:numDelayMustMatchNumData',...
                        'DELAY','DETS');
                end
            end

            % Validate object detections input
            if isempty(newData)
                return;
            end

            if iscell(newData)
                validateattributes(newData{1},{'objectDetection','struct'},...
                    {},mfilename,'newData{:}');
                if isstruct(newData{1})
                    validateStructIsExpectedType(obj,newData{1});
                else
                    validateattributes(newData{1},{'objectDetection'},{},mfilename,'Data',1);
                end
            elseif isstruct(newData)
                validateStructIsExpectedType(obj,newData(1));
            else
                validateattributes(newData(1),{'objectDetection'},{},mfilename,'Data',1);
            end
        end

        function validateStructIsExpectedType(~,data)
            expectedFieldNames = {'Time','SensorIndex'};
            
            for i = 1:numel(expectedFieldNames)
                coder.internal.errorIf(~isfield(data,expectedFieldNames{i}),...
                    'fusion:objectDetectionDelay:incompatibleStruct','objectDetection');
            end
        end

        function flag = isInputSizeMutableImpl(~,index)
            % Return false if input size cannot change
            % between calls to the System object
            if index == 2
                flag = false;
            else
                flag = true;
            end
        end

        function flag = isInputDataTypeMutableImpl(~,~)
            % Return false if input data type cannot change
            % between calls to the System object
            flag = false;
        end

        function num = getNumInputsImpl(obj)
            % Define total number of inputs for system with optional inputs
            num = 2;
            if strcmpi(obj.DelaySource,'Input')
                num = num + 1;
            end
        end

        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s

            % Set private and protected properties
            obj.cDataDelay = fusion.internal.DataDelay.loadobj(s.cDataDelay);

            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end

        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj

            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);

            % Set private and protected properties
            s.cDataDelay = saveobj(obj.cDataDelay);
        end
    end

    methods(Static, Hidden)
        function flag = isAllowedInSystemBlock
            flag = false;
        end
    end
end
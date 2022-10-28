classdef objectDetectionDelay< matlab.System
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
%   <a href="matlab:help matlab.System/reset   ">reset</a>    - Reset the objectDetectionDelay buffer
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

    methods
        function out=objectDetectionDelay
            % Support name-value pair arguments when constructing object
        end

        function out=getNumInputsImpl(~) %#ok<STOUT>
            % Define total number of inputs for system with optional inputs
        end

        function out=isInactivePropertyImpl(~) %#ok<STOUT>
            % Return false if property is visible based on object 
            % configuration, for the command line and System block dialog
        end

        function out=isInputDataTypeMutableImpl(~) %#ok<STOUT>
            % Return false if input data type cannot change
            % between calls to the System object
        end

        function out=isInputSizeMutableImpl(~) %#ok<STOUT>
            % Return false if input size cannot change
            % between calls to the System object
        end

        function out=loadObjectImpl(~) %#ok<STOUT>
            % Set properties in object obj to values in structure s
        end

        function out=releaseImpl(~) %#ok<STOUT>
        end

        function out=resetImpl(~) %#ok<STOUT>
            % Initialize / reset discrete-state properties
        end

        function out=saveObjectImpl(~) %#ok<STOUT>
            % Set properties in structure s to values in object obj
        end

        function out=setDelayParameters(~) %#ok<STOUT>
        end

        function out=setupImpl(~) %#ok<STOUT>
            % Perform one-time calculations, such as computing constants
        end

        function out=stepImpl(~) %#ok<STOUT>
        end

        function out=validateInputsImpl(~) %#ok<STOUT>
            % Validate inputs to the step method at initialization
        end

        function out=validateStructIsExpectedType(~) %#ok<STOUT>
        end

    end
    properties
        %Capacity  Specify the number of data entities to be delayed
        % Specify the number of data entities to be delayed as a positive
        % value. You can specify it as Inf, indicating variable-sized array
        % of data entities or provide a finite positive integer scalar. If
        % finite, the object can support non-dynamic memory allocation in
        % code generation, but will error out when the capacity is
        % exceeded.
        %
        % Default: Inf
        Capacity;

        %DelayDistribution Choose the type of delay to apply
        % Choose the type of delay to apply from
        % [{'Constant'},'Uniform','Normal']. Use the DelayParameters
        % property to define the parameters according to the choice:
        %   Constant - The time delay is constant.
        %   Uniform  - The time delay is uniformly distributed.
        %   Normal   - The time delay is normally distributed.
        %
        % Default = 'Constant'
        DelayDistribution;

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
        DelayParameters;

        %DelaySource  Choose the source of delay
        % Choose the source of delay from {['Property'],'Input'}. If you
        % set the DelaySource to 'Property', you can set the
        % DelayDistribution and DelayParameters properties to control the
        % time delay applied to each detection.
        %
        % If you set the DelaySource to 'Input', you can provide the time
        % delay as an input to the step method.
        DelaySource;

        %SensorIndices  A list of sensor indices to apply the time delay
        % Specify the sensor indices to which time delays apply as a vector
        % of positive integers. Time delay is applied to any detection with
        % SensorIndex that are listed in this property. To delay all the
        % detections, use the value "All".
        %
        % Default: "All"
        SensorIndices;

    end
end

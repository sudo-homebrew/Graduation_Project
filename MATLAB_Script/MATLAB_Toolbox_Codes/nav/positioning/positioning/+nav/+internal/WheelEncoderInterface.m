classdef (Hidden) WheelEncoderInterface < matlab.System
%WHEELENCODERINTERFACE Interface class for wheelEncoderUnicycle,
%   wheelEncoderDifferentialDrive, wheelEncoderBicycle, and
%   wheelEncoderAckermann.
%
%   This class is used to calculate wheel ticks based on ideal pose input
%   and noise properties.
%
%   This class is for internal use only. It may be removed in the future.

%   Copyright 2020 The MathWorks, Inc.

%#codegen

    properties
        % SampleRate Sampling rate (Hz)
        % Specify the sampling rate of the wheel encoder as a positive
        % scalar. The default value is 100.
        SampleRate = 100;
    end
    
    properties (Nontunable)
        %RandomStream Random number source
        %   Specify the source of the random number stream as one of the
        %   following:
        %
        %   'Global stream' - Random numbers are generated using the
        %   current global random number stream.
        %   'mt19937ar with seed' - Random numbers are generated using the
        %   mt19937ar algorithm with the seed specified by the Seed
        %   property.
        %
        %   The default value is 'Global stream'.
        RandomStream = 'Global stream';
        %Seed Initial seed
        %   Specify the initial seed of an mt19937ar random number
        %   generator algorithm as a real, nonnegative integer scalar. This
        %   property applies when you set the RandomStream property to
        %   'mt19937ar with seed'. The default value is 67.
        Seed(1,1) uint32 {mustBeReal} = uint32(67);
    end
    
    properties (Constant, Hidden)
        RandomStreamSet = matlab.system.StringSet({...
            'Global stream', ...
            'mt19937ar with seed'});
    end
    
    properties (Access = private)
        % Random stream object (used in 'mt19937ar with seed' mode).
        pStream;
        % Random number generator state.
        pStreamState;
    end
    
    properties (Abstract)
        TicksPerRevolution;
        WheelRadius;
        WheelRadiusBias;
        WheelPositionAccuracy;
        SlipRatio;
    end
    
    properties (Access = protected)
        pRemAngle;
        pDeltaTime;
        % Used to store input for type casting.
        pInputPrototype;
    end
    
    properties (Access = protected, Constant)
        TICKS_PER_REVOLUTION_DEFAULT = 2048;
        WHEEL_RADIUS_DEFAULT = 0.35;
        WHEEL_RADIUS_BIAS_DEFAULT = 0;
        WHEEL_POSITION_ACCURACY_DEFAULT = 0;
        SLIP_RATIO_DEFAULT = 0;
    end
    
    methods
        function obj = WheelEncoderInterface(varargin)
            setProperties(obj, nargin, varargin{:});
        end

        function set.SampleRate(obj, val)
            validateattributes(val, {'single', 'double'}, ...
                {'real', 'scalar', 'positive', 'finite'}, '', 'SampleRate');
            obj.SampleRate = val;
        end
    end
    
    methods (Access = protected, Abstract)
        wheelDistance = vehicleSpeedToWheelDistance(obj, speed, omega);
    end
    
    methods (Access = protected, Static, Abstract)
        numWheels = getNumWheels;
    end
    
    methods (Access = protected)
        function setupRandomStream(obj)
            % Setup Random Stream object if required.
            if strcmp(obj.RandomStream, 'mt19937ar with seed')
                if isempty(coder.target)
                    obj.pStream = RandStream('mt19937ar', 'seed', obj.Seed);
                else
                    obj.pStream = coder.internal.RandStream('mt19937ar', 'seed', obj.Seed);
                end
            end
        end
        
        function setupImpl(obj, vel, ~, ~)
            setupRandomStream(obj);
            % Store input for type casting.
            obj.pInputPrototype = vel;
        end
        
        function ticks = stepImpl(obj, vel, angvel, orientIn)
            % Convert input orientation to a quaternion, if needed.
            if isa(orientIn, 'quaternion')
                orient = orientIn;
            else
                orient = quaternion(orientIn, 'rotmat', 'frame');
            end
            
            % Convert input to vehicle speed and angular velocity around
            % the z-axis of the vehicle.
            velBody = rotateframe(orient, vel);
            angvelBody = rotateframe(orient, angvel);
            vehicleSpeed = velBody(:,1);
            omega = angvelBody(:,3);
            
            % Extract vehicle and encoder parameters.
            numWheels = obj.getNumWheels;
            wheelRadii = obj.WheelRadius + obj.WheelRadiusBias;
            wheelPosAccuracy = obj.WheelPositionAccuracy;
            slip = obj.SlipRatio;
            % Angular step of encoder.
            ticks2angle = 2*pi ./ obj.TicksPerRevolution;
            
            wheelDistance = vehicleSpeedToWheelDistance(obj, ...
                vehicleSpeed, omega);
            
            % Add slippage effects to vehicle speed.
            wheelDistance = wheelDistance .* (slip + 1);
            
            % Convert distance traveled (m) to angular distance traveled
            % (rad).
            wheelAngleDistance = wheelDistance ./ wheelRadii;
            
            numSamples = numel(vehicleSpeed);
            remAngle = obj.pRemAngle;
            ticks = zeros(numSamples, numWheels);
            % Generate and transpose random numbers to keep generation
            % consistent for vectorized input.
            randNums = stepRandomStream(obj, numWheels, numSamples).';
            for i = 1:numSamples
                angDist = wheelAngleDistance(i,:) + remAngle;
                continuousTicks = (angDist + wheelPosAccuracy ...
                    .* randNums(i,:)) ./ ticks2angle;
                % Add additional tick for zero-crossing.
                ticks(i,:) = fix(continuousTicks) + (abs(sign(remAngle) - sign(continuousTicks)) == 2);
                remAngle = (continuousTicks - ticks(i,:)) .* ticks2angle;
            end
            
            obj.pRemAngle = remAngle;
        end
        
        function noise = stepRandomStream(obj, numSamples, numChans)
            % Noise (random number) generation.
            if strcmp(obj.RandomStream, 'Global stream')
                noise = randn(numSamples, numChans, 'like', ...
                    obj.pInputPrototype);
            else
                noise = randn(obj.pStream, numSamples, numChans, ...
                    class(obj.pInputPrototype));
            end
        end
        
        function resetImpl(obj)
            obj.pRemAngle = zeros(1, obj.getNumWheels);
            obj.pDeltaTime = 1/obj.SampleRate;
            resetRandomStream(obj);
        end
        
        function resetRandomStream(obj)
            if strcmp(obj.RandomStream, 'mt19937ar with seed')
                obj.pStream.reset;
            end
        end
        
        function processTunedPropertiesImpl(obj)
            if isChangedProperty(obj, 'SampleRate')
                obj.pDeltaTime = 1/obj.SampleRate;
            end
        end
        
        function validateInputsImpl(~, vel, angvel, orient)
            validateattributes(vel, {'single', 'double'}, ...
                {'real', '2d', 'ncols', 3});
            numSamples = size(vel, 1);
            validateattributes(angvel, {'single', 'double'}, ...
                {'real', '2d', 'nrows', numSamples, 'ncols', 3});
            if (nargin == 4)
                if isa(orient, 'quaternion')
                    validateattributes(orient, {'quaternion'}, ...
                        {'nrows', numSamples, 'ncols', 1, '2d'});
                else
                    validateattributes(orient, {'single', 'double'}, ...
                        {'real', '3d', 'size', [3 3 numSamples]});
                end
            end
        end

        function s = saveObjectImpl(obj)
            % Save public properties.
            s = saveObjectImpl@matlab.System(obj);
            
            % Save private properties.
            if isLocked(obj)
                s.pRemAngle = obj.pRemAngle;
                s.pDeltaTime = obj.pDeltaTime;
                s.pInputPrototype = obj.pInputPrototype;
                
                if strcmp(obj.RandomStream, 'mt19937ar with seed')
                    if ~isempty(obj.pStream)
                        s.pStreamState = obj.pStream.State;
                    end
                end
            end
        end
        
        function loadObjectImpl(obj, s, wasLocked)
            % Load public properties.
            loadObjectImpl@matlab.System(obj, s, wasLocked);
            
            % Load private properties.
            if wasLocked
                obj.pRemAngle = s.pRemAngle;
                obj.pDeltaTime = s.pDeltaTime;
                obj.pInputPrototype = s.pInputPrototype;
                
                if strcmp(s.RandomStream, 'mt19937ar with seed')
                    obj.pStream = RandStream('mt19937ar', ...
                        'seed', obj.Seed);
                    if ~isempty(s.pStreamState)
                        obj.pStream.State = s.pStreamState;
                    end
                end
            end
        end
        
        function flag = isInactivePropertyImpl(obj, prop)
            flag = false;
            if strcmp(prop, 'Seed')
                if strcmp(obj.RandomStream, 'Global stream')
                    flag = true;
                end
            end
        end
        
        function displayScalarObject(obj)
            displayScalarObjectWithUnits(obj);
        end
    end

    methods (Static, Access = protected)
        function validateTicksPerRevolution(val, numWheels)
            validateattributes(val, {'single', 'double'}, ...
                {'real', 'integer', 'positive'}, '', 'TicksPerRevolution');
            nav.internal.WheelEncoderInterface.validateSize(val, ...
                numWheels, 'TicksPerRevolution');
        end
        function validateWheelRadius(val, numWheels)
            validateattributes(val, {'single', 'double'}, ...
                {'real', 'positive', 'finite'}, '', 'WheelRadius');
            nav.internal.WheelEncoderInterface.validateSize(val, ...
                numWheels, 'WheelRadius');
        end
        function validateWheelRadiusBias(val, numWheels)
            validateattributes(val, {'single', 'double'}, ...
                {'real', 'finite'}, '', 'WheelRadiusBias');
            nav.internal.WheelEncoderInterface.validateSize(val, ...
                numWheels, 'WheelRadiusBias');
        end
        function validateWheelPositionAccuracy(val, numWheels)
            validateattributes(val, {'single', 'double'}, ...
                {'real', 'nonnegative', 'finite'}, '', 'WheelPositionAccuracy');
            nav.internal.WheelEncoderInterface.validateSize(val, ...
                numWheels, 'WheelPositionAccuracy');
        end
        function validateSlipRatio(val, numWheels)
            validateattributes(val, {'single', 'double'}, ...
                {'real', '>=', -1, 'finite'}, '', 'SlipRatio');
            nav.internal.WheelEncoderInterface.validateSize(val, ...
                numWheels, 'SlipRatio');
        end
        function validateTrackWidth(val, numTracks)
            validateattributes(val, {'single', 'double'}, ...
                {'real', 'positive', 'finite'}, '', 'TrackWidth');
            nav.internal.WheelEncoderInterface.validateSize(val, ...
                numTracks, 'TrackWidth');
        end
        function validateTrackWidthBias(val, numTracks)
            validateattributes(val, {'single', 'double'}, ...
                {'real', 'finite'}, '', 'TrackWidthBias');
            nav.internal.WheelEncoderInterface.validateSize(val, ...
                numTracks, 'TrackWidthBias');
        end
        function validateWheelBase(val)
            validateattributes(val, {'single', 'double'}, ...
                {'real', 'positive', 'finite', 'scalar'}, '', 'WheelBase');
        end
        function validateSize(val, numEls, prop)
            % Check that val is a scalar or a numEls-element row vector.
            cond = any([1 1] ~= size(val)) && any([1 numEls] ~= size(val));
            coder.internal.errorIf(cond, 'nav_positioning:WheelEncoderInterface:invalidSize', prop, numEls);
        end
    end
end

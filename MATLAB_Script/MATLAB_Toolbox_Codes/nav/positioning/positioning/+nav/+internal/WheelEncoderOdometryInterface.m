classdef (Hidden) WheelEncoderOdometryInterface < matlab.System
%WHEELENCODERODOMETRYINTERFACE Interface class for
%   wheelEncoderOdometryUnicycle, wheelEncoderOdometryBicycle,
%   wheelEncoderOdometryDifferentialDrive, and wheelEncoderOdometryAckermann
%
%   This class is used to compute odometry based on wheel encoder ticks
%
%   This class is for internal use only. It may be removed in the future.

%   Copyright 2020 The MathWorks, Inc.

%#codegen

    properties
        % SampleRate Sample rate of sensor
        % Sampling rate of the wheel encoder specified as a positive scalar
        % in hertz.
        % Default: 100
        SampleRate = 100;
    end
    properties (Nontunable)
        % InitialPose Initial pose of vehicle
        % Initial pose of the vehicle as a 1-by-3 array of [X, Y, Yaw].
        % X and Y specify the position in meters. Yaw specifies the
        % orientation in radians.
        % Default: [0, 0, 0]
        InitialPose = [0, 0, 0];
    end

    properties (Abstract)
        % Common property for all the objects
        TicksPerRevolution;
        WheelRadius;
    end

    properties (Access = private)
        % Last known pose of the vehicle
        pPrevPose = zeros(1,3);
    end

    properties (Access = protected, Constant)
        % Default value of TicksPerRevolution and WheelRadius
        TICKS_PER_REVOLUTION_DEFAULT = 2048;
        WHEEL_RADIUS_DEFAULT = 0.35;
    end

    methods
        function obj = WheelEncoderOdometryInterface(varargin)
            % Get the expected wheelEncoder<vehicle> object and
            % wheelEncoderOdometry<vehicle> object
            [encType, odomType] = obj.getEncOdomType;
            if nargin == 1
                % If there is single argument then it should be an object
                % of wheelEncoder<vehicle>
                validateattributes(varargin{1}, {encType},...
                    {}, odomType, '');
                setPropertiesFromEncoderObject(obj, varargin{1});
            elseif rem(nargin,2) == 0
                % If there is no argument then set default properties
                % If there is even number of arguments then it must be
                % NV pairs, set it accordingly
                setProperties(obj, nargin, varargin{:});
            else
                % If there is more than 1 argument then it should be
                % name-value pair only.
                % Combination of wheelEncoder<vehicle> system object and
                % N-V pair is not allowed
                coder.internal.error( ...
                    'nav_positioning:WheelEncoderOdometryInterface:incorrectObjectNVPair', ...
                    encType);
            end
        end
        % Set sample rate
        function set.SampleRate(obj, val)
            validateattributes(val, {'single', 'double'}, ...
                               {'real', 'scalar', 'positive', 'finite'},...
                               '', 'SampleRate');
            obj.SampleRate = val;
        end
        % Set initial pose of the vehicle
        function set.InitialPose(obj, val)
            validateattributes(val, {'single', 'double'}, ...
                               {'real', 'finite', 'size', [1,3]}, ...
                               '', 'InitialPose');
            obj.InitialPose = val;
        end
    end
    
    methods (Access = protected, Abstract, Static)
        % Get the expected type of wheelEncoder<vehicle> object which is
        % expected as input
        % Get the wheelEncoderOdometry<vehicle> for which odometry is to be
        % computed
        [encType, odomType] = getEncOdomType();
    end

    methods (Access = protected, Abstract)
        % Function to convert ticks and other input into velocities
        controllerInput = inputToVelOmega(obj, ticks, varargin);
        % Function to copy requisite properties from wheel encoder sensor
        % object into wheel encoder odometry object
        setPropertiesFromEncoderObject(obj, encObj);
    end

    methods (Access = protected)
        function [pose, velocity] = stepImpl(obj, ticks, varargin)
        % Convert wheel encoder ticks and additional input into
        % controlInput (linear velocity and angular velocity).
            if any(any(isnan(ticks))) || any(any(isinf(ticks)))
                % If there is any NaN or Inf in the input then let the code
                % run normally, it will give NaN as output
            else
                validateattributes(ticks, {'numeric'}, {'integer'});
            end
        
            if nargin == 2
                controlInput = inputToVelOmega(obj, ticks);
            elseif nargin == 3
                controlInput = inputToVelOmega(obj, ticks, varargin{1});
            end

            % Pass the control input to get the odometry of the vehicle
            [pose, velocity] = getOdom(obj, controlInput);

            % Update pPrevPose with the current pose of the vehicle.
            % The subsequent poses will be built over this.
            obj.pPrevPose = cast(pose(end, :), 'like', obj.pPrevPose);
        end

        function resetImpl(obj)
        % Reset the current pose of the vehicle to the initial pose
            obj.pPrevPose = obj.InitialPose;
        end

        function s = saveObjectImpl(obj)
        % Save public properties.
            s = saveObjectImpl@matlab.System(obj);

            % Save private properties.
            if isLocked(obj)
                s.pPrevPose = obj.pPrevPose;
            end
        end

        function loadObjectImpl(obj, s, wasLocked)
        % Load public properties.
            loadObjectImpl@matlab.System(obj, s, wasLocked);

            % Load private properties.
            if wasLocked
                obj.pPrevPose = s.pPrevPose;
            end
        end
    end

    methods (Access = private)
        % Integrate the control input, (V, W), to get the odometry of the
        % vehicle
        function [pose, velocity] = getOdom(obj, controlInput)
            pPrevPoseCasted = cast(obj.pPrevPose, 'like', controlInput);
            posX   = pPrevPoseCasted(1);
            posY   = pPrevPoseCasted(2);
            thetaZ = pPrevPoseCasted(3);
            
            freq   = obj.SampleRate;

            numSamples = size(controlInput, 1);
            pose       = zeros(numSamples, 3);
            velocity   = zeros(numSamples, 3);

            for i = 1:numSamples
                vel   = controlInput(i,1);
                omega = controlInput(i,2);
                sinTheta = sin(thetaZ);
                cosTheta = cos(thetaZ);

                if (omega == 0)
                    % If omega is 0, vehicle is moving on a straight path,
                    % get the distance traveled by the vehicle in a time
                    % step and add it to the previous pose of the vehicle
                    posX = posX + vel/freq * cosTheta;
                    posY = posY + vel/freq * sinTheta;
                else
                    % When omega is not 0, integrate and add it to the
                    % previous pose of the vehicle
                    velbyom = vel/omega;
                    thetaZ = thetaZ + omega/freq;
                    posX = posX + velbyom * (sin(thetaZ) - sinTheta);
                    posY = posY - velbyom * (cos(thetaZ) - cosTheta);
                end
                thetaZ = wrapToPiLocal(obj, thetaZ);

                pose(i, :)     = [posX, posY, thetaZ];
                velocity(i, :) = [vel*cos(thetaZ), vel*sin(thetaZ), omega];
            end
        end
    end

    methods (Static, Access = protected)
        % applicable for differential-drive or Ackermann type vehicles
        % Validate ticks per revolution
        function validateTicksPerRevolution(val, numWheels)
            validateattributes(val, {'single', 'double'}, ...
                               {'real', 'finite', 'integer', 'positive'}, ...
                               '', 'TicksPerRevolution');
            nav.internal.WheelEncoderOdometryInterface.validateSize(val, ...
                                          numWheels, 'TicksPerRevolution');
        end
        % Validate wheel radius
        function validateWheelRadius(val, numWheels)
            validateattributes(val, {'single', 'double'}, ...
                               {'real', 'positive', 'finite'}, ...
                               '', 'WheelRadius');
            nav.internal.WheelEncoderOdometryInterface.validateSize(val, ...
                                          numWheels, 'WheelRadius');
        end
        % Checking whether number of elements in TicksPerRevolution and
        % WheelRadius is as per the vehicle to not
        function validateSize(val, numEls, prop)
        % Check that val is a scalar or a numEls-element row vector.
            cond = any([1 1] ~= size(val)) && any([1 numEls] ~= size(val));
            coder.internal.errorIf(cond, ...
                                   'nav_positioning:WheelEncoderOdometryInterface:invalidSize', ...
                                   prop, numEls);
        end
        % Validate track width
        function validateTrackWidth(val)
            validateattributes(val, {'single', 'double'}, ...
                               {'real', 'positive', 'finite', 'scalar'},...
                               '', 'TrackWidth');
        end

        % Applicable for bicycle or Ackermann type vehicles
        % Validate wheel base
        function validateWheelBase(val)
            validateattributes(val, {'single', 'double'}, ...
                               {'real', 'positive', 'finite', 'scalar'},...
                               '', 'WheelBase');
        end
        % Validate number of columns in bicycle and ackermann input
        function validateTicksSize(val, numEls)
        % Check that val is a scalar or a numEls-element row vector.
            cond = any(numEls(1) ~= size(val, 2)) && ...
                   any(numEls(2) ~= size(val, 2));
            coder.internal.errorIf(cond, ...
                                   'nav_positioning:WheelEncoderOdometryInterface:invalidColumns', ...
                                   numEls(1), numEls(2));
        end
    end
    
    methods (Access = protected)
        function num = getNumInputsImpl(obj)
            if isa(obj, 'wheelEncoderOdometryUnicycle')
                num = 2;
            elseif isa(obj, 'wheelEncoderOdometryBicycle')
                num = 2;
            elseif isa(obj, 'wheelEncoderOdometryDifferentialDrive')
                num = 1;
            elseif isa(obj, 'wheelEncoderOdometryAckermann')
                num = 2;
            end
        end
    end
    
    methods (Access = private)
        function theta =  wrapToPiLocal(~, theta)
            if any(abs(theta) > pi, 'all')
                piVal = cast(pi,'like',theta);
                twoPiVal = cast(2*pi,'like',theta);

                lambda = theta + piVal;
                positiveInput = (lambda > 0);
                lambda = mod(lambda, twoPiVal);
                lambda((lambda == 0) & positiveInput) = twoPiVal;

                theta = lambda - piVal;
            end
        end
    end
end

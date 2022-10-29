classdef wheelEncoderOdometryAckermann < ...
        nav.internal.WheelEncoderOdometryInterface & fusion.internal.UnitDisplayer
%WHEELENCODERODOMETRYACKERMANN Compute Ackermann vehicle odometry using wheel encoder ticks
%   whlEncOdom = WHEELENCODERODOMETRYACKERMANN returns a System object,
%   whlEncOdom, that computes odometry of an Ackermann vehicle using wheel
%   encoder ticks and steering angle.
%
%   whlEncOdom = WHEELENCODERODOMETRYACKERMANN(ENCODER) returns a
%   WHEELENCODERODOMETRYACKERMANN System object based on the properties of
%   the wheelEncoderAckermann System object, ENCODER.
%
%   whlEncOdom = WHEELENCODERODOMETRYACKERMANN('Name1', Value1, ..., 'NameN', ValueN)
%   returns a WHEELENCODERODOMETRYACKERMANN System object with each
%   specified property name set to the specified value. You can specify
%   additional name-value pair arguments in any order as
%   (Name1,Value1,...,NameN, ValueN).
%
%   To compute Ackermann vehicle odometry from wheel encoder ticks and
%   steering angle:
%   1) Create the WHEELENCODERODOMETRYACKERMANN object and set its
%   properties.
%   2) Call the object with arguments, as if it were a function.
%
%   [POSE, VELOCITY] = whlEncOdom(TICKS, STEER) computes POSE (position
%   and orientation), and VELOCITY (linear velocity and angular velocity)
%   of the vehicle from wheel encoder ticks and steering angle.
%
%   Input Arguments:
%
%     TICKS       Number of wheel encoder ticks specified as a real finite
%                 N-by-2 array. N is the number of samples in the current
%                 frame. The order of wheel ticks is:
%                 [ticksBackLeft, ticksBackRight]
%
%     STEER       Steering Angle of the vehicle specified as a real finite
%                 N-element column vector in radians. N is the number of
%                 samples in the current frame.
%
%   Output Arguments:
%
%     POSE        Position and Orientation of the vehicle in the local
%                 navigation coordinate system returned as a real finite
%                 N-by-3 array of [X, Y, Yaw]. X and Y specify the position
%                 in meters. Yaw specifies the orientation in radians. N is
%                 the number of samples in the current frame.
%
%     VELOCITY    Linear and Angular velocity of the vehicle in the local
%                 navigation coordinate system returned as a real finite
%                 N-by-3 array of [velX, velY, yawRate]. velX and velY
%                 specify the linear velocity in meters per second. yawRate
%                 specifies the angular velocity in radians per second. N
%                 is the number of samples in the current frame.
%
%   Either single or double datatypes are supported for the inputs to
%   WHEELENCODERODOMETRYACKERMANN. Outputs have the same datatype as the
%   input.
%
%   WHEELENCODERODOMETRYACKERMANN methods:
%
%   step             - Compute Ackermann vehicle odometry using wheel
%                      encoder ticks
%   release          - Allow property value and input characteristics to
%                      change, and release WHEELENCODERODOMETRYACKERMANN
%                      resources
%   clone            - Create WHEELENCODERODOMETRYACKERMANN object with
%                      same property values
%   isLocked         - Display locked status (logical)
%   reset            - Reset the states of the WHEELENCODERODOMETRYACKERMANN
%
%   WHEELENCODERODOMETRYACKERMANN properties:
%
%   SampleRate                - Sample rate of sensor
%   TicksPerRevolution        - Number of encoder ticks per wheel revolution
%   WheelRadius               - Wheel radius
%   TrackWidth                - Distance between wheels on axle
%   WheelBase                 - Distance between front and rear axles
%   InitialPose               - Initial pose of vehicle
%
%       % EXAMPLE: Compute odometry from wheel ticks for Ackermann vehicle
%
%       % Create wheel encoder odometry object for Ackermann vehicle
%       whlEncOdom = wheelEncoderOdometryAckermann;
%
%       % Specify wheel encoder ticks and steering angle.
%       ticks = [5, 5; 2, 2];
%       steer = [0.2; 0.2];
%
%       % Compute odometry
%       [pose, vel] = whlEncOdom(ticks, steer);
%
%   See also WHEELENCODERACKERMANN, WHEELENCODERODOMETRYUNICYCLE,
%   WHEELENCODERODOMETRYBICYCLE, WHEELENCODERODOMETRYDIFFERENTIALDRIVE

%   Copyright 2020 The MathWorks, Inc.

%#codegen

    properties (Constant, Hidden)
        WheelRadiusUnits = 'm';
        WheelBaseUnits   = 'm';
        TrackWidthUnits   = 'm';
        SampleRateUnits  = 'Hz';
    end

    properties
        % TicksPerRevolution Number of encoder ticks per wheel revolution
        % Number of encoder ticks per wheel revolution specified either as
        % a positive scalar integer or two-element vector of positive
        % integers.
        % Default: [2048 2048]
        TicksPerRevolution = ...
            wheelEncoderOdometryAckermann.TICKS_PER_REVOLUTION_DEFAULT *...
            ones(1, wheelEncoderOdometryAckermann.NUM_WHEELS);

        % WheelRadius Wheel radius
        % Wheel radius specified either as a positive scalar or two-element
        % vector in meters.
        % Default: [0.35 0.35]
        WheelRadius = ...
            wheelEncoderOdometryAckermann.WHEEL_RADIUS_DEFAULT * ...
            ones(1, wheelEncoderOdometryAckermann.NUM_WHEELS);

        % TrackWidth Distance between wheels on axles
        % Distance between the wheels on the axle specified as a positive
        % scalar in meters.
        % Default: 1.572
        TrackWidth = 1.572;

        % WheelBase Distance between front and real axles
        % Distance between front and rear axles specified as a positive
        % scalar in meters.
        % Default: 2.818
        WheelBase = 2.818;
    end

    properties (Access = protected, Constant)
        NUM_WHEELS = 2;
    end

    methods
        % Set number of ticks in a revolution of the encoder
        function set.TicksPerRevolution(obj, val)
            nav.internal.WheelEncoderOdometryInterface.validateTicksPerRevolution...
                (val, obj.NUM_WHEELS);
            obj.TicksPerRevolution(:) = val;
        end
        % Set radius of the wheels of the vehicle
        function set.WheelRadius(obj, val)
            nav.internal.WheelEncoderOdometryInterface.validateWheelRadius ...
                (val, obj.NUM_WHEELS);
            obj.WheelRadius(:) = val;
        end
        % Set track width of the vehicle
        function set.TrackWidth(obj, val)
            nav.internal.WheelEncoderOdometryInterface.validateTrackWidth(val);
            obj.TrackWidth = val;
        end
        % Set wheel base of the vehicle
        function set.WheelBase(obj, val)
            nav.internal.WheelEncoderOdometryInterface.validateWheelBase(val);
            obj.WheelBase = val;
        end
    end

    methods (Access = protected, Static)
        % Return wheelEncoderAckermann as the expected
        % wheelEncoder<vehicle> object
        % Return wheelEncoderOdometryAckermann as the expected
        % wheelEncoderOdometry<vehicle> object
        function [encType, odomType] = getEncOdomType()
            encType  = 'wheelEncoderAckermann';
            odomType = 'wheelEncoderOdometryAckermann';
        end
    end
    
    methods (Access = protected)
        % Convert ticks and steering angle into linear and angular velocities
        function velOmega = inputToVelOmega(obj, ticks, steering)
            resolution = 2*pi*obj.WheelRadius ./ obj.TicksPerRevolution;
            velWheel   = ticks(:,1:2) .* repmat(resolution .* obj.SampleRate, size(ticks,1), 1);
            linVelBody = sum(velWheel, 2)/2;
            angVelBody = linVelBody(:,1) .* tan(steering(:)) ./ obj.WheelBase;
            velOmega   = [linVelBody, angVelBody];
        end
        % Copy requisite properties from wheel encoder object into wheel
        % encoder odometry object
        function setPropertiesFromEncoderObject(obj, encObj)
            obj.TicksPerRevolution = encObj.TicksPerRevolution(1:2);
            obj.WheelRadius        = encObj.WheelRadius(1:2);
            obj.TrackWidth         = encObj.TrackWidth(1);
            obj.WheelBase          = encObj.WheelBase;
            obj.SampleRate         = encObj.SampleRate;
        end

        function displayScalarObject(obj)
            displayScalarObjectWithUnits(obj);
        end

        % Validate inputs
        function validateInputsImpl(~, ticks, steer)
            validateattributes(ticks, {'single', 'double'}, ...
                               {'real', '2d'});
            nav.internal.WheelEncoderOdometryInterface.validateTicksSize( ...
                ticks, [2,4]);
            numSamples = size(ticks, 1);
            validateattributes(steer, {'single', 'double'}, ...
                               {'real', '2d', 'nrows', ...
                               numSamples, 'ncols', 1});
        end
    end
end

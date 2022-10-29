classdef wheelEncoderOdometryDifferentialDrive < ...
        nav.internal.WheelEncoderOdometryInterface & fusion.internal.UnitDisplayer
%WHEELENCODERODOMETRYDIFFERENTIALDRIVE Compute differential-drive vehicle odometry using wheel encoder ticks
%   whlEncOdom = WHEELENCODERODOMETRYDIFFERENTIALDRIVE returns a System
%   object, whlEncOdom, that computes odometry of a differential-drive
%   vehicle using wheel encoder ticks.
%
%   whlEncOdom = WHEELENCODERODOMETRYDIFFERENTIALDRIVE(ENCODER) returns a
%   WHEELENCODERODOMETRYDIFFERENTIALDRIVE System object based on the
%   properties of the wheelEncoderDifferentialDrive System object, ENCODER.
%
%   whlEncOdom = WHEELENCODERODOMETRYDIFFERENTIALDRIVE('Name1', Value1, ..., 'NameN', ValueN)
%   returns a WHEELENCODERODOMETRYDIFFERENTIALDRIVE System object with each
%   specified property name set to the specified value. You can specify
%   additional name-value pair arguments in any order as
%   (Name1,Value1,...,NameN, ValueN).
%
%   To compute differential-drive vehicle odometry from wheel encoder ticks:
%   1) Create the WHEELENCODERODOMETRYDIFFERENTIALDRIVE object and set its
%   properties.
%   2) Call the object with arguments, as if it were a function.
%
%   [POSE, VELOCITY] = whlEncOdom(TICKS) computes POSE (position and
%   orientation), and VELOCITY (linear velocity and angular velocity) of
%   the vehicle from wheel encoder ticks.
%
%   Input Arguments:
%
%     TICKS       Number of wheel encoder ticks specified as a real finite
%                 N-by-2 array. N is the number of samples in the current
%                 frame. The order of wheel ticks is:
%                 [ticksLeft, ticksRight]
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
%   WHEELENCODERODOMETRYDIFFERENTIALDRIVE. Outputs have the same datatype
%   as the input.
%
%   WHEELENCODERODOMETRYDIFFERENTIALDRIVE methods:
%
%   step             - Compute differential-drive vehicle odometry using
%                      wheel encoder ticks
%   release          - Allow property value and input characteristics to
%                      change, and release
%                      WHEELENCODERODOMETRYDIFFERENTIALDRIVE resources
%   clone            - Create WHEELENCODERODOMETRYDIFFERENTIALDRIVE object
%                      with same property values
%   isLocked         - Display locked status (logical)
%   reset            - Reset the states of the
%                      WHEELENCODERODOMETRYDIFFERENTIALDRIVE
%
%   WHEELENCODERODOMETRYDIFFERENTIALDRIVE properties:
%
%   SampleRate                - Sample rate of sensor
%   TicksPerRevolution        - Number of encoder ticks per wheel revolution
%   WheelRadius               - Wheel radius
%   TrackWidth                - Distance between wheels on axle
%   InitialPose               - Initial pose of vehicle
%
%       % EXAMPLE: Compute odometry from wheel ticks for Differential Drive vehicle
%
%       % Create wheel encoder odometry object for Differential Drive vehicle
%       whlEncOdom = wheelEncoderOdometryDifferentialDrive;
%
%       % Specify wheel encoder ticks.
%       ticks = [5, 5; 2, 2];
%
%       % Compute odometry
%       [pose, vel] = whlEncOdom(ticks);
%
%   See also WHEELENCODERDIFFERENTIALDRIVE, WHEELENCODERODOMETRYUNICYCLE,
%   WHEELENCODERODOMETRYBICYCLE, WHEELENCODERODOMETRYACKERMANN

%   Copyright 2020 The MathWorks, Inc.

%#codegen

    properties (Constant, Hidden)
        WheelRadiusUnits = 'm';
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
            wheelEncoderOdometryDifferentialDrive.TICKS_PER_REVOLUTION_DEFAULT...
            * ones(1, wheelEncoderOdometryDifferentialDrive.NUM_WHEELS);

        % WheelRadius Wheel radius
        % Wheel radius specified either as a positive scalar or two-element
        % vector in meters.
        % Default: [0.35 0.35]
        WheelRadius = ...
            wheelEncoderOdometryDifferentialDrive.WHEEL_RADIUS_DEFAULT ...
            * ones(1, wheelEncoderOdometryDifferentialDrive.NUM_WHEELS);

        % TrackWidth Distance between wheels on axle
        % Distance between the wheels on the axle specified as a positive
        % scalar in meters.
        % Default: 1.572
        TrackWidth = 1.572;
    end

    properties (Access = protected, Constant)
        % Number of wheels in the vehicle
        NUM_WHEELS = 2;
    end

    methods
        % Set number of ticks in a revolution of the encoder
        function set.TicksPerRevolution(obj, val)
            nav.internal.WheelEncoderOdometryInterface.validateTicksPerRevolution...
                (val, obj.NUM_WHEELS);
            obj.TicksPerRevolution(:) = val(:);
        end
        % Set radius of the wheels of the vehicle
        function set.WheelRadius(obj, val)
            nav.internal.WheelEncoderOdometryInterface.validateWheelRadius ...
                (val, obj.NUM_WHEELS);
            obj.WheelRadius(:) = val(:);
        end
        % Set track width of the vehicle
        function set.TrackWidth(obj, val)
            nav.internal.WheelEncoderOdometryInterface.validateTrackWidth(val);
            obj.TrackWidth = val;
        end
    end

    methods (Access = protected, Static)
        % Return wheelEncoderDifferentialDrive as the expected
        % wheelEncoder<vehicle> object
        % Return wheelEncoderOdometryDifferentialDrive as the expected
        % wheelEncoderOdometry<vehicle> object
        function [encType, odomType] = getEncOdomType()
            encType  = 'wheelEncoderDifferentialDrive';
            odomType = 'wheelEncoderOdometryDifferentialDrive';
        end
    end
    
    methods (Access = protected)
        % Convert ticks linear and angular velocities
        function velOmega = inputToVelOmega(obj, ticks, ~)
            resolution = 2*pi*obj.WheelRadius ./ obj.TicksPerRevolution;
            velWheel   = ticks .* repmat(resolution .* obj.SampleRate, size(ticks,1), 1);
            linVelBody = sum(velWheel, 2)/2;
            angVelBody = (velWheel(:,2) - velWheel(:,1))/obj.TrackWidth;
            velOmega   = [linVelBody, angVelBody];
        end
        % Copy requisite properties from wheel encoder object into wheel
        % encoder odometry object
        function setPropertiesFromEncoderObject(obj, encObj)
            obj.TicksPerRevolution = encObj.TicksPerRevolution;
            obj.WheelRadius        = encObj.WheelRadius;
            obj.TrackWidth         = encObj.TrackWidth;
            obj.SampleRate         = encObj.SampleRate;
        end

        function displayScalarObject(obj)
            displayScalarObjectWithUnits(obj);
        end

        % Validate inputs
        function validateInputsImpl(~, ticks)
            validateattributes(ticks, {'single', 'double'}, ...
                               {'real', '2d', 'ncols', 2});
        end
    end
end

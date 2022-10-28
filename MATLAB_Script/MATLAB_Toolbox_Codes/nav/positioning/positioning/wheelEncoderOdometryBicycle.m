classdef wheelEncoderOdometryBicycle < ...
        nav.internal.WheelEncoderOdometryInterface & fusion.internal.UnitDisplayer
%WHEELENCODERODOMETRYBICYCLE Compute bicycle odometry using wheel encoder ticks
%   whlEncOdom = WHEELENCODERODOMETRYBICYCLE returns a System object,
%   whlEncOdom, that computes odometry of a bicycle using wheel encoder
%   ticks and steering angle.
%
%   whlEncOdom = WHEELENCODERODOMETRYBICYCLE(ENCODER) returns a
%   WHEELENCODERODOMETRYBICYCLE System object based on the properties of
%   the wheelEncoderBicycle System object, ENCODER.
%
%   whlEncOdom = WHEELENCODERODOMETRYBICYCLE('Name1', Value1, ..., 'NameN', ValueN)
%   returns a WHEELENCODERODOMETRYBICYCLE System object with each
%   specified property name set to the specified value. You can specify
%   additional name-value pair arguments in any order as
%   (Name1,Value1,...,NameN, ValueN).
%
%   To compute bicycle odometry from wheel encoder ticks and steering angle:
%   1) Create the WHEELENCODERODOMETRYBICYCLE object and set its
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
%                 N-element column vector. N is the number of samples in
%                 the current frame. The column contain the ticks from the
%                 rear wheel of the bicycle.
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
%   WHEELENCODERODOMETRYBICYCLE. Outputs have the same datatype as the
%   input.
%
%   WHEELENCODERODOMETRYBICYCLE methods:
%
%   step             - Compute bicycle odometry using wheel encoder ticks
%   release          - Allow property value and input characteristics to
%                      change, and release WHEELENCODERODOMETRYBICYCLE
%                      resources
%   clone            - Create WHEELENCODERODOMETRYBICYCLE object with same
%                      property values
%   isLocked         - Display locked status (logical)
%   reset            - Reset the states of the WHEELENCODERODOMETRYBICYCLE
%
%   WHEELENCODERODOMETRYBICYCLE properties:
%
%   SampleRate                - Sample rate of sensor
%   TicksPerRevolution        - Number of encoder ticks per wheel revolution
%   WheelRadius               - Wheel radius
%   WheelBase                 - Distance between the center of front and
%                               rear wheels
%   InitialPose               - Initial pose of vehicle
%
%       % EXAMPLE: Compute odometry from wheel ticks for Bicycle vehicle
%
%       % Create wheel encoder odometry object for Bicycle vehicle
%       whlEncOdom = wheelEncoderOdometryBicycle;
%
%       % Specify wheel encoder ticks and steering angle.
%       ticks = [5; 2];
%       steer = [0.2; 0.2];
%
%       % Compute odometry
%       [pose, vel] = whlEncOdom(ticks, steer);
%
%   See also WHEELENCODERBICYCLE, WHEELENCODERODOMETRYUNICYCLE,
%   WHEELENCODERODOMETRYDIFFERENTIALDRIVE, WHEELENCODERODOMETRYACKERMANN

%   Copyright 2020 The MathWorks, Inc.

%#codegen

    properties (Constant, Hidden)
        WheelRadiusUnits = 'm';
        WheelBaseUnits   = 'm';
        SampleRateUnits  = 'Hz';
    end

    properties
        % TicksPerRevolution Number of encoder ticks per wheel revolution
        % Number of encoder ticks per wheel revolution specified as a
        % positive scalar integer.
        % Default: 2048
        TicksPerRevolution = ...
            wheelEncoderOdometryBicycle.TICKS_PER_REVOLUTION_DEFAULT;

        % WheelRadius Wheel radius
        % Wheel radius specified as a positive scalar in meters.
        % Default: 0.35
        WheelRadius = wheelEncoderOdometryBicycle.WHEEL_RADIUS_DEFAULT;

        % WheelBase Distance between front and real axles
        % Distance between front and rear axles specified as a positive
        % scalar in meters.
        % Default: 2.818
        WheelBase = 2.818;
    end

    methods
        % Set number of ticks in a revolution of the encoder
        function set.TicksPerRevolution(obj, val)
            validateattributes(val, {'single', 'double'}, ...
                               {'real', 'finite', 'integer', 'positive', 'scalar'}, '', ...
                               'TicksPerRevolution');
            obj.TicksPerRevolution = val;
        end
        % Set radius of the wheel of the vehicle
        function set.WheelRadius(obj, val)
            validateattributes(val, {'single', 'double'}, ...
                               {'real', 'positive', 'finite', 'scalar'}, '', ...
                               'WheelRadius');
            obj.WheelRadius = val;
        end
        % Set wheel base of the vehicle
        function set.WheelBase(obj, val)
            nav.internal.WheelEncoderOdometryInterface.validateWheelBase(val);
            obj.WheelBase = val;
        end
    end

    methods (Access = protected, Static)
        % Return wheelEncoderBicycle as the expected
        % wheelEncoder<vehicle> object
        % Return wheelEncoderOdometryBicycle as the expected
        % wheelEncoderOdometry<vehicle> object
        function [encType, odomType] = getEncOdomType()
            encType  = 'wheelEncoderBicycle';
            odomType = 'wheelEncoderOdometryBicycle';
        end
    end
    
    methods (Access = protected)
        % Convert ticks and steering angle into linear and angular velocities
        function velOmega = inputToVelOmega(obj, ticks, steering)
            resolution = 2*pi*obj.WheelRadius / obj.TicksPerRevolution;
            linVelBody = ticks(:,1) .* resolution .* obj.SampleRate;
            angVelBody = linVelBody(:,1) .* tan(steering(:)) ./ obj.WheelBase;
            velOmega   = [linVelBody, angVelBody];
        end
        % Copy requisite properties from wheel encoder object into wheel
        % encoder odometry object
        function setPropertiesFromEncoderObject(obj, encObj)
            obj.TicksPerRevolution = encObj.TicksPerRevolution(1);
            obj.WheelRadius        = encObj.WheelRadius(1);
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
                ticks, [1,2]);
            numSamples = size(ticks, 1);
            validateattributes(steer, {'single', 'double'}, ...
                               {'real', '2d', 'nrows', ...
                               numSamples, 'ncols', 1});
        end
    end
end

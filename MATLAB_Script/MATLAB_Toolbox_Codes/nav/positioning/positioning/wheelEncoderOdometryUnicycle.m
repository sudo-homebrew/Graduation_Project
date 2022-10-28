classdef wheelEncoderOdometryUnicycle < ...
        nav.internal.WheelEncoderOdometryInterface & fusion.internal.UnitDisplayer
%WHEELENCODERODOMETRYUNICYCLE Compute unicycle odometry using wheel encoder ticks
%   whlEncOdom = WHEELENCODERODOMETRYUNICYCLE returns a System object,
%   whlEncOdom, that computes odometry of a unicycle using wheel encoder
%   ticks and angular velocity.
%
%   whlEncOdom = WHEELENCODERODOMETRYUNICYCLE(ENCODER) returns a
%   WHEELENCODERODOMETRYUNICYCLE System object based on the properties of
%   the wheelEncoderUnicycle System object, ENCODER.
%
%   whlEncOdom = WHEELENCODERODOMETRYUNICYCLE('Name1', Value1, ..., 'NameN', ValueN)
%   returns a WHEELENCODERODOMETRYUNICYCLE System object with each
%   specified property name set to the specified value. You can specify
%   additional name-value pair arguments in any order as
%   (Name1,Value1,...,NameN, ValueN).
%
%   To compute unicycle odometry from wheel encoder ticks and angular velocity:
%   1) Create the WHEELENCODERODOMETRYUNICYCLE object and set its
%   properties.
%   2) Call the object with arguments, as if it were a function.
%
%   [POSE, VELOCITY] = whlEncOdom(TICKS, ANGVEL) computes POSE (position
%   and orientation), and VELOCITY (linear velocity and angular velocity)
%   of the vehicle from wheel encoder ticks and angular velocity.
%
%   Input Arguments:
%
%     TICKS       Number of wheel ticks specified as a real finite
%                 N-element column vector. N is the number of samples in
%                 the current frame.
%
%     ANGVEL      Angular velocity of the vehicle in the vehicle body
%                 coordinate system specified as a real finite N-element
%                 column vector in radians per second. N is the number of
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
%   WHEELENCODERODOMETRYUNICYCLE. Outputs have the same datatype as the
%   input.
%
%   WHEELENCODERODOMETRYUNICYCLE methods:
%
%   step             - Compute unicycle odometry using wheel encoder ticks
%   release          - Allow property value and input characteristics to
%                      change, and release WHEELENCODERODOMETRYUNICYCLE
%                      resources
%   clone            - Create WHEELENCODERODOMETRYUNICYCLE object with same
%                      property values
%   isLocked         - Display locked status (logical)
%   reset            - Reset the states of the WHEELENCODERODOMETRYUNICYCLE
%
%   WHEELENCODERODOMETRYUNICYCLE properties:
%
%   SampleRate                - Sample rate of sensor
%   TicksPerRevolution        - Number of encoder ticks per wheel revolution
%   WheelRadius               - Wheel radius
%   InitialPose               - Initial pose of vehicle
%
%       % EXAMPLE: Compute odometry from wheel ticks for Unicycle vehicle
%
%       % Create wheel encoder odometry object for Unicycle vehicle
%       whlEncOdom = wheelEncoderOdometryUnicycle;
%
%       % Specify wheel encoder ticks and angular velocity.
%       ticks = [5; 2];
%       angVel = [0.2; 0.2];
%
%       % Compute odometry
%       [pose, vel] = whlEncOdom(ticks, angVel);
%
%   See also WHEELENCODERUNICYCLE, WHEELENCODERODOMETRYBICYCLE,
%   WHEELENCODERODOMETRYDIFFERENTIALDRIVE, WHEELENCODERODOMETRYACKERMANN

%   Copyright 2020 The MathWorks, Inc.

%#codegen

    properties (Constant, Hidden)
        WheelRadiusUnits = 'm';
        SampleRateUnits  = 'Hz';
    end

    properties
        % TicksPerRevolution Number of encoder ticks per wheel revolution
        % Number of encoder ticks per wheel revolution specified as a
        % positive scalar integer.
        % Default: 2048
        TicksPerRevolution = ...
                wheelEncoderOdometryUnicycle.TICKS_PER_REVOLUTION_DEFAULT;

        % WheelRadius Wheel radius
        % Wheel radius specified as a positive scalar in meters.
        % Default: 0.35
        WheelRadius = wheelEncoderOdometryUnicycle.WHEEL_RADIUS_DEFAULT;
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
    end

    methods (Access = protected, Static)
        % Return wheelEncoderUnicycle as the expected
        % wheelEncoder<vehicle> object
        % Return wheelEncoderOdometryUnicycle as the expected
        % wheelEncoderOdometry<vehicle> object
        function [encType, odomType] = getEncOdomType()
            encType  = 'wheelEncoderUnicycle';
            odomType = 'wheelEncoderOdometryUnicycle';
        end
    end
    
    methods (Access = protected)
        % Convert ticks and angular velocity into linear and angular velocities
        function velOmega = inputToVelOmega(obj, ticks, angVelBody)
            resolution = 2*pi*obj.WheelRadius / obj.TicksPerRevolution;
            linVelBody = ticks .* resolution .* obj.SampleRate;
            velOmega   = [linVelBody, angVelBody];
        end
        % Copy requisite properties from wheel encoder object into wheel
        % encoder odometry object
        function setPropertiesFromEncoderObject(obj, encObj)
            obj.TicksPerRevolution = encObj.TicksPerRevolution;
            obj.WheelRadius        = encObj.WheelRadius;
            obj.SampleRate         = encObj.SampleRate;
        end

        function displayScalarObject(obj)
            displayScalarObjectWithUnits(obj);
        end

        % Validate inputs
        function validateInputsImpl(~, ticks, angVel)
            validateattributes(ticks, {'single', 'double'}, ...
                               {'real', '2d', 'ncols', 1});
            numSamples = size(ticks, 1);
            validateattributes(angVel, {'single', 'double'}, ...
                               {'real', '2d', 'nrows', numSamples, 'ncols', 1});
        end
    end
end

classdef wheelEncoderOdometryDifferentialDrive< nav.internal.WheelEncoderOdometryInterface & fusion.internal.UnitDisplayer
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
%   <a href="matlab:help matlab.System/reset   ">reset</a>            - Reset the states of the
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

    methods
        function out=wheelEncoderOdometryDifferentialDrive
        end

        function out=displayScalarObject(~) %#ok<STOUT>
        end

        function out=getEncOdomType(~) %#ok<STOUT>
        end

        function out=inputToVelOmega(~) %#ok<STOUT>
        end

        function out=setPropertiesFromEncoderObject(~) %#ok<STOUT>
        end

        function out=validateInputsImpl(~) %#ok<STOUT>
        end

    end
    properties
        % Number of wheels in the vehicle
        NUM_WHEELS;

        % TicksPerRevolution Number of encoder ticks per wheel revolution
        % Number of encoder ticks per wheel revolution specified either as
        % a positive scalar integer or two-element vector of positive
        % integers.
        % Default: [2048 2048]
        TicksPerRevolution;

        % TrackWidth Distance between wheels on axle
        % Distance between the wheels on the axle specified as a positive
        % scalar in meters.
        % Default: 1.572
        TrackWidth;

        % WheelRadius Wheel radius
        % Wheel radius specified either as a positive scalar or two-element
        % vector in meters.
        % Default: [0.35 0.35]
        WheelRadius;

    end
end

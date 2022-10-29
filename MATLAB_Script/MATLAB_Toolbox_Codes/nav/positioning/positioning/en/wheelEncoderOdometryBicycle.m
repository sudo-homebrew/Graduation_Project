classdef wheelEncoderOdometryBicycle< nav.internal.WheelEncoderOdometryInterface & fusion.internal.UnitDisplayer
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
%   <a href="matlab:help matlab.System/reset   ">reset</a>            - Reset the states of the WHEELENCODERODOMETRYBICYCLE
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

    methods
        function out=wheelEncoderOdometryBicycle
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
        % TicksPerRevolution Number of encoder ticks per wheel revolution
        % Number of encoder ticks per wheel revolution specified as a
        % positive scalar integer.
        % Default: 2048
        TicksPerRevolution;

        % WheelBase Distance between front and real axles
        % Distance between front and rear axles specified as a positive
        % scalar in meters.
        % Default: 2.818
        WheelBase;

        % WheelRadius Wheel radius
        % Wheel radius specified as a positive scalar in meters.
        % Default: 0.35
        WheelRadius;

    end
end

classdef wheelEncoderBicycle< nav.internal.WheelEncoderInterface & fusion.internal.UnitDisplayer
%WHEELENCODERBICYCLE Simulate wheel encoder sensor readings for bicycle vehicle
%
%   ENCODER = WHEELENCODERBICYCLE returns a System object, ENCODER, that
%   computes a wheel encoder tick reading based on pose input.
%
%   ENCODER = WHEELENCODERBICYCLE(..., 'Name', Value, ...) returns a
%   WHEELENCODERBICYCLE System object with each specified property name set
%   to the specified value. You can specify additional name-value pair
%   arguments in any order as (Name1,Value1,...,NameN, ValueN).
%
%   To simulate wheel encoder sensor readings: 
%   1) Create the WHEELENCODERBICYCLE object and set its properties. 
%   2) Call the object with arguments, as if it were a function.
%
%   [TICKS] = ENCODER(VEL, ANGVEL, ORIENT) computes wheel ticks, TICKS,
%   from velocity (VEL), angular velocity (ANGVEL), and orientation
%   (ORIENT) inputs.
%
%   INPUT ARGUMENTS:
%
%       VEL       Velocity of the vehicle in the local navigation
%                 coordinate system specified as a real finite N-by-3 array
%                 in meters per second. N is the number of samples in the
%                 current frame.
%
%       ANGVEL    Angular velocity of the vehicle in the local navigation
%                 coordinate system specified as a real finite N-by-3 array
%                 in radians per second. N is the number of samples in the
%                 current frame.
%
%       ORIENT    Orientation of the vehicle with respect to the local
%                 navigation coordinate system specified as a quaternion
%                 N-element column vector or a 3-by-3-by-N rotation matrix.
%                 Each quaternion or rotation matrix is a frame rotation
%                 from the local navigation coordinate system to the
%                 current vehicle body coordinate system. N is the number
%                 of samples in the current frame.
%
%   OUTPUT ARGUMENTS:
%
%       TICKS     Number of wheel ticks specified as a real finite N-by-W
%                 array. N is the number of samples in the current frame. W
%                 is the number of wheels on the vehicle. The wheel order
%                 is as follows: [ticksBack, ticksFront].
%
%   Either single or double datatypes are supported for the inputs to
%   WHEELENCODERBICYCLE. Outputs have the same datatype as the input.
%
%   WHEELENCODERBICYCLE methods:
%
%   step             - Simulate wheel encoder readings
%   release          - Allow property value and input characteristics to
%                      change, and release WHEELENCODERBICYCLE resources
%   clone            - Create WHEELENCODERBICYCLE object with same property
%                      values
%   isLocked         - Display locked status (logical)
%   <a href="matlab:help matlab.System/reset   ">reset</a>            - Reset the states of the WHEELENCODERBICYCLE
%
%   WHEELENCODERBICYCLE properties:
%
%   SampleRate                - Sampling rate of sensor (Hz)
%   TicksPerRevolution        - Number of ticks per one revolution
%   WheelRadius               - Wheel radius (m)
%   WheelRadiusBias           - Bias in wheel radius (m)
%   WheelPositionAccuracy     - Deviation from actual wheel position (rad)
%   SlipRatio                 - Amount of wheel slippage/skidding
%   WheelBase                 - Distance between front and rear wheels (m)
%   RandomStream              - Source of random number stream
%   Seed                      - Initial seed of mt19937ar random number
%
%       % EXAMPLE: Generate wheel ticks from vehicle trajectory.
%
%       % Create the sensor
%       encoder = wheelEncoderBicycle;
%       orient = quaternion(1, 0, 0, 0);
%       vel = [1 0 0];
%       angvel = [0 0 0.2];
%
%       % Generate wheel ticks
%       ticks = encoder(vel, angvel, orient);
%
%   See also WHEELENCODERUNICYCLE, WHEELENCODERDIFFERENTIALDRIVE,
%   WHEELENCODERACKERMANN.

 
%   Copyright 2020-2021 The MathWorks, Inc.

    methods
        function out=wheelEncoderBicycle
        end

        function out=displayScalarObject(~) %#ok<STOUT>
        end

        function out=getNumWheels(~) %#ok<STOUT>
        end

        function out=vehicleSpeedToWheelDistance(~) %#ok<STOUT>
        end

    end
    properties
        % SlipRatio Ratio of slip of the wheel
        % Specify the amount of slip or skid of a wheel as a scalar or
        % 2-element vector with values greater than or equal to -1, where
        % each element corresponds to a wheel. The wheel order is as
        % follows: [back, front]. A negative value corresponds to the wheel
        % skidding or under rotation, with -1 corresponding to no wheel
        % movement. A positive value corresponds to the wheel slipping or
        % over rotation, with a larger slip ratio corresponding to more
        % wheel slippage. A value of zero corresponds to no slipping or
        % skidding. The default value is [0 0].
        SlipRatio;

        % TicksPerRevolution Ticks per wheel revolution
        % Specify the number of ticks per one revolution of a wheel as a
        % positive scalar integer or 2-element vector, where each element
        % corresponds to a wheel. The wheel order is as follows: [back,
        % front]. The default value is [2048 2048].
        TicksPerRevolution;

        % WheelBase Distance between front and rear wheels (m)
        % Specify the wheel base as a positive scalar in meters. The
        % default value is 2.818.
        WheelBase;

        % WheelPositionAccuracy Accuracy of wheel position (rad)
        % Specify the standard deviation of the noise in the wheel position
        % as a nonnegative scalar or 2-element vector in radians, where
        % each element corresponds to a wheel. The wheel order is as
        % follows: [back, front]. The default value is [0 0].
        WheelPositionAccuracy;

        % WheelRadius Radius of wheel (m)
        % Specify the wheel radius as a positive scalar or 2-element vector
        % in meters, where each element corresponds to a wheel. The wheel
        % order is as follows: [back, front]. The default value is [0.35
        % 0.35].
        WheelRadius;

        % WheelRadiusBias Bias of the wheel radius (m)
        % Specify the wheel radius bias as a scalar or 2-element vector in
        % meters, where each element corresponds to a wheel. The wheel
        % order is as follows: [back, front]. The default value is [0 0].
        WheelRadiusBias;

    end
end

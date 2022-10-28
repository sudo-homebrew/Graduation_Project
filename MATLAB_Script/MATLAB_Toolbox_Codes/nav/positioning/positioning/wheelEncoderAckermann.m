classdef wheelEncoderAckermann < nav.internal.WheelEncoderInterface ...
        & fusion.internal.UnitDisplayer
%WHEELENCODERACKERMANN Simulate wheel encoder sensor readings for Ackermann vehicle
%
%   ENCODER = WHEELENCODERACKERMANN returns a System object, ENCODER, that
%   computes a wheel encoder tick reading based on pose input.
%
%   ENCODER = WHEELENCODERACKERMANN(..., 'Name', Value, ...) returns a
%   WHEELENCODERACKERMANN System object with each specified property name
%   set to the specified value. You can specify additional name-value pair
%   arguments in any order as (Name1,Value1,...,NameN, ValueN).
%
%   To simulate wheel encoder sensor readings: 
%   1) Create the WHEELENCODERACKERMANN object and set its properties. 
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
%                 is as follows: [ticksBackLeft, ticksBackRight,
%                 ticksFrontLeft, ticksFrontRight].
%
%   Either single or double datatypes are supported for the inputs to
%   WHEELENCODERACKERMANN. Outputs have the same datatype as the input.
%
%   WHEELENCODERACKERMANN methods:
%
%   step             - Simulate wheel encoder readings
%   release          - Allow property value and input characteristics to
%                      change, and release WHEELENCODERACKERMANN resources
%   clone            - Create WHEELENCODERACKERMANN object with same 
%                      property values
%   isLocked         - Display locked status (logical)
%   reset            - Reset the states of the WHEELENCODERACKERMANN
%
%   WHEELENCODERACKERMANN properties:
%
%   SampleRate                - Sampling rate of sensor (Hz)
%   TicksPerRevolution        - Number of ticks per one revolution
%   WheelRadius               - Wheel radius (m)
%   WheelRadiusBias           - Bias in wheel radius (m)
%   WheelPositionAccuracy     - Deviation from actual wheel position (rad)
%   SlipRatio                 - Amount of wheel slippage/skidding
%   TrackWidth                - Distance between wheels on axle (m)
%   TrackWidthBias            - Bias in track width (m)
%   WheelBase                 - Distance between front and rear axles (m)
%   RandomStream              - Source of random number stream
%   Seed                      - Initial seed of mt19937ar random number
%
%       % EXAMPLE: Generate wheel ticks from vehicle trajectory.
%
%       % Create the sensor
%       encoder = wheelEncoderAckermann;
%       orient = quaternion(1, 0, 0, 0);
%       vel = [1 0 0];
%       angvel = [0 0 0.2];
%
%       % Generate wheel ticks
%       ticks = encoder(vel, angvel, orient);
%
%   See also WHEELENCODERUNICYCLE, WHEELENCODERBICYCLE,
%   WHEELENCODERDIFFERENTIALDRIVE.

%   Copyright 2020-2021 The MathWorks, Inc.

%#codegen

    properties
        % TicksPerRevolution Ticks per wheel revolution
        % Specify the number of ticks per one revolution of a wheel as a
        % positive scalar integer or 4-element vector, where each element
        % corresponds to a wheel. The wheel order is as follows: [backLeft,
        % backRight, frontLeft, frontRight]. The default value is [2048
        % 2048 2048 2048].
        TicksPerRevolution = wheelEncoderAckermann.TICKS_PER_REVOLUTION_DEFAULT * ones(1, wheelEncoderAckermann.getNumWheels);
        % WheelRadius Radius of wheel (m)
        % Specify the wheel radius as a positive scalar or 4-element vector
        % in meters, where each element corresponds to a wheel. The wheel
        % order is as follows: [backLeft, backRight, frontLeft,
        % frontRight]. The default value is [0.35 0.35 0.35 0.35].
        WheelRadius = wheelEncoderAckermann.WHEEL_RADIUS_DEFAULT * ones(1, wheelEncoderAckermann.getNumWheels);
        % WheelRadiusBias Bias of the wheel radius (m)
        % Specify the wheel radius bias as a scalar or 4-element vector in
        % meters, where each element corresponds to a wheel. The wheel
        % order is as follows: [backLeft, backRight, frontLeft,
        % frontRight]. The default value is [0 0 0 0].
        WheelRadiusBias = wheelEncoderAckermann.WHEEL_RADIUS_BIAS_DEFAULT * ones(1, wheelEncoderAckermann.getNumWheels);
        % WheelPositionAccuracy Accuracy of wheel position (rad) 
        % Specify the standard deviation of the noise in the wheel position
        % as a nonnegative scalar or 4-element vector in radians, where
        % each element corresponds to a wheel. The wheel order is as
        % follows: [backLeft, backRight, frontLeft, frontRight]. The
        % default value is [0 0 0 0].
        WheelPositionAccuracy = wheelEncoderAckermann.WHEEL_POSITION_ACCURACY_DEFAULT * ones(1, wheelEncoderAckermann.getNumWheels);
        % SlipRatio Ratio of slip of the wheel
        % Specify the amount of slip or skid of a wheel as a scalar or
        % 4-element vector with values greater than or equal to -1, where
        % each element corresponds to a wheel. The wheel order is as
        % follows: [backLeft, backRight, frontLeft, frontRight]. A negative
        % value corresponds to the wheel skidding or under rotation, with
        % -1 corresponding to no wheel movement. A positive value
        % corresponds to the wheel slipping or over rotation, with a larger
        % slip ratio corresponding to more wheel slippage. A value of zero
        % corresponds to no slipping or skidding. The default value is 
        % [0 0 0 0].
        SlipRatio = wheelEncoderAckermann.SLIP_RATIO_DEFAULT * ones(1, wheelEncoderAckermann.getNumWheels);
        
        % TrackWidth Distance between wheels on axle (m)
        % Specify the track width as a positive scalar or 2-element vector
        % in meters, where each element corresponds to a vehicle track. The
        % first element is the rear track and the second element is the
        % front track. The default value is [1.572 1.572].
        TrackWidth = [1.572, 1.572];
        % TrackWidthBias Bias of the track width (m)
        % Specify the track width bias as a scalar or 2-element vector in
        % meters, where each element corresponds to a vehicle track. The
        % first element is the rear track and the second element is the
        % front track. The default value is [0 0].
        TrackWidthBias = [0, 0];
        % WheelBase Distance between front and rear axles (m)
        % Specify the wheel base as a positive scalar in meters. The
        % default value is 2.818.
        WheelBase = 2.818;
    end
    
    properties (Access = protected, Constant)
        NUM_TRACKS = 2;
    end
    
    properties (Constant, Hidden)
        SampleRateUnits = 'Hz';
        TrackWidthUnits = 'm';
        TrackWidthBiasUnits = 'm';
        WheelRadiusUnits = 'm';
        WheelRadiusBiasUnits = 'm';
        WheelBaseUnits = 'm';
        WheelPositionAccuracyUnits = 'rad'
    end
    
    methods
        function set.TicksPerRevolution(obj, val)
            nav.internal.WheelEncoderInterface.validateTicksPerRevolution(val, wheelEncoderAckermann.getNumWheels);
            obj.TicksPerRevolution(:) = val(:);
        end
        function set.WheelRadius(obj, val)
            nav.internal.WheelEncoderInterface.validateWheelRadius(val, wheelEncoderAckermann.getNumWheels);
            obj.WheelRadius(:) = val(:);
        end
        function set.WheelRadiusBias(obj, val)
            nav.internal.WheelEncoderInterface.validateWheelRadiusBias(val, wheelEncoderAckermann.getNumWheels);
            obj.WheelRadiusBias(:) = val(:);
        end
        function set.WheelPositionAccuracy(obj, val)
            nav.internal.WheelEncoderInterface.validateWheelPositionAccuracy(val, wheelEncoderAckermann.getNumWheels);
            obj.WheelPositionAccuracy(:) = val(:);
        end
        function set.SlipRatio(obj, val)
            nav.internal.WheelEncoderInterface.validateSlipRatio(val, wheelEncoderAckermann.getNumWheels);
            obj.SlipRatio(:) = val(:);
        end
        function set.TrackWidth(obj, val)
            nav.internal.WheelEncoderInterface.validateTrackWidth(val, obj.NUM_TRACKS);
            obj.TrackWidth(:) = val(:);
        end
        function set.TrackWidthBias(obj, val)
            nav.internal.WheelEncoderInterface.validateTrackWidthBias(val, obj.NUM_TRACKS);
            obj.TrackWidthBias(:) = val(:);
        end
        function set.WheelBase(obj, val)
            nav.internal.WheelEncoderInterface.validateWheelBase(val);
            obj.WheelBase = val;
        end
    end
    
    methods (Access = protected)
        function wheelDistance = vehicleSpeedToWheelDistance(obj, speed, omega)
            % Convert vehicle speed (m/s) to distance traveled by each
            % wheel (m).
            
            numSamples = size(speed, 1);
            wheelDistance = NaN(numSamples, ...
                wheelEncoderAckermann.getNumWheels, 'like', speed);
            dt = cast(obj.pDeltaTime, 'like', speed);
            trackWidths = cast(obj.TrackWidth + obj.TrackWidthBias, ...
                'like', speed);
            trackWidthBack = repmat(trackWidths(1), numSamples, 1);
            trackWidthFront = repmat(trackWidths(end), numSamples, 1);
            wheelBase = repmat(cast(obj.WheelBase, 'like', speed), ...
                numSamples, 1);
            backWheelTurn = (omega .* trackWidthBack/2);
            wheelDistance(:,1) = dt * (speed - backWheelTurn);
            wheelDistance(:,2) = dt * (speed + backWheelTurn);
            
            zero = zeros(numSamples, 1, 'like', speed);
            
            frontLeftWheelVel = [speed, zero, zero] ...
                + cross([zero, zero, omega], ...
                [wheelBase, trackWidthFront/2, zero]);
            frontLeftWheelSpeed = vecnorm(frontLeftWheelVel, 2, 2);
            frontRightWheelVel = [speed, zero, zero] ...
                + cross([zero, zero, omega], ...
                [wheelBase, -trackWidthFront/2, zero]);
            frontRightWheelSpeed = vecnorm(frontRightWheelVel, 2, 2);
            
            wheelDistance(:,3:4) = dt * [frontLeftWheelSpeed, ...
                frontRightWheelSpeed];          
        end
        
        function displayScalarObject(obj)
            displayScalarObjectWithUnits(obj);
        end
    end
    
    methods (Access = protected, Static)
        function w = getNumWheels
            w = 4;
        end
    end
end


classdef wheelEncoderDifferentialDrive < nav.internal.WheelEncoderInterface ...
        & fusion.internal.UnitDisplayer
%WHEELENCODERDIFFERENTIALDRIVE Simulate wheel encoder sensor readings for differential drive vehicle
%
%   ENCODER = WHEELENCODERDIFFERENTIALDRIVE returns a System object,
%   ENCODER, that computes a wheel encoder tick reading based on pose
%   input.
%
%   ENCODER = WHEELENCODERDIFFERENTIALDRIVE(..., 'Name', Value, ...)
%   returns a WHEELENCODERDIFFERENTIALDRIVE System object with each
%   specified property name set to the specified value. You can specify
%   additional name-value pair arguments in any order as
%   (Name1,Value1,...,NameN, ValueN).
%
%   To simulate wheel encoder sensor readings: 
%   1) Create the WHEELENCODERDIFFERENTIALDRIVE object and set its 
%      properties. 
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
%                 is as follows: [ticksLeft, ticksRight].
%
%   Either single or double datatypes are supported for the inputs to
%   WHEELENCODERDIFFERENTIALDRIVE. Outputs have the same datatype as the
%   input.
%
%   WHEELENCODERDIFFERENTIALDRIVE methods:
%
%   step             - Simulate wheel encoder readings
%   release          - Allow property value and input characteristics to
%                      change, and release WHEELENCODERDIFFERENTIALDRIVE 
%                      resources
%   clone            - Create WHEELENCODERDIFFERENTIALDRIVE object with 
%                      same property values
%   isLocked         - Display locked status (logical)
%   reset            - Reset the states of the 
%                      WHEELENCODERDIFFERENTIALDRIVE
%
%   WHEELENCODERDIFFERENTIALDRIVE properties:
%
%   SampleRate                - Sampling rate of sensor (Hz)
%   TicksPerRevolution        - Number of ticks per one revolution
%   WheelRadius               - Wheel radius (m)
%   WheelRadiusBias           - Bias in wheel radius (m)
%   WheelPositionAccuracy     - Deviation from actual wheel position (rad)
%   SlipRatio                 - Amount of wheel slippage/skidding
%   TrackWidth                - Distance between wheels on axle (m)
%   TrackWidthBias            - Bias in track width (m)
%   RandomStream              - Source of random number stream
%   Seed                      - Initial seed of mt19937ar random number
%
%       % EXAMPLE: Generate wheel ticks from vehicle trajectory.
%
%       % Create the sensor
%       encoder = wheelEncoderDifferentialDrive;
%       orient = quaternion(1, 0, 0, 0);
%       vel = [1 0 0];
%       angvel = [0 0 0.2];
%
%       % Generate wheel ticks
%       ticks = encoder(vel, angvel, orient);
%
%   See also WHEELENCODERUNICYCLE, WHEELENCODERBICYCLE,
%   WHEELENCODERACKERMANN.

%   Copyright 2020-2021 The MathWorks, Inc.

%#codegen

    properties
        % TicksPerRevolution Ticks per wheel revolution
        % Specify the number of ticks per one revolution of a wheel as a
        % positive scalar integer or 2-element vector, where each element
        % corresponds to a wheel. The wheel order is as follows: [left,
        % right]. The default value is [2048 2048].
        TicksPerRevolution = wheelEncoderDifferentialDrive.TICKS_PER_REVOLUTION_DEFAULT * ones(1, wheelEncoderDifferentialDrive.getNumWheels);
        % WheelRadius Radius of wheel (m)
        % Specify the wheel radius as a positive scalar or 2-element vector
        % in meters, where each element corresponds to a wheel. The wheel
        % order is as follows: [left, right]. The default value is [0.35
        % 0.35].
        WheelRadius = wheelEncoderDifferentialDrive.WHEEL_RADIUS_DEFAULT * ones(1, wheelEncoderDifferentialDrive.getNumWheels);
        % WheelRadiusBias Bias of the wheel radius (m)
        % Specify the wheel radius bias as a scalar or 2-element vector in
        % meters, where each element corresponds to a wheel. The wheel
        % order is as follows: [left, right]. The default value is [0 0].
        WheelRadiusBias = wheelEncoderDifferentialDrive.WHEEL_RADIUS_BIAS_DEFAULT * ones(1, wheelEncoderDifferentialDrive.getNumWheels);
        % WheelPositionAccuracy Accuracy of wheel position (rad) 
        % Specify the standard deviation of the noise in the wheel position
        % as a nonnegative scalar or 2-element vector in radians, where
        % each element corresponds to a wheel. The wheel order is as
        % follows: [left, right]. The default value is [0 0].
        WheelPositionAccuracy = wheelEncoderDifferentialDrive.WHEEL_POSITION_ACCURACY_DEFAULT * ones(1, wheelEncoderDifferentialDrive.getNumWheels);
        % SlipRatio Ratio of slip of the wheel
        % Specify the amount of slip or skid of a wheel as a scalar or
        % 2-element vector with values greater than or equal to -1, where
        % each element corresponds to a wheel. The wheel order is as
        % follows: [left, right]. A negative value corresponds to the wheel
        % skidding or under rotation, with -1 corresponding to no wheel
        % movement. A positive value corresponds to the wheel slipping or
        % over rotation, with a larger slip ratio corresponding to more
        % wheel slippage. A value of zero corresponds to no slipping or
        % skidding. The default value is [0 0].
        SlipRatio = wheelEncoderDifferentialDrive.SLIP_RATIO_DEFAULT * ones(1, wheelEncoderDifferentialDrive.getNumWheels);
        
        % TrackWidth Distance between wheels on axle (m)
        % Specify the track width as a positive scalar in meters. The
        % default value is 1.572.
        TrackWidth = 1.572;
        % TrackWidthBias Bias of the track width (m)
        % Specify the track width bias as a scalar in meters. The default
        % value is 0.
        TrackWidthBias = 0;
    end
    
    properties (Access = protected, Constant)
        NUM_TRACKS = 1;
    end
    
    properties (Constant, Hidden)
        SampleRateUnits = 'Hz';
        TrackWidthUnits = 'm';
        TrackWidthBiasUnits = 'm';
        WheelRadiusUnits = 'm';
        WheelRadiusBiasUnits = 'm';
        WheelPositionAccuracyUnits = 'rad'
    end
    
    methods
        function set.TicksPerRevolution(obj, val)
            nav.internal.WheelEncoderInterface.validateTicksPerRevolution(val, wheelEncoderDifferentialDrive.getNumWheels);
            obj.TicksPerRevolution(:) = val(:);
        end
        function set.WheelRadius(obj, val)
            nav.internal.WheelEncoderInterface.validateWheelRadius(val, wheelEncoderDifferentialDrive.getNumWheels);
            obj.WheelRadius(:) = val(:);
        end
        function set.WheelRadiusBias(obj, val)
            nav.internal.WheelEncoderInterface.validateWheelRadiusBias(val, wheelEncoderDifferentialDrive.getNumWheels);
            obj.WheelRadiusBias(:) = val(:);
        end
        function set.WheelPositionAccuracy(obj, val)
            nav.internal.WheelEncoderInterface.validateWheelPositionAccuracy(val, wheelEncoderDifferentialDrive.getNumWheels);
            obj.WheelPositionAccuracy(:) = val(:);
        end
        function set.SlipRatio(obj, val)
            nav.internal.WheelEncoderInterface.validateSlipRatio(val, wheelEncoderDifferentialDrive.getNumWheels);
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
    end
    
    methods (Access = protected)
        function wheelDistance = vehicleSpeedToWheelDistance(obj, speed, omega)
            % Convert vehicle speed (m/s) to distance traveled by each
            % wheel (m).
            dt = obj.pDeltaTime;
            trackWidth = obj.TrackWidth + obj.TrackWidthBias;
            wheelDistance = dt*(speed + (omega .* trackWidth/2).*[-1, 1]);
        end
        
        function displayScalarObject(obj)
            displayScalarObjectWithUnits(obj);
        end
    end
    
    methods (Access = protected, Static)
        function w = getNumWheels
            w = 2;
        end
    end
end

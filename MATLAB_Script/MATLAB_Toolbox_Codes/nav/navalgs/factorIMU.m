classdef factorIMU
%FACTORIMU Represents a factor that converts raw IMU readings into
%   constraints between poses, velocities and IMU biases. 
%
%   F = FACTORIMU(ID,SAMPLERATE,GYROBIASNOISE,ACCELBIASNOISE,
%   GYRONOISE,ACCELNOISE,GYROREADINGS,ACCELREADINGS) creates a factorIMU
%   object, F, with the node identification number set to ID, and with the
%   sample rate, gyroscope bias noise, accelerometer bias noise, gyroscope
%   noise, accelerometer noise, gyroscope readings, and accelerometer
%   readings all set to the corresponding values, respectively.
%
%   F = FACTORIMU(...,Name=Value,...) returns a factorIMU object, F, with
%   each specified property name set to the specified value. You can
%   specify additional name-value pair arguments in any order as
%   (Name1=Value1,...,NameN=ValueN).
%
%   FACTORIMU methods:
%      nodeType          - Retrieve the node type for a specified node ID
%      predict           - Estimate pose and velocity based on raw
%                          measurements
%       
%   FACTORIMU properties:
%      NodeID                    - Node ID number for factor graph
%      SampleRate                - IMU sampling rate
%      GyroscopeBiasNoise        - Process noise for gyroscope bias
%      AccelerometerBiasNoise    - Process noise for accelerometer bias
%      GyroscopeNoise            - Gyroscope measurement noise
%      AccelerometerNoise        - Accelerometer measurement noise
%      GyroscopeReadings         - Collected raw gyroscope readings
%      AccelerometerReadings     - Collected raw accelerometer readings
%      ReferenceFrame            - Reference frame
%
%   Example:
%      % Add an IMU factor to a factor graph
%      nodeID = [1,2,3,4,5,6];
%      sampleRate = 400; % Hz
%      gyroBiasNoise = 1.5e-9 * eye(3);
%      accelBiasNoise = diag([9.62e-9,9.62e-9,2.17e-8]);
%      gyroNoise = 6.93e-5 * eye(3);
%      accelNoise = 2.9e-6 * eye(3);
%         
%      gyroReadings = [ -0.0151    0.0299    0.0027
%                       -0.0079    0.0370   -0.0014
%                       -0.0320    0.0306    0.0035
%                       -0.0043    0.0340   -0.0066
%                       -0.0033    0.0331   -0.0011];
%      accelReadings = [   1.0666    0.0802    9.9586
%                          1.1002    0.0199    9.6650
%                          1.0287    0.3071   10.1864
%                          0.9077   -0.2239   10.2989
%                          1.2322    0.0174    9.8411];
%         
%      f = factorIMU( ...
%                   nodeID, sampleRate, gyroBiasNoise, ...
%                   accelBiasNoise, gyroNoise, accelNoise, ...
%                   gyroReadings, accelReadings, ...
%                   ReferenceFrame="NED" ...
%                   );
%         
%      G = factorGraph;
%      G.addFactor(f)
%
%   References:
%
%   [1] C. Foster, L. Carlone, F. Dellaert and D. Scaramuzza, "On-Manifold
%       Preintegration for Real-Time Visual-Inertial Odometry," IEEE 
%       Transactions on Robotics, Vol. 33, No. 1, pp. 1-21, Feb. 2017,
%       doi: 10.1109/TRO.2016.2597321
%
%
%   See also factorGraph

%   Copyright 2021 The MathWorks, Inc.

    properties (Hidden, Constant)
        FactorType = "IMU_F";
    end

    properties (SetAccess=protected)
        %NodeID Node ID numbers this factor connects in the factor graph
        %   Expects a 1-by-6 vector.
        %
        %   Must be specified at construction
        NodeID

        %SampleRate IMU sampling rate in Hz. Expects a positive scalar 
        %   larger than 100.
        %
        %   Must be specified at construction
        SampleRate

        %GyroscopeBiasNoise  Gyroscope bias process noise covariance
        %   Expects a 3-by-3 matrix.
        %
        %   Must be specified at construction   
        GyroscopeBiasNoise

        %AccelerometerBiasNoise Accelerometer bias process noise covariance
        %   Expects a 3-by-3 matrix.
        %
        %   Must be specified at construction
        AccelerometerBiasNoise 

        %GyroscopeNoise Gyroscope measurement noise covariance 
        %   Expects a 3-by-3 matrix.
        %
        %   Must be specified at construction
        GyroscopeNoise

        %AccelerometerNoise Accelerometer measurement noise covariance
        %   Expects a 3-by-3 matrix.
        %
        %   Must be specified at construction
        AccelerometerNoise

        %GyroscopeReadings A collection of raw gyroscope readings to be
        %   pre-integrated. Expects an N-by-3 matrix, where N is the number
        %   of readings. GyroscopeReadings and AccelerometerReadings must
        %   have the same size.
        %
        %   Must be specified at construction
        GyroscopeReadings

        %AccelerometerReadings A collection of raw accelerometer readings
        %   to be pre-integrated. Expects an N-by-3 matrix, where N is the 
        %   number of readings. GyroscopeReadings and AccelerometerReadings 
        %   must have the same size.
        %
        %   Must be specified at construction
        AccelerometerReadings
    end

    properties

        %ReferenceFrame Reference frame
        %   Specify the reference frame for the local coordinate system as
        %   "ENU" (East-North-Up) or "NED" (North-East-Down).
        %   
        %   Default: "ENU"
        ReferenceFrame = "ENU"

    end

    methods
        function obj = factorIMU(ids, sampleRate, gyroBiasNoise, accelBiasNoise, ...
                                gyroNoise, accelNoise, gyroReadings, accelReadings, varargin)
            %factorIMU Constructor
            narginchk(8,10);

            % input validation
            nav.algs.internal.validation.validateNodeID_FactorConstruction(ids, 6, 'factorIMU', 'ids');
            obj.NodeID = double(ids);
            
            validateattributes(sampleRate, 'numeric', ...
                {'scalar', 'real', 'nonempty','finite','nonnan','nonsparse', '>=', 100}, 'factorIMU', 'sampleRate');
            obj.SampleRate = double(sampleRate);

            validateattributes(gyroBiasNoise, 'numeric', ...
                {'size', [3, 3], 'real', 'finite','nonnan', 'nonsparse'}, 'factorIMU', 'gyroBiasNoise');
            obj.GyroscopeBiasNoise = double(gyroBiasNoise);

            validateattributes(accelBiasNoise, 'numeric', ...
                {'size', [3, 3], 'real', 'finite', 'nonnan', 'nonsparse'}, 'factorIMU', 'accelBiasNoise');
            obj.AccelerometerBiasNoise = double(accelBiasNoise);

            validateattributes(gyroNoise, 'numeric', ...
                {'size', [3, 3], 'real', 'finite', 'nonnan', 'nonsparse'}, 'factorIMU', 'gyroNoise');
            obj.GyroscopeNoise = double(gyroNoise);

            validateattributes(accelNoise, 'numeric', ...
                {'size', [3, 3], 'real', 'finite', 'nonnan', 'nonsparse'}, 'factorIMU', 'accelNoise');
            obj.AccelerometerNoise = double(accelNoise);

            validateattributes(gyroReadings, 'numeric', ...
                {'2d', 'ncols', 3, 'real', 'nonempty','finite','nonnan', 'nonsparse'}, 'factorIMU', 'gyroReadings');
            validateattributes(accelReadings, 'numeric', ...
                {'2d', 'ncols', 3, 'real', 'nonempty','finite','nonnan', 'nonsparse'}, 'factorIMU', 'accelReadings');

            coder.internal.errorIf(size(gyroReadings,1) ~= size(accelReadings, 1), ...
                'nav:navalgs:factors:MismatchedIMUReadings');
            obj.GyroscopeReadings = double(gyroReadings);
            obj.AccelerometerReadings = double(accelReadings);

            obj = matlabshared.fusionutils.internal.setProperties(obj, nargin-8, varargin{:});
        end

        function [predictedPose, predictedVel] = predict(obj, prevPose, prevVel, prevBias)
            %predict Predict pose and velocity based on the previously
            %   estimated pose, velocity, and IMU biases and the collected
            %   raw IMU readings as saved in GyroscopeReadings and 
            %   AccelerometerReadings properties.
            %
            %   [PREDICTEDPOSE,PREDICTEDVEL] = PREDICT(F,PREVPOSE,PREVVEL,
            %   PREVBIAS) updates the pose, PREDICTEDPOSE, and velocity, 
            %   PREDICTEDVEL, based on the IMU readings and the initial 
            %   values, PREVPOSE, PREVVEL, and PREVBIAS. PREVPOSE is a 
            %   7-element vector containing the 3D position and the
            %   orientation quaternion. PREVVEL is a 3-element vector
            %   containing the 3D velocity. PREVBIAS is a 6-element vector
            %   containing the gyroscope and accelerometer 3D biases.

            internalObj = obj.createBuiltinObject();
            prevPose = [prevPose(1:3), prevPose(5:7), prevPose(4)];
            result = internalObj.predict(prevPose, prevVel, prevBias);
            poseTmp = result.PredictedPose;
            predictedPose = [poseTmp(1:3), poseTmp(7), poseTmp(4:6)] ;
            predictedVel = result.PredictedVel;
        end

        function obj = set.ReferenceFrame(obj, refFrame)
            %set.ReferenceFrame
            obj.ReferenceFrame = validatestring(refFrame, ["ENU", "NED"], 'factorIMU','refFrame');
        end

        function type = nodeType(obj, id)
            %nodeType Retrieve the node type for a specified node ID.
            narginchk(2,2);
            nav.algs.internal.validation.validateNodeID_FactorQuery(id, obj.NodeID, 'factorIMU', 'id');
            if find(obj.NodeID == id) == 1 || find(obj.NodeID == id) == 4
                type = nav.internal.factorgraph.NodeTypes.SE3; % for the first and fourth IDs
            elseif find(obj.NodeID == id) == 2 || find(obj.NodeID == id) == 5
                type = nav.internal.factorgraph.NodeTypes.Velocity3; % for the second and fifth IDs
            else
                type = nav.internal.factorgraph.NodeTypes.IMUBias; % for the third and sixth IDs
            end
        end 
    end

    methods (Access=?factorGraph)
        function internalObj = createBuiltinObject(obj)
            %createBuiltinObject
            gyroBiasN = obj.GyroscopeBiasNoise';
            accelBiasN = obj.AccelerometerBiasNoise';
            gyroN = obj.GyroscopeNoise';
            accelN = obj.AccelerometerNoise';
            gyroRaw = obj.GyroscopeReadings';
            accelRaw = obj.AccelerometerReadings';

            % Define gravitational vector to be added to rotated
            % accelerometer readings to obtain linear accelerations without
            % gravity.
            if strcmp(obj.ReferenceFrame, "ENU")
                gravitationalAcceleration = [0,0,fusion.internal.ConstantValue.Gravity];
            else % strcmp(obj.ReferenceFrame, "NED")
                gravitationalAcceleration = [0,0, -fusion.internal.ConstantValue.Gravity];
            end
            
            internalObj = ...
                nav.algs.internal.builtin.FactorIMU(int32(obj.NodeID), ...
                obj.SampleRate, gravitationalAcceleration, ...
                gyroBiasN(:), ...
                accelBiasN(:), ...
                gyroN(:), ...
                accelN(:), ...
                gyroRaw(:), ...
                accelRaw(:));
        end

    end


end
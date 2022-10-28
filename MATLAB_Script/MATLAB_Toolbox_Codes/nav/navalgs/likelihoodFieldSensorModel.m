classdef likelihoodFieldSensorModel < handle
%LIKELIHOODFIELDSENSORMODEL Create likelihood field range sensor model
%   LIKELIHOODFIELDSENSORMODEL creates a likelihood field sensor model
%   object for range sensors. This object contains specific sensor
%   model parameters. You can use this object to specify the sensor
%   model parameters in the monteCarloLocalization object.
%
%   LF = likelihoodFieldSensorModel creates a likelihood field
%   sensor model object for range sensors.
%
%   likelihoodFieldSensorModel properties:
%       Map                        - Occupancy grid representing the map
%       SensorPose                 - Pose of range sensor relative to the robot
%       SensorLimits               - Minimum and maximum range limits of the sensor in meters
%       NumBeams                   - Number of beams to be used for likelihood computation
%       MeasurementNoise           - Standard deviation for measurement noise
%       RandomMeasurementWeight    - Weight for probability of random measurement
%       ExpectedMeasurementWeight  - Weight for probability of expected measurement
%       MaxLikelihoodDistance      - Maximum distance to find nearest obstacle
%
%   Example:
%
%       % Create likelihood field sensor model object
%       sensorModel = likelihoodFieldSensorModel;
%
%   See also monteCarloLocalization, odometryMotionModel

%   Copyright 2015-2019 The MathWorks, Inc.
%
%   References:
%
%   [1] S. Thrun, W. Burgard and D. Fox, Probabilistic Robotics.
%   Cambridge, MA: MIT Press, 2005.

%#codegen

    properties (Dependent)
        %Map Occupancy grid representing the map
        %   Map is a occupancyMap or a
        %   binaryOccupancyMap object. The property
        %   contains a map assigned for likelihood computation.
        %
        %   Default: Empty binaryOccupancyMap object
        Map
    end

    properties
        %SensorPose Pose of the range sensor relative to the robot
        %   A vector defining the pose of the range sensor relative to the
        %   robot's coordinate frame as [X Y Yaw], where X, Y are
        %   translations along X and Y axes in meters, and Yaw is rotation
        %   along Z-axis in radians.
        %
        %   Default: [0 0 0]
        SensorPose = [0 0 0]

        %SensorLimits Minimum and maximum range of sensor in meters
        %   A vector [MIN MAX] representing the measurement limits of the
        %   range sensor in meters. The sensor cannot detect obstacles closer
        %   than MIN distance and further than MAX distance.
        %
        %   Default: [0 12]
        SensorLimits = [0 12]

        %NumBeams Number of beams to be used for likelihood computation
        %   The number of beams to be used for the likelihood computation.
        %   The computational efficiency can be improved by specifying a
        %   smaller value to NumBeams than the actual number of beams
        %   available from the sensor.
        %
        %   Default: 60
        NumBeams = 60

        %MeasurementNoise Standard deviation for measurement noise
        %   The standard deviation of the zero mean Gaussian representing
        %   the noise in measurements.
        %
        %   Default: 0.2
        MeasurementNoise = 0.2

        %RandomMeasurementWeight Weight for probability of random measurement
        %   The weight on the probability of getting a random range
        %   measurement.
        %
        %   Default: 0.05
        RandomMeasurementWeight = 0.05

        %ExpectedMeasurementWeight Weight for probability of expected measurement
        %   The weight on the probability of getting a correct range
        %   measurement within the noise limits specified in
        %   MeasurementNoise property.
        %
        %   Default: 0.95
        ExpectedMeasurementWeight = 0.95

        %MaxLikelihoodDistance Maximum distance to find nearest obstacle
        %   The maximum distance to search for nearest obstacles on the
        %   map, specified in meters.
        %
        %   Default: 2.0
        MaxLikelihoodDistance = 2.0
    end

    properties (Access = private)
        %InternalMap Internal property that holds the map object
        InternalMap = binaryOccupancyMap.empty
    end

    methods
        function set.Map(obj, map)
            validateattributes(map, {'binaryOccupancyMap', ...
                                'occupancyMap'}, {'scalar'});
            obj.InternalMap = map.copy;
        end

        function map = get.Map(obj)
            map = obj.InternalMap.copy;
        end

        function set.SensorPose(obj, value)
            validateattributes(value, {'double'}, ...
                               {'real', 'nonnan', 'finite', 'vector', 'numel', 3});
            obj.SensorPose = value(:).';
        end

        function set.SensorLimits(obj, value)
            validateattributes(value, {'double'}, ...
                               {'real', 'nonnan', 'finite', 'nonnegative', 'numel', 2});
            obj.SensorLimits = [min(value) max(value)];
        end

        function set.NumBeams(obj, value)
            validateattributes(value, {'double'}, ...
                               {'scalar', 'nonnan', 'finite', 'positive', 'integer'});
            obj.NumBeams = value;
        end

        function set.MeasurementNoise(obj, value)
            validateattributes(value, {'double'}, ...
                               {'real', 'nonnan', 'scalar', 'finite', 'nonnegative'});
            obj.MeasurementNoise = value;
        end

        function set.RandomMeasurementWeight(obj, value)
            validateattributes(value, {'double'}, ...
                               {'real', 'nonnan', 'scalar', 'finite', 'nonnegative'});
            obj.RandomMeasurementWeight = value;
        end

        function set.ExpectedMeasurementWeight(obj, value)
            validateattributes(value, {'double'}, ...
                               {'real', 'nonnan', 'scalar', 'finite', 'nonnegative'});
            obj.ExpectedMeasurementWeight = value;
        end

        function set.MaxLikelihoodDistance(obj, value)
            validateattributes(value, {'double'}, ...
                               {'real', 'nonnan', 'scalar', 'finite', 'nonnegative'});
            obj.MaxLikelihoodDistance = value;
        end
    end


    methods(Hidden)
        function cpObj = clone(obj)
        %clone Hidden clone method to create a copy of this class

            cpObj = likelihoodFieldSensorModel;

            % Save all public properties
            cpObj.SensorPose = obj.SensorPose;
            cpObj.SensorLimits = obj.SensorLimits;
            cpObj.NumBeams = obj.NumBeams;
            cpObj.MeasurementNoise = obj.MeasurementNoise;
            cpObj.RandomMeasurementWeight = obj.RandomMeasurementWeight;
            cpObj.ExpectedMeasurementWeight = obj.ExpectedMeasurementWeight;
            cpObj.MaxLikelihoodDistance = obj.MaxLikelihoodDistance;

            % Make a deep copy of the handles
            cpObj.InternalMap = obj.InternalMap.copy;
        end
    end
end

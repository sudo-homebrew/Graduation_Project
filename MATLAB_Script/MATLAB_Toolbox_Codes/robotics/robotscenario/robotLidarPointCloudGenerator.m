classdef robotLidarPointCloudGenerator < matlab.System
%robotLidarPointCloudGenerator Generate point cloud from meshes
%   The robotLidarPointCloudGenerator System object generates detections
%   from a statistical simulated lidar sensor. The system object uses a
%   statistical sensor model to simulate lidar detections with added random
%   noise. All detections are with respect to the coordinate frame of the
%   vehicle-mounted sensor.
%
%   LIDAR = ROBOTLIDARPOINTCLOUDGENERATOR creates a statistical sensor
%   model to generate point cloud for a lidar. This sensor model will have
%   default properties.
%
%   LIDAR = ROBOTLIDARPOINTCLOUDGENERATOR(Name=Value) sets properties
%   using one or more name-value pair arguments.
%
%   robotLidarPointCloudGenerator properties:
%     UpdateRate                - Update rate of lidar sensor
%     MaxRange                  - Maximum detection range
%     RangeAccuracy             - Accuracy of range measurements
%     AzimuthResolution         - Azimuthal resolution of lidar sensor
%     ElevationResolution       - Elevation resolution of lidar sensor
%     AzimuthLimits             - Azimuthal limits of lidar sensor
%     ElevationLimits           - Elevation limits of lidar sensor
%     HasNoise                  - Add noise to lidar sensor measurements
%     HasOrganizedOutput        - Output generated data as organized point
%                                 cloud locations
%
%   To generate lidar point clouds, create the
%   robotLidarPointCloudGenerator object and set its properties, call the
%   object with arguments, as if it were a function using the following
%   syntax:
%
%   PTCLOUD = lidar(TGTS, SIMTIME) generates a lidar pointCloud object
%   PTCLOUD from the specified target object data, TGTS, at the current
%   simulation time SIMTIME. pointCloud object stores 3-D point cloud.
%   Target object data is specified as a structure or structure array. The
%   current simulation time is specified as a positive real scalar in
%   seconds. The lidar object calls the lidar point cloud generator at
%   regular intervals to generate new point clouds at a frequency defined
%   by the updateRate property.
%
%   The fields of the TGTS structure are:
%
%       Position        A 3-element vector defining the xyz-position of
%                       the target with respect to the sensor frame.
%
%       Orientation     A quaternion object or a 3-by-3 matrix defining
%                       the orientation of the target with respect to
%                       the sensor frame.
%
%       Mesh            An extendedObjectMesh object representing the
%                       geometry of the target in its own coordinate frame.
%
%   [PTCLOUD,ISVALIDTIME] = lidar(TGTS,SIMTIME) additionally returns
%   ISVALIDTIME which specifies if the specified SIMTIME is a multiple of
%   the sensor's update interval (1/UpdateRate).
%
%   Note:
%       The Location property of the returned PTCLOUD object is reported in
%       the robotSensor coordinate frame.
%
%   Example:
%
%       % Create a default lidar sensor
%       lidar = robotLidarPointCloudGenerator("HasOrganizedOutput", false);
%
%       % Create a floor
%       tgts.Mesh = scale(extendedObjectMesh('cuboid'), [100,100,2]);
%       % setup floor's relative position and orientation to the lidar
%       tgts.Position = [0 0 -10];
%       tgts.Orientation = quaternion([1 0 0 0]);
%
%       % Generate point cloud from floor
%       ptcloud = lidar(tgts, 0);
%
%       % Visualize the point cloud and mesh
%       figure;
%       show(translate(tgts.Mesh, tgts.Position));
%       hold on
%       scatter3(ptcloud.Location(:,1), ptcloud.Location(:,2), ...
%       ptcloud.Location(:,3));
%
%   See also robotSensor, robotScenario, robotPlatform

%   Copyright 2021-2022 The MathWorks, Inc.

    properties (Dependent, Nontunable)
        %UpdateRate Update rate of lidar sensor
        %   Update rate of the lidar sensor, specified as a positive real
        %   scalar in Hz. This property sets the frequency at which new
        %   detections happen.
        %
        %   Default: 10
        UpdateRate

        %MaxRange Maximum detection range of lidar sensor
        %   Maximum detection range of the lidar sensor, specified as a
        %   positive real scalar in meters. The sensor does not detect
        %   objects beyond this range.
        %
        %   Default: 120
        MaxRange

        %RangeAccuracy Accuracy of range measurements
        %   Accuracy of the range measurements, specified as a positive
        %   real scalar in meters. This property sets the
        %   one-standard-deviation accuracy of the sensor range
        %   measurements.
        %
        %   Default: 0.002
        RangeAccuracy

        %AzimuthResolution Azimuthal resolution of lidar sensor
        %  Azimuthal resolution of lidar sensor, specified as a positive
        %  real scalar in degrees. The azimuthal resolution defines the
        %  minimum separation in azimuth angle at which the lidar sensor
        %  can distinguish two targets.
        %
        %   Default: 0.16
        AzimuthResolution

        %ElevationResolution Elevation resolution of lidar sensor
        %   Elevation resolution of lidar sensor, specified as a positive
        %   real scalar with units in degrees. The elevation resolution
        %   defines the minimum separation in elevation angle at which the
        %   lidar can distinguish two targets.
        %
        %   Default: 1.25
        ElevationResolution

        %AzimuthLimits Azimuthal limits of lidar sensor
        %   Azimuth limits of the lidar sensor, specified as a two-element
        %   vector of the form [min max]. Units are in degrees.
        %
        %   Default: [-180 180]
        AzimuthLimits

        %ElevationLimits Elevation limits of lidar sensor
        %   Elevation limits of the lidar sensor, specified as a
        %   two-element vector of the form [min max]. Units are in degrees.
        %
        %   Default: [-20 20]
        ElevationLimits

        %HasNoise Add noise to lidar sensor measurements
        %   Add noise to lidar sensor measurements, specified as true or
        %   false. Set this property to true to add noise to the sensor
        %   measurements. Otherwise, the measurements have no noise. The
        %   sensor adds random Gaussian noise to each point with mean equal
        %   to zero and standard deviation specified by the RangeAccuracy
        %   property.
        %
        %   Default: true
        HasNoise

        %HasOrganizedOutput Output data as organized point cloud locations
        %   Output generated data as organized point cloud locations,
        %   specified as true or false. When this property is set as true,
        %   the Location property of the pointCloud object is an
        %   M-by-N-by-3 matrix of organized point cloud. M is the number of
        %   elevation channels, and N is the number of azimuth channels.
        %   When this property is set as false, the Location property of
        %   the pointCloud object is an M-by-3 matrix of unorganized list
        %   of points. M is the product of the numbers of elevation and
        %   azimuth channels.
        %
        %   Default: true
        HasOrganizedOutput
    end

    properties (Access = {?robotLidarPointCloudGenerator, ?matlab.unittest.TestCase})
        %SensorImpl Implementation of the point cloud generator
        SensorImpl
    end

    methods
        function obj = robotLidarPointCloudGenerator(varargin)
        % Support name-value pair arguments when constructing object
            obj.SensorImpl = uav.internal.sensor.LidarPointCloudGenerator(varargin{:});
        end

        function set.UpdateRate(obj, r)
            obj.SensorImpl.UpdateRate = r;
        end

        function r = get.UpdateRate(obj)
            r = obj.SensorImpl.UpdateRate;
        end

        function set.MaxRange(obj, r)
            obj.SensorImpl.MaxRange = r;
        end

        function r = get.MaxRange(obj)
            r = obj.SensorImpl.MaxRange;
        end

        function set.RangeAccuracy(obj, r)
            obj.SensorImpl.RangeAccuracy = r;
        end

        function r = get.RangeAccuracy(obj)
            r = obj.SensorImpl.RangeAccuracy;
        end

        function set.AzimuthResolution(obj, r)
            obj.SensorImpl.AzimuthResolution = r;
        end

        function r = get.AzimuthResolution(obj)
            r = obj.SensorImpl.AzimuthResolution;
        end

        function set.ElevationResolution(obj, r)
            obj.SensorImpl.ElevationResolution = r;
        end

        function r = get.ElevationResolution(obj)
            r = obj.SensorImpl.ElevationResolution;
        end

        function set.AzimuthLimits(obj, r)
            obj.SensorImpl.AzimuthLimits = r;
        end

        function r = get.AzimuthLimits(obj)
            r = obj.SensorImpl.AzimuthLimits;
        end
        function set.ElevationLimits(obj, r)
            obj.SensorImpl.ElevationLimits = r;
        end

        function r = get.ElevationLimits(obj)
            r = obj.SensorImpl.ElevationLimits;
        end

        function set.HasNoise(obj, r)
            obj.SensorImpl.HasNoise = r;
        end

        function r = get.HasNoise(obj)
            r = obj.SensorImpl.HasNoise;
        end

        function set.HasOrganizedOutput(obj, r)
            obj.SensorImpl.HasOrganizedOutput = r;
        end

        function r = get.HasOrganizedOutput(obj)
            r = obj.SensorImpl.HasOrganizedOutput;
        end

    end

    methods(Access = protected)

        function setupImpl(obj, varargin)
        % Perform one-time calculations, such as computing constants
            obj.SensorImpl.setup(varargin{:});
        end

        function cobj = cloneImpl(obj)
        % Override system.object clone behavior

            cobj = cloneImpl@matlab.System(obj);
            % clone SensorImpl
            cobj.SensorImpl = clone(obj.SensorImpl);
        end

        function [pt, isvalidtime] = stepImpl(obj, tgts, time)
        % Implement algorithm. Calculate y as a function of input u and
        % discrete states.

            [pt, config] = obj.SensorImpl(tgts, time);
            isvalidtime = config.IsValidTime;
        end

        function resetImpl(obj)
        % Initialize / reset discrete-state properties

            obj.SensorImpl.reset();
        end

        function releaseImpl(obj)
        % Release resources, such as file handles

            obj.SensorImpl.release();
        end

        function loadObjectImpl(obj,s,wasLocked)
        %saveObjectImpl overrides default load behavior
        %   Set properties in object obj to values in structure s

        % Set private properties and states
            obj.SensorImpl = s.SensorImpl;
            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end

        function s = saveObjectImpl(obj)
        %saveObjectImpl overrides default save behavior
        %   Set properties in structure s to values in object obj, then
        %   save s

        % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);
            % Set private properties and states
            s.SensorImpl = obj.SensorImpl;
        end
    end

end

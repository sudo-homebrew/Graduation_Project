classdef robotSensor < handle
%robotSensor Sensor for robot scenario
%   The robotSensor object creates a sensor that is rigidly attached to a
%   robot platform, specified as a robotPlatform object. You can specify
%   different mounting positions and orientations. Configure this object to
%   automatically generate readings at fixed rate from a sensor specified
%   as an insSensor, gpsSensor, or robotLidarPointCloudGenerator System
%   object.
%
%   SENSOR = ROBOTSENSOR(NAME, PLATFORM, SENSORMODEL) creates a sensor with
%   the specified name NAME and sensor model SENSORMODEL, which set the
%   Name and SensorModel properties respectively. The sensor is added to
%   the platform PLATFORM specified as a robotPlatform object. The PLATFORM
%   argument sets the MountingBodyName property.
%
%   SENSOR = ROBOTSENSOR(___, Name=Value) sets properties using one or
%   more name-value pair arguments in addition to the input arguments in
%   the previous syntax. You can specify the MountingLocation,
%   MountingAngles, or UpdateRate properties as name-value pairs.
%
%   robotSensor properties:
%       Name                   - Sensor name
%       MountingAngles         - Sensor orientation on platform
%       MountingBodyName       - Name of sensor mounted platform body
%       MountingLocation       - Sensor position on platform
%       UpdateRate             - Update rate of sensor
%       SensorModel            - Sensor model for generating readings
%
%   robotSensor methods:
%       read                   - Gather latest reading from robot sensor
%
%   Example:
%
%      % Create scenario and platform.
%      scenario = robotScenario(UpdateRate=1, StopTime=5);
%      platform = robotPlatform("ROBOT", scenario);
%
%      % Create a GPS sensor and attach to platform.
%      gps = robotSensor("GPS", platform, ...
%                               gpsSensor(VelocityAccuracy=0),...
%                               MountingBodyName="ROBOT", ...
%                               UpdateRate=scenario.UpdateRate);
%
%      % Gather initial readings at time 0.
%      setup(scenario)
%      [isUpdated, t, lla, vel, groundspeed, course] = read(gps);
%
%      % Simulate the scenario and gather readings.
%      while advance(scenario)
%           updateSensors(scenario)
%           [isUpdated, t, lla, vel, groundspeed, course] = read(gps);
%      end
%
%   See also robotScenario, robotPlatform, insSensor, gpsSensor,
%   robotLidarPointCloudGenerator

%   Copyright 2021-2022 The MathWorks, Inc.

    properties (Dependent, SetAccess = private)
        %Name Sensor name
        %   Sensor name, specified as a string scalar. Choose a name to
        %   identify this specific sensor.
        Name

        %SensorModel Sensor model for generating readings
        %   Sensor model for generating readings, specified as an
        %   insSensor, gpsSensor, or robotLidarPointCloudGenerator System
        %   object.
        SensorModel

        %MountingLocation Sensor position on platform
        %   Sensor position on platform, specified as a vector of the form
        %   [x y z] in the platform frame. Units are in meters.
        %
        %   Default: [0 0 0]
        MountingLocation

        %MountingAngles Sensor orientation on platform
        %   Sensor orientation on platform, specified as a vector of the
        %   form [z y x] where z, y, and x are rotations about the z-axis,
        %   y-axis, and x-axis, sequentially, in degrees. The orientation
        %   is relative to the platform body frame.
        %
        %   Default: [0 0 0]
        MountingAngles

        %MountingBodyName Name of sensor mounted platform body
        %   Name of sensor mounted platform body, specified as a string
        %   scalar. The RigidBodyTree based robot platform can have
        %   multiple bodies, any valid body can be selected to mount
        %   sensor.
        %
        %   Default: PLATFORM.NAME
        MountingBodyName

        %UpdateRate Update rate of sensor
        %   Update rate of the sensor, specified as a positive scalar in
        %   Hz. By default, the object uses the UpdateRate property of the
        %   specified sensor model object. The sensor update interval
        %   (1/UpdateRate) must be a multiple of the update interval for
        %   the associated robotScenario object.
        UpdateRate

    end

    properties (Access = {?robotSensor, ?uav.internal.scenario.SceneInterface, ?matlab.unittest.TestCase})
        %SensorImpl Implementation of robot sensor
        SensorImpl

        %PlatformImpl Implementation of the robot platform
        PlatformImpl
    end

    methods
        function name = get.Name(obj)
            name = obj.SensorImpl.Name;
        end

        function smodel = get.SensorModel(obj)
            smodel = obj.SensorImpl.SensorImpl.getSensorModel;
        end

        function r = get.UpdateRate(obj)
            r = obj.SensorImpl.UpdateRate;
        end

        function location = get.MountingLocation(obj)
        % get valid mounting body name
            validMountingBodyName = ...
                robotics.internal.scenario.rigidbody.getValidPlatformBodyName(obj.PlatformImpl, obj.MountingBodyName);
            tform = obj.SensorImpl.SensorImpl.TFTree.getTransform(validMountingBodyName, obj.Name);
            location = tform2trvec(tform);
        end

        function angles = get.MountingAngles(obj)
            validMountingBodyName = ...
                robotics.internal.scenario.rigidbody.getValidPlatformBodyName(obj.PlatformImpl, obj.MountingBodyName);
            tform = obj.SensorImpl.SensorImpl.TFTree.getTransform(validMountingBodyName, obj.Name);
            angles = rad2deg(tform2eul(tform));
        end

        function mountBodyName = get.MountingBodyName(obj)
            mountBodyName = obj.SensorImpl.MountingBodyName;
        end
    end

    methods
        function obj = robotSensor(name, platform, sensorModel, varargin)
        %robotSensor

        % validate inputs
            validateattributes(platform, "robotPlatform", "scalar", "robotSensor", "platform");

            name = convertCharsToStrings(name);
            validateattributes(name, ["char", "string"], "scalartext", "robotSensor", "name");
            combinedName = platform.Name+"/"+name;
            if any(strcmp(combinedName, [platform.Sensors.Name]))
                error(message("robotics:robotscenario:scenario:SensorNameNotUnique", name, platform.Name));
            end

            % parse inputs
            p = inputParser;
            p.addParameter("MountingLocation", [0 0 0],...
                           @(x)validateattributes(x, "numeric", {"nrows", 1, "ncols", 3, "2d"}, "robotSensor", "MountingLocation"));
            p.addParameter("MountingAngles", [0 0 0],...
                           @(x)validateattributes(x, "numeric", {"nrows", 1, "ncols", 3, "2d"}, "robotSensor", "MountingAngles"));
            p.addParameter("MountingBodyName", [],...
                           @(x)validateattributes(x, ["char", "string"], "scalartext", "robotSensor", "MountingBodyName"));
            p.addParameter("UpdateRate", [],...
                           @(x)validateattributes(x, "numeric", ["scalar", "positive", "real", "finite"], "robotSensor", "UpdateRate"));
            p.parse(varargin{:});
            tform = trvec2tform(double(p.Results.MountingLocation))*eul2tform(deg2rad(double(p.Results.MountingAngles)));

            if isempty(p.Results.MountingBodyName)
                % default, consider robot platform to mount sensor
                mountingBodyName = platform.PlatformImpl.Name;
            else
                mountingBodyName = p.Results.MountingBodyName;
                % validate input platform body name
                validBodyList = robotics.internal.scenario.rigidbody.getPlatformBodyList(platform);
                if ~any(strcmp(validBodyList, mountingBodyName))
                    error(message("shared_uav_rst:robotuav:scenario:InvalidSensorMountingBodyName", mountingBodyName, strjoin(validBodyList,' , ')));
                end
            end

            if isempty(p.Results.UpdateRate)
                obj.SensorImpl = robotics.internal.scenario.Sensor(platform.ScenarioImpl.Parent, ...
                                                                   platform, combinedName, tform, sensorModel, mountingBodyName);
            else
                obj.SensorImpl = robotics.internal.scenario.Sensor(platform.ScenarioImpl.Parent, ...
                                                                   platform, combinedName, tform, sensorModel, mountingBodyName, ...
                                                                   double(p.Results.UpdateRate));
            end

            sceneRate = platform.ScenarioImpl.UpdateRate;
            sensorRate = obj.SensorImpl.UpdateRate;
            if floor(sceneRate/sensorRate) ...
                    ~= sceneRate/sensorRate
                error(message('robotics:robotscenario:scenario:SensorUpdateRateMismatch',sceneRate,sensorRate));
            end

            obj.PlatformImpl = platform.PlatformImpl;
            platform.PlatformImpl.addSensor(obj.SensorImpl);
            platform.Sensors(end+1) = obj;
        end

        function varargout = read(obj)
        %READ Gather latest reading from robot sensor
        %   [ISUPDATED,T,SENSORREADINGS] = READ(SENSOR) gathers the
        %   simulated sensor output sensor readings from the latest update
        %   of the robot platform associated with the specified sensor
        %   SENSOR. The function returns an indicator ISUPDATED of whether
        %   the reading was updated at the simulation step in the scenario
        %   with timestamp T. The simulated sensor readings SENSORREADINGS
        %   depends on the type of sensor specified in the SENSOR input
        %   argument.
        %
        %   Example:
        %
        %      % Create scenario and platform.
        %      scenario = robotScenario(UpdateRate=1, StopTime=10);
        %      platform = robotPlatform("ROBOT", scenario);
        %
        %      % Add INS sensor on platform.
        %      ins = robotSensor("INS", platform,...
        %                        insSensor(RollAccuracy=0));
        %
        %      % Setup simulation.
        %      setup(scenario)
        %
        %      % Read INS sensor readings.
        %      [isUpdated, t, measurements] = read(ins);

            [varargout{1:nargout}] = obj.SensorImpl.read();
        end
    end
end

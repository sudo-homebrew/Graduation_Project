classdef robotPlatform < handle
%robotPlatform Create robot platform in scenario
%
%   The robotPlatform object represents a robot platform in a given robot
%   scenario. Use the platform to define and track the trajectory of an
%   object in the scenario.
%
%   PLATFORM = ROBOTPLATFORM(NAME, SCENARIO) creates a platform with a
%   specified name NAME and adds it to the scenario, specified as a
%   robotScenario object. Specify the NAME argument as a string scalar. The
%   NAME argument sets the Name property.
%
%   PLATFORM = ROBOTPLATFORM(___, Name=Value) specifies options using one
%   or more name-value pair arguments. You can specify properties as
%   name-value pair arguments as well.
%
%   Name-Value Pair Arguments:
%
%      BaseTrajectory              - Trajectory for robot platform base
%                                    motion, specified as a
%                                    waypointTrajectory object. By default,
%                                    the platform is assumed to be
%                                    stationary and at the scenario origin.
%                                    To move the platform at each
%                                    simulation step of the scenario, use
%                                    the move object function.
%                                     Note: The robotPlatform object must
%                                           specify the same ReferenceFrame
%                                           property as specified in the
%                                           waypointTrajectory object.
%                                    Default: []
%      IsBinaryOccupied            - Occupied state of binary occupancy
%                                    map, specified as true or false. Set
%                                    the value as true if robot platform is
%                                    incorporated in the binary occupancy
%                                    map.
%                                    Default : False
%      InitialBaseAcceleration     - Initial acceleration of robot platform
%                                    base, specified as a vector of the
%                                    form [ax ay az]. Only specify this
%                                    name-value pair if not specifying the
%                                    BaseTrajectory property.
%                                    Default: [0 0 0]
%      InitialBaseAngularVelocity  - Initial angular velocity of robot
%                                    platform base, specified as a vector
%                                    of the form [wx wy wz]. The magnitude
%                                    of the vector defines the angular
%                                    speed in radians per second. The
%                                    xyz-coordinates define the axis of
%                                    clockwise rotation. Only specify this
%                                    name-value pair if not specifying the
%                                    BaseTrajectory property.
%                                    Default: [0 0 0]
%      InitialBaseOrientation      - Initial robot platform base
%                                    orientation, specified as a vector of
%                                    the form [w x y z], representing a
%                                    quaternion. Only specify this
%                                    name-value pair if not specifying the
%                                    BaseTrajectory property.
%                                    Default: [1 0 0 0]
%      InitialBasePosition         - Initial robot platform base position,
%                                    specified as a vector of the form [x y
%                                    z]. Only specify this name-value pair
%                                    if not specifying the BaseTrajectory
%                                    property.
%                                    Default: [0 0 0]
%      InitialBaseVelocity         - Initial velocity of robot platform
%                                    base, specified as a vector of the
%                                    form [vx vy vz]. Only specify this
%                                    name-value pair if not specifying the
%                                    BaseTrajectory property.
%                                    Default: [0 0 0]
%      ReferenceFrame              - Reference frame for computing robot
%                                    platform motion, specified as "ENU" or
%                                    "NED", which matches any reference
%                                    frame in the robotScenario. All
%                                    platform motion is computed relative
%                                    to this inertial frame.
%                                    Default: "ENU"
%      RigidBodyTree               - Rigid body tree robot platform,
%                                    specified as a rigidBodyTree object.
%                                    Default: []
%      StartTime                   - Initial time of the platform
%                                    trajectory, specified as a scalar in
%                                    seconds.
%                                    Default: 0
%
%   robotPlatform properties:
%      Name                - Identifier for robot platform
%      ReferenceFrame      - Reference frame for computing robot platform
%                            motion
%      BaseTrajectory      - Trajectory for base of robot platform
%      BaseMesh            - Robot platform base body mesh
%      BaseMeshColor       - Robot platform base body mesh color
%      BaseMeshTransform   - Transform between robot platform base body and
%                            mesh frames
%      RigidBodyTree       - Rigid body tree robot platform
%      IsBinaryOccupied    - Status of robot platform in binary occupancy
%                            map
%      Sensors             - Sensors mounted on robot platform
%
%   robotPlatform methods:
%      move                - Move robot platform in scenario at current
%                            simulation step
%      read                - Read robot platform motion vector
%      updateMesh          - Update robot platform body mesh
%
%   Example:
%
%      % Create scenario with ground plane.
%      scenario = robotScenario(UpdateRate=1, StopTime=5);
%      addMesh(scenario,"Plane", Size=[5 3], Position=[2 0 0],...
%                         Color=[0.2 0.2 0.2]);
%
%      % Create waypoint trajectory.
%      traj = waypointTrajectory(...
%                         SampleRate=10, ...
%                         TimeOfArrival=[0 2 5],...
%                         Waypoints=[1 0 0; 2 0 0; 3 0 0],...
%                         ReferenceFrame="ENU");
%
%      % Create rigidBodyTree with loadrobot.
%      robot = loadrobot("amrPioneer3AT");
%
%      % Create robot platform with trajectory and rigidBodyTree.
%      platform = robotPlatform("TurtleBot", scenario, ...
%                         BaseTrajectory=traj, RigidBodyTree=robot);
%
%      % Setup the scenario and gather initial platform pose.
%      setup(scenario);
%      motion = read(platform);
%
%      % Visualize the scene.
%      ax = show3D(scenario);
%
%      % Create a rate object running at 1 Hz.
%      r = rateControl(1);
%
%      % Advance the scene, gather platform's motion and visualize.
%      while advance(scenario)
%           motion = read(platform);
%           show3D(scenario, Parent=ax);
%           % Pause Simulation for visualization.
%           waitfor(r);
%      end
%
%   See also robotScenario, robotSensor

%   Copyright 2021-2022 The MathWorks, Inc.

    properties (Dependent, SetAccess = private)
        %Name Identifier for robot platform
        %   Identifier for the robot platform, specified as a string scalar
        %   or character vector. The name must be unique within the
        %   scenario.
        Name

        %ReferenceFrame Reference frame for computing robot platform motion
        %   Reference frame for computing robot platform motion, specified
        %   as "ENU" or "NED", which matches any reference frame in the
        %   robotScenario. The object computes all platform motion relative
        %   to this inertial frame.
        %
        %   Default: "ENU"
        ReferenceFrame

        %BaseTrajectory Trajectory for robot platform base motion
        %   Trajectory for robot platform base motion, specified as a
        %   waypointTrajectory object. By default, the object assumes the
        %   base of the platform is stationary and at the scenario origin.
        %   When specified as a waypointTrajectory object, base of the
        %   platform is moved along the trajectory during the scenario
        %   simulation. To move the platform at each simulation step of the
        %   scenario, use the move object function.
        %
        %   Default: []
        BaseTrajectory

        %BaseMesh Robot platform base body mesh
        %   Robot platform base body mesh, specified as an
        %   extendedObjectMesh object. The body mesh describes the 3-D
        %   model of the platform for visualization purposes. The body mesh
        %   is used to generate 3-D point cloud. The default mesh is a
        %   cuboid of the form [xlength ylength zlength] in meters.
        %
        %   Default: [1 0.5 0.3]
        BaseMesh

        %BaseMeshColor Robot platform base body mesh color
        %   Robot platform base body mesh color when displayed in the
        %   scenario, specified as an RGB triplet
        %
        %   Default: [1 0 0]
        BaseMeshColor

        %BaseMeshTransform Transform between Robot platform base body and
        %mesh frames
        %   Transform between robot platform base body and mesh frame,
        %   specified as a 4-by-4 homogeneous transformation matrix that
        %   maps points in the platform mesh frame to points in the body
        %   frame.
        %
        %   Default: eye(4)
        BaseMeshTransform

        %RigidBodyTree Rigid body tree robot platform
        %   Rigid body tree robot platform, specified as a rigidBodyTree
        %   object.
        %
        %   Default: []
        RigidBodyTree

        %IsBinaryOccupied Status of robot platform in binary occupancy map
        %   Status of robot platform in binary occupancy map, specified as
        %   true or false.
        %
        %   Default: false
        IsBinaryOccupied
    end

    properties (SetAccess = {?robotPlatform, ?robotSensor})
        %Sensors Sensors mounted on robot platform
        %   Sensors mount on robot platform, specified as an array of
        %   robotSensor objects.
        %
        %   Default: []
        Sensors
    end

    properties (Access={?robotPlatform, ...
                        ?robotSensor,...
                        ?robotics.SensorAdaptor, ...
                        ?robotics.internal.scenario.Sensor , ...
                        ?uav.internal.scenario.SensorInterface , ...
                        ?robotics.internal.scenario.sensor.SensorAdaptor, ...
                        ?uav.internal.scenario.sensor.utils.PointCloudGeneratorUtils,...
                        ?matlab.unittest.TestCase})
        %ScenarioImpl Implementation of the robot scenario
        ScenarioImpl

        %PlatformImpl Implementation of the robot platform
        PlatformImpl
    end

    methods

        function traj = get.BaseTrajectory(obj)
            traj = obj.PlatformImpl.BaseTrajectory;
        end

        function name = get.ReferenceFrame(obj)
            name = obj.PlatformImpl.ReferenceFrame;
        end

        function name = get.Name(obj)
            name = obj.PlatformImpl.Name;
        end

        function mesh = get.BaseMesh(obj)
            mesh = obj.PlatformImpl.BaseMesh.Mesh;
        end

        function color = get.BaseMeshColor(obj)
            color = obj.PlatformImpl.BaseMesh.Color;
        end

        function transform = get.BaseMeshTransform(obj)
            transform = obj.PlatformImpl.BaseMesh.Transform;
        end

        function robot = get.RigidBodyTree(obj)
            robot = obj.PlatformImpl.RigidBodyTree;
        end

        function isBinaryOccupied = get.IsBinaryOccupied(obj)
            isBinaryOccupied = obj.PlatformImpl.OccupancyMapDetails.IsBinaryOccupied;
        end
    end

    methods
        function obj = robotPlatform(name, scenario, varargin)
        %robotPlatform

        % validate inputs
            validateattributes(scenario, "robotScenario", "scalar", "robotPlatform", "scenario");
            name = convertCharsToStrings(name);
            validateattributes(name, ["char", "string"], "scalartext", "robotPlatform", "name");
            if any(strcmp(name, [scenario.Platforms.Name]))
                error(message("robotics:robotscenario:scenario:PlatformNameNotUnique", name));
            end

            % parse inputs
            p = inputParser;
            p.addParameter("ReferenceFrame", uav.internal.scenario.Scene.ENUFrameName, ...
                           @(x)validateattributes(x, ["char", "string"], "scalartext", "robotPlatform", "ReferenceFrame"));
            p.addParameter("BaseTrajectory", [],...
                           @(x)validateattributes(x, "waypointTrajectory", "scalar", "robotPlatform", "BaseTrajectory"));
            p.addParameter("InitialBasePosition", [0 0 0], ...
                           @(x)validateattributes(x, "numeric", {"nrows", 1, "ncols", 3, "2d"}, "robotPlatform", "InitialBasePosition"));
            p.addParameter("InitialBaseOrientation", [1 0 0 0]);
            p.addParameter("InitialBaseVelocity", [0 0 0], ...
                           @(x)validateattributes(x, "numeric", {"nrows", 1, "ncols", 3, "2d"}, "robotPlatform", "InitialBaseVelocity"));
            p.addParameter("InitialBaseAcceleration", [0 0 0], ...
                           @(x)validateattributes(x, "numeric", {"nrows", 1, "ncols", 3, "2d"}, "robotPlatform", "InitialBaseAcceleration"));
            p.addParameter("InitialBaseAngularVelocity", [0 0 0], ...
                           @(x)validateattributes(x, "numeric", {"nrows", 1, "ncols", 3, "2d"}, "robotPlatform", "InitialBaseAngularVelocity"));
            p.addParameter("StartTime", scenario.ScenarioImpl.StartTime, ...
                           @(x)validateattributes(x, "numeric", {">=", scenario.ScenarioImpl.StartTime}, "robotPlatform", "StartTime"));
            p.addParameter("RigidBodyTree", {},...
                           @(x)validateattributes(x, "rigidBodyTree", "scalar", "robotPlatform", "RigidBodyTree"));
            p.addParameter("IsBinaryOccupied", false, ...
                           @(x)validateattributes(x,["logical", "numeric"], ["scalar", "binary"], "robotPlatform", "IsBinaryOccupied"));
            p.parse(varargin{:});

            if scenario.IsRunning
                error(message("robotics:robotscenario:scenario:CannotAddPlatformToRunningScenario"));
            end

            obj.ScenarioImpl = scenario.ScenarioImpl;
            traj = p.Results.BaseTrajectory;

            if isempty(traj)
                % validate initial base orientation
                q = robotics.internal.validation.validateQuaternion(...
                    p.Results.InitialBaseOrientation, ...
                    "robotPlatform", "InitialBaseOrientation");

                % initial motion vector
                motion = [double(p.Results.InitialBasePosition), ...
                          double(p.Results.InitialBaseVelocity), ...
                          double(p.Results.InitialBaseAcceleration), ...
                          double(q), ...
                          double(p.Results.InitialBaseAngularVelocity)];

                obj.PlatformImpl = robotics.internal.scenario.Platform(...
                    obj.ScenarioImpl, ...
                    p.Results.ReferenceFrame, name, ...
                    double(p.Results.StartTime), motion, p.Results.RigidBodyTree,...
                    p.Results.IsBinaryOccupied);
            else
                % error-out if platform and trajectory reference frames are
                % mismatched
                if ~strcmp(p.Results.ReferenceFrame, traj.ReferenceFrame)
                    error(message("robotics:robotscenario:scenario:MismatchTrajectoryReferenceFrame", ...
                                  traj.ReferenceFrame, p.Results.ReferenceFrame));
                end
                obj.PlatformImpl = robotics.internal.scenario.Platform(...
                    obj.ScenarioImpl, p.Results.ReferenceFrame, name, ...
                    double(p.Results.StartTime), traj, p.Results.RigidBodyTree,...
                    p.Results.IsBinaryOccupied, varargin);
            end
            obj.ScenarioImpl.addPlatform(obj.PlatformImpl);
            obj.Sensors = robotSensor.empty;
            scenario.Platforms(end+1) = obj;
        end

        function updateMesh(obj, type, varargin)
        %updateMesh Update body mesh of robot platform
        %   UPDATEMESH(PLATFORM,TYPE,NAME=VALUE) updates the body mesh of
        %   the robot platform by specifying the mesh type as "Cuboid",
        %   "Custom", "GroundVehicle" or "RigidBodyTree" and specifies
        %   additional options using name-value arguments.
        %
        %   Name-Value arguments:
        %
        %       Color            - Robot platform body mesh color,
        %                          specified as a RGB triplet, except for
        %                          RigidBody mesh.
        %                          Default : [1 0 0]
        %       Faces            - Faces of the custom robot platform mesh,
        %                          specified as an N-by-3 matrix of
        %                          positive integers. The three elements in
        %                          each row are the indices of the three
        %                          points in the vertices forming a
        %                          triangle face. N is the number of faces.
        %       IsBinaryOccupied - Occupied state of binary occupancy map,
        %                          specified as true or false. Set the
        %                          value as true if robot platform is
        %                          incorporated in the binary occupancy
        %                          map.
        %                          Default : False
        %       Object           - Rigid body tree robot platform,
        %                          specified as a rigidBodyTree object.
        %                          Default : []
        %       Offset           - Transformation of mesh relative to the
        %                          body frame, specified as a 4-by-4
        %                          homogeneous transformation matrix. The
        %                          matrix maps points in the platform mesh
        %                          frame to points in the body frame.
        %       Orientation      - Relative mesh orientation in the body
        %                          frame, specified as a quaternion vector
        %                          of the form [w x y z] or a quaternion
        %                          object.
        %                          Default : [1 0 0 0]
        %       Position         - Relative mesh position in the body
        %                          frame, specified as a vector of the form
        %                          [x y z] in meters.
        %                          Default : [0 0 0]
        %       Scale            - Scale of the GroundVehicle robot
        %                          platform mesh, specified as a scalar.
        %                          Scale is unitless.
        %                          Default : 1
        %       Size             - Size of the cuboid robot platform mesh,
        %                          specified as a three-element vector of
        %                          form [LENGTHX, LENGTHY, LENGTHZ] in
        %                          meters.
        %                          Default : [1 0.5 0.3]
        %       Vertices         - Vertices of the custom robot platform
        %                          mesh, specified as an N-by-3 matrix of
        %                          real scalars. The first, second, and
        %                          third element of each row represents the
        %                          x-, y-, and z-position of each vertex,
        %                          respectively. N is the number of
        %                          vertices.
        %
        %   Example:
        %
        %      % Create scenario.
        %      scenario = robotScenario(UpdateRate=1,StopTime=10);
        %
        %      % Create three platforms.
        %      platformA = robotPlatform("RobotA", scenario);
        %      platformB = robotPlatform("RobotB", scenario);
        %      platformC = robotPlatform("RobotC", scenario);
        %
        %      % Create a rigid body tree.
        %      robot = loadrobot("amrPioneer3AT");
        %
        %      % Update platforms.
        %      updateMesh(platformA,"GroundVehicle",...
        %                           Scale=2, Position=[2 3 0]);
        %      updateMesh(platformB,"Cuboid", Size=[1.5 1 0.5],...
        %                           Color=[1 0.5 0],Position=[4 5 0]);
        %      updateMesh(platformC,"RigidBodyTree", Object=robot,...
        %                           Position=[1 1 0]);
        %
        %
        %      % Visualize the scenario.
        %      show3D(scenario);

            type = validatestring(type, uav.internal.scenario.Constants.RobotBodyMeshType, "updateMesh", "type");

            p = inputParser;
            addParameter(p,'Position',[0 0 0],...
                         @(x)validateattributes(x,"numeric",{"2d", "nrows", 1, "ncols", 3, "real", "finite"},"updateMesh", "Position"));
            addParameter(p,'Orientation',[1 0 0 0],...
                         @(x)validateattributes(x,"numeric",{"2d", "nrows", 1, "ncols", 4, "real", "finite"},"updateMesh", "Orientation"));
            addParameter(p,'IsBinaryOccupied', {}, ...
                         @(x)validateattributes(x,["logical", "numeric"], ["scalar", "binary"], "updateMesh", "IsBinaryOccupied"));
            addParameter(p,'Offset',{});

            switch type
              case "GroundVehicle"
                addParameter(p,'Color', [1 0 0],...
                             @(x)validateattributes(x,"numeric",{"2d", "nrows", 1, "ncols", 3, "real", "finite",">=", 0, "<=", 1},"updateMesh", "Color"));
                addParameter(p, 'Scale', 1,...
                             @(x)validateattributes(x,"numeric",["scalar", "positive", "finite"], "updateMesh", "Scale"));
              case "Cuboid"
                addParameter(p,'Color', [1 0 0],...
                             @(x)validateattributes(x,"numeric",{"2d", "nrows", 1, "ncols", 3, "real", "finite",">=", 0, "<=", 1},"updateMesh", "Color"));
                addParameter(p,'Size', ...
                             uav.internal.scenario.Constants.DefaultCuboidSize, ...
                             @(x)validateattributes(x,"numeric",{"2d", "nrows", 1, "ncols", 3, "positive", "finite"},"updateMesh", "Size"));
              case "Custom"
                addParameter(p,'Color', [1 0 0],...
                             @(x)validateattributes(x,"numeric",{"2d", "nrows", 1, "ncols", 3, "real", "finite",">=", 0, "<=", 1},"updateMesh", "Color"));
                addParameter(p,'Vertices', {},...
                             @(x)validateattributes(x,"numeric",{"2d","finite", "real", "ncols", 3, "nonempty"},"updateMesh", "Vertices"));
                addParameter(p,'Faces', {},...
                             @(x)validateattributes(x,"numeric",{"2d","finite", "integer", "ncols", 3},"updateMesh", "Faces"));
              case "RigidBodyTree"
                addParameter(p,'Object', {},...
                             @(x)validateattributes(x,"rigidBodyTree", ["scalar", "nonempty"], "updateMesh", "Object"));
            end
            parse(p,varargin{:});

            switch type
              case "GroundVehicle"
                color = double(p.Results.Color);
                geometries{1} = p.Results.Scale;
                geometries = cellfun(@double, geometries, "UniformOutput", false);
              case "Cuboid"
                color = double(p.Results.Color);
                geometries{1} = p.Results.Size;
                geometries = cellfun(@double, geometries, "UniformOutput", false);
              case "Custom"
                color = double(p.Results.Color);
                % validate input vertices and faces
                validateattributes(p.Results.Vertices,"numeric", {"2d","finite", "real", "ncols", 3}, "updateMesh", "Vertices");
                validateattributes(p.Results.Faces, "numeric",{"2d","finite", "integer", "ncols", 3}, "updateMesh", "Faces");
                geometries{1} = double(p.Results.Vertices);
                geometries{2} = p.Results.Faces;
                geometries = cellfun(@double, geometries, "UniformOutput", false);
              case "RigidBodyTree"
                color = [1 0 0];
                if isempty(p.Results.Object)
                    error(message("robotics:robotscenario:scenario:MissingRBTObject"));
                else
                    geometries{1} = p.Results.Object;
                end
            end

            if ~isempty(p.Results.Offset)
                % validate input offset
                offset = double(p.Results.Offset);
                validateattributes(offset, "numeric", {"2d", "nrows", 4, "ncols", 4, "real", "finite"}, "updateMesh", "offset");
            else
                % validate input position and orientation
                position = p.Results.Position;
                orientation = p.Results.Orientation;
                validateattributes(norm(orientation), ["quaternion", "numeric"], {'nonzero'}, "updateMesh", "Orientation norm");
                orientation = robotics.internal.validation.validateQuaternion(orientation, "updateMesh", "Orientation");
                offset = trvec2tform(double(position))*quat2tform(double(orientation));
            end

            if ~isempty(p.Results.IsBinaryOccupied)
                % change only if updateMesh contains occupancy status
                % otherwise preserve existing status
                obj.PlatformImpl.OccupancyMapDetails = ...
                    uav.internal.scenario.map.OccupancyMapInfo(p.Results.IsBinaryOccupied);
            end

            obj.PlatformImpl.setMesh(lower(type), geometries, color, offset);

        end

        function motion = read(obj)
        %READ Read robot platform motion vector
        %   MOTION = READ(PLATFORM) reads the latest motion of the robot
        %   platform base in the scenario. The robot platform motion at the
        %   current instance in the scenario is returned as a 16-element
        %   vector including position, velocity, acceleration, orientation,
        %   and angular velocity as [x y z vx vy vz ax ay az qw qx qy qz wx
        %   wy wz].
        %
        %   Example:
        %
        %      % Create scenario.
        %      scenario = robotScenario(UpdateRate=1, StopTime=10);
        %
        %      % Create platform at specific initial position.
        %      platform = robotPlatform("Robot", scenario,...
        %                               InitialBasePosition=[2 0 0]);
        %
        %      % Setup scenario and get initial motion vector.
        %      setup(scenario);
        %      motion = read(platform);

            motion = obj.PlatformImpl.read("base", obj.ScenarioImpl.CurrentTime);

        end

        function move(obj, type, motion)
        %MOVE Move robot platform in scenario
        %   MOVE(PLATFORM,TYPE,MOTION) moves the robot platform of TYPE
        %   "base" in the scenario according to the specified motion
        %   MOTION. The robot platform motion at the current instance in
        %   the scenario is specified as a 16-element vector including
        %   position, velocity, acceleration, orientation, and angular
        %   velocity as [x y z vx vy vz ax ay az qw qx qy qz wx wy wz].
        %
        %   Example:
        %
        %      % Create scenario.
        %      scenario = robotScenario(UpdateRate=1, StopTime=10);
        %
        %      % Create platform at specific initial position.
        %      platform = robotPlatform("Robot", scenario,...
        %                               InitialBasePosition=[0 3 0]);
        %
        %      % Setup scenario and move platform with motion vector.
        %      setup(scenario);
        %      move(platform,"base",[0 5 0 0 1 0 0 0 0 1 0 0 0 0 0 0])

            narginchk(3,3);
            type = validatestring(type, uav.internal.scenario.Constants.RobotPlatformBodyName, "move", "type");
            validateattributes(motion, "numeric", {"vector", "numel", 16, "finite", "real"}, "move", "motion");
            validateattributes(norm(motion(10:13)), "numeric", "nonzero", "move", "orientation norm");
            robotics.internal.validation.validateQuaternion(motion(10:13), "move", "orientation");

            obj.PlatformImpl.move(type, double(reshape(motion, 1, 16)), obj.ScenarioImpl.CurrentTime);

        end

    end

end

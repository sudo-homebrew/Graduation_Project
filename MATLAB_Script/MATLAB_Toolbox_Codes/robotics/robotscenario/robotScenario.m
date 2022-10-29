classdef robotScenario < handle
%robotScenario Generate robot simulation scenario
%
%   The robotScenario object generates a simulation scenario consisting of
%   static meshes, robot platforms, and sensors in a 3-D environment.
%
%   SCENARIO = ROBOTSCENARIO creates an empty robot scenario with default
%   property values. The default inertial frames are the north-east-down
%   (NED) and the east-north-up (ENU) frames.
%
%   SCENARIO = ROBOTSCENARIO(Name=Value) configures a robotScenario object
%   with properties using one or more Name,Value pair arguments.
%
%   robotScenario properties:
%      UpdateRate        - Simulation update rate
%      StopTime          - Stop time of simulation
%      HistoryBufferSize - Maximum number of steps stored in scenario
%      ReferenceLocation - Scenario origin in geodetic coordinates
%      MaxNumFrames      - Maximum number of frames in scenario
%      CurrentTime       - Current simulation time (read-only)
%      IsRunning         - Indicate whether scenario is running (read-only)
%      TransformTree     - Transformation information between frames
%                          (read-only)
%      InertialFrames    - Names of inertial frames in scenario (read-only)
%      Meshes            - Static meshes in scenario (read-only)
%      Platforms         - Robot platforms in scenario (read-only)
%
%   robotScenario methods:
%      setup              - Prepare robot scenario for simulation
%      advance            - Advance robot scenario simulation by one time
%                           step
%      updateSensors      - Update sensor readings in robot scenario
%      restart            - Reset simulation of robot scenario
%      addInertialFrame   - Define new inertial frame in robot scenario
%      addMesh            - Add new static mesh to robot scenario
%      binaryOccupancyMap - Create 2-D binary occupancy map from robot
%                           scenario
%      show3D             - Visualize robot scenario in 3-D
%
%   Example 1 : Create scenario and get occupancy map
%   -------------------------------------------------
%      % Create scenario.
%      scenario = robotScenario(UpdateRate=1,StopTime=10);
%
%      % Add plane and box mesh in the scenario.
%      addMesh(scenario,"Plane",Position=[5 5 0], Size=[10 10],...
%                       Color=[0.7 0.7 0.7])
%      addMesh(scenario,"Box",Position=[5 5 1], Size=[2 2 2],...
%                       Color=[1 0.5 0], IsBinaryOccupied=true)
%
%      % Visualize the scenario.
%      figure
%      show3D(scenario);
%
%      % Get 2D occupancy map.
%      occupancyMap = binaryOccupancyMap(scenario);
%
%      % Visualize 2D occupancy map.
%      figure
%      show(occupancyMap);
%
%   Example 2 : Add robot platform and simulate scene
%   -------------------------------------------------
%      % Create scenario.
%      scenario = robotScenario(UpdateRate=1, StopTime=10);
%
%      % Create robot platform with waypoint trajectory.
%      platform = robotPlatform("Robot", scenario,...
%                   BaseTrajectory=waypointTrajectory(...
%                                      TimeOfArrival=[0 2], ...
%                                      Waypoints=[1 0 0; 4 0 0],...
%                                      ReferenceFrame="ENU"));
%
%      % Visualize the scenario.
%      figure
%      ax = show3D(scenario);
%
%      % Setup simulation.
%      setup(scenario)
%
%      % Start simulation.
%      advance(scenario);
%
%      % Visualize the scenario after advance.
%      show3D(scenario,Parent=ax);
%
%      % Reset simulation.
%      restart(scenario)
%
%      % Setup simulation again.
%      setup(scenario);
%
%      % Visualize the scenario after restart.
%      show3D(scenario,Parent=ax);
%
%   See also robotPlatform, robotSensor

%   Copyright 2021-2022 The MathWorks, Inc.

    properties (Dependent, SetAccess = private)
        %UpdateRate Simulation update rate
        %   Simulation update rate, specified as a positive scalar in Hz.
        %   The step size of the scenario when using an advance object
        %   function is equal to the inverse of the update rate.
        %
        %   Default: 10
        UpdateRate

        %StopTime Stop time of simulation
        %    Stop time of the simulation, specified as a nonnegative scalar
        %    in seconds. A scenario stops advancing when it reaches the
        %    stop time.
        %
        %   Default: inf
        StopTime

        %HistoryBufferSize Maximum number of steps stored in scenario
        %   Maximum number of steps stored in scenario, specified as a
        %   positive integer greater than 1. This property determines the
        %   maximum number of frames of platform poses stored in the
        %   scenario. If the number of simulated steps exceeds the value of
        %   this property, then the scenario stores only latest steps.
        %
        %   Default: 100
        HistoryBufferSize

        %ReferenceLocation Scenario origin in geodetic coordinates
        %   Scenario origin in geodetic coordinates, specified as a
        %   three-element vector of scalars in the form [latitude longitude
        %   altitude]. latitude and longitude are geodetic coordinates in
        %   degrees. altitude is the height above the WGS84 reference
        %   ellipsoid in meters.
        %
        %   Default: [0 0 0]
        ReferenceLocation

        %MaxNumFrames Maximum number of frames in scenario
        %   Maximum number of frames in the scenario, specified as a
        %   positive integer. The combined number of inertial frames,
        %   platforms, and sensors added to the scenario must be less than
        %   or equal to the value of this property.
        %
        %   Default: 50
        MaxNumFrames

        %CurrentTime Current simulation time
        %   Current simulation time, specified as a nonnegative scalar.
        %
        %   This property is read-only.
        CurrentTime

        %IsRunning Indicate whether scenario is running
        %   Indicate whether the scenario is running, specified as true or
        %   false. After a scenario simulation starts, it runs until it
        %   reaches the stop time.
        %
        %   This property is read-only.
        IsRunning

        %TransformTree Transformation information between frames
        %   Transformation information between all the frames in the
        %   scenario, specified as a transformTree object. This property
        %   contains the transformation information between the inertial,
        %   platform, and sensor frames associated with the scenario.
        %
        %   This property is read-only.
        TransformTree

        %InertialFrames Names of inertial frames in scenario
        %   Names of the inertial frames in the scenario, specified as a
        %   vector of strings.
        %
        %   This property is read-only.
        InertialFrames

        %Meshes Static meshes in scenario
        %   Static meshes in the scenario, specified as a 1-by-n cell array
        %   of extendedObjectMesh.
        %
        %   This property is read-only.
        Meshes
    end

    properties (SetAccess = {?robotScenario, ?robotPlatform})
        %Platforms robot platforms in scenario
        %   Robot platforms in the scenario, specified as an array of
        %   robotPlatform objects.
        %
        %   Default: robotPlatform
        %
        %   This property is read-only.
        Platforms
    end

    properties (Access = {?robotScenario,...
                          ?robotPlatform, ...
                          ?matlab.unittest.TestCase})
        %ScenarioImpl Implementation of the scenario
        ScenarioImpl
    end

    methods
        function r = get.UpdateRate(obj)
            r = obj.ScenarioImpl.UpdateRate;
        end

        function t = get.StopTime(obj)
            t = obj.ScenarioImpl.StopTime;
        end

        function s = get.HistoryBufferSize(obj)
            s = obj.TransformTree.MaxNumTransforms;
        end

        function r = get.ReferenceLocation(obj)
            r = obj.ScenarioImpl.GeoFrame.GeoOrigin;
        end

        function n = get.MaxNumFrames(obj)
            n = obj.TransformTree.MaxNumFrames;
        end

        function t = get.CurrentTime(obj)
            t = obj.ScenarioImpl.CurrentTime;
        end

        function isRunning = get.IsRunning(obj)
            isRunning = obj.ScenarioImpl.IsRunning;
        end

        function t = get.TransformTree(obj)
            t = obj.ScenarioImpl.TFTree;
        end

        function names = get.InertialFrames(obj)
            names = obj.ScenarioImpl.InertialFrameNames;
        end

        function meshes = get.Meshes(obj)
            meshes = cell(size(obj.ScenarioImpl.Meshes));
            for idx = 1:numel(meshes)
                meshes{idx} = obj.ScenarioImpl.Meshes{idx}.Mesh;
            end
        end
    end

    methods
        function obj = robotScenario(varargin)
        %robotScenario

        % parse inputs
            p = inputParser;
            p.addParameter("StopTime", inf, ...
                           @(x)validateattributes(x, "numeric", {"scalar", "real", ">=", uav.internal.scenario.Scene.StartTimeConstant}, "robotScenario", "StopTime"));
            p.addParameter("UpdateRate", 10, ...
                           @(x)validateattributes(x, "numeric", ["scalar", "real", "positive", "finite"], "robotScenario", "UpdateRate"));
            p.addParameter("HistoryBufferSize", 100, ...
                           @(x)validateattributes(x, "numeric", {"scalar", "real", "positive", "integer", ">=", 2}, "robotScenario", "HistoryBufferSize"));
            p.addParameter("ReferenceLocation", [0 0 0], ...
                           @(x)validateattributes(x, "numeric", {"2d", "nrows", 1, "ncols", 3, "real", "finite"}, "robotScenario", "ReferenceLocation"));
            p.addParameter("MaxNumFrames", 50, ...
                           @(x)validateattributes(x, "numeric", {"scalar", "real", "positive", "integer", ">=", 3}, "robotScenario", "MaxNumFrames"));
            p.parse(varargin{:});

            validateattributes(p.Results.ReferenceLocation(1), "numeric", {">=", -90, "<=", 90}, "robotScenario", "ReferenceLocationLatitude");
            validateattributes(p.Results.ReferenceLocation(2), "numeric", {">=", -180, "<=", 180}, "robotScenario", "ReferenceLocationLongitude");

            obj.ScenarioImpl = robotics.internal.scenario.Scene(...
                double(p.Results.UpdateRate), double(p.Results.StopTime), ...
                double(p.Results.HistoryBufferSize), double(p.Results.ReferenceLocation), ...
                double(p.Results.MaxNumFrames));

            obj.Platforms = robotPlatform.empty;

            obj.ScenarioImpl.Parent = obj;
        end

        function binaryMap = binaryOccupancyMap(obj, varargin )
        %binaryOccupancyMap Create 2-D binary occupancy map from robot scenario
        %   BINARYMAP = BINARYOCCUPANCYMAP(SCENARIO, NAME=VALUE) creates
        %   binary occupancy map based on mesh elements from scenario
        %   defined with 'IsBinaryOccupied' status true. The mesh elements
        %   are processed in the 3D convex hull form. Further, mesh element
        %   is considered as occupied region only if it lies inside map
        %   height limits [Hmin, Hmax] and map size [width, height]. These
        %   properties are specified by one or more Name,Value pair
        %   arguments.
        %
        %       GridOriginInLocal - Origin of occupancy map grid in local
        %                           coordinates, specified as a two-element
        %                           vector of the form [xLocal yLocal].
        %                           Default : [0 0]
        %       HeightResolution  - Resolution of occupancy map grid in
        %                           Z-axis, specified as a scalar in cells
        %                           per meter.
        %                           Default : 10
        %       MapHeightLimits   - Minimum and maximum values of map
        %                           height, specified as a two-element
        %                           vector of the form [Hmin Hmax], from
        %                           which static meshes are considered for
        %                           occupancy map grid.
        %                           Default : [0 1]
        %       MapResolution     - Resolution of occupancy map grid in
        %                           XY-axis, specified as a scalar in cells
        %                           per meter.
        %                           Default : 10
        %       MapSize           - Size of occupancy map grid, specified
        %                           as a two-element vector of the form
        %                           [width height] in meter.
        %                           Default : [10 10]
        %
        %   Example:
        %
        %      % Create scenario.
        %      scenario = robotScenario(UpdateRate=1, StopTime=10);
        %
        %      % Add plane, box and cylinder mesh in the scenario.
        %      addMesh(scenario,"Plane", Size=[10 10],...
        %                                IsBinaryOccupied=false)
        %      addMesh(scenario,"Box", Position=[-2 -2 0.5],...
        %                                IsBinaryOccupied=true)
        %      addMesh(scenario,"Cylinder", Position=[2 2 0.5],...
        %                                IsBinaryOccupied=true)
        %
        %      % Get 2D occupancy map.
        %      occupancyMap = binaryOccupancyMap(scenario, ...
        %                               MapHeightLimits=[-1 1], ...
        %                               GridOriginInLocal=[-5 -5]);
        %
        %      % Visualize 2D occupancy map.
        %      show(occupancyMap);

            p = inputParser;
            p.addParameter("GridOriginInLocal",[0 0],...
                           @(x)validateattributes(x, "numeric", {"2d", "nrows", 1, "ncols", 2, "real", "finite"}, "binaryOccupancyMap", "GridOriginInLocal"));
            p.addParameter("MapSize",[10 10],...
                           @(x)validateattributes(x, "numeric", {"2d", "nrows", 1, "ncols", 2, "real", "finite", "positive"}, "binaryOccupancyMap", "MapSize"));
            p.addParameter("MapHeightLimits",[0 1],...
                           @(x)validateattributes(x, "numeric", {"2d", "nrows", 1, "ncols", 2, "real", "finite", "increasing"}, "binaryOccupancyMap", "MapHeightLimits"));
            p.addParameter("MapResolution",10,...
                           @(x)validateattributes(x, "numeric", ["scalar", "real", "positive", "integer"], "binaryOccupancyMap", "MapResolution"));
            p.addParameter("HeightResolution", 10,...
                           @(x)validateattributes(x, "numeric", ["scalar", "real", "positive", "integer"], "binaryOccupancyMap", "HeightResolution"));
            p.parse(varargin{:});

            % get occupied map in image form
            mapImage = obj.ScenarioImpl.getBinaryOccupiedImage( p.Results.GridOriginInLocal,...
                                                                p.Results.MapSize, p.Results.MapHeightLimits,...
                                                                p.Results.MapResolution, p.Results.HeightResolution,...
                                                                obj.CurrentTime);

            % convert binary occupied image into binary occupancy map
            binaryMap = binaryOccupancyMap(mapImage, p.Results.MapResolution );
            binaryMap.GridOriginInLocal = [p.Results.GridOriginInLocal(1) p.Results.GridOriginInLocal(2)];

        end

        function addInertialFrame(obj, base, name, varargin)
        %addInertialFrame Define new inertial frame in robot scenario
        %   ADDINERTIALFRAME(SCENARIO, BASE, NAME, POSITION, ORIENTATION)
        %   adds a new inertial frame with scalar string NAME to the
        %   scenario. The new frame is defined by its POSITION and
        %   ORIENTATION relative to the inertial frame identified by
        %   BASE, which is a string scalar referring to the base
        %   inertial frame name. The POSITION is a 1x3 vector. The
        %   ORIENTATION is a 1x4 vector or a scalar quaternion.
        %
        %   ADDINERTIALFRAME(SCENARIO, BASE, NAME, TFORM) adds a new
        %   inertial frame relative to the inertial frame based on
        %   TFORM, which is a 4x4 transform matrix that maps points in
        %   the new inertial frame to BASE.
        %
        %   Example:
        %
        %      % Create scenario.
        %      scenario = robotScenario(UpdateRate=1, StopTime=10);
        %
        %      % Define internal frame in the scenario.
        %      addInertialFrame(scenario,...
        %                       "ENU", "robot", eul2tform([pi/4 0 0]))
        %
        %      % Add plane and box mesh with reference frame in the scenario.
        %      addMesh(scenario,...
        %              "Plane", Size=[10 10], ReferenceFrame="robot")
        %      addMesh(scenario,...
        %              "Box", Position=[-2 -2 0.5], ReferenceFrame="robot")
        %
        %      % Visualize the scenario.
        %      show3D(scenario);

            base = validatestring(base, obj.InertialFrames, "addInternailFrame", "base");
            name = convertCharsToStrings(name);
            validateattributes(name, "string", "scalartext", "addInertialFrame", "name");
            if any(strcmp(name, [obj.TransformTree.info.FrameNames]))
                error(message("robotics:robotscenario:scenario:InertialFrameAlreadyExist", name));
            end
            if nargin == 4
                tform = varargin{1};
                validateattributes(tform, "numeric", {"2d", "nrows", 4, "ncols", 4, "finite"}, "addInertialFrame", "tform");
            else
                position = varargin{1};
                validateattributes(position, "numeric", {"nrows", 1, "ncols", 3, "finite"}, "addInertialFrame", "position");
                validateattributes(varargin{2}, ["quaternion", "numeric"], {'finite'}, "addInertialFrame", "orientation");
                validateattributes(norm(varargin{2}), ["quaternion", "numeric"], {'nonzero'}, "addInertialFrame", "orientation norm");
                orientation = robotics.internal.validation.validateQuaternion(varargin{2}, "addInertialFrame", "orientation");
                tform = trvec2tform(double(position))*quat2tform(double(orientation));
            end

            obj.ScenarioImpl.addInertialFrame(base, name, double(tform));
        end

        function addMesh(obj, type, varargin)
        %addMesh Add new static mesh to robot scenario
        %   ADDMESH(SCENARIO,TYPE,NAME=VALUE) adds a static mesh to the
        %   robot scenario by specifying the mesh type as "Box",
        %   "Cylinder", "Plane", "Sphere", or "Custom" and specifies
        %   additional options using name-value pair arguments.
        %
        %       Color            - Mesh color, specified as a RGB triplet.
        %                          Default : [1 0 0]
        %       IsBinaryOccupied - Occupied state of binary occupancy map,
        %                          specified as true or false. Set the
        %                          value as true if static mesh is
        %                          considered as an obstacle in the
        %                          scenario and it is incorporated in the
        %                          binary occupancy map.
        %                          Default : False
        %       Position         - Position of static mesh in robot
        %                          scenario, specified as a three-element
        %                          vector of [x y z] in meters.
        %                          Default : [0 0 0]
        %       Size             - Size of static mesh, specified as
        %                          geometry parameters depending on the
        %                          mesh type, except for the custom mesh.
        %                           - Type:"Box", Size: [LENGTHX, LENGTHY, LENGTHZ]
        %                             Default : [1 1 1]
        %                           - Type:"Cylinder", Size: [LENGTH, RADIUS].
        %                             Default : [1 1]
        %                           - Type:"Plane", Size: [LENGTHX, LENGTHY].
        %                             Default : [1 1]
        %                           - Type:"Sphere", Size:  RADIUS.
        %                             Default : 1
        %       Faces            - Faces of static custom mesh, specified
        %                          as an N-by-3 matrix array of positive
        %                          integers. The three elements in each row
        %                          are the indices of the three points in
        %                          the vertices forming a triangle face. N
        %                          is the number of faces.
        %       Vertices         - Vertices of static custom mesh,
        %                          specified as an N-by-3 matrix of real
        %                          scalars. The first, second, and third
        %                          element of each row represents the x-,
        %                          y-, and z-position of each vertex,
        %                          respectively. N is the number of
        %                          vertices.
        %      ReferenceFrame    - Reference frame of mesh geometry,
        %                          specified as an inertial frame name
        %                          defined in the InertialFrames property
        %                          of the robotScenario object, SCENARIO.
        %                          You can add new inertial frames to the
        %                          scenario using the addInertialFrame
        %                          object function. The scenario only
        %                          accepts frames that have z-axis rotation
        %                          with respect to the "ENU" frame.
        %                          Default: "ENU"
        %
        %   Example:
        %
        %      % Create scenario.
        %      scenario = robotScenario(UpdateRate=1, StopTime=10);
        %
        %      % Add plane, box, cylinder and sphere mesh in the scenario.
        %      addMesh(scenario,"Plane",Size=[15 15], Color=[0.7 0.7 0.7])
        %      addMesh(scenario,"Box", Position=[-3 -3 0.5])
        %      addMesh(scenario,"Cylinder",...
        %              Position=[-2 4 0.5], Color=[0 0 1])
        %      addMesh(scenario,"Sphere",Position=[2 7 1], Color=[0 1 0])
        %
        %      % Add custom mesh in the scenario with vertices and faces
        %      vertices = [0 0 0;0 0 2;0 2 0;0 2 2;...
        %                  2 0 0;2 0 2;2 2 0;2 2 2];
        %      faces = [1 3 7;1 7 5;1 6 2;1 5 6;1 2 4;1 4 3;...
        %               3 4 8;3 8 7;5 8 6;5 7 8;2 8 4;2 6 8];
        %      addMesh(scenario,"Custom",Vertices=vertices, Faces=faces,...
        %              Position=[4 -4 1], Color=[1 0.5 0])
        %
        %      % Visualize the scenario.
        %      show3D(scenario);

            type = validatestring(type, uav.internal.scenario.Constants.RobotScenarioMeshShapeType, "addMesh", "type");

            p = inputParser;
            addParameter(p,'Position',[0 0 0], @(x)validateattributes(x, ...
                                                                      "numeric", {"2d", "nrows", 1, "ncols", 3, "real", "finite"}, "addMesh", "Position"));

            addParameter(p,'Color', [1 0 0], @(x)validateattributes(x, ...
                                                                    "numeric", {"2d", "nrows", 1, "ncols", 3, "real", "finite",">=", 0, "<=", 1}, "addMesh", "Color"));

            addParameter(p,'IsBinaryOccupied', false, @(x)validateattributes(x, ...
                                                                             ["logical", "numeric"], ["scalar", "binary"], "addMesh", "IsBinaryOccupied"));

            p.addParameter("ReferenceFrame", uav.internal.scenario.Scene.ENUFrameName, ...
                           @(x)validateattributes(x, ["char", "string"], "scalartext", "addMesh", "ReferenceFrame"));

            switch type
              case "Custom"
                addParameter(p,'Faces', {});
                addParameter(p,'Vertices', {});
              otherwise
                addParameter(p,'Size', {});
            end
            parse(p,varargin{:});

            shapeTransform = [quat2rotm([1 0 0 0]),p.Results.Position';0 0 0 1];

            switch type
              case "Cylinder"
                if isempty(p.Results.Size)
                    % default size
                    cylinderSize = [1 1];
                else
                    validateattributes(p.Results.Size, "numeric", ...
                                       {"2d", "nrows", 1, "ncols", 2, "real", "finite",...
                                        "nonzero","nonnegative"}, "addMesh", "Size");
                    cylinderSize = p.Results.Size;
                end
                geometries = {[0,0,cylinderSize(1)], [-cylinderSize(2)/2,cylinderSize(2)/2]};
                baseType = "cylinder";
                geometries = cellfun(@double, geometries, "UniformOutput", false);

              case "Sphere"
                if isempty(p.Results.Size)
                    % default size
                    sphereSize = 1;
                else
                    validateattributes(p.Results.Size, "numeric", ...
                                       ["scalar", "real", "finite","nonzero","nonnegative"],...
                                       "addMesh", "Size");
                    sphereSize = p.Results.Size;
                end
                geometries = {[0,0,0,sphereSize(1)]};
                baseType = "sphere";
                geometries = cellfun(@double, geometries, "UniformOutput", false);
              case "Box"
                if isempty(p.Results.Size)
                    % default size
                    boxSize = [1 1 1];
                else
                    validateattributes(p.Results.Size, "numeric", ...
                                       {"2d", "nrows", 1, "ncols", 3, "real", "finite",...
                                        "nonzero","nonnegative"}, "addMesh", "Size");
                    boxSize = p.Results.Size;
                end
                x1 = [-boxSize(1)/2,-boxSize(2)/2];
                x2 = [boxSize(1)/2,-boxSize(2)/2];
                x3 = [boxSize(1)/2,boxSize(2)/2];
                x4 = [-boxSize(1)/2,boxSize(2)/2];
                geometries = {[x1;x2;x3;x4], [-boxSize(3)/2,boxSize(3)/2]};
                baseType = "polygon";
                geometries = cellfun(@double, geometries, "UniformOutput", false);
              case "Plane"
                if isempty(p.Results.Size)
                    % default size
                    planeSize = [1 1];
                else
                    validateattributes(p.Results.Size, "numeric", ...
                                       {"2d", "nrows", 1, "ncols", 2, "real", "finite",...
                                        "nonzero","nonnegative"}, "addMesh", "Size");
                    planeSize = p.Results.Size;
                end
                x1 = [-planeSize(1)/2,-planeSize(2)/2];
                x2 = [planeSize(1)/2,-planeSize(2)/2];
                x3 = [planeSize(1)/2,planeSize(2)/2];
                x4 = [-planeSize(1)/2,planeSize(2)/2];
                % constant height of plane
                planeHeight = -0.025;
                geometries = {[x1;x2;x3;x4], [planeHeight,0]};
                baseType = "polygon";
                geometries = cellfun(@double, geometries, "UniformOutput", false);
              case "Custom"
                % validates vertices and faces
                validateattributes(p.Results.Vertices, "numeric", ...
                                   {"2d","finite", "real", "ncols", 3, "nonempty"}, "addMesh", "Vertices");
                validateattributes(p.Results.Faces, "numeric", ...
                                   {"2d","finite", "integer", "ncols", 3, "nonempty"}, "addMesh", "Faces");
                baseType = "custom";
                geometries{1} = double(p.Results.Vertices);
                geometries{2} = p.Results.Faces;
            end

            color = double(p.Results.Color);

            refFrame = validatestring(p.Results.ReferenceFrame, obj.InertialFrames, "addMesh", "ReferenceFrame");

            obj.ScenarioImpl.addMesh(baseType, geometries, color, refFrame, shapeTransform, p.Results.IsBinaryOccupied);

        end

        function setup(obj)
        %SETUP Prepare robot scenario for simulation
        %   SETUP(SCENARIO) prepares the robot scenario for simulation,
        %   sets poses of the platforms to their initial values, and
        %   generates initial sensor readings.
        %
        %   Example:
        %
        %      % Create scenario.
        %      scenario = robotScenario(UpdateRate=1, StopTime=10);
        %
        %      % Setup simulation.
        %      setup(scenario)

            obj.ScenarioImpl.setup();
        end

        function advanced = advance(obj)
        %ADVANCE Advance robot scenario simulation by one time step
        %   ADVANCED = ADVANCE(SCENARIO) advances the robot scenario
        %   simulation by one time step. The UpdateRate property of the
        %   robotScenario object determines the time step during
        %   simulation. The function returns the running status of the
        %   simulation as true or false. The function only updates a
        %   platform location if the platform has an assigned trajectory.
        %
        %   Example:
        %
        %      % Create scenario.
        %      scenario = robotScenario(UpdateRate=1, StopTime=10);
        %
        %      % Setup simulation.
        %      setup(scenario)
        %
        %      % Advance simulation.
        %      advanced = advance(scenario);

            if obj.ScenarioImpl.IsSetup
                advanced = obj.ScenarioImpl.advance();
            else
                error(message("robotics:robotscenario:scenario:AdvanceScenarioBeforeSetup"));
            end
        end

        function updateSensors(obj)
        %UPDATESENSORS Update sensor readings in robot scenario
        %   UPDATESENSORS(SCENARIO) updates all sensor readings based on
        %   latest states of all platforms in the SCENARIO.
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
        %
        %      % Simulate the scenario and gather INS sensor readings.
        %      advance(scenario);
        %      updateSensors(scenario)
        %      [isUpdated, t, measurements] = read(ins);

            if obj.ScenarioImpl.IsSetup
                obj.ScenarioImpl.updateSensors();
            else
                error(message("robotics:robotscenario:scenario:UpdateSensorBeforeSetup"));
            end
        end

        function restart(obj)
        %RESTART Reset simulation of robot scenario
        %   RESTART(SCENARIO) resets the simulation of the robot scenario
        %   scene. The function resets poses of the platforms and sensor
        %   readings to NaN, resets the CurrentTime property of the
        %   scenario to zero, and resets the IsRunning property of the
        %   scenario to false.
        %
        %   Example:
        %
        %      % Create scenario.
        %      scenario = robotScenario(UpdateRate=1, StopTime=10);
        %
        %      % Setup simulation.
        %      setup(scenario)
        %
        %      % Reset simulation.
        %      restart(scenario);

            obj.ScenarioImpl.restart();

        end

        function [ax, plotFrames] = show3D(obj, varargin)
        %SHOW3D Visualize robot scenario in 3-D
        %   [AX, PLOTFRAMES] = SHOW3D(SCENARIO) visualizes the latest
        %   states of the platforms and sensor frames in the SCENARIO,
        %   along with all static meshes. AX is the handle to the
        %   figure handle. PLOTFRAMES is a nested struct to query
        %   hgtransforms for each visualization frames created by
        %   show3D method. For example, PLOTFRAMES.ENU is the
        %   hgtransform you can use as parent to plot in the ENU frame.
        %
        %   [AX, PLOTFRAMES] = SHOW3D(SCENARIO, T) visualizes the
        %   SCENARIO at time T.
        %
        %   [AX, PLOTFRAMES] = SHOW3D(___, Name=Value) additionally
        %   take in the following name value pairs:
        %
        %      Parent           - Parent axis for plotting.
        %
        %      FastUpdate       - Set to true to fast update plot in an
        %                         existing plot. Parent must be
        %                         provided to perform fast update.
        %
        %                         Default: false
        %
        %      View             - Set or modify view point of plot with
        %                         string input "Top", "Side" or "3D".
        %
        %                         Default: "3D"
        %
        %   Example:
        %
        %      % Create scenario.
        %      scenario = robotScenario(UpdateRate=1, StopTime=10);
        %
        %      % Add plane mesh in the scenario.
        %      addMesh(scenario,"Plane", Size=[10 10], Color=[0.7 0.7 0.7])
        %
        %      % Visualize the scenario in Top view.
        %      ax = show3D(scenario, View="Top");
        %
        %      % Add box mesh in the scenario.
        %      addMesh(scenario,"Box", Color=[1 0.5 0])
        %
        %      % Update plot with Side view.
        %      show3D(scenario, Parent=ax, View="Side");

            p = inputParser();
            p.addOptional("Time", obj.ScenarioImpl.CurrentTime, ...
                          @(x)validateattributes(x, "numeric", {"scalar", ">=", obj.ScenarioImpl.StartTime}, "show3D", "time"));
            p.addParameter("Parent", [], ...
                           @(x)validateattributes(x, "matlab.graphics.axis.Axes", "scalar", "show3D", "Parent"));
            p.addParameter("FastUpdate", false, ...
                           @(x)validateattributes(x, ["logical", "numeric"], ["scalar", "binary"], "show3D", "FastUpdate"));
            p.addParameter("View", "3D", ...
                           @(x)validateattributes(x, ["char","string"], ["scalartext", "nonempty"], "show3D", "View"));
            p.parse(varargin{:});

            viewPoint = validatestring(char(p.Results.View), {'Top','Side','3D'}, 'show3D', 'View');

            [ax, plotFrames] = visualizeScene(obj, double(p.Results.Time),...
                                              p.Results.FastUpdate, p.Results.Parent, viewPoint);

        end
    end

    methods (Access = private)
        function tform = getNormalizedTform(obj, tgt, base, t)
        %getNormalizedTfrom get transform with normalized and
        %orthogonal rotation matrix

        % Due to floating point computation accuracy, the
        % rotation matrix part in the transform is slightly
        % off.
        % Normalize the rotation matrix's determinant to 1 and
        % make the rotation matrix orthogonal
            tform = obj.TransformTree.getTransform(tgt, base, t);
            if ~any(isnan(tform))
                [u, ~, v] = svd(tform(1:3,1:3));
                tform(1:3,1:3) = u*v';
            end
        end

        function [ax, plotFrames] = visualizeScene(obj, plotTime, fastUpdate, parent, viewPoint )

            if fastUpdate
                % Update the existing transforms if fast update is true

                if isempty(parent)
                    error(message("robotics:robotscenario:scenario:ShowFastUpdateEmptyAx"));
                end

                ax = parent;
                plotFrames = struct;

                % view point should be same as previously configured in
                % other than 3D view
                if ~strcmp(viewPoint,"3D") && ~isequal(uav.internal.scenario.plot.PlotViewAngle.ViewPoint(viewPoint), parent.View)
                    error(message("robotics:robotscenario:scenario:MismatchViewPoint"));
                end

                for idx = 1:numel(obj.InertialFrames)
                    htform = findobj(ax, "-depth", 2, "Tag", obj.getHGTransformTag(obj.InertialFrames(idx)));
                    if isempty(htform)
                        error(message("robotics:robotscenario:scenario:MissingTransformInPlot", "Inertial Frame", obj.InertialFrames(idx)));
                    end
                    plotFrames.(obj.InertialFrames(idx)) = htform(1);
                end

                for idx = 1:numel(obj.ScenarioImpl.Platforms)
                    plat = obj.ScenarioImpl.Platforms(idx);
                    % get platform name and reference frame
                    platName = plat.Name;
                    platRefFrame = plat.ReferenceFrame;

                    htform = findobj(ax, "-depth", 2, "Tag", obj.getHGTransformTag(platName+".BodyFrame"));
                    if isempty(htform)
                        error(message("robotics:robotscenario:scenario:MissingTransformInPlot", "Platform", platName));
                    end
                    % get platform transform
                    platTform = obj.getNormalizedTform(platRefFrame, platName, plotTime);
                    if ~any(isnan(platTform))
                        htform(1).Matrix = platTform;
                        htform(1).Visible = true;
                    else
                        htform(1).Visible = false;
                    end
                    plotFrames.(platName).BodyFrame = htform(1);

                    htform = findobj(ax, "-depth", 3, "Tag", obj.getHGTransformTag(platName+".MeshFrame"));
                    if isempty(htform)
                        error(message("robotics:robotscenario:scenario:MissingTransformInPlot", "Platform", platName));
                    end
                    plotFrames.(platName).MeshFrame = htform(1);

                    if plat.IsRigidBody
                        % get current platform transform with respect to
                        % ENU frame and use RigidBodyTree internal fast
                        % show method.
                        rbPlatTform = obj.getNormalizedTform(obj.ScenarioImpl.ENUFrameName, platName, plotTime);
                        rbPlatTform = rbPlatTform * plat.MeshOffset;
                        % use RigidBodyTree visualization
                        robotics.internal.scenario.rigidbody.RigidBodyTreeUtil.show3D(...
                            plat.RigidBodyTree, ax, fastUpdate, rbPlatTform);
                    end

                    for sIdx = 1:numel(plat.Sensors)
                        sensor = plat.Sensors{sIdx};
                        sensorName = extractAfter(sensor.Name, plat.Name+"/");
                        validMountingBodyName = ...
                            robotics.internal.scenario.rigidbody.getValidPlatformBodyName(plat, sensor.MountingBodyName);

                        htform = findobj(ax, "-depth", 3, "Tag", obj.getHGTransformTag(sensor.Name));
                        if isempty(htform)
                            error(message("robotics:robotscenario:scenario:MissingTransformInPlot", "Sensor", sensor.Name));
                        end
                        plotFrames.(validMountingBodyName).(sensorName) = htform(1);
                    end
                end

            else
                % Create new plot if not fast update
                if isempty(parent)
                    ax = newplot;
                    axis(ax, "equal");
                else
                    ax = parent;
                end

                if ~ishold(ax)
                    cla(ax);
                    hold(ax, "on");
                    cleanup = onCleanup(@()hold(ax, "off"));
                end

                plotFrames = struct;

                % plot all obstacle meshes
                for idx = 1:numel(obj.ScenarioImpl.Meshes)
                    show3D(obj.ScenarioImpl.Meshes{idx}, ax);
                end
                xlabel(ax, message("robotics:robotscenario:scenario:Show3DXLabel").getString);
                ylabel(ax, message("robotics:robotscenario:scenario:Show3DYLabel").getString);
                zlabel(ax, message("robotics:robotscenario:scenario:Show3DZLabel").getString);
                view(ax, uav.internal.scenario.plot.PlotViewAngle.ViewPoint(viewPoint));

                % create hgtransform for each inertial frames
                enuFrameName = obj.ScenarioImpl.ENUFrameName;
                for idx = 1:numel(obj.InertialFrames)
                    plotFrames.(obj.InertialFrames(idx)) = hgtransform(...
                        "Parent", ax, ...
                        "Matrix", obj.getNormalizedTform(enuFrameName, obj.InertialFrames(idx), obj.ScenarioImpl.StartTime), ...
                        "Tag", obj.getHGTransformTag(obj.InertialFrames(idx)));
                end

                % create hgtransform for each platform
                for idx = 1:numel(obj.ScenarioImpl.Platforms)
                    plat = obj.ScenarioImpl.Platforms(idx);
                    platName = plat.Name;
                    platRefFrame = plat.ReferenceFrame;

                    % get current platform transform with respect to
                    % platform reference frame and get mesh details
                    platTform = obj.getNormalizedTform(platRefFrame, platName, plotTime);
                    platMeshTform = plat.MeshOffset;

                    if ~any(isnan(platTform))
                        plotFrames.(platName).BodyFrame = hgtransform(...
                            "Parent", plotFrames.(platRefFrame), ...
                            "Matrix", platTform, ...
                            "Tag", obj.getHGTransformTag(platName+".BodyFrame"));
                    else
                        % visualization should be off if platform
                        % transform contains NaN values.
                        plotFrames.(platName).BodyFrame = hgtransform(...
                            "Parent", plotFrames.(platRefFrame), ...
                            "Matrix", eye(4), ...
                            "Tag", obj.getHGTransformTag(platName+".BodyFrame"));
                        plotFrames.(platName).BodyFrame.Visible = false;
                    end

                    plotFrames.(platName).MeshFrame = hgtransform(...
                        "Parent", plotFrames.(platName).BodyFrame, ...
                        "Matrix", platMeshTform, ...
                        "Tag", obj.getHGTransformTag(platName+".MeshFrame"));

                    if plat.IsRigidBody
                        % get current platform transform with respect to
                        % ENU frame
                        rbtPlatTform = obj.getNormalizedTform(enuFrameName, platName, plotTime);
                        rbtPlatTform = rbtPlatTform * platMeshTform;
                        % use RigidBodyTree visualization for RigidBodyTree
                        % based robot platform
                        robotics.internal.scenario.rigidbody.RigidBodyTreeUtil.show3D(...
                            plat.RigidBodyTree, ax, fastUpdate, rbtPlatTform);
                    else
                        % for robot platform other than RigidBodyTree
                        platMeshFaces = plat.BaseMesh.Mesh{1}.Faces;
                        platMeshVertices = plat.BaseMesh.Mesh{1}.Vertices;
                        platMeshColor = plat.BaseMesh.Color{1};

                        patch("Faces", platMeshFaces, ...
                              "Vertices", platMeshVertices, ...
                              "Parent", plotFrames.(platName).MeshFrame, ...
                              "FaceColor", platMeshColor, ...
                              "LineStyle", "none");
                    end

                    % create hgtransform for its sensors
                    for sIdx = 1:numel(plat.Sensors)
                        sensor = plat.Sensors{sIdx};
                        sensorName = extractAfter(sensor.Name, plat.Name+"/");
                        validMountingBodyName = ...
                            robotics.internal.scenario.rigidbody.getValidPlatformBodyName(plat, sensor.MountingBodyName);
                        plotFrames.(validMountingBodyName).(sensorName) = hgtransform(...
                            "Parent", plotFrames.(plat.Name).BodyFrame, ...
                            "Matrix", obj.getNormalizedTform(validMountingBodyName, sensor.Name, plotTime), ...
                            "Tag", obj.getHGTransformTag(sensor.Name));
                    end

                end
            end
        end
    end

    methods (Static)
        function obj = loadobj(obj)
        %loadobj customize load process

        % Assign parent
            if isempty(obj.ScenarioImpl.Parent)
                obj.ScenarioImpl.Parent = obj;
            end
        end
    end

    methods (Static, Access = private)
        function tag = getHGTransformTag(tag)
            tag = "Robot_Scenario_Frame_"+tag;
        end
    end
end

classdef Importer < robotics.manip.internal.InternalAccess
%Base class for importer objects that reference URDFshared models

%   Copyright 2018-2021 The MathWorks, Inc.

    properties (Abstract)

        %ParseInertial Indicates whether to parse inertial parameters
        ParseInertial

        %DataFormat Defines data format for the RigidBodyTree object
        DataFormat

        %MaxNumBodies Maximum number of bodies on the RigidBodyTree object
        %   This property is used only during codegen.
        MaxNumBodies
    end

    methods (Access = ?robotics.manip.internal.InternalAccess)
        function robot = importURDFShared(obj, model, varargin)
        %importURDFShared Import rigidBodyTree object from matlabshared.multibody.internal.urdf.Model
        %   VARARGIN holds the following arguments:
        %   MESHPATH   - Mesh path provided by user.
        %   PATHTOURDF - Directory containing the URDF.
        %   SOURCEDATA - Description file type, either URDF or SDF.
        %   FILEINPUT  - Name of the input description file.
            if nargin > 2
                meshPath = varargin{1};
                pathToURDF = varargin{2};
                if nargin > 4
                    sourceData = varargin{3};
                    fileInput = varargin{4};
                else
                    sourceData = 'URDF';
                    fileInput = '';
                end
            else
                meshPath = {};
                pathToURDF = '';
                sourceData = 'URDF';
                fileInput = '';
            end

            linkNames = keys(model.Links);

            bodies = containers.Map;

            % processing urdf links
            % shared_multibody makes sure all link names are unique


            for i = 1:model.Links.Count
                urdflink = model.Links(linkNames{i});
                body = rigidBody(urdflink.Name);

                % inertial Information

                if obj.ParseInertial
                    if ~isempty(urdflink.Inertial)
                        % URDF contains inertial information
                        xyz = urdflink.Inertial.Origin.xyz;
                        rpy = urdflink.Inertial.Origin.rpy;

                        mass = urdflink.Inertial.Mass;
                        % in URDF, the origin of the inertia frame collocates with CoM
                        ixx = urdflink.Inertial.Inertia.Ixx;
                        iyy = urdflink.Inertial.Inertia.Iyy;
                        izz = urdflink.Inertial.Inertia.Izz;
                        ixy = urdflink.Inertial.Inertia.Ixy;
                        ixz = urdflink.Inertial.Inertia.Ixz;
                        iyz = urdflink.Inertial.Inertia.Iyz;

                        ypr = [rpy(3) rpy(2) rpy(1)];
                        R = eul2rotm(ypr, 'ZYX');  % orientation of inertia frame rel body frame

                        com = xyz;
                        I = [ixx, ixy, ixz; ixy, iyy, iyz; ixz iyz izz];
                        Io = robotics.manip.internal.inertiaTransform(I, mass, R, com);

                        body.Mass = mass;
                        body.CenterOfMass = com;

                        body.Inertia = robotics.manip.internal.flattenInertia(Io);
                    else
                        % URDF does not contain inertial information.
                        % Assign default values (zeros)
                        body.Mass = 0;
                        body.CenterOfMass = [0 0 0];
                        body.Inertia = [0 0 0 0 0 0];
                    end
                end


                if ~isempty(urdflink.Visual)
                    for k = 1:length(urdflink.Visual)
                        xyz = urdflink.Visual(k).Origin.xyz;
                        rpy = urdflink.Visual(k).Origin.rpy;
                        ypr = [rpy(3) rpy(2) rpy(1)];
                        R = eul2rotm(ypr, 'ZYX');
                        T = [R xyz(:); [0 0 0 1]];

                        type = urdflink.Visual(k).Geometry.Type;
                        color = [];
                        if ~isempty(urdflink.Visual(k).Material)
                            color = urdflink.Visual(k).Material.Color;
                        end

                        switch type
                          case 'mesh'
                            parameter = cell(1,2);
                            parameter{2} = urdflink.Visual(k).Geometry.Scale; % scale
                            fn = urdflink.Visual(k).Geometry.FileName; % mesh filename
                            parameter{1} = robotics.manip.internal.RobotDataImporter.findMeshFilePath(fn, pathToURDF, meshPath, sourceData, fileInput);
                          case 'box'
                            parameter = urdflink.Visual(k).Geometry.Size; % [xl, yl, zl]
                          case 'cylinder'
                            parameter = [urdflink.Visual(k).Geometry.Radius, urdflink.Visual(k).Geometry.Length]; % [radius, length]
                          case 'sphere'
                            parameter = urdflink.Visual(k).Geometry.Radius; % radius
                        end

                        type = validatestring(type, {'Mesh', 'Box', 'Cylinder', 'Sphere'});

                        % Note: if the type is mesh, we will ignore the normals read from stl
                        addVisualInternal(getInternalBody(obj, body), type, parameter, T, color );

                    end
                end

                % If the link has <collision> data associated with it, parse it
                % for the parameters of the collision geometry.
                if ~isempty(urdflink.Collision)
                    bodyInternal = getInternalBody(obj, body);
                    bodyInternal.CollisionsInternal =...
                        robotics.manip.internal.CollisionSet(length(urdflink.Collision));
                    for k = 1:length(urdflink.Collision)
                        collisionParser =...
                            robotics.manip.internal.URDFCollisionParameterParser;

                        [type, parameter, T] = ...
                            collisionParser.extractCollisionParameters(...
                            urdflink.Collision(k), ...
                            meshPath, ...
                            pathToURDF, ...
                            sourceData, ...
                            fileInput);

                        %Add the collision geometry to the RigidBody
                        %bodyInternal based on the parsed type and parameters.
                        addCollisionFromImporter(...
                            bodyInternal, ...
                            type, ...
                            parameter, ...
                            T);
                    end
                end


                bodies(urdflink.Name) = body;
            end

            baseName = [];
            parents = containers.Map;
            childrenSets = containers.Map;
            for i = 1:model.Links.Count
                childrenSets(linkNames{i}) = {};
            end

            % processing urdf joints
            % shared_multibody makes sure all joint names are unique
            jointNames = keys(model.Joints);
            for i=1:model.Joints.Count
                urdfjoint = model.Joints(jointNames{i});

                % joint name and type
                jtype = urdfjoint.Type;
                if strcmp(jtype, 'continuous')
                    jtype = 'revolute';
                end
                jnt = rigidBodyJoint(urdfjoint.Name, jtype);

                % fixed transform
                if ~isempty(urdfjoint.Origin)
                    xyz = urdfjoint.Origin.xyz;
                    rpy = urdfjoint.Origin.rpy;

                    ypr = [rpy(3) rpy(2) rpy(1)];
                    R = eul2rotm(ypr, 'ZYX');
                    jnt.setFixedTransform([R, reshape(xyz, 3,1);[0 0 0], 1]);
                end

                % Set the joint limits and home position for non-fixed
                % joints
                if ~strcmp(jnt.Type, 'fixed')
                    % Set the home position. This is a property of the
                    % urdfshared model. For imports from URDF, the value is
                    % always zero since the property does not exist in URDF,
                    % but rather is used for conversion between RBT and
                    % Simscape Multibody.

                    % Joint limits and home positions sometimes have
                    % conflicting values. If a home position is set and
                    % joint limits are applied that no longer allow that
                    % home position, a warning is thrown and the home
                    % position is moved. However, if joint limits are set
                    % and a home position is applied outside these bounds,
                    % an error is thrown. To avoid the error case, this
                    % function first changes the joint limits to an
                    % unbounded interval, making sure to store the default
                    % values set in the rigidBodyJoint constructor. Then,
                    % once the home position has been set, the joint limits
                    % are reapplied, ensuring that the robot is imported
                    % and the home position values are shifted (and a
                    % warning is thrown) if they are not compatible with
                    % the joint limits.
                    defaultPositionLimits = jnt.PositionLimits;
                    jnt.PositionLimits = [-inf, inf];
                    jnt.HomePosition = urdfjoint.HomePosition;

                    % Apply joint limits. If the joint
                    % is continuous, or if the URDFshared joint has
                    % specified joint limits that may differ from the
                    % defaults, the limits must be updated.
                    if strcmp(urdfjoint.Type, 'continuous')
                        % Continuous joints have infinite joint limits.
                        % Setting the values is redundant but is left here
                        % to ensure a clear overview of the different cases
                        jnt.PositionLimits = [-inf, inf];

                    elseif ~isempty(urdfjoint.Limit)
                        % If the source URDFshared joint has joint limits,
                        % update the joint to have those limits
                        lb = urdfjoint.Limit.Lower;
                        ub = urdfjoint.Limit.Upper;

                        jnt.PositionLimits = [lb, ub];
                    else
                        % If the source URDFshared joint doesn't have any
                        % joint limits, use the default values
                        jnt.PositionLimits = defaultPositionLimits;
                    end
                end

                % joint axis
                if ismember(jnt.Type, {'revolute', 'prismatic'})
                    if ~isempty(urdfjoint.Axis)
                        jnt.JointAxis = urdfjoint.Axis.xyz;
                    end
                end

                % identify parents, childrenSet, robot base name
                childName = urdfjoint.ChildLink;
                child = bodies(childName);
                child.Joint = jnt;

                parentName = urdfjoint.ParentLink;
                parent = bodies(parentName);
                parentLink = model.Links(parentName);
                if strcmp(parentLink.ParentJoint, '')
                    % shared_multibody makes sure there is only one
                    % root link (base frame) in the model
                    baseName = parentName;
                end

                parents(childName) = parent;

                if isKey(childrenSets, {parentName})
                    tmp = childrenSets(parentName);
                    tmp = [tmp, {child}]; %#ok<AGROW>
                    childrenSets(parentName) = tmp;
                else
                    childrenSets(parentName) = {child};
                end
            end

            % Assemble robot
            robot = rigidBodyTree('MaxNumBodies', obj.MaxNumBodies);
            if isempty(jointNames) % when the model contains only one link and no joint
                                   % Override base to match internal representation with
                                   % visual information.
                overrideBase(obj, robot, bodies(linkNames{1}));
            else
                overrideBase(obj, robot, bodies(baseName));
                bodiesToAdd = childrenSets(baseName);
                while ~isempty(bodiesToAdd)
                    body = bodiesToAdd{1};

                    parent = parents(body.Name);
                    robot.addBody(body, parent.Name);
                    bodiesToAdd = bodiesToAdd(2:end);
                    bodiesToAdd = [childrenSets(body.Name), bodiesToAdd]; %#ok<AGROW>

                end
            end

            % Assign gravity
            if ~isempty(model.Gravity)
                robot.Gravity = model.Gravity;
            end

        end
    end

    methods (Access = {?robotics.manip.internal.InternalAccess})
        function overrideBase(~, robot, body)
        %overrideBase
            robot.TreeInternal.Base = copy(body.BodyInternal);
            robot.TreeInternal.Base.Index = 0;
        end

        function bodyIn = getInternalBody(~, body)
        %getInternalBody
            bodyIn = body.BodyInternal;
        end
    end
end

classdef RigidBody < robotics.manip.internal.InternalAccess
    %This class is for internal use only. It may be removed in the future.

    %RIGIDBODY Internal implementation of rigidBody.

    %   Copyright 2016-2022 The MathWorks, Inc.

    %#codegen

    properties (Dependent)
        %Name Name of the body
        Name

        %Joint The joint associated with the rigid body
        Joint

        %Mass Mass of the rigid body
        Mass

        %CenterOfMass Center of mass coordinates with respect to the body
        %   frame.
        CenterOfMass

        %Inertia Inertia of the rigid body relative to the body frame.
        %   Only 6 independent elements are specified. The
        %   order of elements is [Ixx Iyy Izz Iyz Ixz Ixy].
        Inertia

    end


    properties (Access = ?robotics.manip.internal.InternalAccess)
        %Index Index (a double number) of the body
        Index

        %NameInternal Name of the body, for internal use
        NameInternal

        %JointInternal The (tree) joint associated with the body. Access to
        %   this handle is restricted to certain classes.
        JointInternal

        %ParentIndex Index of the body's parent
        ParentIndex

        %ChildrenIndices
        ChildrenIndices

        %MassInternal
        MassInternal

        %CenterOfMassInternal
        CenterOfMassInternal

        %InertiaInternal Inertia tensor of the rigid body relative to body frame
        InertiaInternal

        %SpatialInertia 6x6 spatial inertia matrix
        SpatialInertia

        %VisualsInternal
        VisualsInternal

        %CollisionsInternal
        CollisionsInternal = robotics.manip.internal.CollisionSet(0);
    end



    methods
        function obj = RigidBody(bodyInput, jointInput, isBase)
            %RigidBody Constructor
            %   Create a RigidBody object using either name or RigidBody
            %   struct as input. 
            %   OBJ = RigidBody(BODYINPUT) creates a RigidBody OBJ with the
            %   name BODYINPUT
            %
            %   OBJ = RigidBody(BODYINPUT, JOINTINPUT, ISBASE) creates a
            %   RigidBody from the rigidBody struct representation
            %   BODYINPUT and the rigidBodyJoint struct representation
            %   JOINTINPUT. ISBASE is a logical input that is set to true
            %   when creating a base rigidBody from the struct representation.

            if nargin == 1
                %Instantiate the body given a RigidBody name as input
                bname = bodyInput;

                validateattributes(bname,{'char'},{'nonempty','row'},...
                    'rigidBody','bname');

                coder.varsize('bn', [1, inf], [0, 1]);
                bn = bname;
                obj.NameInternal = bn;
                jtype = 'fixed';
                jname = [bname '_jnt'];
                obj.JointInternal = rigidBodyJoint(jname, jtype);
                obj.Index = -1;
                obj.ParentIndex = -1;

                obj.MassInternal = 1;
                obj.CenterOfMassInternal = [0 0 0];
                obj.InertiaInternal = eye(3);
                obj.SpatialInertia = eye(6);
                obj.VisualsInternal = {};
                obj.CollisionsInternal = ...
                    robotics.manip.internal.CollisionSet(0);
            else
                % Directly instantiate an internal RigidBody from a
                % structure using the Simulink-compatible format. For a
                % given rigidBodyTree object, tree, the following
                % function call can be used to generate a
                % Simulink-compatible structure representation of the tree:
                %
                % treeStruct = robotics.manip.internal.RBTRepresentationUtility.populateRigidBodyTreeStruct(tree);
                %
                % The structure used for bodies is stored inside the parent
                % tree, e.g. in treeStruct.Bodies(1).

                bodyStruct = bodyInput;
                jointStruct = jointInput;

                bodyName = char(bodyStruct.Name(1:bodyStruct.NameLength)');
                %The body name length may change later
                coder.varsize('bodyName', [1, 256], [0, 1]);
                obj.NameInternal = bodyName;
                obj.ParentIndex = bodyStruct.ParentIndex;

                obj.ChildrenIndices = bodyStruct.ChildrenIndices';

                obj.MassInternal = bodyStruct.Mass;
                obj.CenterOfMassInternal = bodyStruct.CenterOfMass';
                obj.InertiaInternal = bodyStruct.Inertia;
                obj.SpatialInertia = bodyStruct.SpatialInertia;

                if ~isBase
                    switch jointStruct.Type
                        case uint8(0)
                            type = 'fixed';
                        case uint8(1)
                            type = 'revolute';
                        otherwise
                            type = 'prismatic';
                    end

                    obj.JointInternal = rigidBodyJoint(char(jointStruct.Name(1:jointStruct.NameLength)'), type);
                    obj.JointInternal.JointToParentTransform = jointStruct.JointToParentTransform;
                    obj.JointInternal.ChildToJointTransform = jointStruct.ChildToJointTransform;

                    obj.JointInternal.MotionSubspace = jointStruct.MotionSubspace(:,1:max(1,jointStruct.VelocityNumber));
                    obj.JointInternal.InTree = true;

                    obj.JointInternal.PositionLimitsInternal = jointStruct.PositionLimits(1:max(1,jointStruct.PositionNumber),:);
                    obj.JointInternal.JointAxisInternal = jointStruct.JointAxis(:)';
                    obj.JointInternal.HomePositionInternal = jointStruct.HomePosition(1:max(1,jointStruct.PositionNumber), :);
                else
                    jtype = 'fixed';
                    jname = [obj.NameInternal '_jnt'];
                    obj.JointInternal = rigidBodyJoint(jname, jtype);
                end
                %Initialize Visuals and Collisions
                obj.VisualsInternal = {};
                obj.CollisionsInternal = ...
                    robotics.manip.internal.CollisionSet(0);

                %Populate CollisionsInternal with collision data stored in
                %the RigidBody struct
                if ~isempty(bodyStruct.CollisionGeom)
                    %MaxElements need to be defined for the
                    %CollisionSet when using codegen
                    nColls = coder.const(length(bodyStruct.CollisionGeom));
                    obj.CollisionsInternal = ...
                        robotics.manip.internal.CollisionSet(nColls);
                    
                    for i=coder.unroll(1:nColls)
                        collisionGeometry = robotics.manip.internal.CollisionGeomStruct.populateCollisionGeomFromStruct(bodyStruct.CollisionGeom(i));
                        obj.CollisionsInternal.add(collisionGeometry);
                    end
                end
            end
        end



        function newbody = copy(obj)
            %COPY Creates a deep copy of the rigid body object. The copied
            %   body will not retain any parent/children information as it
            %   is a standalone body.

            newbody = robotics.manip.internal.RigidBody(obj.NameInternal);
            newbody.JointInternal = copy(obj.JointInternal);

            newbody.MassInternal = obj.MassInternal;
            newbody.CenterOfMassInternal = obj.CenterOfMassInternal;
            newbody.InertiaInternal = obj.InertiaInternal;
            newbody.SpatialInertia = obj.SpatialInertia;

            if coder.target('matlab')
                newbody.VisualsInternal = obj.VisualsInternal;
            end
            newbody.CollisionsInternal = copy(obj.CollisionsInternal);
        end

        function addVisual(obj, type, parameter, varargin)
            %addVisual
            validType = validateAddVisualInputsSimple(obj, type, parameter, varargin{:});
            if ~isempty(which(parameter))
                parameter = which(parameter);
            end
            errCode = addVisualInternal(obj, validType, parameter, varargin{:});
            prefix = 'robotics:robotmanip:rigidbody:';
            switch(errCode)
                case 0
                    %SUCCESS No warning needed
                case 1
                    warning(message([prefix 'InvalidMeshFileExtension'], parameter, 'Visual'));
                case 2
                    warning(message([prefix 'MeshFileNotOpened'], parameter, 'Visual'));
                case 3
                    warning(message([prefix 'EmptyMeshFile'], parameter, 'Visual'));
                case 4
                    warning(message([prefix 'InvalidSTLFileFormat'], parameter, 'Visual'));
                case -2
                    warning(message('robotics:robothoops:hoopsconvert:CorruptedInputCADFile', parameter));
                case -1
                    warning(message('robotics:robothoops:hoopsconvert:CannotSuccessfullyReadInputCADFile', parameter));
                otherwise
                    warning(message('MATLAB:polyfun:stlFailedToRead', parameter));
            end
        end

        function clearVisual(obj)
            %clearVisual
            obj.VisualsInternal = {};
        end

        function clearCollision(obj)
            %clearCollision
            obj.CollisionsInternal = obj.CollisionsInternal.clearSet();
        end
    end


    methods (Access = {?robotics.manip.internal.InternalAccess})
        function validType = validateAddVisualInputs(~, type, parameter, tform)
            %validateAddVisualInputs
            narginchk(3,4);

            validType = validatestring(type, {'Mesh', 'Box', 'Cylinder', 'Sphere'}, 'addVisual', 'type');
            if nargin == 4
                validateattributes(tform, {'single','double'}, {'nonempty', 'nonnan', 'finite', 'size',[4, 4]}, 'addVisual', 'tform');
            end
            switch validType
                case 'mesh'
                    validateattributes(parameter, {'cell', 'char'}, {'nonempty'}, 'addVisual', 'parameter');
                    if ischar(parameter)
                        robotics.internal.validation.validateString(parameter, false, 'addVisual', 'filename');
                    else
                        robotics.internal.validation.validateString(parameter{1}, false, 'addVisual', 'filename');
                        if length(parameter) > 1
                            validateattributes(parameter{2}, {'single','double'}, {'vector', 'numel', 3, 'positive', 'finite', 'nonnan'}, 'addVisual', 'scales');
                        end
                    end
                case 'box'
                    validateattributes(parameter, {'single','double'}, {'vector', 'numel', 3, 'positive', 'finite', 'nonnan'}, 'addVisual', 'parameter');
                case 'cylinder'
                    validateattributes(parameter, {'single','double'}, {'vector', 'numel', 2, 'positive', 'finite', 'nonnan'}, 'addVisual', 'parameter');
                case 'sphere'
                    validateattributes(parameter, {'single','double'}, {'scalar', 'positive', 'finite', 'nonnan'}, 'addVisual', 'parameter');
            end
        end


        function validType = validateAddVisualInputsSimple(~, type, parameter, tform)
            %validateAddVisualInputsSimple
            narginchk(3,4);
            validType = validatestring(type, {'Mesh'}, 'addVisual', 'type');
            robotics.internal.validation.validateString(parameter, false, 'addVisual', 'filename');
            if nargin == 4
                validateattributes(tform, {'single','double'}, {'nonempty', 'nonnan', 'finite', 'size',[4, 4]}, 'addVisual', 'tform');
            end

        end

        function addCollisionInternal(obj, varargin)
            %addCollisionInternal Adds the collision geometry.
            %   The internal function to which rigidBody/addCollision is
            %   forwarded to.

            %The input can be a collision object, or a type and its parameters.
            %Optionally, a local frame can also be input.
            narginchk(2, 4);

            %The input is a collision type with its parameters
            if((isstring(varargin{1}) || ischar(varargin{1})) && ...
                    (nargin == 4 || nargin == 3))
                geomType = varargin{1};
                parameters = varargin{2};
                if(nargin == 4)
                    tform = varargin{3};
                else
                    tform = eye(4);
                end

                %Use the CollisionInputValidator to validate type-parameter
                %inputs
                robotics.manip.internal.CollisionInputValidator.validateTypeParameterInputs(...
                    geomType, parameters, tform);

                %The factory outputs a meshFile and any errors associated with
                %it
                [collisionGeometry, stlreadErrorCode, meshFile] = ...
                    robotics.manip.internal.CollisionGeometryFactory.makeCollisionGeometryFrom(...,
                    geomType, parameters, tform);

                robotics.manip.internal.CollisionInputValidator.throwInvalidMeshError(...,
                    stlreadErrorCode, meshFile);
            else
                %The input was a collision geometry object
                collisionObj = varargin{1};
                if(nargin == 2)
                    tform = eye(4);
                else
                    tform = varargin{2};
                end

                %Use the CollisionInputValidator to validate the collision
                %object input
                robotics.manip.internal.CollisionInputValidator.validateCollisionObjectInput(...
                    collisionObj, tform);

                %The collisionObjs are handle classes, hence we will ask the
                %factory to create a copy.
                collisionGeometry = ...
                    robotics.manip.internal.CollisionGeometryFactory.makeCollisionGeometryFromCollisionObj(...,
                    copy(collisionObj), tform * collisionObj.Pose);
            end
            if(~isempty(collisionGeometry))
                obj.CollisionsInternal.add(collisionGeometry);
            end
        end

        function addCollisionFromImporter(obj, type, parameter, tform)
            %addCollisionFromImporter Called by Importer to add a collision geometry

            %Validate the type of collision geometry
            type = ...
                robotics.manip.internal.CollisionInputValidator.validateType(type);
            collisionGeometry = ...
                robotics.manip.internal.CollisionGeometryFactory.makeCollisionGeometryFrom(...,
                type, parameter, tform);

            %Note that unlike addCollision the importer doesn't throw
            %an error if the mesh file is invalid
            if(~isempty(collisionGeometry))
                obj.CollisionsInternal.add(collisionGeometry);
            end
        end

        function errCode = addVisualInternal(obj, type, parameter, tform, color)
            %addVisualInternal

            rbgeo = robotics.manip.internal.RigidBodyGeometry();
            if nargin > 3
                rbgeo.Tform = tform;
            end
            if (nargin > 4) && ~isempty(color) % if color is specified, replace the default color
                rbgeo.Color = color.rgba;
            end

            errCode = 0;
            switch type
                case 'Mesh'
                    scale = [1 1 1];
                    if ischar(parameter)
                        geometryFile = parameter;
                    else
                        geometryFile = parameter{1};
                        if length(parameter)>1
                            scale = parameter{2};
                        end
                    end

                    %read CAD file
                    result = robotics.manip.internal.readCADFile(geometryFile);

                    if result.IsSuccess
                        F = result.Faces;
                        V = [scale(1)*result.Vertices(:,1), scale(2)*result.Vertices(:,2), scale(3)*result.Vertices(:,3)];
                        F = robotics.core.internal.PrimitiveMeshGenerator.flipFace(F);
                        rbgeo.SourceData = {'Mesh', geometryFile};
                    end
                    errCode = result.ErrorCode;
                case 'Box'
                    [F, V] = robotics.core.internal.PrimitiveMeshGenerator.boxMesh(parameter);
                    scale = [1 1 1];
                    rbgeo.SourceData = {'box', parameter};
                case 'Cylinder'
                    [F, V] = robotics.core.internal.PrimitiveMeshGenerator.cylinderMesh(parameter);
                    scale = [1 1 1];
                    rbgeo.SourceData = {'cylinder', parameter};
                case 'Sphere'
                    [F, V] = robotics.core.internal.PrimitiveMeshGenerator.sphereMesh(parameter);
                    scale = [1 1 1];
                    rbgeo.SourceData = {'sphere', parameter};
            end

            if ~isempty(rbgeo.SourceData)
                rbgeo.Faces = F;
                rbgeo.Vertices = V;
                rbgeo.Scale=scale;
                obj.VisualsInternal{end+1} = rbgeo;
            end
        end
    end


    methods

        function jnt = get.Joint(obj)
            %get.Joint Getter for the body's Joint property
            if obj.Index == 0
                robotics.manip.internal.error('rigidbody:NoSuchPropertyForBase', 'Joint');
            end
            jnt = obj.JointInternal;
        end

        function set.Joint(obj, jnt)
            %set.Joint Setter for the body's Joint property
            validateattributes(jnt, {'rigidBodyJoint'}, ...
                {'scalar'}, 'rigidBody', 'Joint')
            if obj.Index == 0
                robotics.manip.internal.error('rigidbody:NoSuchPropertyForBase', 'Joint');
            end

            if obj.Index > 0
                robotics.manip.internal.error('rigidbody:BodyAlreadyAddedToRobot', 'Joint', 'replaceJoint');
            end

            obj.JointInternal = copy(jnt);
        end

        function bname = get.Name(obj)
            bname = obj.NameInternal;
        end

        function set.Name(obj, bname)
            %set.Name
            validateattributes(bname,{'char'},{'nonempty','row'},...
                'rigidBody','Name');
            if obj.Index == 0
                robotics.manip.internal.error('rigidbody:UseBaseNameProperty');
            end
            if obj.Index > 0
                robotics.manip.internal.error('rigidbody:BodyAlreadyAddedToRobot', 'Name', 'replaceBody');
            end
            obj.NameInternal = bname;
        end

        function set.Inertia(obj, I)
            %set.Inertia
            if obj.Index == 0
                robotics.manip.internal.error('rigidbody:NoSuchPropertyForBase', 'Inertia');
            end

            validateattributes(I,{'double'},{'nonempty','vector','nonnan', 'finite', 'real','numel', 6},...
                'rigidBody','Inertia');
            inertiaInternal = [ I(1), I(6), I(5); I(6), I(2), I(4); I(5), I(4), I(3)];

            if ~robotics.core.internal.isPositiveSemidefinite(inertiaInternal)
                robotics.manip.internal.error('rigidbody:InertiaMatrixNotPSD');
            end
            obj.InertiaInternal = inertiaInternal;
            obj.SpatialInertia = robotics.manip.internal.spatialInertia(obj.MassInternal, obj.CenterOfMassInternal, obj.InertiaInternal);
        end

        function I = get.Inertia(obj)
            %get.Inertia
            if obj.Index == 0
                robotics.manip.internal.error('rigidbody:NoSuchPropertyForBase', 'Inertia');
            end

            I = robotics.manip.internal.flattenInertia(obj.InertiaInternal);

        end

        function set.Mass(obj, m)
            %set.Mass
            if obj.Index == 0
                robotics.manip.internal.error('rigidbody:NoSuchPropertyForBase', 'Mass');
            end
            validateattributes(m, {'double'},{'nonempty','scalar','nonnan', 'finite', 'real','nonnegative' },...
                'rigidBody','Mass');
            obj.MassInternal = m;
            obj.SpatialInertia = robotics.manip.internal.spatialInertia(obj.MassInternal, obj.CenterOfMassInternal, obj.InertiaInternal);
        end

        function m = get.Mass(obj)
            %get.Mass
            if obj.Index == 0
                robotics.manip.internal.error('rigidbody:NoSuchPropertyForBase', 'Mass');
            end
            m = obj.MassInternal;
        end

        function set.CenterOfMass(obj, com)
            %set.CenterOfMass
            if obj.Index == 0
                robotics.manip.internal.error('rigidbody:NoSuchPropertyForBase', 'CenterOfMass');
            end

            validateattributes(com, {'double'},{'nonempty','vector','nonnan', 'finite', 'real', 'numel', 3},...
                'rigidBody','CenterOfMass');
            obj.CenterOfMassInternal = com(:)';
            obj.SpatialInertia = robotics.manip.internal.spatialInertia(obj.MassInternal, obj.CenterOfMassInternal, obj.InertiaInternal);
        end

        function com = get.CenterOfMass(obj)
            %get.CenterOfMass
            if obj.Index == 0
                robotics.manip.internal.error('rigidbody:NoSuchPropertyForBase', 'CenterOfMass');
            end

            com = obj.CenterOfMassInternal;

        end
    end


end

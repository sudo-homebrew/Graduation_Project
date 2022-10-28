classdef rigidBody < robotics.manip.internal.InternalAccess
%rigidBody Create a rigid body
%   rigidBody creates a rigid body. A rigid body is the building block
%   for any tree-structured robot manipulator. Each rigid body has a
%   joint attached to it.
%
%   body = rigidBody(bodyname) creates a rigid body with name
%   as bodyname.
%
%   body = rigidBody(bodyname, 'MaxNumCollisions', N) allows the user to
%   specify an upper bound on the number of collision geometries allowed in
%   the body when generating code.
%
%   rigidBody properties:
%       Name            - Name of the rigid body
%       Joint           - The joint associated with the rigid body
%       Mass            - Mass of the rigid body (kg)
%       CenterOfMass    - Center of mass of the rigid body (m)
%       Inertia         - Inertia of the rigid body (kg*m^2)
%       Parent          - Parent of the body
%       Children        - Children of the body
%       Visuals         - Visual geometries assigned to the body
%       Collisions      - Collision geometries assigned to the body
%
%   rigidBody methods:
%       copy            - Create a deep copy of the rigid body
%       addVisual       - Add a visual geometry to the rigid body
%       addCollision    - Add a collision geometry to the rigid body
%       clearVisual     - Clear all the attached visual geometries
%       clearCollision  - Clear all the attached collision geometries
%
%   Example:
%
%       % Create a rigid body named B1
%       body1 = rigidBody('B1');
%
%       % By default the body comes with a fixed joint named 'B1_jnt'.
%       % Replace it with a revolute joint named 'J1'.
%       body1.Joint = rigidBodyJoint('J1', 'revolute');
%
%   See also rigidBodyJoint, rigidBodyTree.

%   Copyright 2016-2021 The MathWorks, Inc.

%#codegen


    properties (Dependent)
        %Name Name of the body
        %
        %   A nonempty string must be provided upon object construction.
        Name

        %Joint The joint associated with the rigid body
        %   This joint is a tree joint (i.e. not a loop joint).
        %
        %   Default: A rigidBodyJoint handle object with 'fixed' joint type
        %   and '<bodyname>_jnt' joint name.
        Joint

        %Mass Mass of the rigid body
        %   Body mass cannot be negative. Unit: kilogram (kg).
        %
        %   Default: 1
        Mass

        %CenterOfMass Center of mass vector of the rigid body,
        %   relative to the body frame. Unit: meter (m).
        %
        %   Default: [0 0 0]
        CenterOfMass

        %Inertia Inertia of the rigid body relative to the body frame.
        %   The 6 independent elements are specified in the order of
        %   [Ixx Iyy Izz Iyz Ixz Ixy]. Unit: kilogram-meter-squared (kg*m^2)
        %
        %   Default: [1 1 1 0 0 0]
        Inertia

        %Parent Parent of the body
        %
        %   This property will be updated automatically once the body is
        %   added to a robot. By default it's not defined.
        Parent

        %Children Children of the body
        %
        %   Default: {1x0 cell}
        %   This property will be updated automatically once the body is
        %   added to a robot
        Children

        %Visuals List of visual geometries assigned to the body
        %
        %   Default: {}
        Visuals

        %Collisions Collision geometries assigned to the body
        %
        %   Default: {}
        Collisions
    end

    properties (Access = {?robotics.manip.internal.InternalAccess})

        %BodyInternal
        BodyInternal

        %TreeInternal
        TreeInternal

    end



    methods
        function obj = rigidBody(bname, varargin )
        %rigidBody Constructor

            bname = convertStringsToChars(bname);

            if coder.target('MATLAB')
                parser = inputParser;
                parser.addParameter('BodyInternal', robotics.manip.internal.RigidBody(bname) );
                parser.addParameter('TreeInternal', robotics.manip.internal.RigidBodyTree(1) );
                parser.addParameter('MaxNumCollisions', 0);
                parser.parse(varargin{:});
                bodyInternal = parser.Results.BodyInternal;
                treeInternal = parser.Results.TreeInternal;

                validateattributes(bodyInternal, {'robotics.manip.internal.RigidBody'}, ...
                                   {'scalar', 'nonempty'},'rigidBody', 'the second input argument');
                validateattributes(treeInternal, {'robotics.manip.internal.RigidBodyTree'}, ...
                                   {'scalar', 'nonempty'},'rigidBody', 'the third input argument');

            else
                defaultMaxNumCollisions = 0;
                params = struct('BodyInternal', uint32(0), ...
                                'TreeInternal', uint32(0), ...
                                'MaxNumCollisions', uint32(0));

                pstruct = coder.internal.parseParameterInputs(params, [], varargin{:});

                maxNumCollisions =...
                    coder.internal.getParameterValue(pstruct.MaxNumCollisions, ...
                                                     defaultMaxNumCollisions,...
                                                     varargin{:});
                bodyInternal = coder.internal.getParameterValue(pstruct.BodyInternal, ...
                                                                robotics.manip.internal.RigidBody(bname), varargin{:});
                treeInternal = coder.internal.getParameterValue(pstruct.TreeInternal,...
                                                                robotics.manip.internal.RigidBodyTree(1), varargin{:});

                coder.internal.assert(isa(bodyInternal, 'robotics.manip.internal.RigidBody'), ...
                                      'robotics:robotmanip:rigidbody:InvalidConstructorNameValuePair');
                coder.internal.assert(isa(treeInternal, 'robotics.manip.internal.RigidBodyTree'), ...
                                      'robotics:robotmanip:rigidbody:InvalidConstructorNameValuePair');
                %if maxNumCollisions is specified, then clear the collisions
                %internal and reconstruct it.
                if(maxNumCollisions ~= defaultMaxNumCollisions)
                    bodyInternal.CollisionsInternal = ...
                        robotics.manip.internal.CollisionSet(maxNumCollisions);
                end
            end
            obj.BodyInternal = bodyInternal;
            obj.TreeInternal = treeInternal;
        end


        function newbody = copy(obj)
        %COPY Creates a deep copy of the rigid body object
        %   NEWBODY = COPY(BODY) creates a selective deep copy of the
        %   body object. Some internal properties will be reset to
        %   default instead of being copied (for example, Parent and
        %   Children)
        %
        %   Example:
        %       % Create a rigid body named B1
        %       b1 = rigidBody('B1');
        %
        %       % make a deep copy
        %       b2 = copy(b1)
        %
        %   See also rigidBody

            newbody = rigidBody('-', 'BodyInternal', copy(obj.BodyInternal));
        end


        function addVisual(obj, varargin)
        %addVisual Add visual geometry data to rigid body
        %   addVisual(BODY, 'Mesh', FILENAME) appends a polygon mesh
        %   as specified in FILENAME on top of any visual geometry
        %   currently stored in the body. The frame of the polygon
        %   mesh is assumed to coincide with the frame of BODY. The
        %   mesh file must be in STL or DAE format.
        %
        %   addVisual(BODY, 'Mesh', FILENAME, TFORM) specify a TFORM for
        %   the polygon mesh relative to the body frame. TFORM is
        %   specified as a 4x4 homogeneous transform.
        %
        %   Example:
        %       % Create a rigid body named B1
        %       b1 = rigidBody('B1');
        %
        %       % Attach a Mesh visual to body B1
        %       T = trvec2tform([0.1 0.1 0.2]) * eul2tform([0 0 pi], 'ZYX');
        %       addVisual(b1, 'Mesh', 'groundvehicle.stl', T);

        %Convert optional inputs to strings. This is done with explicit
        %variable sizes to ensure code generation support
            numInputs = numel(varargin);
            Inputs = cell(1,numInputs);
            for i = 1:numInputs
                Inputs{i} = convertStringsToChars(varargin{i});
            end

            obj.BodyInternal.addVisual(Inputs{:});
        end

        function addCollision(obj, varargin)
        %addCollision Add collision geometry data to the rigid body
        %   addCollision(BODY, TYPE, PARAMETERS) appends a collision
        %   geometry to the BODY.
        %
        %  TYPE specifies the type of geometry as a string and determines
        %  the PARAMETERS input:
        %     - "box": [x,y,z]
        %     - "cylinder": [radius,length]
        %     - "sphere": [radius]
        %     - "mesh": N-by-3 matrix of vertices or
        %               STL or DAE file name as a string
        %
        %   addCollision(BODY, COLLISIONOBJ) appends a collision geometry to
        %   the BODY. The COLLISIONOBJ is a collision object of type
        %   collisionBox, collisionCylinder, collisionSphere, or
        %   collisionMesh. This function uses the Pose property of the
        %   COLLISIONOBJ to transform the vertices into the rigid body
        %   frame.
        %
        %   addCollision(__,TFORM) specifies a transformation for the
        %   collision geometry relative to the body frame.  If specifying a
        %   collision object, this function uses TFORM*COLLISIONOBJ.Pose to
        %   transform the vertices into the rigidBody frame.
        %
        %   Example:
        %       % Create a rigid body named B1
        %       b1 = rigidBody('B1');
        %
        %       % Attach a box collision to body B1
        %       T = trvec2tform([0.1 0.1 0.2]) * eul2tform([0 0 pi], 'ZYX');
        %       addCollision(b1, "box", [1, 2, 3], T);

            obj.BodyInternal.addCollisionInternal(varargin{:})
        end

        function clearVisual(obj)
        %clearVisual Clear all the visuals attached to the rigid body
        %   clearVisuals(BODY) clears all the visual geometries that
        %   are currently attached to BODY.
        %
        %   Example:
        %       % Import a robot model from URDF
        %       lbr = importrobot('iiwa14.urdf');
        %
        %       % Clear the visual geometries associated with body 1
        %       clearVisual(lbr.Bodies{1});

            obj.BodyInternal.clearVisual();
        end

        function clearCollision(obj)
        %clearCollision Clear all the collisions attached to the rigid body
        %   clearCollision(BODY) clears all the collisions geometries that
        %   are currently attached to BODY.
        %
        %   Example:
        %       % Import a robot model from URDF
        %       lbr = importrobot('iiwa14.urdf');
        %
        %       % Clear the collision geometries associated with body 1
        %       clearCollision(lbr.Bodies{1});

            obj.BodyInternal.clearCollision();
        end

        function tf = eq(obj1, obj2)
        %eq Override == behavior
            tf = isequal(obj1.BodyInternal, obj2.BodyInternal);
        end

    end


    methods
        function value = get.Name(obj)
        %get.Name
            value = obj.BodyInternal.Name;
        end

        function set.Name(obj, value)
        %set.Name
            value = convertStringsToChars(value);
            obj.BodyInternal.Name = value;
        end

        function value = get.Joint(obj)
        %get.Joint
            value = obj.BodyInternal.Joint;
        end

        function set.Joint(obj, value)
        %set.Joint
            obj.BodyInternal.Joint = value;
        end


        function value = get.Mass(obj)
        %get.Mass
            value = obj.BodyInternal.Mass;
        end

        function set.Mass(obj, value)
        %set.Mass
            obj.BodyInternal.Mass = value;
        end

        function value = get.CenterOfMass(obj)
        %get.CenterOfMass
            value = obj.BodyInternal.CenterOfMass;
        end

        function set.CenterOfMass(obj, value)
        %set.CenterOfMass
            obj.BodyInternal.CenterOfMass = value;
        end

        function value = get.Inertia(obj)
        %get.Inertia
            value = obj.BodyInternal.Inertia;
        end

        function set.Inertia(obj, value)
        %set.Inertia
            obj.BodyInternal.Inertia = value;
        end


        function parent = get.Parent(obj)
        %get.Parent
            parent = rigidBody('-', 'BodyInternal', obj.TreeInternal.parentOf(obj.Name),...
                               'TreeInternal', obj.TreeInternal);
        end

        function children = get.Children(obj)
        %get.Children
            childrenInternal = obj.TreeInternal.childrenOf(obj.Name);
            l = length(childrenInternal);
            children = repmat({obj}, 1, l);
            for i = coder.unroll(1:obj.TreeInternal.MaxNumBodies)
                if i <= l
                    children{i} = rigidBody('-', 'BodyInternal', childrenInternal{i}, ...
                                            'TreeInternal', obj.TreeInternal);
                end
            end
        end

        function value = get.Visuals(obj)
        %set.Visuals
            value = {};
            if coder.target('matlab')
                for i = 1:length(obj.BodyInternal.VisualsInternal)
                    value{end+1} = obj.BodyInternal.VisualsInternal{i}.getTag;
                end
            end
        end

        function value = get.Collisions(obj)
        %get.Collisions
        %   Collisions is a cell-array of strings which captures the
        %   collision geometry data of the rigid body
            value = {};
            if coder.target('matlab')
                value = obj.BodyInternal.CollisionsInternal.Tags;
            end
        end

    end

    methods(Static)
        function obj = loadobj(objFromMAT)
        %loadobj Provide backward-compatibility support when a 16b
        %   rigidBody object is loaded from a MAT file. This function is
        %   to be invoked by the "load" command.

            if isstruct(objFromMAT) % if this is a 16b object

                newObj = rigidBody(objFromMAT.NameInternal);
                newObj.Joint = objFromMAT.JointInternal;
                if objFromMAT.Index > 0 % if this rigid body is in tree, continue to extract its children indices
                    newObj.BodyInternal.Index = objFromMAT.Index;
                    cs = size(objFromMAT.ChildrenInternal, 2);
                    childrenIndices = zeros(1,cs);
                    for i = 1:cs
                        childrenIndices(i) = objFromMAT.ChildrenInternal{i}.BodyInternal.Index;
                    end
                    newObj.BodyInternal.ChildrenIndices = childrenIndices;
                end

                obj = newObj;
            else
                obj = objFromMAT;
            end
        end
    end

end

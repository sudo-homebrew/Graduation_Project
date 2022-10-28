classdef rigidBodyTree < robotics.manip.internal.InternalAccess
%rigidBodyTree Create a tree-structured robot
%   ROBOT = rigidBodyTree() creates a default model that contains no
%   rigid bodies.
%
%   ROBOT = rigidBodyTree('MaxNumBodies', N) allows the user to specify
%   an upper bound on the number of bodies allowed in the robot when
%   generating code.
%
%   ROBOT = rigidBodyTree('DataFormat', FMT) allows the user to specify
%   the data format for the RigidBodyTree object. The possible values
%   for FMT are 'struct', 'column' and 'row'. The default is 'struct',
%   but to use the dynamics methods change the data format to 'row' or
%   'column'.
%
%
%   rigidBodyTree properties:
%       NumBodies              - Number of bodies
%       Bodies                 - Cell array of rigid bodies
%       Base                   - Base of the robot
%       BodyNames              - Cell array of Names of rigid bodies
%       BaseName               - Name of robot base
%       Gravity                - Gravitational acceleration (m/s^2)
%       DataFormat             - Input/output data format
%
%
%   rigidBodyTree methods:
%       addBody               - Add a body to robot
%       removeBody            - Remove a body from robot
%       replaceBody           - Replace a body in the robot
%       replaceJoint          - Replace the joint of one of robot's body
%       addSubtree            - Attach a subtree to current robot
%       subtree               - Get a subtree from robot as a new robot
%       getBody               - Get robot's body handle by name
%       geometricJacobian     - Compute the geometric Jacobian
%       copy                  - Copy robot
%       getTransform          - Get transform between two body frames
%       homeConfiguration     - Return the home configuration for robot
%       randomConfiguration   - Return a random configuration for robot
%       showdetails           - Display details of robot
%       show                  - Plot robot body frames
%       massMatrix            - Compute joint-space mass matrix
%       inverseDynamics       - Compute required joint torques given desired motion
%       forwardDynamics       - Compute resultant joint accelerations given joint torques and states
%       centerOfMass          - Compute center of mass position and Jacobian
%       velocityProduct       - Compute joint torques that cancel velocity induced forces
%       gravityTorque         - Compute joint torques that compensate gravity
%       externalForce         - Formulate external force matrix
%       checkCollision        - Check if robot is in collision
%       writeAsFunction       - Create rigidBodyTree generating function
%
%
%   Example:
%
%       % Create a robot
%       rob = rigidBodyTree()
%
%       % Add a body to robot as child of robot's base
%       body1 = rigidBody('Link1');
%       addBody(rob, body1, rob.BaseName);
%
%       % Display robot details
%       showdetails(rob);
%
%   See also rigidBody, rigidBodyJoint.

%   Copyright 2016-2021 The MathWorks, Inc.

%#codegen

    properties (SetAccess = private, GetAccess = ?robotics.manip.internal.InternalAccess)
        TreeInternal
    end

    properties(SetAccess = private, Dependent)
        %NumBodies Number of rigid bodies in the robot
        %
        %   Default: 0
        NumBodies

        %Bodies Cell array of rigidBody handles
        %
        %   Default: {}
        Bodies

        %Base Base of the robot
        %
        %   1x1 rigidBody with no access to Joint and Parent properties
        Base
    end

    properties (Dependent)
        %BodyNames Names of all rigid bodies in the robot
        %
        %   Default: {}
        BodyNames

        %BaseName Name of the base of the robot
        %
        %   Default: 'base'
        BaseName

        %Gravity Gravitational acceleration experienced by the robot
        %   Unit: meter-per-second-squared (m/s^2)
        %
        %   Default: [0 0 0]
        Gravity

        %DataFormat Input/output data format for kinematics/dynamics function
        %   This char vector indicates the input and output data format
        %   for all the kinematics and dynamics functions of RigidBodyTree
        %   object. It takes either one of the three values: 'struct',
        %   'row', 'column'.
        %
        %   Default: 'struct'
        DataFormat
    end


    properties (Dependent, Access = ?robotics.manip.internal.InternalAccess)
        %MaxNumBodies Upper bound on number of bodies allowed in the tree.
        %
        %   Default: 0 (MATLAB execution only, this property must be
        %            specified by user through constructor name-value pair
        %            input when generating code)
        MaxNumBodies

        %ShowTag Tag to identify the graphic objects that belong to the robot.
        %   The tag is a randomly generated string.
        ShowTag

        %EstimatedMaxReach Estimated max reach of the manipulator
        %
        %   Default: 1 (meter)
        EstimatedMaxReach

        %NumNonFixedBodies Number of rigid bodies with non-fixed joint in the robot
        %
        %   Default: 0
        NumNonFixedBodies

    end

    methods
        function obj = rigidBodyTree(varargin)
        %rigidBodyTree Constructor
        %   Please see the class documentation for more details on different call syntaxes.
        %   See also rigidBodyTree.

            if nargin==1 && isa(varargin{1}, 'robotics.manip.internal.RigidBodyTree')
                %For internal use, create tree if TreeInternal is known
                obj.TreeInternal = varargin{1};
            else
                if coder.target('MATLAB')
                    parser = inputParser;
                    parser.addParameter('MaxNumBodies', 0);
                    parser.addParameter('DataFormat', 'struct');
                    parser.parse(varargin{:});
                    maxNumBodies = parser.Results.MaxNumBodies;
                    dataFormat = parser.Results.DataFormat;
                else
                    params = struct('MaxNumBodies', uint32(0), ...
                                    'DataFormat', uint32(0));

                    pstruct = coder.internal.parseParameterInputs(params, [], varargin{:});
                    maxNumBodies = coder.internal.getParameterValue(pstruct.MaxNumBodies, nan, varargin{:});
                    dataFormat = coder.internal.getParameterValue(pstruct.DataFormat, 'struct', varargin{:});

                    coder.internal.assert(~isnan(maxNumBodies), 'robotics:robotmanip:rigidbodytree:MaxNumBodiesRequiredForCodegen');
                end

                obj.TreeInternal = robotics.manip.internal.RigidBodyTree(maxNumBodies, dataFormat);
                if(~coder.target('MATLAB'))

                    %There is no special constructor for the rigidBodyTree/Base.
                    %Hence, the "MaxNumCollisions" unlike any other
                    %rigidBody, is assumed constant in the case of the
                    %rigidBodyTree/Base during code generation
                    baseMaxNumCollisions = 10;

                    obj.TreeInternal.Base.CollisionsInternal = ...
                        robotics.manip.internal.CollisionSet(baseMaxNumCollisions);
                end
            end
        end



        function bodyhandle = getBody(obj, bodyname)
        %getBody Get robot's body handle by name
        %   BDY = getBody(ROBOT, BODYNAME) returns BDY, which is the
        %   handle to the rigid body named BODYNAME in ROBOT.
        %
        %   Example:
        %       % Load predefined robot models
        %       load exampleRobots
        %
        %       % Get the body named 'right_wrist' from Baxter robot
        %       rightwrist = getBody(baxter, 'right_wrist')

            bodyInternal = obj.TreeInternal.getBody(bodyname);
            bodyhandle = rigidBody('-', 'BodyInternal', bodyInternal, ...
                                   'TreeInternal', obj.TreeInternal);

        end


        function addBody(obj, bodyin, parentName)
        %addBody Add a body to the robot
        %   addBody(ROBOT, BODYIN, PARENTNAME) adds BODYIN to ROBOT by
        %   attaching it to the body with name PARENTNAME.
        %
        %   Example:
        %       % Create a robot. By default the cell array of bodies
        %       % is empty
        %       rob = rigidBodyTree();
        %
        %       % Create a rigid body. By default the body contains a
        %       % fixed joint
        %       b1 = rigidBody('link1')
        %
        %       % Change the body's joint to revolute
        %       b1.Joint = rigidBodyJoint('jnt1', 'revolute');
        %
        %       % Add this body to robot, attaching to base
        %       addBody(rob, b1, 'base');
        %
        %   See also rigidBodyTree, removeBody, subtree,
        %   addSubtree

            narginchk(3,3);
            validateattributes(bodyin, {'rigidBody'}, ...
                               {'scalar', 'nonempty'},'addBody', 'bodyin');

            obj.TreeInternal.addBody(bodyin.BodyInternal, parentName);

        end


        function replaceJoint(obj, bodyname, joint)
        %replaceJoint Replace the joint of one of robot's body
        %   replaceJoint(ROBOT, BODYNAME, JNT) replaces the joint of
        %   a rigidBody object with name BODYNAME inside ROBOT with JNT.
        %   When a rigid body is part of a rigid body tree, its joint
        %   cannot be changed through direct assignment. It can only be
        %   changed using this method.
        %
        %   Example:
        %       % Load predefined robot models
        %       load exampleRobots
        %
        %       % Create a replacement joint
        %       jnt = rigidBodyJoint('JR', 'revolute');
        %
        %       % Replace the joint of body L3 in PUMA robot
        %       replaceJoint(puma1, 'L3', jnt)
        %
        %   See also replaceBody

            narginchk(3,3);
            obj.TreeInternal.replaceJoint(bodyname, joint);
        end


        function replaceBody(obj, bodyname, newbody)
        %replaceBody Replace a body in the robot
        %   replaceBody(ROBOT, BODYNAME, NEWBODY) changes all the
        %   properties of body with BODYNAME in ROBOT with those in
        %   NEWBODY excepts the Parent and Children properties
        %
        %   Example:
        %       % Load predefined robot models
        %       load exampleRobots
        %
        %       % Create a replacement body
        %       bd = rigidBody('LR');
        %
        %       % Swap the body called 'L3' in PUMA robot with the replacement
        %       replaceBody(puma1, 'L3', bd);
        %
        %   See also replaceJoint
            narginchk(3,3);
            validateattributes(newbody, {'rigidBody'}, ...
                               {'scalar', 'nonempty'},'replaceBody', 'newbody');
            b = copy(newbody);
            obj.TreeInternal.replaceBody(bodyname, b.BodyInternal);
        end



        function newrobot = removeBody(obj, bodyname)
        %removeBody Remove a body from robot
        %   removeBody(ROBOT, BODYNAME) removes the set of bodies in
        %   the subtree starting at the body with name BODYNAME from ROBOT,
        %   body BODYNAME is included.
        %
        %   NEWROBOT = removeBody(ROBOT, BODYNAME) removes the set of
        %   bodies in the subtree starting from the body with name BODYNAME
        %   from ROBOT, including body BODYNAME, and returns the removed
        %   subtree as a new robot NEWROBOT. In the new robot, the
        %   parent name of body BODYNAME is the name of the base.
        %
        %   Example:
        %       % Load predefined robot models
        %       load exampleRobots
        %
        %       % Detach Baxter robot's right arm
        %       rightarm = removeBody(baxter, 'right_upper_shoulder')
        %
        %       % Baxter robot model is modified
        %       showdetails(baxter);
        %
        %   See also subtree, addSubtree, addBody

            newRobotInternal = obj.TreeInternal.removeBody(bodyname);

            newrobot = rigidBodyTree('MaxNumBodies', newRobotInternal.MaxNumBodies, 'DataFormat', newRobotInternal.DataFormat);
            newrobot.TreeInternal = newRobotInternal;

        end


        function newrobot = subtree(obj, bodyname)
        %SUBTREE Get a subtree from robot
        %   NEWROBOT = SUBTREE(ROBOT, BODYNAME) creates a new
        %   RigidBodyTree object NEWROBOT from a subtree in ROBOT
        %   starting from its body with name BODYNAME. The base name of
        %   NEWROBOT will be the name of the parent of body BODYNAME. The
        %   original robot ROBOT will not be affected at all by this
        %   method. If BODYNAME is the base name of ROBOT, a copy of ROBOT
        %   is made.
        %
        %   Example:
        %       % Load predefined robot models
        %       load exampleRobots
        %
        %       % Get the right arm of Baxter robot
        %       rightarm = SUBTREE(baxter, 'right_upper_shoulder')
        %
        %       % Baxter robot model is intact
        %       showdetails(baxter)
        %
        %   See also addSubtree, removeBody

            newRobotInternal = obj.TreeInternal.subtree(bodyname);

            newrobot = rigidBodyTree('MaxNumBodies', newRobotInternal.MaxNumBodies, 'DataFormat', newRobotInternal.DataFormat);
            newrobot.TreeInternal = newRobotInternal;
        end

        function newrobot = copy(obj)
        %COPY Copy the tree-structured robot
        %   NEWROBOT = COPY(ROBOT) returns a deep copy of ROBOT. NEWROBOT
        %   and ROBOT are two different RigidBodyTree objects with the
        %   same properties.
        %
        %   Example:
        %       % Load predefined robot models
        %       load exampleRobots
        %
        %       % Make a copy of PUMA robot
        %       puma1_cpy = COPY(puma1)

            newRobotInternal = obj.TreeInternal.copy();
            newrobot = rigidBodyTree('MaxNumBodies', obj.MaxNumBodies, 'DataFormat', obj.DataFormat);
            newrobot.TreeInternal = newRobotInternal;
        end


        function addSubtree(obj, bodyname, subtreerobot)
        %addSubtree Add a subtree to current robot
        %   addSubTree(ROBOT, BODYNAME, SUBTREEROBOT) attaches SUBTREEROBOT,
        %   which is RigidBodyTree object, to ROBOT at the body with
        %   BODYNAME.
        %
        %   Example;
        %       % Load predefined robot models
        %       load exampleRobots
        %
        %       % Swap arms for Baxter robot
        %       % Detach Baxter robot's right arm
        %       rightarm = removeBody(baxter, 'right_upper_shoulder');
        %
        %       % Detach Baxter robot's left arm
        %       leftarm = removeBody(baxter, 'left_upper_shoulder');
        %
        %       % Attach right arm to the left arm mount
        %       addSubtree(baxter, 'left_arm_mount', rightarm);
        %
        %       % Attach left arm to the right arm mount
        %       addSubtree(baxter, 'right_arm_mount', leftarm);
        %
        %   See also subtree, removeBody

            validateattributes(subtreerobot, {'rigidBodyTree'},...
                               {'scalar'}, 'addSubtree', 'subtreerobot');
            obj.TreeInternal.addSubtree(bodyname, subtreerobot.TreeInternal);
        end




        function Q = homeConfiguration(obj)
        %homeConfiguration Return the home configuration for robot
        %   Q = homeConfiguration(ROBOT) returns the home
        %   configuration of ROBOT as predefined in the robot model.
        %   The configuration Q is returned as an array of structs.
        %   The structure array contains one struct for each non-fixed
        %   joint. Each struct contains two fields
        %       - JointName
        %       - JointPosition
        %   The sequence of structs in the array is the same as that
        %   displayed by SHOWDETAILS
        %
        %
        %   Example;
        %       % Load predefined robot models
        %       load exampleRobots
        %
        %       % Get the predefined home configuration for PUMA robot
        %       Q = homeConfiguration(puma1)
        %
        %   See also showdetails, randomConfiguration

            Q = obj.TreeInternal.homeConfiguration();
        end


        function Q = randomConfiguration(obj)
        %randomConfiguration Return a random configuration for robot
        %   Q = randomConfiguration(ROBOT) returns a random
        %   configuration of ROBOT that falls within the predefined joint
        %   limits. The configuration is returned as an array of structs.
        %   The structure array contains one struct for each non-fixed
        %   joint. Each struct contains two fields
        %       - JointName
        %       - JointPosition
        %   The sequence of structs in the array is the same as that
        %   displayed by SHOWDETAILS
        %
        %   Example;
        %       % Load predefined robot models
        %       load exampleRobots
        %
        %       % Get a random configuration for PUMA robot
        %       Q = randomConfiguration(puma1)
        %
        %   See also showdetails, homeConfiguration

            Q = obj.TreeInternal.randomConfiguration();
        end



        function [isColliding, separationDist, witnessPts] = checkCollision(obj, config, varargin)
        %checkCollision Check if robot is in collision
        %   [ISSELFCOLLIDING, SELFSEPARATIONDIST, SELFWITNESSPTS] = checkCollision(ROBOT, CONFIG)
        %   checks if the ROBOT, specified as a rigidBodyTree object, is in
        %   self-collision at the specific configuration, CONFIG.
        %
        %   [ISCOLLIDING, SEPARATIONDIST, WITNESSPTS] = checkCollision(ROBOT, CONFIG, WORLDOBJECTS)
        %   checks if the ROBOT is in self-collision or is colliding with a
        %   set of collision objects specified in WORLDOBJECTS.
        %
        %   INPUT ARGUMENTS:
        %
        %       ROBOT - Rigid body tree robot model, specified as a
        %       rigidBodyTree object. Add collision geometry data to your model
        %       using addCollision on each rigid body.
        %
        %       CONFIG - Joint configuration of the rigid body tree,
        %       specified as an N-element vector, where N is the number of
        %       non-fixed joints. The DataFormat property of ROBOT
        %       determines the vector's shape. Only "row" or "column" data
        %       formats are supported.
        %
        %       WORLDOBJECTS - Cell-array of collision object containing
        %       collisionBox, collisionCylinder, collisionSphere, and
        %       collisionMesh objects.
        %
        %   OUTPUT ARGUMENTS:
        %
        %       ISSELFCOLLIDING - Logical scalar indicating self-collision
        %
        %       SELFSEPARATIONDIST - Separation distance between the bodies
        %       on the robot, including the base, returned as a square
        %       matrix. The matrix indices correspond to the indices in the
        %       Bodies property of the ROBOT and the last row and column
        %       correspond to the base.
        %
        %       SELFWITNESSPTS - Witness points between bodies of the robot
        %       including the base, specified as a 3(N+1)-by-2(N+1) matrix,
        %       where N is the number of bodies, returned as follows:
        %
        %       [Wr1_1       Wr1_2        ...     Wr1_(N)    Wr1_(N+1);
        %        Wr2_1       Wr2_2        ...     Wr2_(N)    Wr2_(N+1);
        %        .           .            .       .         .
        %        .           .            .       .         .
        %        .           .            .       .         .
        %        Wr(N+1)_1   Wr(N+1)_2    ...     Wr(N+1)_N  Wr(N+1)_(N+1)]
        %
        %       Each element, Wr, in the above matrix is a 3-by-2 matrix
        %       that gives the nearest [x y z] points on the two
        %       corresponding bodies in the robot. The final row and column
        %       correspond to the robot base.
        %
        %       ISCOLLIDING - Indicates both self-collision and collision
        %       with WORLDOBJECTS respectively as a two-element vector. If
        %       "IgnoreSelfCollision" is "on", ISCOLLIDING is a logical
        %       scalar indicating collision with WORLDOBJECTS.
        %
        %       SEPARATIONDIST - Separation distance between bodies and
        %       world objects, specified as an (N+1)-by-(N+M+1) matrix,
        %       where N is the number of bodies, and M is the number of
        %       world objects. The first N row or column indices correspond
        %       to the bodies on the robot. The N+1 row or column index
        %       corresponds to the robot base.  The last M column indices
        %       correspond to the world objects list. If
        %       "IgnoreSelfCollision" is "on", SEPARATIONDIST is an
        %       (N+1)-by-M matrix with column indices corresponding to the
        %       world objects.
        %
        %       WITNESSPTS - Witness points between the bodies of the robot,
        %       including the base, and the world objects. It is a
        %       3*(N+1)-by-2*(N+1+M) matrix, where N is the number of bodies
        %       and M is the number of world objects, returned as follows:
        %
        %       [Wr1_1       Wr1_2     ...    Wr1_(N+1)     Wo1_1     Wo1_2      ... W1_M;
        %        Wr2_1       Wr2_2     ...    Wr2_(N+1)     Wo2_1     Wo2_2      ... W2_M;
        %        .           .         .      .             .         .          .   .
        %        .           .         .      .             .         .          .   .
        %        .           .         .      .             .         .          .   .
        %        Wr(N+1)_1   Wr(N+1)_2 ...    Wr(N+1)_(N+1) Wo(N+1)_1 Wo(N+1)_2  ... W(N+1)_M]
        %
        %       Each element in the above matrix, is a 3-by-2 matrix that
        %       gives the nearest [x y z] points on the corresponding world
        %       object/body. Wr elements specify the SELFWITNESSPTS.
        %       Woi_j are the witness points between the robot bodies and
        %       base, and the world objects.  The j indices correspond to
        %       the world object list, and the i indices correspond to the
        %       robot bodies, with N+1 as the base. If "IgnoreSelfCollision"
        %       is "on", WITNESSPTS only contains Wo elements.
        %
        %   If a pair is in collision the separation distance is set to NaN,
        %   and the witness points are set to NaN(3, 2). If a collision pair
        %   is ignored, the separation distance is set to Inf, and the
        %   witness points are set to Inf(3, 2). Note that adjacent bodies
        %   are ignored when checking for self-collision.
        %
        %   [ISCOLLIDING, SEPARATIONDIST, WITNESSPTS] = checkCollision(__, Name, Value)
        %   provides additional options specified by one more Name Value
        %   pair arguments. You can specify several name-value pair
        %   arguments in any order as Name1, Value1, ..., NameN, ValueN:
        %
        %      'Exhaustive'             - Exhaustively check for all collisions.
        %                                 Setting this to 'off'
        %                                 returns when the first collision
        %                                 is found. If returned early,
        %                                 separation distance and witness
        %                                 points values are Inf for
        %                                 incomplete checks.
        %                                 Setting this to 'on' will compute the
        %                                 separation distance and witness
        %                                 points for all pairs.
        %
        %                                 Default: 'off'
        %
        %      'IgnoreSelfCollision'    - Skip checking for collisions between robot bodies and base
        %
        %                                 Default: 'off'
        %   Example:
        %       %Load the robot
        %       robot = loadrobot("kukaIiwa14", "DataFormat", "row");
        %       show(robot);
        %       hold on;
        %
        %       %Create the world objects
        %       box = collisionBox(0.2, 0.2, 0.2);
        %       sphere = collisionSphere(0.2);
        %
        %       %Set the world poses
        %       sphere.Pose = trvec2tform([0.5, 0, 0]);
        %       box.Pose = trvec2tform([0, 0, 0.5]);
        %       show(box);
        %       show(sphere);
        %       hold off;
        %
        %       %Check for robot's collision with the world objects
        %       isColliding = checkCollision(robot, robot.homeConfiguration(), {box, sphere});
        %
        %   See also checkCollision, collisionBox, collisionSphere,
        %    collisionMesh

        %Parse for the optional inputs in varargin

        % check the number of input arguments
        % obj, config, world, 'Exhaustive', 'on'['off'], 'IgnoreSelfCollision', 'on'['off']
            narginchk(2, 7);

            [worldObjects, isExhaustiveParamValue, ignoreSelfCollisionParamValue] = ...
                robotics.manip.internal.parseCheckCollisionInputs(varargin{:});

            %Validate the parsed inputs
            if(~obj.validateWorldObjects(worldObjects))
                robotics.manip.internal.error('rigidbodytree:InvalidWorldObjects');
            end
            isExhaustiveParamValue = validatestring(...
                isExhaustiveParamValue, {'on', 'off'}, 'checkCollision');
            ignoreSelfCollisionParamValue = validatestring(...
                ignoreSelfCollisionParamValue, {'on', 'off'}, 'checkCollision');

            ignoreSelfCollision =...
                strcmpi(ignoreSelfCollisionParamValue, 'on');
            isExhaustive = strcmpi(isExhaustiveParamValue, 'on');
            if strcmp(obj.TreeInternal.DataFormatInternal,'struct')
                robotics.manip.internal.error('rigidbodytree:CheckCollisionUsesVectorsOnly');
            end
            obj.TreeInternal.validateConfiguration(config);
            [isColliding, separationDist, witnessPts] = ...
                obj.TreeInternal.checkCollision(config,...
                                                worldObjects,...
                                                ignoreSelfCollision, ...
                                                isExhaustive);
        end

        function T = getTransform(obj, varargin)
        %getTransform Get the transform between two body frames
        %   T1 = getTransform(ROBOT, Q, BODYNAME1) computes a
        %   transform T1 that converts points originally expressed in
        %   BODYNAME1 frame to be expressed in the robot's base frame
        %   under configuration Q.
        %
        %   T2 = getTransform(ROBOT, Q, BODYNAME1, BODYNAME2) computes
        %   a transform T2 that converts points originally expressed in
        %   BODYNAME1 frame to be expressed in BODYNAME2 frame
        %   under configuration Q.
        %
        %   Example:
        %       % Load predefined robot models
        %       load exampleRobots
        %
        %       % Get the transform that takes points in body 'L6' frame
        %       % and express them in the base coordinates for PUMA robot
        %       % under home configuration
        %       T = getTransform(puma1, puma1.homeConfiguration, 'L6');
        %
        %       % Get the transform that takes points in body 'L2' frame
        %       % and express them in body 'L5' frame for PUMA robot
        %       % under a user-defined configuration
        %       Q = puma1.homeConfiguration;
        %       q = [ 0 0 pi/2 0 0 0];
        %       Q = arrayfun(@(x,y) setfield(x, 'JointPosition', y), Q, q);
        %       T = getTransform(puma1, Q, 'L2', 'L5');

            T = obj.TreeInternal.getTransform(varargin{:});
        end

        function Jac = geometricJacobian(obj, Q, endeffectorname)
        %geometricJacobian Compute the geometric Jacobian
        %   JAC = geometricJacobian(ROBOT, Q, ENDEFFECTORNAME) computes
        %   the geometric Jacobian for the body ENDEFFECTORNAME in ROBOT
        %   under the configuration Q. The Jacobian matrix JAC is of size
        %   6xN, where N is the number of degrees of freedom. The
        %   Jacobian maps joint-space velocity to the Cartesian space
        %   end-effector velocity relative to the base coordinate frame.
        %
        %   Example:
        %       % Load predefined robot models
        %       load exampleRobots
        %
        %       % Get the Jacobian for right_wrist body in Baxter robot
        %       % under a random configuration
        %       jac = geometricJacobian(baxter,...
        %                      baxter.randomConfiguration,'right_wrist');

            Jac = obj.TreeInternal.geometricJacobian(Q, endeffectorname);
        end



        function showdetails(obj)
        %showdetails Display details of the robot
        %   showdetails(ROBOT) displays details of each body in the
        %   robot including the body name, associated joint name and
        %   type, and its parent name and children names.
        %
        %   Example:
        %       % Load predefined robot models
        %       load exampleRobots
        %
        %       % Display details of Baxter robot
        %       showdetails(baxter);
        %
        %   See also show

            obj.TreeInternal.showdetails();
        end

        function ax = show(obj, varargin)
        %SHOW Plot robot body frames
        %   SHOW(ROBOT) plots in MATLAB figure the body frames of
        %   ROBOT under the predefined home configuration.
        %
        %   SHOW(ROBOT, Q) plots in MATLAB figure the body frames of
        %   ROBOT under configuration Q.
        %
        %   AX = SHOW(ROBOT, ___) returns the axes handle under which
        %   the robot is plotted.
        %
        %   SHOW(___, Name, Value) provides additional options specified
        %   by one or more Name, Value pair arguments. Name must appear
        %   inside single quotes (''). You can specify several name-value
        %   pair arguments in any order as Name1, Value1, ..., NameN, ValueN:
        %
        %      'Parent'         - Handle of the axes in which the body
        %                         frames of the robot are to be rendered
        %
        %      'PreservePlot'   - When hold is on and show method is called
        %                         repeatedly, this Boolean parameter
        %                         determines whether the graphic
        %                         objects resulted from previous calls
        %                         of show method are preserved
        %                         (true) or cleared (false).
        %                         When 'PreservePlot' value is true,
        %                         'FastUpdate' value must be false.
        %
        %                         Default: true
        %
        %      'Frames'         - A char vector to turn on and off the
        %                         display of the body frames. The value
        %                         can be either 'on' or 'off'.
        %
        %                         Default: 'on'
        %
        %      'Visuals'        - A char vector to turn on and off the
        %                         display of the body visual meshes.
        %                         The value can be either 'on' or 'off'.
        %
        %                         Default: 'on'
        %
        %      'Collisions'     - A char vector to turn on and off the
        %                         display of the body collision geometries.
        %                         The value can be either 'on' or 'off'.
        %
        %                         Default: 'off'
        %
        %      'Position'       - Position of the robot [x,y,z,yaw].
        %
        %                         Default: [0,0,0,0]
        %
        %      'FastUpdate'     - Specify this parameter to improve the
        %                         performance, when show method is called
        %                         multiple times. The function uses
        %                         HGTransforms for performance improvement.
        %                         Set the value to true for faster
        %                         performance. Otherwise, set the value to
        %                         false. 'FastUpdate' preserves the visual
        %                         state of the robot until its value is
        %                         changed.
        %                         When 'FastUpdate' value is true,
        %                         'PreservePlot' value must be false.
        %
        %                         Default: false
        %
        %   Example:
        %       % Load predefined robot models
        %       load exampleRobots
        %
        %       % Render Baxter robot frames in 3D plot at home
        %       % configuration
        %       show(baxter, baxter.randomConfiguration);
        %
        %       % Open plot browser for a list of body names
        %       plotbrowser
        %
        %       % Plot Puma in the same plot as Baxter
        %       hold on
        %       show(puma1, puma1.homeConfiguration)
        %
        %
        %       % Plot LBR robot into two subplots with different configuration
        %       figure
        %       subplot(1,2,1);
        %       show(lbr, lbr.randomConfiguration);
        %       hold on
        %
        %       subplot(1,2,2);
        %       show(lbr, lbr.randomConfiguration);
        %       hold on
        %
        %       % After executing the following four lines. You should
        %       % still see one robot arm in both subplots.
        %       subplot(1,2,1)
        %       show(lbr, lbr.randomConfiguration, 'PreservePlot', false);
        %
        %       subplot(1,2,2)
        %       show(lbr, lbr.randomConfiguration, 'PreservePlot', false);
        %
        %
        %       % Playback a trajectory for a robot in two subplots
        %       % Make a copy of lbr robot
        %       lbrCpy = copy(lbr);
        %
        %       figure
        %
        %       % Start from home configuration
        %       Q = lbr.homeConfiguration;
        %       i = 1;
        %       for i = 1:50
        %           % Increment joint_a2 and joint_a4
        %           Q(2).JointPosition = Q(2).JointPosition + 0.02;
        %           Q(4).JointPosition = Q(4).JointPosition - 0.02;
        %
        %           % On the left subplot, preserve all previous
        %           % drawings, on the right subplot, only keep the
        %           % most recent drawing. Note the 'Parent' parameter
        %           % selects in which axis the robot is drawn
        %           show(lbr, Q, 'PreservePlot', false, 'Parent', subplot(1,2,1));
        %           show(lbrCpy, Q, 'Parent', subplot(1,2,2));
        %           hold on
        %           drawnow
        %       end
        %
        %       figure;
        %       robot = loadrobot("rethinkBaxter");
        %       show(robot, 'Collisions', 'on', 'Visuals', 'off');
        %
        %       %Animate robot configurations using FastUpdate
        %       figure
        %       robot = loadrobot("kinovaGen3","DataFormat","column");
        %       robotConfigs = trapveltraj([randomConfiguration(robot) randomConfiguration(robot)],100);
        %       for i = 1:100
        %           robot.show(robotConfigs(:,i),'PreservePlot',false,'FastUpdate',true);
        %           drawnow;
        %       end
        %
        %   See also showdetails

            ax = obj.TreeInternal.show(varargin{:});
        end

        function H = massMatrix(obj, varargin)
        %massMatrix Compute the mass matrix for given configuration
        %   H = massMatrix(ROBOT) returns the joint-space mass
        %   matrix, H, of ROBOT for ROBOT's home configuration.
        %
        %   H = massMatrix(ROBOT, Q) returns the joint-space mass
        %   matrix, H, of ROBOT for the given configuration Q.
        %
        %   Joint configuration Q must be specified as a pNum-by-1 or
        %   an 1-by-pNum vector, depending on the DataFormat property
        %   of ROBOT, where pNum is the position number of ROBOT.
        %
        %   The returned mass matrix H is a positive-definite symmetric
        %   matrix with size vNum-by-vNum, where vNum is the velocity
        %   number of ROBOT (degrees of freedom).
        %
        %   Examples:
        %       % Load example robot
        %       load exampleRobots.mat
        %
        %       % Set lbr robot dynamics input data format to 'column'
        %       lbr.DataFormat = 'column';
        %
        %       % Generate a random configuration for lbr
        %       q = lbr.randomConfiguration
        %
        %       % Get the mass matrix at configuration q
        %       H = massMatrix(lbr, q);

            narginchk(1,2);
            q = validateDynamicsFunctionInputs(obj.TreeInternal, false, varargin{:});
            H = robotics.manip.internal.RigidBodyTreeDynamics.massMatrix(obj.TreeInternal, q);
        end

        function tau = inverseDynamics(obj, varargin)
        %inverseDynamics Compute required joint torques for desired motion.
        %   TAU = inverseDynamics(ROBOT) computes joint torques TAU
        %   required for ROBOT to statically hold its home
        %   configuration with no external forces applied.
        %
        %   TAU = inverseDynamics(ROBOT, Q) computes the required joint
        %   torques for ROBOT to statically hold the given
        %   configuration Q with no external forces applied.
        %
        %   TAU = inverseDynamics(ROBOT, Q, QDOT) computes the joint
        %   torques required for ROBOT given the joint configuration Q
        %   and joint velocities QDOT while assuming zero joint accelerations
        %   and no external forces. (Set Q = [] if the desired joint
        %   configuration is home configuration.)
        %
        %   TAU = inverseDynamics(ROBOT, Q, QDOT, QDDOT) computes the
        %   joint torques required for ROBOT given the joint
        %   configuration Q, joint velocities QDOT and joint accelerations
        %   QDDOT while assuming no external forces are applied. (Set
        %   QDOT = [] to indicate zero joint velocities.)
        %
        %   TAU = inverseDynamics(ROBOT, Q, QDOT, QDDOT, FEXT) computes
        %   the joint torques required for ROBOT given the joint
        %   configuration Q, joint velocities QDOT, joint accelerations
        %   QDDOT and the external forces FEXT. (Set QDDOT = [] to
        %   indicate zero joint accelerations.)
        %
        %   If ROBOT's DataFormat property is set to 'column', the
        %   input variables must be formatted as
        %   - Joint configuration, Q - pNum-by-1 vector
        %   - Joint velocities, QDOT - vNum-by-1 vector
        %   - Joint accelerations, QDDOT - vNum-by-1 vector
        %   - External forces, FEXT - 6-by-NB matrix
        %
        %   where:
        %   - pNum is the position number of ROBOT
        %   - vNum is the velocity number of ROBOT (degrees of freedom)
        %   - NB is the number of bodies in ROBOT
        %   - Each column of FEXT represents a wrench
        %     -- top 3 elements: moment
        %     -- bottom 3 elements: linear force
        %
        %   If the DataFormat property of ROBOT is set to 'row', then
        %   all the vectors/matrices above need to be transposed.
        %
        %   The returned joint torques TAU is either a vNum-by-1 or an
        %   1-by-vNum vector, depending on the DataFormat property.
        %
        %   Examples:
        %       % Load example robot
        %       load exampleRobots.mat
        %
        %       % Set lbr robot dynamics input data format to 'column'
        %       lbr.DataFormat = 'column';
        %
        %       % Generate a random configuration for lbr
        %       q = lbr.randomConfiguration
        %
        %       % Compute the required joint torques for lbr to
        %       % statically hold that configuration
        %       tau = inverseDynamics(lbr, q);
        %
        %   See also forwardDynamics, externalForce

            narginchk(1,5);
            [q, qdot, qddot, fext] = validateDynamicsFunctionInputs(obj.TreeInternal, true, varargin{:});
            tau = robotics.manip.internal.RigidBodyTreeDynamics.inverseDynamics(obj.TreeInternal, q, qdot, qddot, fext);
            tau = resultPostProcess(obj.TreeInternal, tau);
        end


        function qddot = forwardDynamics(obj, varargin)
        %forwardDynamics Compute resultant joint acceleration given joint torques and states
        %   QDDOT = forwardDynamics(ROBOT) computes the resultant joint
        %   accelerations due to gravity when ROBOT is at its home
        %   configuration, with zero joint velocities and no external
        %   forces.
        %
        %   QDDOT = forwardDynamics(ROBOT, Q) computes the resultant
        %   joint accelerations due to gravity when ROBOT is at joint
        %   configuration Q, with zero joint velocities and no external
        %   forces.
        %
        %   QDDOT = forwardDynamics(ROBOT, Q, QDOT) computes the joint
        %   accelerations resulted from gravity and joint velocities
        %   QDOT when ROBOT is at Joint configuration Q. (Set Q = [] if
        %   the joint configuration is home configuration.)
        %
        %   QDDOT = forwardDynamics(ROBOT, Q, QDOT, TAU) computes the
        %   joint accelerations due to gravity, joint velocities QDOT
        %   and joint torques TAU when ROBOT is at joint configuration Q.
        %   (Set QDOT = [] to indicate zero joint velocities.)
        %
        %   QDDOT = forwardDynamics(ROBOT, Q, QDOT, TAU, FEXT) computes
        %   the joint accelerations due to gravity, joint velocities
        %   QDOT, torques applied to the joints TAU, and external forces
        %   applied to each body FEXT, when ROBOT is at configuration Q.
        %   (Set TAU = [] to indicate zero joint torques.)
        %
        %   If ROBOT's DataFormat property is set to 'column', the
        %   input variables must be formatted as
        %   - Joint configuration, Q - pNum-by-1 vector
        %   - Joint velocities, QDOT - vNum-by-1 vector
        %   - Joint torques, TAU - vNum-by-1 vector
        %   - External forces, FEXT - 6-by-NB matrix
        %
        %   where:
        %   - pNum is the position number of ROBOT
        %   - vNum is the velocity number of ROBOT (degrees of freedom)
        %   - NB is the number of bodies in ROBOT
        %   - Each column of FEXT represents a wrench
        %     -- top 3 elements: moment
        %     -- bottom 3 elements: linear force
        %
        %   If the DataFormat property of ROBOT is set to 'row', then
        %   all the vectors/matrices above need to be transposed.
        %
        %   The returned joint accelerations QDDOT is either a vNum-by-1
        %   or an 1-by-vNum vector, depending on the DataFormat property.
        %
        %   Examples:
        %       % Load example robot
        %       load exampleRobots.mat
        %
        %       % Set lbr robot dynamics input data format to 'row'
        %       lbr.DataFormat = 'row';
        %
        %       % Set the gravity
        %       lbr.Gravity = [0 0 -9.81];
        %
        %       % Get the home configuration for lbr
        %       q = lbr.homeConfiguration
        %
        %       % The end-effector (body 'tool0') experiences a wrench.
        %       % Use the 2 lines below to generate the corresponding
        %       % external force matrix, fext. Note that if the external
        %       % wrench is specified in the body 'tool0' frame, the
        %       % joint configuration, q, must be specified as the fourth
        %       % input argument for externalForce method.
        %       wrench = [0 0 0.5 0 0 0.3];
        %       fext = externalForce(lbr, 'tool0', wrench, q)
        %
        %       % Compute the resultant joint acceleration due to gravity
        %       % with the external force applied to the end-effector when
        %       % lbr is at its home configuration.
        %       qddot = forwardDynamics(lbr, q, [], [], fext);
        %
        %   See also inverseDynamics, externalForce

            narginchk(1,5);
            [q, qdot, tau, fext] = validateDynamicsFunctionInputs(obj.TreeInternal, false, varargin{:});
            qddot = robotics.manip.internal.RigidBodyTreeDynamics.forwardDynamicsCRB(obj.TreeInternal, q, qdot, tau, fext);
            qddot = resultPostProcess(obj.TreeInternal, qddot);
        end

        function [com, comJac] = centerOfMass(obj, varargin)
        %centerOfMass Compute the center of mass position and Jacobian
        %   COM = centerOfMass(ROBOT) computes the center of mass
        %   position of ROBOT at its home configuration relative to the
        %   base frame.
        %
        %   COM = centerOfMass(ROBOT, Q) computes the center of mass
        %   position of ROBOT at the specified joint configuration Q
        %   relative to the base frame.
        %
        %   [COM, COMJAC] = centerOfMass(ROBOT, ...) also returns the
        %   center of mass Jacobian COMJAC as the second output argument.
        %   COMJAC relates center of mass velocity to joint
        %   velocities.
        %
        %   COM is a 3-by-1 vector, and COMJAC is a 3-by-vNum matrix,
        %   where vNum is the velocity number of ROBOT (i.e. the
        %   degrees of freedom).
        %
        %   Examples:
        %       % Load example robot
        %       load exampleRobots.mat
        %
        %       % Set lbr robot dynamics input data format to 'row'
        %       lbr.DataFormat = 'row';
        %
        %       % Compute the center of mass position and Jacobian at
        %       % home configuration
        %       [com, comJac] = centerOfMass(lbr);

            narginchk(1,2);
            q = validateDynamicsFunctionInputs(obj.TreeInternal, false, varargin{:});
            [com, totalmass, cmm] = robotics.manip.internal.RigidBodyTreeDynamics.centroidalMomentumMatrix(obj.TreeInternal, q);
            comJac = cmm(4:6,:)/totalmass;
            com = resultPostProcess(obj.TreeInternal, com);
        end

        function c = velocityProduct(obj, varargin)
        %velocityProduct Compute the joint torques that cancel velocity dependent term
        %   C = velocityProduct(ROBOT, Q, QDOT) computes joint torques C
        %   required for ROBOT to cancel the forces induced by joint
        %   velocities QDOT at the joint configuration Q. (Set Q = []
        %   if the joint configuration is home configuration.)
        %
        %   C = velocityProduct(ROBOT) and
        %   C = velocityProduct(ROBOT, Q) are also valid signatures,
        %   however they will always return zero vectors since zero
        %   joint velocity is assumed by default.
        %
        %   NOTE that if ROBOT's DataFormat property is set to
        %   'column', the joint configuration Q must be specified as a
        %   pNum-by-1 vector, where pNum is the position number of the
        %   robot; and the joint velocities QDOT must be specified as a
        %   vNum-by-1 vector, where vNum is the velocity number (degrees
        %   of freedom for ROBOT).
        %
        %   Also NOTE that the output of this function is not affected
        %   by the Gravity property of ROBOT.
        %
        %   Examples:
        %       % Load example robot
        %       load exampleRobots.mat
        %
        %       % Set lbr robot dynamics input data format to 'row'
        %       lbr.DataFormat = 'row';
        %
        %       % Set the desired joint velocity vector (row)
        %       qdot = [0 0 0.2 0.3 0 0.1 0];
        %
        %       % Compute the joint torques induced by velocity qdot at
        %       % ROBOT's home configuration, notice the negative sign
        %       tau = - velocityProduct(lbr, [], qdot);
        %
        %   See also inverseDynamics, gravityTorque

            narginchk(1,3);
            [q, qdot, qddot, fext] = validateDynamicsFunctionInputs(obj.TreeInternal, true, varargin{:});

            c = robotics.manip.internal.RigidBodyTreeDynamics.inverseDynamics(obj.TreeInternal, q, qdot, qddot, fext) ...
                - robotics.manip.internal.RigidBodyTreeDynamics.inverseDynamics(obj.TreeInternal, q, 0*qdot, qddot, fext);
            c = resultPostProcess(obj.TreeInternal, c);
        end

        function gTorq = gravityTorque(obj, varargin)
        %gravityTorque Compute required joint torques to compensate gravity
        %   GTORQ = gravityTorque(ROBOT) computes GTORQ, the joint torques
        %   required to hold ROBOT at its home configuration.
        %
        %   GTORQ = gravityTorque(ROBOT, Q) computes the joint torques
        %   required to hold ROBOT at configuration Q.
        %
        %   Joint configuration Q must be specified as a pNum-by-1 or
        %   an 1-by-pNum vector, depending on the DataFormat property
        %   of ROBOT.
        %
        %   Examples:
        %       % Load example robot
        %       load exampleRobots.mat
        %
        %       % Set lbr robot dynamics input data format to 'row'
        %       lbr.DataFormat = 'row';
        %
        %       % Get a random configuration for lbr
        %       q = lbr.randomConfiguration
        %
        %       % Compute the gravity compensation torque
        %       gtau = gravityTorque(lbr, q);
        %
        %   See also inverseDynamics, velocityProduct

            narginchk(1,2);
            [q, qdot, qddot, fext] = validateDynamicsFunctionInputs(obj.TreeInternal, true, varargin{:});
            gTorq = robotics.manip.internal.RigidBodyTreeDynamics.inverseDynamics(obj.TreeInternal, q, qdot, qddot, fext);
            gTorq = resultPostProcess(obj.TreeInternal, gTorq);
        end

        function fext = externalForce(obj, bodyName, wrench, varargin)
        %externalForce Compose external force matrix relative to base
        %   FEXT = externalForce(ROBOT, BODYNAME, WRENCH) composes the
        %   external force matrix, FEXT, that applies an external WRENCH
        %   to body BODYNAME. WRENCH is assumed to be relative to
        %   the base frame.
        %
        %   FEXT = externalForce(ROBOT, BODYNAME, WRENCH, Q) is similar
        %   to the signature above, but WRENCH is assumed relative to
        %   the BODYNAME frame. The fourth input argument, joint
        %   configuration Q, is used to convert WRENCH to the base
        %   frame as required by FEXT.
        %
        %   Depending on the DataFormat property of ROBOT, FEXT is
        %   either a 6-by-vNum ('column') or vNum-by-6 ('row') matrix,
        %   where vNum is the velocity number of ROBOT (degrees of
        %   freedom). FEXT contains WRENCH in the correct column or row
        %   (relative to base frame) that corresponds to the given
        %   BODYNAME. The first 3 elements in WRENCH are assumed to be
        %   moment, the last 3 are assumed linear force.
        %
        %   Examples:
        %       % Load example robot
        %       load exampleRobots.mat
        %
        %       % Set lbr robot dynamics input data format to 'row'
        %       lbr.DataFormat = 'column';
        %
        %       % Get the home configuration for lbr
        %       q = lbr.homeConfiguration
        %
        %       % Set external force on link_4, wrench expressed in base frame
        %       fext1 = externalForce(lbr, 'link_1', [0 0 0.0 0.1 0 0]);
        %
        %       % Set external force on tool0, the end-effector, wrench
        %       % is expressed in tool0 frame
        %       fext2 = externalForce(lbr, 'tool0', [0 0 0.0 0.1 0 0], q);
        %
        %       % Compute the joint torques required to balance the
        %       % external forces
        %       tau = inverseDynamics(lbr, q, [], [], fext1+fext2);
        %
        %   See also inverseDynamics, forwardDynamics

            narginchk(3,4);
            fext = externalForce(obj.TreeInternal, bodyName, wrench, varargin{:});
        end


        function writeAsFunction(obj, filename)
        %writeAsFunction Create rigidBodyTree generating function
        %   writeAsFunction(ROBOT, FILENAME) creates a function file that
        %   constructs the rigidBodyTree object. The generated function is
        %   Code Generation supported.
        %
        %   Examples:
        %       % Load a robot
        %       rbt = importrobot("iiwa14.urdf");
        %
        %       % Generate a function with the robot name that constructs
        %       % the rigidBodyTree
        %       rbt.writeAsFunction("iiwa14.m");
        %
        %   See also loadrobot, importrobot

            validateattributes(filename,{'char','string'},{'nonempty','scalartext'},'writeAsFunction','filename');
            filename=char(filename);

            [path, name, ext] = fileparts(filename);
            %Only supports file generation in current directory
            if ~isempty(path)
                fileInputArgName = 'filename';
                coder.internal.error('robotics:robotmanip:rigidbodytree:OnlyLocalFilesSupported', fileInputArgName);
            end
            %Verify that the function name is valid
            if ~isvarname(name)
                coder.internal.error('robotics:robotmanip:rigidbodytree:InvalidFunctionName');
            end

            if ~isempty(ext) && ~strcmp(ext, '.m')
                coder.internal.error('robotics:robotmanip:rigidbodytree:IncorrectFileExt');
            end
            obj.TreeInternal.saveAsMATLABFunction(filename);
        end
    end


    % Property access methods
    methods

        function nb = get.NumBodies(obj)
            nb = obj.TreeInternal.NumBodies;
        end

        function bnames = get.BodyNames(obj)
        %get.BodyNames
            bnames = obj.TreeInternal.BodyNames;
        end

        function basename = get.BaseName(obj)
        %get.BaseName
            basename = obj.TreeInternal.Base.Name;
        end

        function set.BaseName(obj, baseName)
        %set.BaseName
            obj.TreeInternal.BaseName = baseName;
        end

        function bodies = get.Bodies(obj)
        %get.Bodies
            bodies = repmat({obj.Base}, 1, obj.NumBodies);
            j = 1;
            for i = coder.unroll(1:obj.TreeInternal.MaxNumBodies)
                if i <= obj.NumBodies
                    bodies{j} = rigidBody('-', 'BodyInternal', obj.TreeInternal.Bodies{i}, ...
                                          'TreeInternal', obj.TreeInternal);
                    j = j + 1;
                end
            end
        end

        function base = get.Base(obj)
        %get.Base
            base = rigidBody('-', 'BodyInternal', obj.TreeInternal.Base, ...
                             'TreeInternal', obj.TreeInternal);
        end

        function set.Gravity(obj, g)
        %set.Gravity
            obj.TreeInternal.Gravity = g;
        end

        function g = get.Gravity(obj)
        %get.Gravity
            g = obj.TreeInternal.Gravity;
        end

        function maxNB = get.MaxNumBodies(obj)
        %get.MaxNumBodies
            maxNB = obj.TreeInternal.MaxNumBodies;
        end


        % below for internal use only
        function tag = get.ShowTag(obj)
        %get.ShowTag
            tag = obj.TreeInternal.ShowTag;
        end

        function set.ShowTag(obj, tag)
        %set.ShowTag
            obj.TreeInternal.ShowTag = tag;
        end

        function maxreach = get.EstimatedMaxReach(obj)
        %get.EstimatedMaxReach
            maxreach = obj.TreeInternal.EstimatedMaxReach;
        end

        function set.EstimatedMaxReach(obj, maxreach)
        %set.ShowTag
            obj.TreeInternal.EstimatedMaxReach = maxreach;
        end

        function numNonFixed = get.NumNonFixedBodies(obj)
        %get.NumNonFixedBodies
            numNonFixed = obj.TreeInternal.NumNonFixedBodies;
        end

        function fmt = get.DataFormat(obj)
        %get.DataFormat
            fmt = obj.TreeInternal.DataFormat;
        end

        function set.DataFormat(obj, fmt)
        %set.DataFormat
            obj.TreeInternal.DataFormat = fmt;
        end

    end % setter/getter methods

    methods(Static)
        function obj = loadobj(objFromMAT)
        %loadobj Provide backward-compatibility support when a 16b
        %   RigidBodyTree object is loaded from a MAT file. This function
        %   is to be invoked by the "load" command.

            if isstruct(objFromMAT) % if this is a 16b object
                newObj = rigidBodyTree( );

                newObj.BaseName = objFromMAT.Base.Name;

                % reconstruct the parent list
                for i = 1:objFromMAT.NumBodies
                    c = objFromMAT.Bodies{i}.BodyInternal.ChildrenIndices;
                    for j = 1:length(c)
                        objFromMAT.Bodies{c(j)}.BodyInternal.ParentIndex = i;
                    end
                end

                % rebuild the robot from 16b metadata
                for i = 1:objFromMAT.NumBodies
                    pid = objFromMAT.Bodies{i}.BodyInternal.ParentIndex;
                    if pid == -1 % parent is base
                        newObj.addBody( objFromMAT.Bodies{i}, objFromMAT.Base.Name);
                    else
                        newObj.addBody( objFromMAT.Bodies{i}, objFromMAT.Bodies{pid}.Name);
                    end
                end

                newObj.TreeInternal.VisualizationInfo.ShowTag = objFromMAT.ShowTag;
                obj = newObj;
            else
                obj = objFromMAT;
            end
            %Re-initialize VisualizationInfo to ensure all RBTs have
            %unique showtag
            obj.TreeInternal.VisualizationInfo = robotics.manip.internal.VisualizationInfo();
        end
    end

    methods(Access=private)
        function isValid = validateWorldObjects(~, worldObjects)
        %validateWorldObjects Validation of input worldObjects for checkCollision
        %   The worldObjects are passed as a cell-array of collision
        %   objects. The function returns true if the worldObjects are a
        %   valid input to checkCollision

            isValid = true;
            validateattributes(worldObjects, {'cell'}, {}, ...
                               'validateWorldObjects',...
                               'worldObjects');
            for i = 1:length(worldObjects)
                if(~isa(worldObjects{i}, 'robotics.core.internal.CollisionGeometryBase'))
                    isValid = false;
                    break;
                end
            end
        end
    end

    methods (Static, Access = ?robotics.manip.internal.InternalAccess)
        function obj = fromTreeInternal(treeInternal)
        %fromTreeInternal Create object from object of internal class
            obj = rigidBodyTree('MaxNumBodies', ...
                                treeInternal.MaxNumBodies, ...
                                'DataFormat', ...
                                treeInternal.DataFormat);
            obj.TreeInternal = treeInternal;
        end
    end

end

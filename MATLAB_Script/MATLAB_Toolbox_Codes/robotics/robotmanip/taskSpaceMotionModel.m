classdef taskSpaceMotionModel < robotics.manip.internal.InternalAccess
    %TASKSPACEMOTIONMODEL Model rigid body tree motion given task-space reference inputs
    %   The taskSpaceMotionModel object models the closed-loop task-space
    %   motion of a manipulator, specified as a rigid body tree object. The
    %   motion model behavior is defined by the MOTIONTYPE property.
    %
    %   MOTIONMODEL = taskSpaceMotionModel('RigidBodyTree', tree) creates
    %   a motion control model for the specified RigidBodyTree object given
    %   by TREE.
    %
    %   MOTIONMODEL = taskSpaceMotionModel(___, 'PropertyName', PropertyValue, ..) 
    %   sets additional properties specified as name-value pairs. You can
    %   specify multiple properties in any order.
    %
    %   TASKSPACEMOTIONMODEL Properties:
    %      RigidBodyTree     - Rigid body tree robot model
    %      EndEffectorName   - Name of the end effector body
    %      Kp                - Proportional gain for PD control
    %      Kd                - Derivative gain for PD control
    %      JointDamping      - Damping constant for each joint
    %      MotionType        - Type of motion computed by the motion model. Options are:
    %         - "PDControl": Uses proportional-derivative control based on
    %         the specified KP and KD properties.
    %
    %   TASKSPACEMOTIONMODEL Methods:
    %      derivative             - Compute the time derivative of the vehicle state
    %      copy                   - Create a copy of the object 
    %
    %   Example:
    %      % Load robot
    %      robot = loadrobot("kinovaGen3", "DataFormat", "column", "Gravity", [0 0 -9.81]);
    %
    %      % Set up simulation, starting from home position at zero velocity
    %      tspan = 0:0.02:1;
    %      initialState = [homeConfiguration(robot); zeros(7,1)];
    %
    %      % Define a reference state with a target position and zero velocity
    %      refPose = trvec2tform([0.6 -.1 0.5]);
    %      refVel = zeros(6,1);
    %
    %      % Model the behavior with as a system under PD control
    %      motionModel = taskSpaceMotionModel("RigidBodyTree", robot, "EndEffectorName", "EndEffector_Link");
    %      
    %      % Simulate the behavior over 1 second, using a stiff solver to 
    %      % more efficiently capture the robot dynamics
    %      [t,robotState] = ode15s(@(t,state)derivative(motionModel, state, refPose, refVel), tspan, initialState);
    %
    %      % Plot the robot's initial position and mark the target with an X
    %      figure
    %      show(robot, initialState(1:7));
    %      hold all
    %      plot3(refPose(1,4), refPose(2,4), refPose(3,4), "x", "MarkerSize", 20);
    %
    %      % Observe response by plotting the robot in a 5 Hz loop
    %      r = rateControl(5);    %
    %      for i = 1:size(robotState,1)
    %           show(robot, robotState(i, 1:7)', "PreservePlot", false);
    %           waitfor(r);
    %      end
    %
    %   See also jointSpaceMotionModel
    
    %   Copyright 2019-2020 The MathWorks, Inc.
    
    %#codegen
    
    properties (Dependent)
        %RigidBodyTree - Rigid body tree robot model
        %   Rigid body tree robot model, specified as a
        %   rigidBodyTree object that defines the inertial and
        %   kinematic properties of the manipulator.
        RigidBodyTree
        
        %EndEffectorName - Name of the end effector body
        %   This property defines the body that will be used as the end
        %   effector, and for which the task space motion is defined. The
        %   property must correspond to a body name in the associate
        %   rigidBodyTree object. If the rigid body tree is
        %   updated without also updating the end effector, the body with
        %   the highest index is assigned by default.
        EndEffectorName
        
        %JointDamping - Joint damping constant
        %   Damping constant for each joint, specified as a scalar or
        %   N-element vector where N in the number of non-fixed joints in
        %   the associated rigidBodyTree object.
        JointDamping
    end
    
    properties
        
        %MotionType - Type of motion computed by the motion model
        %   Type of motion, specified as a string scalar or character
        %   vector that defines the closed-loop joint-space behavior that
        %   the object models. Options are:
        %      - "PDControl": Uses proportional-derivative control based on
        %         the specified KP and KD properties. Under PD Control, the
        %         following control law is used:
        %            tau = J'*(Kp*[eOri; ePos] + Kd*(refVel - J*qdot) - JointDamping*qdot + G(q)
        %         where J is the geometric Jacobian, eOri and ePos are
        %         the orientation and position errors for the X, Y,
        %         and Z axes, G(q) is the gravity torque, and Kp and Kd are
        %         the provided proportional and derivative gain matrices.
        %         Note that q and qdot are the joint configuration and
        %         velocity, obtained from the state vector [q; qdot].
        MotionType
        
        %Kp - Proportional gain for PD Control
        %   Proportional gain for PD control, specified as a 6x6 matrix.
        %   Under PD Control, the following control law is used:
        %      tau = J'*(Kp*[eOri; ePos] + Kd*(refVel - J*qdot) - JointDamping*qdot + G(q)
        %   where J is the geometric Jacobian, eOri and ePos are the
        %   orientation and position errors for the X, Y, and Z axes, G(q)
        %   is the gravity torque, and Kp and Kd are the provided
        %   proportional and derivative gain matrices. Note that q and qdot
        %   are the joint configuration and velocity, obtained from the
        %   state vector [q; qdot].
        Kp
        
        %Kd - Derivative gain for PD Control
        %   Derivative gain for PD control, specified as a 6x6 matrix.
        %   Under PD Control, the following control law is used:
        %      tau = J'*(Kp*[eOri; ePos] + Kd*(refVel - J*qdot) - JointDamping*qdot + G(q)
        %   where J is the geometric Jacobian, eOri and ePos are the
        %   orientation and position errors for the X, Y, and Z axes, G(q)
        %   is the gravity torque, and Kp and Kd are the provided
        %   proportional and derivative gain matrices. Note that q and qdot
        %   are the joint configuration and velocity, obtained from the
        %   state vector [q; qdot].
        Kd
    end
    
    properties (Constant, Access=?robotics.manip.internal.InternalAccess)
        MotionTypeSet = {'PDControl'}
        
        MotionTypeDefault = 'PDControl'
        
        KpDefault = 500*eye(6);
        
        KdDefault = 100*eye(6);
        
        EndEffectorNameDefault = 'tool'
        
        JointDampingDefault = 1;
    end
    
    properties (SetAccess = private, Hidden)
        %NumJoints - the number of non-fixed joints in the associated rigidBodyTree object
        NumJoints
    end
    
    properties (Access = ?robotics.manip.internal.InternalAccess, Hidden)
        %RigidBodyTreeInternal - A robotics.manip.internal.RigidBodyTree instantiated from TreeStruct
        RigidBodyTreeInternal
        
        %RigidBodyTreeFormat - The DataFormat property of the user-facing rigid body tree
        RigidBodyTreeFormat
        
        %EENameInternal
        EENameInternal
        
        %JointDampingInternal
        JointDampingInternal
    end
    
    methods
        function obj = taskSpaceMotionModel(varargin)
            %TASKSPACEMOTIONMODEL Constructor
            
            narginchk(0,12);
            
            % Convert strings to chars case by case for codegen support
            charInputs = cell(1,nargin);
            [charInputs{:}] = convertStringsToChars(varargin{:});
            
            % Parse inputs
            names = {'RigidBodyTree', 'MotionType', 'EndEffectorName', 'Kp', 'Kd', 'JointDamping'};
            defaults = {twoJointRigidBodyTree, obj.MotionTypeDefault, ...
                obj.EndEffectorNameDefault, obj.KpDefault, obj.KdDefault, ...
                obj.JointDampingDefault};
            parser = robotics.core.internal.NameValueParser(names, defaults);
            parse(parser, charInputs{:});
            
            % Set parameters
            obj.setRigidBodyTreeForConstruction(parameterValue(parser, names{1}));
            obj.MotionType = parameterValue(parser, names{2});
            obj.EndEffectorName = parameterValue(parser, names{3});
            obj.Kp = parameterValue(parser, names{4});
            obj.Kd = parameterValue(parser, names{5});
            obj.JointDamping = parameterValue(parser, names{6});
        end
        
        function stateDot = derivative(obj, state, refPose, refVel, varargin)
            %DERIVATIVE Compute time derivative of model states
            %   STATEDOT = derivative(TASKMOTION, STATE, REFPOSE, REFVEL, CMDS)
            %   computes the state derivative of the motion model based on
            %   the current state and reference commands.
            %
            %   STATEDOT = derivative(TASKMOTION, STATE, REFPOSE, REFVEL, FEXT)
            %   computes the state derivative based on the current state,
            %   reference commands, and any external forces on the
            %   manipulator.
            %
            %   Input Arguments:
            %
            %      TASKMOTION   - jointSpaceMotionModel object.
            %
            %      STATE        - Joint positions and velocities, 
            %                     specified as a 2xN element vector, 
            %                     [Q; QDOT].
            %
            %      REFPOSE      - The reference position of the end
            %                     effector in the task space in m,
            %                     specified as a 4x4 homogeneous
            %                     transformation matrix.
            %
            %      REFVEL       - The reference velocities of the end
            %                     effector in the task space, specified as
            %                     a 6-element vector [OMEGA V]. OMEGA
            %                     represents the three angular velocities
            %                     about x, y, and z, in rad/s, and V
            %                     represents the 3 linear velocities along
            %                     x, y, and z, in m/s.
            %
            %      FEXT          - External forces, specified as a 6xM 
            %                      matrix, where M is the number of bodies
            %                      in the associated rigidBodyTree
            %                      object.
            %
            %   Output Arguments:
            %
            %      STATEDOT   - State derivative based on current state and
            %                   specified control commands, returned as a
            %                   2xN matrix, [QDOT; QDDOT] representing the
            %                   joint velocities and accelerations,
            %                   respectively.
            
            % Validate inputs
            narginchk(4,5);
            validateattributes(state, {'double'}, {'nonempty', 'vector', 'numel', 2*obj.NumJoints, 'nonnan', 'real'}, 'derivative', 'state');
            robotics.internal.validation.validateHomogeneousTransform(refPose, 'derivative', 'refPose');
            validateattributes(refPose, {'double'}, {'2d'}, 'derivative', 'refPose');
            validateattributes(refVel, {'double'}, {'nonempty', 'vector', 'numel', 6, 'nonnan', 'real'}, 'derivative', 'refVel');
            
            % Get the robot position and velocity from the state
            q = state(1:obj.NumJoints);
            qDot = state((obj.NumJoints + 1):end);
            
            % Parse the optional arguments
            if nargin > 4
                fExt = varargin{1};
                validateattributes(fExt, {'double'}, {'nonempty', '2d', 'size', [6 obj.RigidBodyTreeInternal.NumBodies], 'finite','real'}, 'derivative', 'fExt');
            else
                fExt = zeros(6, obj.RigidBodyTreeInternal.NumBodies);
            end
            
            % Initialize outputs
            stateDot = zeros(2*obj.NumJoints,1);
            
            % For position control, pass all inputs as column vectors
            tau = PDControl(obj, q(:), qDot(:), refPose, refVel(:));
            
            fwdDynamics = robotics.manip.internal.RigidBodyTreeDynamics.forwardDynamicsCRB(obj.RigidBodyTreeInternal, q(:), qDot(:), tau(:), fExt);
            stateDot(:,:) = [qDot(:); fwdDynamics]; 
        end
        
        function newMotionModel = copy(obj)
            %COPY Copy the motion model
            %   NEWMOTIONMODEL = COPY(MOTIONMODEL) returns a deep copy of
            %   MOTIONMODEL. NEWMOTIONMODEL and MOTIONMODEL are two
            %   different taskSpaceMotionModel objects with the same
            %   properties.
            %   
            %   Example:
            %       % Create a task-space motion model
            %       robot = loadrobot('kinovaGen3');
            %       mm = taskSpaceMotionModel("RigidBodyTree", robot, "EndEffectorName", "EndEffector_Link");
            %
            %       % Make a copy
            %       newMotionModel = COPY(mm)
            
            newMotionModel = taskSpaceMotionModel(...
                'RigidBodyTree', copy(obj.RigidBodyTree), ...
                'EndEffectorName', obj.EndEffectorName, ...
                'MotionType', obj.MotionType, ...
                'Kp', obj.Kp, ...
                'Kd', obj.Kd, ...
                'JointDamping', obj.JointDamping ...
                );
        end
    end
    
    %% Get/Set methods
    methods        
        function set.RigidBodyTree(obj, tree)
            %SET.RIGIDBODYTREE Set method for obj.RigidBodyTree
            %   This method relates the user-defined RigidBodyTree method
            %   to the internal representation
            
            oldNumJoints = obj.NumJoints;
            obj.setRigidBodyTreeForConstruction(tree);
            obj.setTreeDependentProperties(oldNumJoints);
        end
        
        function tree = get.RigidBodyTree(obj)
            %GET.RIGIDBODYTREE Get method for obj.RigidBodyTree
            %   This method relates the internal RigidBodyTree
            %   representation to the user-facing RigidBodyTree.
                             
            %Create a RigidBodyTree from the robotics.manip.internal.RigidBodyTree
            treeInternal = copy(obj.RigidBodyTreeInternal, obj.RigidBodyTreeFormat);
            tree = rigidBodyTree(treeInternal);
        end
        
        function set.EndEffectorName(obj, eeName)
            %SET.ENDEFFECTORNAME Set method for obj.EndEffectorName
            %   This method checks that the specified end effector name
            %   actually exists in the associated rigid body tree. If not,
            %   a default value is used and a warning is cast.
            
            validateattributes(eeName, {'string', 'char'}, {'scalartext'}, 'taskSpaceMotionModel', 'EndEffectorName');
            nameMatch = false(1, obj.RigidBodyTreeInternal.NumBodies);
            nameMatch(:) = strcmp(eeName, obj.RigidBodyTreeInternal.BodyNames);
            if any(nameMatch)
                obj.EENameInternal = eeName;
            else
                obj.EENameInternal = obj.RigidBodyTreeInternal.BodyNames{end};
                coder.internal.warning('robotics:robotmanip:motionmodels:UnresolvedEndEffectorName', eeName, obj.EENameInternal);
            end
        end
        
        function eeName = get.EndEffectorName(obj)
            %GET.RIGIDBODYTREE Get method for obj.RigidBodyTree
            %   This method relates the internal RigidBodyTree
            %   representation to the user-facing RigidBodyTree.
                 
            eeName = obj.EENameInternal;
        end
        
        function set.MotionType(obj, inputString)
            %SET.MotionType Setter method for MotionType
            
            obj.MotionType = validatestring(inputString, obj.MotionTypeSet, 'taskSpaceMotionModel', 'MotionType');
        end
        
        function set.Kp(obj, kp)
            %SET.KP Setter method for proportional gain, Kp
            
            validateattributes(kp, {'double'}, {'nonempty', 'square', 'finite', 'real', 'ncols', 6}, 'taskSpaceMotionModel', 'Kp');
            obj.Kp = kp;
        end        
        
        function set.Kd(obj, kd)
            %SET.KD Setter method for derivative gain, Kd
            
            validateattributes(kd, {'double'}, {'nonempty', 'square', 'finite', 'real', 'ncols', 6}, 'taskSpaceMotionModel', 'Kd');
            obj.Kd = kd;
        end
        
        function set.JointDamping(obj, b)
            %SET.JOINTDAMPING Set method for obj.JointDamping
            
            validateattributes(b, {'double'}, {'nonempty', 'vector', 'finite', 'real', 'nonnegative'}, 'taskSpaceMotionModel', 'JointDamping');
            
            % Ensure that the vector is either scalar or has the right
            % number of elements, and convert to column representation
            obj.JointDampingInternal = obj.validateVectorProperty(b, obj.NumJoints, 'JointDamping');
        end
        
        function b = get.JointDamping(obj)
            %GET.JOINTDAMPING Get method for obj.RigidBodyTree
                             
            % Return the value as a row for command line readability
            b = obj.JointDampingInternal(:)';
        end
    end
    
    %% Helper methods
        
    methods (Access = protected)
        function u = PDControl(obj, q, qDot, refPose, refVel)
            %PDCONTROL Compute control input under PD control
            %   This method computes the control law defined by the
            %   task-space PD controller of the rigid body tree, end
            %   effector, and parameters defined in the associated
            %   taskSpaceMotionControl object. The method takes the current
            %   joint configuration Q and joint velocity QDOT states as
            %   inputs, as well as the desired end effector orientation and
            %   position REFPOSE and velocity REFVEL. All vector-valued
            %   inputs are provided as column vectors.
            
            J = geometricJacobian(obj.RigidBodyTreeInternal, q, obj.EndEffectorName);
            currentPose = getTransform(obj.RigidBodyTreeInternal, q, obj.EndEffectorName);            
            
            % Compute rotation error and get euler angles of difference
            oriErrorRotm = refPose(1:3,1:3)*currentPose(1:3,1:3)';
            oriErrorEul = rotm2eul(oriErrorRotm, 'XYZ')';

            % Get difference of position vectors
            posError = tform2trvec(refPose)' - tform2trvec(currentPose)';
            
            % Define position and velocity error vectors
            posError = [oriErrorEul; posError];
            velError = refVel - J*qDot;

            f = (obj.Kp*posError + obj.Kd*velError);

            % Convert end effector forces and torques to joint forces and
            % torques
            u = J'*f;
            
            % Add joint damping to reduce superfluous joint motion
            u = u - obj.JointDampingInternal.*qDot;

            % Add gravity compensation
            u = addGravityCompensation(obj, u, q);
        end
        
        function uWithGrav = addGravityCompensation(obj, u, q)
            %addGravityCompensation Add Gravity compensation

            fExt = zeros(6, obj.RigidBodyTreeInternal.NumBodies);
            gravTorque = robotics.manip.internal.RigidBodyTreeDynamics.inverseDynamics(...
                obj.RigidBodyTreeInternal, q, zeros(size(q)), zeros(size(q)), fExt);
            uWithGrav = u + gravTorque;
        end
        
        function setRigidBodyTreeForConstruction(obj, tree)
            %setRigidBodyTreeForConstruction Set rigid body tree for construction
            %   This method relates the user-defined RigidBodyTree method
            %   to the internal representation. In the set method, some
            %   additional items are also executed. To ensure codegen
            %   compatibility, the additional set-time tasks cannot also
            %   occur in the constructor, so the constructor-only tasks are
            %   split out into this method.
            
            if isa(tree, 'robotics.manip.internal.RigidBodyTree')
                % This is an internal path that is used to ensure the
                % function and related tools are codegen compatible
                obj.RigidBodyTreeInternal = tree.copy('column');
            else
                validateattributes(tree, {'rigidBodyTree'}, {'nonempty'}, 'taskSpaceMotionModel', 'RigidBodyTree');
                newRobotInternal = tree.TreeInternal.copy('column'); 
                obj.RigidBodyTreeInternal = newRobotInternal;
            end
            obj.NumJoints = obj.RigidBodyTreeInternal.VelocityNumber;
            obj.RigidBodyTreeFormat = tree.DataFormat;
        end
        
        function setTreeDependentProperties(obj, oldNumJoints)
            %setTreeDependentProperties Set properties that depend on the associated RigidBodyTree
            
            % Ensure that dependent properties maintain sizes that match
            % the rigid body tree size, unless this set method is called at
            % construction time (at that point the pre-existing properties
            % are undefined, so oldNumJoints = 0 and the warning is
            % intentionally not thrown)
            if ~(oldNumJoints == 0) && (obj.NumJoints ~= oldNumJoints)
                coder.internal.warning('robotics:robotmanip:motionmodels:TaskSpaceDifferentNumberJoints');
                obj.JointDamping = obj.JointDampingDefault;
            end
            
            % Check if this tree has an end effector with a matching name
            % and update with a default value if none is found
            if ~isempty(obj.EENameInternal)
                % Update with a new, valid end effector by triggering the
                % set method, which will replace the end effector with one
                % in the tree and throw a warning as needed
                obj.EndEffectorName = obj.EENameInternal;
            end
        end
    end
    
    methods (Static, Access = protected)        
        function propValueInternal = validateVectorProperty(propValue, numElements, propName)
            %validateVectorProperty Validate properties that have either 1 or N elements
            %   This function provides validation and a targeted error
            %   message for the property PROPNAME with value PROPVALUE,
            %   which is a vector that may have either 1 or NUMELEMENTS
            %   elements. This function also ensures that the internal
            %   value is always converted to column representation, even
            %   though the user-facing value is returned as a row.
            
            if isscalar(propValue)
                propValueInternal = propValue*ones(numElements,1);
            else
                validateattributes(propValue, {'double'}, {'vector', 'numel', numElements}, 'taskSpaceMotionModel', propName);
                propValueInternal = propValue(:);
            end
        end
    end
end

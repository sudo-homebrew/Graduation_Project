classdef jointSpaceMotionModel < robotics.manip.internal.InternalAccess
    %JOINTSPACEMOTIONMODEL Model rigid body tree motion given joint-space inputs
    %   The jointSpaceMotionModel object models the closed-loop joint-space
    %   motion of a manipulator, specified as a rigid body tree object. The
    %   motion model behavior is defined by the MOTIONTYPE property.
    %
    %   MOTIONMODEL = jointSpaceMotionModel('RigidBodyTree', tree) creates
    %   a motion control model for the specified RigidBodyTree object given
    %   by TREE.
    %
    %   MOTIONMODEL = jointSpaceMotionControlModel(___, 'PropertyName', PropertyValue, ..) 
    %   sets additional properties specified as name-value pairs. You can
    %   specify multiple properties in any order.
    %
    %   JOINTSPACEMOTIONMODEL Properties:
    %      RigidBodyTree      - Rigid body tree robot model
    %      NaturalFrequency   - Natural frequency of error dynamics
    %      DampingRatio       - Damping ratio of error dynamics
    %      Kp                 - Proportional gain for PD control
    %      Kd                 - Derivative gain for PD control
    %      MotionType         - Type of motion computed by the motion model. Options are:
    %         - "ComputedTorqueControl": (default): Compensates for full-body 
    %         dynamics and assigns the error dynamics specified in the
    %         NATURALFREQUENCY and DAMPINGRATIO properties.
    %
    %         - "IndependentJointMotion": Models each joint as an
    %         independent second order system using the error dynamics
    %         specified by the NATURALFREQUENCY and DAMPINGRATIO
    %         properties.
    %
    %         - "PDControl": Uses proportional-derivative control on the
    %         joints based on the specified KP and KD properties.
    %    
    %   JOINTSPACEMOTIONMODEL Methods:
    %      derivative                       - Time derivative of the model states
    %      updateErrorDynamicsFromStep      - Update the damping ratio and natural frequency given a unit step response overshoot and settling time
    %      copy                             - Create a copy of the object 
    %
    %   Example:
    %      % Load robot
    %      robot = loadrobot("kinovaGen3", "DataFormat", "column", "Gravity", [0 0 -9.81]);
    %
    %      % Set up simulation, starting from home position at zero velocity
    %      tspan = 0:0.01:1;
    %      initialState = [homeConfiguration(robot); zeros(7,1)];
    %
    %      % Define a reference state with a target position, zero
    %      % velocity, and zero acceleration
    %      targetState = [pi/4; pi/3; pi/2; -pi/3; pi/4; -pi/4; 3*pi/4; zeros(7,1); zeros(7,1)];
    %
    %      % Model the behavior with as a system under computed torque
    %      % control with error dynamics defined by a moderately fast step
    %      % response with 5% overshoot
    %      motionModel = jointSpaceMotionModel("RigidBodyTree", robot);
    %      updateErrorDynamicsFromStep(motionModel, .3, .05);
    %
    %      % Simulate the behavior over 1 second using ode45
    %      [t,robotState] = ode45(@(t,state)derivative(motionModel, state, targetState), tspan, initialState);
    %
    %      % Plot response
    %      figure
    %      plot(t, robotState(:, 1:motionModel.NumJoints));
    %      hold all;
    %      plot(t, targetState(1:motionModel.NumJoints)*ones(1,length(t)), "--");
    %      title("Joint Position (Solid) vs Reference (Dashed)");
    %      xlabel("Time (s)")
    %      ylabel("Position (rad)");
    %
    %   See also taskSpaceMotionModel
    
    %   Copyright 2019 The MathWorks, Inc.
    
    %#codegen
    
    properties (Dependent)
        %RigidBodyTree - Rigid body tree robot model
        %   Rigid body tree robot model, specified as a
        %   rigidBodyTree object that defines the inertial and
        %   kinematic properties of the manipulator.
        RigidBodyTree
        
        %NaturalFrequency - Natural frequency of error dynamics
        %   Natural frequency of error dynamics, specified as a scalar of
        %   N-element vector, where N is the number of non-fixed joints in
        %   the associated rigidBodyTree object. You must set the
        %   MotionType property to "ComputedTorqueControl" or
        %   "IndependentJointMotion"."
        NaturalFrequency
        
        %DampingRatio - Damping ratio of error dynamics
        %   Damping ratio of the second-order error dynamics, specified as
        %   a scalar or N-element vector, where N is the number of
        %   non-fixed joints in the associated rigidBodyTree
        %   object. You must set the MotionType property to
        %   "ComputedTorqueControl" or "IndependentJointMotion".
        DampingRatio
        
        %Kp - Proportional gain for PD Control
        %   Proportional gain for PD control, specified as a scalar or NxN
        %   matrix where N in the number of non-fixed joints in the
        %   associated rigidBodyTree object. You must set the
        %   MotionType property to "PDControl". The value may be provided
        %   as a scalar or as an NxN matrix, where N is the number of
        %   non-fixed joints in the associated rigidBodyTree
        %   object.
        Kp
        
        %Kd - Derivative gain for PD Control
        %   Derivative gain for PD control, specified as a scalar or NxN
        %   matrix where N in the number of non-fixed joints in the
        %   associated rigidBodyTree object. You must set the
        %   MotionType property to "PDControl".
        Kd
    end
    
    properties
        
        %MotionType - Type of motion computed by the motion model
        %   Type of motion, specified as a string scalar or character
        %   vector that defines the closed-loop joint-space behavior that
        %   the object models. Options are:
        %         - "ComputedTorqueControl": (default): Compensates for
        %         full-body dynamics and assigns the error dynamics
        %         specified in the NATURALFREQUENCY and DAMPINGRATIO
        %         properties.
        %
        %         - "IndependentJointMotion": Models each joint as an
        %         independent second order system using the error dynamics
        %         specified by the NATURALFREQUENCY and DAMPINGRATIO
        %         properties.
        %
        %         - "PDControl": Uses proportional-derivative control on the
        %         joints based on the specified KP and KD properties.
        MotionType
    end
    
    properties (Constant, Access=?robotics.manip.internal.InternalAccess)
        MotionTypeSet = {'ComputedTorqueControl', 'PDControl', 'IndependentJointMotion'}
        
        MotionTypeDefault = 'ComputedTorqueControl'
        
        NaturalFrequencyDefault = 10
        
        DampingRatioDefault = 1
        
        KpDefault = 100
        
        KdDefault = 10
    end
    
    properties (SetAccess = private, Hidden)
        NumJoints = 0
        
    end
    
    properties (Access = ?robotics.manip.internal.InternalAccess, Hidden)        
        %DampingRatioInternal
        DampingRatioInternal
        
        %NaturalFrequencyInternal
        NaturalFrequencyInternal
        
        %KpInternal
        KpInternal
        
        %KdInternal
        KdInternal
        
        %RigidBodyTreeInternal
        RigidBodyTreeInternal
        
        %RigidBodyTreeFormat
        RigidBodyTreeFormat
    end
    
    methods
        function obj = jointSpaceMotionModel(varargin)
            %JOINTSPACEMOTIONMODEL Constructor
            
            narginchk(0,12);
            
            % Convert strings to chars
            charInputs = cell(1,nargin);
            [charInputs{:}] = convertStringsToChars(varargin{:});
            
            % Parse inputs
            names = {'RigidBodyTree', 'MotionType', 'NaturalFrequency', 'DampingRatio', 'Kp', 'Kd'};
            defaults = {twoJointRigidBodyTree, obj.MotionTypeDefault, obj.NaturalFrequencyDefault, obj.DampingRatioDefault, obj.KpDefault, obj.KdDefault};
            parser = robotics.core.internal.NameValueParser(names, defaults);
            parse(parser, charInputs{:});
            obj.RigidBodyTree = parameterValue(parser, names{1});
            obj.MotionType = parameterValue(parser, names{2});
            obj.NaturalFrequency = parameterValue(parser, names{3});
            obj.DampingRatio = parameterValue(parser, names{4});
            obj.Kp = parameterValue(parser, names{5});
            obj.Kd = parameterValue(parser, names{6});      
        end
        
        function stateDot = derivative(obj, state, cmds, varargin)
            %DERIVATIVE Compute time derivative of model states
            %   STATEDOT = derivative(JOINTMOTION, STATE, CMDS) computes
            %   the state derivative of the motion model based on the
            %   current state and reference motion commands.
            %
            %   STATEDOT = derivative(JOINTMOTION, STATE, CMDS, FEXT)
            %   computes the state derivative based on the current state,
            %   reference motion commands, and any external forces on the
            %   manipulator.
            %
            %   Input Arguments:
            %
            %      JOINTMOTION   - jointSpaceMotionModel object.
            %
            %      STATE         - Joint positions and velocities, 
            %                      specified as a 2xN element vector, 
            %                      [Q; QDOT].
            %
            %      CMDS          - Control commands indicating desired 
            %                      motion. This input depends on the
            %                      MOTIONTYPE property of JOINTMOTION. For
            %                      "PDControl", CMDS is a vector with 2*N
            %                      elements, [QREF; QREFDOT], for joint
            %                      positions and velocities. For
            %                      "ComputedTorqueControl" and
            %                      "IndependentJointMotion", CMDS is a
            %                      vector with 3*N elements, [QREF;
            %                      QREFDOT; QREFDDOT], which includes joint
            %                      accelerations.
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
            %                   2*N-element vector, [QDOT; QDDOT]
            %                   representing the joint velocities and
            %                   accelerations, respectively.
            
            % Validate inputs
            narginchk(3,4);
            if strcmp(obj.MotionType, 'PDControl')
                cmdLength = 2*obj.NumJoints;
            else
                cmdLength = 3*obj.NumJoints;
            end
                
            validateattributes(state, {'double'}, {'nonempty', 'vector', 'numel', 2*obj.NumJoints, 'finite', 'real'}, 'derivative', 'state');
            validateattributes(cmds, {'double'}, {'nonempty', 'vector', 'nonnan', 'numel', cmdLength, 'real'}, 'derivative', 'cmds');

            % Get the robot position and velocity from the state
            q = state(1:obj.NumJoints);
            qDot = state((obj.NumJoints + 1):end);
            
            % Make sure everything is passed as a column
            q = q(:);
            qDot = qDot(:);
            cmds = cmds(:);
            
            % Parse the optional arguments
            if nargin > 3
                fExt = varargin{1};
                validateattributes(fExt, {'double'}, {'nonempty', '2d', 'size', [6 obj.RigidBodyTreeInternal.NumBodies], 'finite', 'real'}, 'derivative', 'fExt');
            else
                fExt = zeros(6, obj.RigidBodyTreeInternal.NumBodies);
            end
            
            % Initialize outputs
            stateDot = zeros(2*obj.NumJoints,1);
                       
            % Compute the control and dynamics model
            tau = control(obj, q, qDot, cmds);
            qDDot = computeModelDynamics(obj, q, qDot, cmds, tau, fExt);
            
            stateDot(:,:) = [qDot; qDDot]; 
        end
        
        function updateErrorDynamicsFromStep(obj, settlingTime, overshoot, varargin)
            %UPDATEERRORDYNAMICSFROMSTEP Update second order behavior using step response parameters
            %   updateErrorDynamicsFromStep(OBJ, SETTLINGTIME, OVERSHOOT) 
            %   updates the values of the NATURALFREQUENCY and DAMPINGRATIO
            %   properties given the desired step response.
            %
            %   updateErrorDynamicsFromStep(OBJ, SETTLINGTIME, OVERSHOOT, JOINTINDEX) 
            %   updates the natural frequency and damping properties for a
            %   specific joint. In this case, the values of SETTLINGTIME
            %   and OVERSHOOT must be provided as scalars since they apply
            %   to a single joint.
            %
            %   Input Arguments:
            %
            %      OBJ            - jointSpaceMotionModel object
            %
            %      SETTLINGTIME   - Settling time required to reach a 2% 
            %                       tolerance band in seconds, specified as
            %                       a scalar or an N-element vector.
            %
            %      OVERSHOOT      - The overshoot relative to a unit step, 
            %                       specified as a scalar or an N-element
            %                       vector.
            %
            %      JOINTINDEX      - The index of the joint for which the 
            %                       natural frequency and damping ratio
            %                       will be updated given unit step error
            %                       dynamics. In this case, settling time
            %                       and overshoot must be specified as
            %                       scalars.
            
            narginchk(3,4);
            
            % Validate input attributes other than size
            validateattributes(settlingTime, {'double'}, {'nonempty','finite','real','positive'}, 'getSysParamsFromStepParams', 'settlingTime');
            validateattributes(overshoot, {'double'}, {'nonempty','finite','real','nonnegative','<',1}, 'getSysParamsFromStepParams', 'overshoot');
            
            % Validate joint index and size of settling time and overshoot.
            if nargin > 3
                jointIndex = varargin{1};
                validateattributes(jointIndex, {'double'}, {'finite', 'real', 'integer', 'positive', '<=', obj.NumJoints}, 'updateErrorDynamicsFromStep', 'jointIndex');
                validateattributes(settlingTime, {'double'}, {'scalar'}, 'updateErrorDynamicsFromStep', 'settlingTime');
                validateattributes(overshoot, {'double'}, {'scalar'}, 'updateErrorDynamicsFromStep', 'overshoot');
            else
                obj.validateVariableJointDependentSize(settlingTime, obj.NumJoints, 'settlingTime');
                obj.validateVariableJointDependentSize(overshoot, obj.NumJoints, 'overshoot');
                jointIndex = 1:obj.NumJoints;
            end
            
            % Compute second order parameters from step response parameters
            numJointsToBeUpdated = max(numel(settlingTime), numel(overshoot));
            zeta = zeros(numJointsToBeUpdated, 1);
            wn = zeros(numJointsToBeUpdated, 1);
            for i = 1:numJointsToBeUpdated
                settlingTimeIndex = min(i,numel(settlingTime));
                overshootIndex = min(i,numel(overshoot));
                [zeta(i), wn(i)] = robotics.manip.internal.SecondOrderSysHelper.getSysParamsFromStepParams(settlingTime(settlingTimeIndex), overshoot(overshootIndex));
            end
            
            % If jointIndex is given, the value is only applied to the
            % specified joint. Otherwise, jointIndex is an array such that
            % all values in the vector are updated to the computed value.
            wnVector = obj.NaturalFrequencyInternal;
            zetaVector = obj.DampingRatioInternal;
            wnVector(jointIndex) = wn;
            zetaVector(jointIndex) = zeta;
                
            obj.NaturalFrequency = wnVector;
            obj.DampingRatio = zetaVector;
        end
        
        function newMotionModel = copy(obj)
            %COPY Copy the motion model
            %   NEWMOTIONMODEL = COPY(MOTIONMODEL) returns a deep copy of
            %   MOTIONMODEL. NEWMOTIONMODEL and MOTIONMODEL are two
            %   different jointSpaceMotionModel objects with the same
            %   properties.
            %   
            %   Example:
            %       % Create a task-space motion model
            %       robot = loadrobot('kinovaGen3');
            %       mm = jointSpaceMotionModel("RigidBodyTree", robot, "Kp", 140);
            %
            %       % Make a copy
            %       newMotionModel = COPY(mm)
            
            newMotionModel = jointSpaceMotionModel(...
                'RigidBodyTree', copy(obj.RigidBodyTree), ...
                'MotionType', obj.MotionType, ...
                'NaturalFrequency', obj.NaturalFrequency, ...
                'DampingRatio', obj.DampingRatio, ...
                'Kp', obj.Kp, ...
                'Kd', obj.Kd ...
                );
        end
    end
    
    %% Get/Set methods
    methods   
        function set.RigidBodyTree(obj, tree)
            %SET.RIGIDBODYTREE Set method for obj.RigidBodyTree
            %   This method relates the user-defined RigidBodyTree method
            %   to the internal representation
            
            if isa(tree, 'robotics.manip.internal.RigidBodyTree')
                % This is an internal path that is used to ensure the
                % function and related tools are codegen compatible
                obj.RigidBodyTreeInternal = tree.copy('column');
            else
                validateattributes(tree, {'rigidBodyTree'}, {'nonempty'}, 'jointSpaceMotionModel', 'RigidBodyTree');
                newRobotInternal = tree.TreeInternal.copy('column'); 
                obj.RigidBodyTreeInternal = newRobotInternal;
            end
            
            obj.RigidBodyTreeFormat = tree.DataFormat;
            oldNumJoints = obj.NumJoints;
            obj.NumJoints = obj.RigidBodyTreeInternal.VelocityNumber;
            
            % Ensure that dependent properties maintain sizes that match
            % the rigid body tree size, unless this set method is called at
            % construction time (at which point the pre-existing properties
            % are undefined)
            if ~(oldNumJoints == 0) && (obj.NumJoints ~= oldNumJoints)
                coder.internal.warning('robotics:robotmanip:motionmodels:JointSpaceDifferentNumberJoints');
                obj.NaturalFrequency = obj.NaturalFrequencyDefault;
                obj.DampingRatio = obj.DampingRatioDefault;
                obj.Kp = obj.KpDefault;
                obj.Kd = obj.KdDefault;
            end
        end
        
        function tree = get.RigidBodyTree(obj)
            %GET.RIGIDBODYTREE Get method for obj.RigidBodyTree
            %   This method relates the internal RigidBodyTree
            %   representation to the user-facing RigidBodyTree.
                             
            %Create a RigidBodyTree from the robotics.manip.internal.RigidBodyTree
            treeInternal = copy(obj.RigidBodyTreeInternal, obj.RigidBodyTreeFormat);
            tree = rigidBodyTree(treeInternal);
        end
        
        function set.MotionType(obj, inputString)
            %SET.MOTIONTYPE Setter method for MotionType
            obj.MotionType = validatestring(inputString, obj.MotionTypeSet, 'jointSpaceMotionModel', 'MotionType');
        end
        
        function set.DampingRatio(obj, zeta)
            %SET.DAMPINGRATIO Setter method for damping ratio, zeta
            
            validateattributes(zeta, {'double'}, {'nonempty', 'vector', 'finite', 'real', 'positive'}, 'jointSpaceMotionModel', 'DampingRatio');
            obj.DampingRatioInternal = obj.validateVectorProperty(zeta, obj.NumJoints, 'DampingRatio');
        end
        
        function zeta = get.DampingRatio(obj)
            %GET.DAMPINGRATIO Getter method for damping ratio, zeta
            
            % Return the values as a row so that it is easily viewed from
            % the command window
            zeta = obj.DampingRatioInternal(:)';
        end       
        
        function set.NaturalFrequency(obj, wN)
            %SET.NATURALFREQUENCY Setter method for natural frequency, wN
            
            validateattributes(wN, {'double'}, {'nonempty', 'vector', 'finite', 'real', 'positive'}, 'jointSpaceMotionModel', 'NaturalFrequency');
            obj.NaturalFrequencyInternal = obj.validateVectorProperty(wN, obj.NumJoints, 'NaturalFrequency');
        end
        
        function wN = get.NaturalFrequency(obj)
            %GET.NATURALFREQUENCY Getter method for natural frequency, wN
            
            % Return the values as a row so that it is easily viewed from
            % the command window
            wN = obj.NaturalFrequencyInternal(:)';
        end   
        
        function set.Kp(obj, kp)
            %SET.KP Setter method for proportional gain, Kp
            
            validateattributes(kp, {'double'}, {'nonempty', 'square', 'finite', 'real'}, 'jointSpaceMotionModel', 'Kp');
            obj.KpInternal = obj.validateSquareProperty(kp, obj.NumJoints, 'Kp');
        end     
        
        function kp = get.Kp(obj)
            %GET.KP Getter method for proportional gain, Kp
            
            kp = obj.KpInternal;
        end       
        
        function set.Kd(obj, kd)
            %SET.KD Setter method for derivative gain, Kd
            
            validateattributes(kd, {'double'}, {'nonempty', 'square', 'finite', 'real'}, 'jointSpaceMotionModel', 'Kd');
            obj.KdInternal = obj.validateSquareProperty(kd, obj.NumJoints, 'Kd');
        end       
        
        function kd = get.Kd(obj)
            %GET.KD Getter method for derivative gain, Kd
            
            kd = obj.KdInternal;
        end   
    end

    %% Helper methods
    methods (Access = protected)        
        function u = control(obj, q, qDot, refCmds)
            %Control compute the control law for closed loop motions
            %   This method computes the control law based on the value of
            %   the MOTIONTYPE property. Since Independent Joint Motion
            %   models the complete closed-loop behavior, no control is
            %   computed.
            
            % Initialize controller output
            u = zeros(obj.NumJoints, 1);
            
            % Specify the control method
            switch obj.MotionType
                case 'PDControl'
                    ref = refCmds(1:obj.NumJoints);
                    refDot = refCmds((obj.NumJoints + 1):(2*obj.NumJoints));
                    u = pdControl(obj, q, qDot, ref, refDot); 
                                        
                case 'ComputedTorqueControl'
                    ref = refCmds(1:obj.NumJoints);
                    refDot = refCmds((obj.NumJoints + 1):(2*obj.NumJoints));
                    refDDot = refCmds((2*obj.NumJoints + 1):(3*obj.NumJoints));
                    u = computedTorqueControl(obj, q, qDot, ref, refDot, refDDot);
                    
                otherwise
                    % No controller is used and u = 0
            end
        end
        
        function qddot = computeModelDynamics(obj, q, qDot, refCmds, tau, fExt)
            %computeModelDynamics Compute the dynamics for the motion model
            %   This method computes the dynamics. For motion models that
            %   compute torque input using a controller, this model simply
            %   applies that torque to the rigid body tree at the current
            %   state, given external forces. For models that consider a
            %   holistic view, i.e. independent joint control, this method
            %   computes the resultant dynamics given the low fidelity
            %   motion model.
            
            if strcmp(obj.MotionType, 'IndependentJointMotion')
                zeta = obj.DampingRatioInternal;
                wn = obj.NaturalFrequencyInternal;
                qRef = refCmds(1:obj.NumJoints);
                qRefDot = refCmds((obj.NumJoints + 1):(2*obj.NumJoints));
                qRefDDot = refCmds((2*obj.NumJoints + 1):(3*obj.NumJoints));
                qddot = qRefDDot - wn.^2.*obj.jointPosDiff(q, qRef) - 2.*zeta.*wn.*obj.jointVelDiff(qDot, qRefDot);
            else
                qddot = robotics.manip.internal.RigidBodyTreeDynamics.forwardDynamicsCRB(obj.RigidBodyTreeInternal, q, qDot, tau, fExt);
            end
            
        end
        
        function u = computedTorqueControl(obj, q, qDot, qRef, qRefDot, qRefDDot)
            %computedTorqueControl Compute control input under computed torque control
            %   This method computes a control law under computed torque
            %   control. Computed torque control compensates for the rigid
            %   body tree dynamics and assigns the error dynamics set by
            %   the associated second-order parameters: 
            %   qDDot = wn^2*(q - ref) + 2*zeta*wn*(qDot-refDot)
            %   For a RigidBodyTree with N non-fixed joints, the method
            %   accepts Nx1 joint configuration and velocity vectors, Q and
            %   QDOT, as well as the Nx1 joint reference configuration,
            %   velocity, and acceleration vectors.
            
            M = robotics.manip.internal.RigidBodyTreeDynamics.massMatrix(obj.RigidBodyTreeInternal, q);
            fExt = zeros(6, obj.RigidBodyTreeInternal.NumBodies);
            jointTorqWithVel = robotics.manip.internal.RigidBodyTreeDynamics.inverseDynamics(obj.RigidBodyTreeInternal, q, qDot, zeros(size(qDot)), fExt);
            jointTorqWithoutVel = robotics.manip.internal.RigidBodyTreeDynamics.inverseDynamics(obj.RigidBodyTreeInternal, q, zeros(size(qDot)), zeros(size(qDot)), fExt);
            CqDot = jointTorqWithVel - jointTorqWithoutVel; %Velocity product
            
            % Define error dynamics
            zeta = obj.DampingRatioInternal;
            wn = obj.NaturalFrequencyInternal;

            a = qRefDDot - (wn.^2).*obj.jointPosDiff(q, qRef) - (2*zeta.*wn).*obj.jointVelDiff(qDot, qRefDot);
            u = M*a + CqDot;
            u = addGravityCompensation(obj, u, q);
        end  
        
        function u = pdControl(obj, q, qDot, qRef, qRefDot)
            %pdControl Compute control input under PD control
            %   This method computes the control law defined by the
            %   joint-space PD controller of the rigid body tree, end
            %   effector, and parameters defined in the associated
            %   taskSpaceMotionControl object. The method takes the current
            %   joint configuration Q and joint velocity QDOT states as
            %   inputs, as well as the desired joint configuration and
            %   velocity, QREF and velocity QREFDOT.
            %
            %   The control method places proportional and derivative gains
            %   on the error between the desired joint configurations and
            %   velocities, respectively. Gains may be entered as scalars,
            %   or as NxN matrices, where N is the number of non-fixed
            %   joints in the associated rigid body tree. Provided gravity
            %   compensation is enabled, the corresponding system will be
            %   stable for all gains > 0.
            
            u = -obj.Kp*obj.jointPosDiff(q, qRef) - obj.Kd*obj.jointVelDiff(qDot, qRefDot);
            
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
    end
    
    methods (Static, Access = protected)
        function qbar = jointPosDiff(q, qRef)
            %jointPosDiff Compute the difference between actual and desired joint configurations
            qbar = q - qRef;            
        end
        
        function qdbar = jointVelDiff(qd, qdRef)
            %jointVelDiff Compute the difference between actual and desired joint velocities
            qdbar = qd - qdRef;            
        end
        
        function propValueInternal = validateVectorProperty(propValue, numElements, propName)
            
            if isscalar(propValue)
                propValueInternal = propValue*ones(numElements,1);
            else
                validateattributes(propValue, {'double'}, {'vector', 'numel', numElements}, 'jointSpaceMotionModel', propName);
                propValueInternal = propValue(:);
            end
        end
        
        function propValueInternal = validateSquareProperty(propValue, numCols, propName)
            
            if isscalar(propValue)
                propValueInternal = propValue*diag(ones(numCols,1));
            else
                validateattributes(propValue, {'double'}, {'square', 'ncols', numCols}, 'jointSpaceMotionModel', propName);
                propValueInternal = propValue;
            end
        end
        
        function validateVariableJointDependentSize(varIn, numJts, varName)
            %validateJointDependentVectorSize Check that VARIN is either a scalar or a vector with dimension N
            %   This method verifies that the input is either a scalar or a
            %   vector with N elements, where N is the number of non-fixed
            %   joints in the associated rigid body tree. A detailed error
            %   is thrown if the validation fails.
            
            isInvalidSize = (numel(varIn) > 1) && (numel(varIn) ~= numJts);
            if isInvalidSize
                coder.internal.error('robotics:robotmanip:motionmodels:TreeDependentParameterVectorSize', varName, num2str(numJts));
            end
        end
    end
end

classdef manipulatorStateSpace < nav.StateSpace
%MANIPULATORSTATESPACE State space for rigid body tree robot models
%   The MANIPULATORSTATESPACE object represents the joint configuration 
%   space of a rigid body tree robot model. For a given rigid body 
%   tree, the non-fixed joints in the model form the state space. The
%   NumStateVariables property corresponds to the number of non-fixed
%   joints or the size of the configuration.
%
%   Typically, the manipulator state space works with sampling-based path 
%   planners in the Navigation Toolbox(TM), including the plannerRRT and 
%   plannerBiRRT objects. The MANIPULATORSTATESPACE object derives from the 
%   nav.StateSpace class and is specified in the StateSpace property of 
%   the path planners.
%
%   SS = manipulatorStateSpace creates a default state space for a rigid 
%   body tree with two revolute joints.
%
%   SS = manipulatorStateSpace(ROBOT) creates a state space for the 
%   specified rigidBodyTree object, ROBOT.
%
%   SS = manipulatorStateSpace(ROBOT,NUMSTATEVARS) specifies the number of
%   state variables, which is the number of non-fixed joints in the robot 
%   model. This syntax is required for code generation.
%   
%   MANIPULATORSTATESPACE Properties:
%       RigidBodyTree          - Rigid body tree robot model
%       Name                   - Name of state space object
%       NumStateVariables      - Dimension of the state space
%       StateBounds            - Min and max bounds of joint positions
%
%   MANIPULATORSTATESPACE Methods:
%       enforceStateBounds     - Limit state to state bounds
%       distance               - Distance between two states
%       interpolate            - Interpolate between states
%       sampleGaussian         - Sample state using Gaussian distribution
%       sampleUniform          - Sample state using uniform distribution
%
%   Example:
%   ss = manipulatorStateSpace;
%   sv = manipulatorCollisionBodyValidator(ss);
%   planner = plannerRRT(ss,sv);
%   plannedPath = plan(planner, sampleUniform(ss), sampleUniform(ss));
%
%   See also manipulatorCollisionBodyValidator, manipulatorRRT
%

%   Copyright 2021 The MathWorks, Inc.

%#codegen


    properties(Access=private)
        %StateSpaceInternal The internal implementation of the state space
        StateSpaceInternal
    end

    properties (Access = {?nav.algs.internal.InternalAccess})
        %SkipStateValidation Skip validation in certain member functions
        %   This switch is used by internal functions only
        SkipStateValidation = false
    end


    methods(Access=protected, Hidden)
        function updateStateBounds(obj, bounds)
            obj.StateSpaceInternal.adjustJointBounds(bounds);
        end
    end

    properties(SetAccess=immutable)
        %RigidBodyTree Rigid body tree robot model
        %   Rigid body tree robot model, specified as a rigidBodyTree
        %   object. After you create the object, this property is
        %   read-only.
        RigidBodyTree
    end

    methods
        function obj = manipulatorStateSpace(robot, numStateVars)
        %Constructor
            if(nargin == 0)
                robot = twoJointRigidBodyTree("row");
                copyOfRobot = copy(robot);
                numStateVars = 2;
            else
                validateattributes(robot, ...
                                   {'rigidBodyTree'}, {'nonempty'}, ...
                                   'manipulatorStateSpace', 'rigidBodyTree');
                copyOfRobot = copy(robot);
                copyOfRobot.DataFormat = "row";
                if(coder.target('MATLAB'))
                    numStateVars = ...
                        robotics.manip.internal.RigidBodyTreeUtils.getConfigurationInfo(robot);
                else
                    coder.internal.assert(nargin > 1, ...
                                          'robotics:robotmanip:manipulatorplanning:RequiresNumStateVariablesForCodegen');
                end
            end
            [~, jointBounds] = ...
                robotics.manip.internal.RigidBodyTreeUtils.getConfigurationInfo(robot);
            ssInternal = ...
                robotics.manip.internal.ManipulatorStateSpace(copyOfRobot);
            obj@nav.StateSpace(...
                "manipulatorStateSpace", numStateVars, jointBounds);
            obj.StateSpaceInternal = ssInternal;
            obj.RigidBodyTree = copyOfRobot;
        end

        function newObj = copy(obj)
        %copy Create a deep copy of the state space
            newObj = manipulatorStateSpace(obj.RigidBodyTree, obj.NumStateVariables);
        end

        function boundedState = enforceStateBounds(obj, state)
        %enforceStateBounds Ensure state lies within state bounds
        %   BOUNDEDSTATE = enforceStateBounds(SPACE, STATE) clamps the values
        %   for input STATE within the StateBounds of the state space, SPACE.
        %   The output BOUNDEDSTATE is given as the closest state that is
        %   within the StateBounds limits.

            if(~obj.SkipStateValidation)
                nav.internal.validation.validateStateMatrix(...
                    state, nan, obj.NumStateVariables, ...
                    'enforceStateBounds', 'state');
            end
            boundedState = state(:, 1:obj.NumStateVariables);
            numStates = size(state, 1);

            % Find all states in STATE that exceed the upper limits
            % corresponding to StateBounds. The resulting logical matrix is of
            % size N-by-C where a "true" entry at (n,c) is read as "n"th state's
            % dimension "c" exceeds that dimension's upper limit.
            upperViolations = boundedState > ...
                repmat(obj.StateSpaceInternal.JointBounds(:, 2)', numStates, 1);

            % The correction for violating the upper limit violations is to clip
            % it to the upper limit
            correction = upperViolations .* ...
                repmat(obj.StateSpaceInternal.JointBounds(:, 2)', numStates, 1);
            boundedState(upperViolations) = 0;
            boundedState = boundedState + correction;

            % Similarly, we find the violations of lower limit
            lowerViolations = boundedState < ...
                repmat(obj.StateSpaceInternal.JointBounds(:, 1)', numStates, 1);

            % The correction for violating the lower limit violations is to clip
            % them to the lower limit.
            correction = lowerViolations .* ... 
                repmat(obj.StateSpaceInternal.JointBounds(:, 1)', numStates, 1);
            boundedState(lowerViolations) = 0;
            boundedState = boundedState + correction;
        end

        function state = sampleUniform(obj, varargin)
        %sampleUniform Sample state using uniform distribution
        %   STATE = sampleUniform(SS) samples a single random state within
        %   the bounds of the state space, SS.StateBounds, using a uniform
        %   distribution.
        %
        %   STATE = sampleUniform(SS,NUMSAMPLES) samples multiple random 
        %   stated based on the input NUMSAMPLES. The output STATE is a 
        %   NUMSAMPLES-by-N matrix where N is the dimension of the state 
        %   (size of joint configuration).

            narginchk(1, 4);
            if(~obj.SkipStateValidation)
                obj.validateSampleUniformInput(varargin{:});
            end
            state = obj.StateSpaceInternal.sampleUniform(varargin{:});
        end

        function state = sampleGaussian(obj, meanState, stdDev, varargin)
        %sampleGaussian Sample state using Gaussian distribution
        %   STATE = sampleGaussian(SS,MEANSTATE,STDDEV) samples a
        %   state from a Gaussian (normal) distribution truncated to the 
        %   specified state bounds with mean MEANSTATE and standard 
        %   deviation STDDEV.
        %
        %   STATE = sampleGaussian(SS,MEANSTATE,STDDEV,NUMSAMPLES) samples  
        %   multiple states based on NUMSAMPLES, specified as a positive 
        %   integer.
            narginchk(3,4);
            lowerBound = obj.StateSpaceInternal.JointBounds(:,1)';
            upperBound = obj.StateSpaceInternal.JointBounds(:,2)';
            numSamples = 1;
            if(~obj.SkipStateValidation)
                [meanState, stdDev, numSamples] = ...
                    obj.validateSampleGaussianInput(...
                    meanState, stdDev, varargin{:});
                if(~all(meanState >= lowerBound & meanState <= upperBound))
                    robotics.manip.internal.error(...
                        'manipulatorplanning:SampleGaussianMeanOutOfBounds');
                end
            end
            state = matlabshared.fusionutils.internal.truncatedGaussian(lowerBound, ...
                                                                        upperBound, ...
                                                                        meanState, ...
                                                                        stdDev,...
                                                                        numSamples);
        end

        function interpolatedState = interpolate(obj, state1, state2, ratios)
        %interpolate Interpolate between states
        %   INTERPSTATE = interpolate(SS,STATE1,STATE2,RATIOS) interpolates
        %   between two states in the state space at the given
        %   ratios. RATIOS is a row vector of values between 0 and 1 that 
        %   indicate where to sample between the two states. The output 
        %   INTERPSTATE is matrix of size R-by-N where R is the number of 
        %   elements in RATIO, and N is the size of the joint configuration 
        %   corresponding to the dimension of the state.
            narginchk(4, 4);
            if ~obj.SkipStateValidation
                [state1, state2, ratios] = ...
                    obj.validateInterpolateInput(state1, state2, ratios);
            end

            interpolatedState = ...
                obj.StateSpaceInternal.interpolate(state1, state2, ratios);
        end

        function dist = distance(obj, state1, state2)
        %distance Distance between two states
        %   DIST = distance(SS,STATE1,STATE2) calculates the distance
        %   between two states. States are a single state vector or
        %   matrix of states as row vectors. The distance between two 
        %   states is the Euclidean norm of the difference between the 
        %   state vectors. For revolute joints with infinite bounds, the 
        %   difference in joint values is calculated using angdiff().
        %
        %   STATE1 and STATE2 should be matrices of compatible sizes.  When
        %   STATE1 and STATE2 are single states, the output DIST is a scalar.
        %   When either of them is a M-by-N matrix (where M is the number of
        %   states, and N is the size of the joint configuration),  the other
        %   state should be a 1-by-N or an M-by-N matrix, and in that case, DIST
        %   is a column vector of size R. When STATE1 and STATE2 are each of
        %   size M-by-N, each row in DIST is the distance between the
        %   corresponding states in STATE1 and STATE2 in that row.

            narginchk(3, 3);
            if(~obj.SkipStateValidation)
                state1NumRows = size(state1,1);
                state2NumRows = size(state2,1);
                nav.internal.validation.validateStateMatrix(...
                    state1, nan, obj.NumStateVariables, ...
                    'distance', 'state1');
                nav.internal.validation.validateStateMatrix(...
                    state2, nan, obj.NumStateVariables, ...
                    'distance', 'state2');

                % If the number of rows in state1 and state2 are not same, we
                % need to check for compatibility.
                % If the number of rows are not same, and neither state is a row
                % vector, the sizes are incompatible.
                if(state1NumRows ~= state2NumRows) && ...
                        ~(state1NumRows == 1 || state2NumRows == 1)
                    nav.internal.validation.validateStateMatrix(...
                        state1, state2NumRows, obj.NumStateVariables, ...
                        'distance', 'state1'); %validate they have compatible sizes
                end
            end
            dist = obj.StateSpaceInternal.distance(state1, state2);
        end

    end

    methods
        function robot = get.RigidBodyTree(obj)
        %get.RigidBodyTree Returns a deep copy of the rigidBodyTree
            robot = copy(obj.RigidBodyTree);
        end
    end
end

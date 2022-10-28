classdef stateSpaceSE3 < nav.StateSpace & ...
        matlabshared.tracking.internal.CustomDisplay & ...
        matlabshared.planning.internal.EnforceScalarHandle & ...
        nav.algs.internal.InternalAccess
    %stateSpaceSE3 SE(3) state space
    %   The stateSpaceSE3 object stores parameters and has functions for
    %   the state space composed of a state vector with variables:
    %   [x, y, z, qw, qx, qy, qz]. x, y, and z are Cartesian coordinates,
    %   and qw, qx, qy, and qz represent the orientation angle in quaternion.
    %   The object uses Euclidean distance calculation and linear
    %   interpolation for translation component of the state. The object
    %   uses quaternion distance calculation and SLERP interpolation for
    %   rotation component of the state.
    %
    %   SPACE = stateSpaceSE3 creates an SE(3) state space object, SPACE,
    %   with default state bounds for x, y, and z. The state variables
    %   qw, qx, qy, and qz corresponding to orientation are not bounded.
    %
    %   SPACE = stateSpaceSE3(BOUNDS) creates a state space object with
    %   BOUNDS for x, y, z, qw, qx, qy, and qz, specified as a 7-by-2
    %   matrix (one row each for x, y, z, qw, qx, qy, and qz). Each row is
    %   a 1-by-2 vector of [min,max]. The BOUNDS input sets the value of
    %   the StateBounds property.
    %
    %   stateSpaceSE3 properties:
    %      Name              - Name of state space
    %      NumStateVariables - Number of state variables in space
    %      StateBounds       - Bounds of state variables x, y, and z
    %      WeightXYZ         - Weight for x,y,z distance calculation
    %      WeightQuaternion  - Weight for quaternion distance calculation
    %
    %   stateSpaceSE3 methods:
    %      copy               - Create copy of state space object
    %      distance           - Distance between two states
    %      enforceStateBounds - Ensure state lies within state bounds
    %      interpolate        - Interpolate between two states
    %      sampleUniform      - Sample state using uniform distribution
    %      sampleGaussian     - Sample state using Gaussian distribution
    %
    %   Example:
    %      % Create an SE(3) state space.
    %      space = stateSpaceSE3
    %
    %      % Calculate distance between two states.
    %      dist = distance(space,[2 10 3 0.2 0 0 0.8],[0 -2.5 4 0.7 0.3 0 0])
    %
    %      % Interpolate half-way between 2 states
    %      state = interpolate(space, [2 10 3 0.2 0 0 0.8], ...
    %         [0 -2.5 4 0.7 0.3 0 0], 0.5)
    %
    %   See also stateSpaceSE2, validatorOccupancyMap3D.

    %   Copyright 2020-2021 The MathWorks, Inc.

    %#codegen

    properties
        %WeightXYZ - Weight applied to x, y and z distance calculation
        %   Weight applied to x, y and z distance calculation, specified as
        %   a nonnegative real scalar. The value of weight for translations
        %   are chosen to be greater than the weight for rotations.
        %
        %   In the object, the distance calculated as:
        %
        %   sqrt(WeightXYZ*(dx^2 + dy^2 + dz^2) + WeightQuaternion*(dq^2))
        %
        %   WeightXYZ is weight applied to x, y, and z coordinates, and
        %   WeightQuaternion is the weight applied to the orientation angle
        %   in quaternion. dx, dy, and dz are the distances in the x, y,
        %   and z direction, respectively. dq is the quaternion distance.
        %
        %   Default: 1
        WeightXYZ = 1

        %WeightQuaternion - Weight applied to quaternion distance calculation
        %   Weight applied to quaternion distance calculation, specified as
        %   a nonnegative real scalar. The value of weight for rotations are
        %   chosen to be lesser than the weight for translations.
        %
        %   In the object, the distance calculated as:
        %
        %   sqrt(WeightXYZ*(dx^2 + dy^2 + dz^2) + WeightQuaternion*(dq^2))
        %
        %   WeightXYZ is weight applied to x, y, and z coordinates, and
        %   WeightQuaternion is the weight applied to the orientation angle
        %   in quaternion. dx, dy, and dz are the distances in the x, y,
        %   and z direction, respectively. dq is the quaternion distance.
        %
        %   Default: 0.1
        WeightQuaternion = 0.1
    end

    properties (Constant, Access = protected)
        %DefaultStateBounds Default bounds for SE3 state space
        %   By default, X, Y and Z are bounded by a reasonable value.
        DefaultStateBounds = ...
            [-100, 100; ...
             -100, 100; ...
             -100,  100;
             inf, inf;
             inf, inf;
             inf, inf;
             inf, inf]
    end

    properties (Access = {?nav.algs.internal.InternalAccess})
        %SkipStateValidation Skip validation in certain member functions
        %   This switch is used by internal functions only
        SkipStateValidation
    end

    methods
        function obj = stateSpaceSE3(bounds)
        %STATESPACESE3 Construct SE3 state space object

            narginchk(0,1);
            name = 'SE3';
            obj@nav.StateSpace(name, 7, stateSpaceSE3.DefaultStateBounds);

            if nargin ~= 0
                obj.StateBounds = bounds;
            end

            obj.SkipStateValidation = false;
        end
    end

    methods

        function disnce = distance(obj, state1, state2)
        %DISTANCE Distance between two states
        %   DIST = DISTANCE(SPACE, STATE1, STATE2) computes the distance
        %   between STATE1 and STATE2. The states are specified as an
        %   M-by-7 matrices, where each row is a different state. The
        %   function calculates the distance between each row in the
        %   two matrices and returns a vector of M distances.
        %
        %   Example:
        %      % Create an SE(3) state space.
        %       space = stateSpaceSE3
        %
        %       % Calculate distance between two states.
        %       dist = distance(space,[2 10 3 0.2 0 0 0.8],[0 -2.5 4 0.7 0.3 0 0])
        %
        %       % Calculate Euclidean distance between two states.
        %       space.WeightQuaternion = 0;
        %       distEuc = distance(space,[2 10 3 0.2 0 0 0.8; 4 5 2 1 2 4 2],[62 5 33 0.2 0 0 0.8; 9 9 3 3 1 3.1 7])

            narginchk(3,3);


            % Get number of states for each input
            s1 = size(state1, 1);
            s2 = size(state2, 1);

            if s1 ~= s2
                % If inputs do not have same number of states, ensure that
                % one of them only contains a single state
                if ~obj.SkipStateValidation
                    nav.internal.validation.validateStateMatrix(state1, nan, 7, 'distance', 'state1');
                    nav.internal.validation.validateStateMatrix(state2, nan, 7, 'distance', 'state2');
                end

                % Find difference between states
                if s1 == 1
                    newState1 = repelem(state1, s2, 1);
                    stateDiff = abs(newState1(:,1:3) - state2(:,1:3));
                    qDist = dist(quaternion(newState1(:,4:7)),quaternion(state2(:,4:7)));
                elseif s2 == 1
                    newState2 = repelem(state2, s1, 1);
                    stateDiff = abs(newState2(:,1:3) - state1(:,1:3));
                    qDist = dist(quaternion(newState2(:,4:7)),quaternion(state1(:,4:7)));
                else
                    % Throw error for incorrect size
                    nav.internal.validation.validateStateMatrix(state1, size(state2,1), 7, 'distance', 'state1');
                    nav.internal.validation.validateStateMatrix(state2, size(state1,1), 7, 'distance', 'state2');
                    stateDiff = state1(:,1:3); % Required for codegen
                    qDist = dist(quaternion(state1(:,4:7)),quaternion(state1(:,4:7)));
                end
            else %otherwise both states have to have the same number of rows
                if ~obj.SkipStateValidation
                    nav.internal.validation.validateStateMatrix(state1, nan, 7, 'distance', 'state1');
                    nav.internal.validation.validateStateMatrix(state2, size(state1,1), 7, 'distance', 'state2');
                end

                % Find difference between states
                stateDiff = abs(state1(:,1:3) - state2(:,1:3));
                qDist = dist(quaternion(state1(:,4:7)),quaternion(state2(:,4:7)));
            end

            dx = stateDiff(:,1);
            dy = stateDiff(:,2);
            dz = stateDiff(:,3);


            % Distance for SE3 is implemented as Cartesian product of R3
            % and SO3 (For Quaternion).
            disnce = sqrt(obj.WeightXYZ * (dx.*dx + dy.*dy + dz.*dz) + obj.WeightQuaternion * (qDist.*qDist));
        end

        function interpState = interpolate(obj, state1, state2, ratios)
        %INTERPOLATE Interpolate between two states
        %   INTERPSTATE = INTERPOLATE(SPACE, STATE1, STATE2, RATIOS) computes
        %   interpolated states between two states at points given by RATIOS,
        %   which is specified as a vector of values between [0,1].
        %   The ratio values represent the distance along the path segment
        %   that connects STATE1 and STATE2.
        %
        %   Example:
        %      % Create an SE(3) state space.
        %      space = stateSpaceSE3
        %
        %      % Interpolate half-way between 2 states
        %      state = interpolate(space, [2 10 3 0.2 0 0 0.8], [0 -2.5 4 0.7 0.3 0 0], 0.5)
        %
        %      % Interpolate multiple points with a fixed interval
        %      states = interpolate(space, [2 10 3 0.2 0 0 0.8], [0 -2.5 4 0.7 0.3 0 0], [0:0.02:1])

            narginchk(4,4);

            if ~obj.SkipStateValidation
                [state1, state2, ratios] = obj.validateInterpolateInput(state1, state2, ratios);
            end

            % Find difference between states
            stateDiff = state2(1:3) - state1(1:3);

            %Allocate memory
            interpState = zeros(size(ratios(:),1),obj.NumStateVariables);

            % Calculate interpolated states
            interpState(:,1:3) = repelem(state1(1:3), numel(ratios),1) + ratios(:)*stateDiff;

            %To support codegen write a loop for slerp
            for i=1:size(ratios(:),1)
                interpState(i,4:7) = compact(slerp(quaternion(state1(4:7)),quaternion(state2(4:7)),ratios(i)));
            end
        end

        function state = sampleGaussian(obj, meanState, stdDev, varargin) %#ok<INUSD,STOUT>
        %Unsupported for stateSpaceSE3.
        %
        %   See also sampleUniform.

            coder.internal.error('nav:navalgs:statespace:UnsupportedMethod', 'sampleGaussian');
        end

        function state = sampleUniform(obj, varargin)
        %sampleUniform Sample state using uniform distribution
        %   STATE = sampleUniform(SPACE) Samples the state variables x,
        %   y, and z within the StateBounds property of the SPACE object
        %   using a uniform probability distribution. The state variables
        %   corresponding to orientation are bound to unit quaternion
        %   using a uniform distribution of random rotations.
        %
        %   STATE = sampleUniform(SPACE,NUMSAMPLES) returns a number of
        %   state samples within the StateBounds property of the SPACE
        %   object. The number is equal to numSamples.
        %
        %   Example:
        %      % Create an SE(3) state space.
        %      space = stateSpaceSE3([-10 10; -10 10; -10 10; inf inf; inf inf; inf inf; inf inf])
        %
        %      % Sample 3 states within full state bounds
        %      state = sampleUniform(space, 3)
        %
        %   See also enforceStateBounds.

            narginchk(1,4);

            if nargin >= 3
                % Unsupported for stateSpaceSE3
                coder.internal.error('nav:navalgs:statespace:UnsupportedMethodSignature', 'sampleUniform');
            end

            numSamples = obj.validateSampleUniformInput(varargin{:});
            stateBounds = obj.StateBoundsInternal(1:3,:);

            % Generate all random samples initially
            stateXYZ = rand(3, numSamples)';
            stateQuaternion = compact(randrot(1, numSamples)');

            % Convert nearState and distVec to mu/sigma for uniform
            % distributions
            halfLimits = stateBounds / 2;
            mu = halfLimits(:,1) + halfLimits(:,2);
            sig = halfLimits(:,2) - halfLimits(:,1);

            % Scale the random samples to their corresponding spaces
            stateXYZ = matlabshared.tracking.internal.sampleUniformImpl(stateXYZ, mu(:)', sig(:)');
            state = [stateXYZ, stateQuaternion];
        end

        function boundedState = enforceStateBounds(obj, state)
        %enforceStateBounds Ensure state's x, y and z lies within state
        %bounds.
        %   BOUNDEDSTATES = enforceStateBounds(SPACE,STATES) reduces
        %   the state variables x, y, and z of the input STATES to
        %   state bounds specified in the StateBounds property of the
        %   SPACE object.
        %
        %   Example:
        %      % Create an SE(3) state space.
        %      space = stateSpaceSE3([-1 1; -2 2; -10 10; -inf inf; -inf inf; -inf inf; -inf inf])
        %
        %      % Enforce state bounds for single state
        %      boundedState = enforceStateBounds(space, [[2 10 3 2 0 0 0.8]])
        %
        %      % Enforce state bounds for multiple states
        %      boundedStates = enforceStateBounds(space, [2 10 3 2 0 0 0.8; 223 100 3 2 2 12 5])
            nav.internal.validation.validateStateMatrix(state, nan, 7, 'enforceStateBounds', 'state');

            boundedState = state;
            stateBoundsInternal = obj.StateBoundsInternal;

            %Quaternion values should not be enforced.
            stateBoundsInternal(4:obj.NumStateVariables,1) = -inf;
            stateBoundsInternal(4:obj.NumStateVariables,2) = inf;

            % Saturate values on the lower boundaries
            boundedState = max(boundedState, stateBoundsInternal(:,1)');

            % Saturate values on the upper boundaries
            boundedState = min(boundedState, stateBoundsInternal(:,2)');
        end

        function copyObj = copy(obj)
        %COPY Create deep copy of state space object
        %   SPACE2 = copy(SPACE1) creates a deep copy of the space object,
        %   SPACE2, from the state space object, SPACE1.
        %
        %   Example:
        %      % Create SE3 object and set custom weight
        %      space = stateSpaceSE3;
        %      space.WeightQuaternion = 2/3
        %
        %      % Make a deep copy
        %      space2 = copy(space)
        %
        %      % Verify that property values are the same
        %      isequal(space.WeightQuaternion, space2.WeightQuaternion)

            copyObj = stateSpaceSE3(obj.StateBounds);
            obj.copyProperties(copyObj);
        end
    end

    methods
        function set.WeightXYZ(obj, weight)
        %set.WeightXY Setter for WeightXYZ property
            validateattributes(weight, {'numeric'}, {'nonempty', 'scalar', ...
                                'real', 'nonnan', 'finite', 'nonnegative'}, 'stateSpaceSE3', 'WeightXYZ');

            obj.WeightXYZ = double(weight);
        end

        function set.WeightQuaternion(obj, weight)
        %set.WeightTheta Setter for WeightQuaternion property
            validateattributes(weight, {'numeric'}, {'nonempty', 'scalar', ...
                                'real', 'nonnan', 'finite', 'nonnegative'}, 'stateSpaceSE3', 'WeightQuaternion');

            obj.WeightQuaternion = double(weight);
        end
    end

    methods (Access = protected)
        function propgrp = getPropertyGroups(obj)
        %getPropertyGroups Custom property group display
        %   This function overrides the function in the
        %   CustomDisplay base class.
            propList = struct(...
                "Name", obj.Name,...
                "StateBounds", obj.StateBounds,...
                "NumStateVariables", obj.NumStateVariables,...
                "WeightXYZ", obj.WeightXYZ,...
                "WeightQuaternion", obj.WeightQuaternion);
            propgrp = matlab.mixin.util.PropertyGroup(propList);
        end

        function copyProperties(obj, copyObj)
        %copyProperties Copy property data from this object to new object
            copyObj.WeightXYZ = obj.WeightXYZ;
            copyObj.WeightQuaternion = obj.WeightQuaternion;
        end
    end
end

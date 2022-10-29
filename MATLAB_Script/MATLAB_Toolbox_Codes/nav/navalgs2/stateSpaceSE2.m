classdef stateSpaceSE2 < nav.StateSpace & ...
        matlabshared.tracking.internal.CustomDisplay & ...
        matlabshared.planning.internal.EnforceScalarHandle & ...
        nav.algs.internal.InternalAccess
    %stateSpaceSE2 SE(2) state space
    %   The stateSpaceSE2 object stores parameters and has functions for
    %   the state space composed of a state vector with variables:
    %   [x y theta].
    %   This is the simplest SE(2) state space using Euclidean distance
    %   calculations and linear interpolation for both translation and
    %   rotation components of the state.
    %
    %   SPACE = stateSpaceSE2 creates an SE(2) state space object, SPACE,
    %   with default state bounds for x, y, and theta.
    %
    %   SPACE = stateSpaceSE2(BOUNDS) creates a state space object with
    %   BOUNDS for x, y, and theta, specified as a 3-by-2 matrix (one row
    %   each for x, y, and theta).
    %   Each row is a 1-by-2 vector of [min,max].
    %
    %   stateSpaceSE2 properties:
    %      Name              - Name of state space
    %      NumStateVariables - Number of state variables in space
    %      StateBounds       - Bounds of state variables
    %      WeightXY          - Weight applied to x and y distance calculation
    %      WeightTheta       - Weight applied to theta distance calculation
    %
    %   stateSpaceSE2 methods:
    %      copy               - Create copy of object
    %      distance           - Distance between two states
    %      enforceStateBounds - Ensure state lies within state bounds
    %      interpolate        - Interpolate between two states
    %      sampleUniform      - Sample state using uniform distribution
    %      sampleGaussian     - Sample state using Gaussian distribution
    %
    %   See also stateSpaceDubins, stateSpaceReedsShepp.

    %   Copyright 2019-2021 The MathWorks, Inc.

    %#codegen

    properties
        %WeightXY - Weight applied to x and y distance calculation
        %   There is no general method for choosing the weights. In motion planning
        %   problems, rotations are usually given smaller weight than
        %   translations.
        %   The distance in SE2 is calculated based on the following
        %   formula:
        %   sqrt(WeightXY*(dx^2 + dy^2) + WeightTheta*(dtheta^2))
        %
        %   Default: 1
        %
        %   See also WeightTheta, distance.
        WeightXY = 1

        %WeightTheta - Weight applied to theta distance calculation
        %   The distance in SE2 is calculated based on the following
        %   formula:
        %   sqrt(WeightXY*(dx^2 + dy^2) + WeightTheta*(dtheta^2))
        %
        %   Default: 0.1
        %
        %   See also WeightXY, distance.
        WeightTheta = 0.1
    end

    properties (Constant, Access = protected)
        %DefaultStateBounds Default bounds for SE2 state space
        %   By default, X and Y are bounded by a reasonable value and Theta
        %   can take on values on the whole unit circle.
        DefaultStateBounds = ...
            [-100, 100; ...
             -100, 100; ...
             -pi,  pi]
    end

    properties (Access = {?nav.algs.internal.InternalAccess})
        %SkipStateValidation Skip validation in certain member functions
        %   This switch is used by internal functions only
        SkipStateValidation
    end

    methods
        function obj = stateSpaceSE2(bounds)
        %STATESPACESE2 Construct SE2 state space object

            narginchk(0,1);
            name = 'SE2';
            obj@nav.StateSpace(name, 3, stateSpaceSE2.DefaultStateBounds);

            if nargin ~= 0
                obj.StateBounds = bounds;
            end

            obj.SkipStateValidation = false;
        end
    end

    methods

        function dist = distance(obj, state1, state2)
        %DISTANCE Distance between two states
        %   DIST = DISTANCE(SPACE, STATE1, STATE2) computes the distance
        %   between STATE1 and STATE2. These states are M-by-3
        %   matrices, where each row is a different state. The function
        %   calculates the distance between each row in the two matrices and
        %   returns a vector of M distances.
        %
        %
        %   Example:
        %      space = stateSpaceSE2
        %
        %      % Calculate a distance between 2 states
        %      dist = DISTANCE(space, [2 10 -pi], [0 -2.5 -pi/4])
        %
        %      % Calculate Euclidean distance between multiple states
        %      space.WeightTheta = 0;
        %      dists = DISTANCE(space, [2 10 -pi; 2 10 0], [0 -2.5 -pi/4; 0 -2.5 -pi/2])

            narginchk(3,3);


            % Get number of states for each input
            s1 = size(state1, 1);
            s2 = size(state2, 1);

            if s1 ~= s2
                % If inputs do not have same number of states, ensure that
                % one of them only contains a single state
                if ~obj.SkipStateValidation
                    nav.internal.validation.validateStateMatrix(state1, nan, 3, 'distance', 'state1');
                    nav.internal.validation.validateStateMatrix(state2, nan, 3, 'distance', 'state2');
                end

                % Find difference between states
                if s1 == 1
                    stateDiff = abs(repelem(state1, s2, 1) - state2);
                elseif s2 == 1
                    stateDiff = abs(repelem(state2, s1, 1) - state1);
                else
                    % Throw error for incorrect size
                    nav.internal.validation.validateStateMatrix(state1, size(state2,1), 3, 'distance', 'state1');
                    nav.internal.validation.validateStateMatrix(state2, size(state1,1), 3, 'distance', 'state2');
                    stateDiff = state1; % Required for codegen
                end
            else %otherwise both states have to have the same number of rows
                if ~obj.SkipStateValidation
                    nav.internal.validation.validateStateMatrix(state1, nan, 3, 'distance', 'state1');
                    nav.internal.validation.validateStateMatrix(state2, size(state1,1), 3, 'distance', 'state2');
                end

                % Find difference between states
                stateDiff = abs(state1 - state2);
            end

            dx = stateDiff(:,1);
            dy = stateDiff(:,2);
            dtheta = stateDiff(:,3);

            % Restrict theta to [-pi pi]
            dtheta = robotics.internal.wrapToPi(dtheta);

            % Distance for SE2 is implemented as Cartesian product of R2
            % and SO2.
            dist = sqrt(obj.WeightXY * (dx.*dx + dy.*dy) + obj.WeightTheta * (dtheta.*dtheta));
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
        %      space = stateSpaceSE2
        %
        %      % Interpolate half-way between 2 states
        %      state = INTERPOLATE(space, [2 10 -pi], [0 -2.5 -pi/4], 0.5)
        %
        %      % Interpolate multiple points with a fixed interval
        %      states = INTERPOLATE(space, [2 10 -pi], [0 -2.5 -pi/4], [0:0.02:1])

            narginchk(4,4);

            if ~obj.SkipStateValidation
                [state1, state2, ratios] = obj.validateInterpolateInput(state1, state2, ratios);
            end

            % Find difference between states
            stateDiff = state2 - state1;

            % Restrict dTheta to [-pi pi]
            stateDiff(:,3) = robotics.internal.wrapToPi(stateDiff(:,3));

            % Calculate interpolated states
            interpState = repelem(state1, numel(ratios),1) + ratios(:)*stateDiff;

            % Restrict theta to [-pi pi]
            interpState(:,3) = robotics.internal.wrapToPi(interpState(:,3));
        end

        function state = sampleGaussian(obj, meanState, stdDev, varargin)
        %sampleGaussian Sample state using Gaussian distribution
        %   STATE = sampleGaussian(SPACE, MEANSTATE, STDDEV) samples a
        %   STATE using a Gaussian (normal) distribution with the given
        %   mean, MEANSTATE, and standard deviation, STDDEV.
        %
        %   STATE = sampleGaussian(SPACE, MEANSTATE, STDDEV, NUMSAMPLES)
        %   returns NUMSAMPLES state samples. NUMSAMPLES is a positive
        %   integer.
        %
        %   Example:
        %      space = stateSpaceSE2
        %
        %      % Sample single state
        %      state = sampleGaussian(space, [5 10 pi/2], [0.5 0.5 pi/16])
        %
        %      % Sample 10 states at once
        %      states = sampleGaussian(space, [5 10 pi/2], [0.5 0.5 pi/16], 10)
        %
        %   See also sampleUniform.

            narginchk(3,4);

            [meanState, stdDev, numSamples] = obj.validateSampleGaussianInput(meanState, stdDev, varargin{:});

            % Generate all random samples initially
            state = randn(obj.NumStateVariables, numSamples)';

            % Scale the random samples to their corresponding spaces
            state = matlabshared.tracking.internal.sampleGaussianImpl(state, meanState, diag(stdDev));

            % Make sure all state samples are within state bounds. This
            % saturation is not ideal, since it distorts the normal
            % distribution on the state boundaries, but similar to what OMPL is doing.
            state = obj.enforceStateBounds(state);
        end

        function state = sampleUniform(obj, varargin)
        %sampleUniform Sample state using uniform distribution
        %   STATE = sampleUniform(SPACE) samples a state within the
        %   StateBounds using a uniform probability distribution.
        %
        %   STATE = sampleUniform(SPACE, NUMSAMPLES) returns NUMSAMPLE
        %   state samples within the StateBounds. NUMSAMPLE is a positive
        %   integer.
        %
        %   STATE = sampleUniform(SPACE, NEARSTATE, DISTVEC, NUMSAMPLE)
        %   uniformly samples NUMSAMPLE states in a sub-region of the state
        %   space. NEARSTATE and DISTVEC are both vectors with
        %   NumStateVariables elements, where NEARSTATE defines the center
        %   of the sampled region and DISTVEC is the maximum distance from
        %   NEARSTATE allowed in each dimension.
        %
        %   Example:
        %      space = stateSpaceSE2([-10 10; -10 10; -pi pi])
        %
        %      % Sample 3 states within full state bounds
        %      state = sampleUniform(space, 3)
        %
        %      % Sample 10 states within small window near [2,2]
        %      states = sampleUniform(space, [2 2 0], [0.25 0.25 pi], 10)
        %
        %   See also sampleGaussian.

            narginchk(1,4);

            [numSamples, stateBounds, sampleNear] = obj.validateSampleUniformInput(varargin{:});

            if sampleNear
                % Calculate theta bounds without wrapping
                boundLow  = obj.StateBounds(3,1);
                boundHigh = obj.StateBounds(3,2);
                theta = varargin{1}(3);
                dTheta = varargin{2}(3);

                % Ensure sampling region adheres to state bounds
                stateBounds(3,:) = [min(max(theta-dTheta, boundLow),boundHigh),...
                                    min(max(boundLow, theta+dTheta),boundHigh)];
            else
                % Otherwise the sampling domain is between the StateBounds
                stateBounds = obj.StateBoundsInternal;
            end

            % Generate all random samples initially
            state = rand(obj.NumStateVariables, numSamples)';

            % Convert nearState and distVec to mu/sigma for uniform
            % distributions
            halfLimits = stateBounds / 2;
            mu = halfLimits(:,1) + halfLimits(:,2);
            sig = halfLimits(:,2) - halfLimits(:,1);

            % Scale the random samples to their corresponding spaces
            state = matlabshared.tracking.internal.sampleUniformImpl(state, mu(:)', sig(:)');
            state(:,3) = matlabshared.tracking.internal.wrapToPi(state(:,3));
        end

        function boundedState = enforceStateBounds(obj, state)
        %enforceStateBounds Ensure state lies within state bounds
        %   BOUNDEDSTATE = enforceStateBounds(SPACE, STATE) clamps the
        %   values for input STATE within the StateBounds of the state space,
        %   SPACE. The output BOUNDEDSTATE is given as the closest state
        %   that is within the StateBounds limits.
        %
        %   Example:
        %      space = stateSpaceSE2([-1 1; -2 2; -pi/2 pi/2])
        %
        %      % Enforce state bounds for single state
        %      boundedState = enforceStateBounds(space, [-10 5.5 pi/4])
        %
        %      % Enforce state bounds for multiple states
        %      boundedStates = enforceStateBounds(space, [-10 5.5 pi/4; 0.1 -2.33 pi])

            nav.internal.validation.validateStateMatrix(state, nan, 3, 'enforceStateBounds', 'state');

            boundedState = state;

            % Wrap angles first for theta state variable
            bounds = repelem(obj.StateBoundsInternal(:)',size(state,1),1);
            boundedState(:,3) = robotics.internal.wrapToPi(boundedState(:,3));

            % Then saturate values on the boundaries
            boundedState = min(max(boundedState, bounds(:,1:obj.NumStateVariables)),bounds(:,obj.NumStateVariables+1:end));
        end

        function copyObj = copy(obj)
        %COPY Create deep copy of state space object
        %   COPYSPACE = COPY(SPACE) creates a deep copy of the state space
        %   object SPACE and return the new object in COPYSPACE. All
        %   data of SPACE is also present in COPYSPACE.
        %
        %   Example:
        %      % Create SE2 object and set custom weight
        %      space = stateSpaceSE2;
        %      space.WeightTheta = 2/3
        %
        %      % Make a deep copy
        %      space2 = COPY(space)
        %
        %      % Verify that property values are the same
        %      isequal(space.WeightTheta, space2.WeightTheta)

            copyObj = stateSpaceSE2(obj.StateBounds);
            obj.copyProperties(copyObj);
        end
    end

    methods
        function set.WeightXY(obj, weight)
        %set.WeightXY Setter for WeightXY property
            validateattributes(weight, {'numeric'}, {'nonempty', 'scalar', ...
                                'real', 'nonnan', 'finite', 'nonnegative'}, 'stateSpaceSE2', 'WeightXY');

            obj.WeightXY = double(weight);
        end

        function set.WeightTheta(obj, weight)
        %set.WeightTheta Setter for WeightTheta property
            validateattributes(weight, {'numeric'}, {'nonempty', 'scalar', ...
                                'real', 'nonnan', 'finite', 'nonnegative'}, 'stateSpaceSE2', 'WeightTheta');

            obj.WeightTheta = double(weight);
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
                "WeightXY", obj.WeightXY,...
                "WeightTheta", obj.WeightTheta);
            propgrp = matlab.mixin.util.PropertyGroup(propList);
        end

        function copyProperties(obj, copyObj)
        %copyProperties Copy property data from this object to new object
            copyObj.WeightXY = obj.WeightXY;
            copyObj.WeightTheta = obj.WeightTheta;
        end
    end
end

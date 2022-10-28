classdef stateSpaceReedsShepp < stateSpaceSE2
%STATESPACEREEDSSHEPP State space for Reeds-Shepp vehicles
%   The Reeds-Shepp state space is based on the SE(2) space with three
%   state variables: x, y, and theta.
%
%   SPACE = stateSpaceReedsShepp creates a Reeds-Shepp state space object,
%   SPACE, with default state bounds for x, y, and theta.
%
%   SPACE = stateSpaceReedsShepp(BOUNDS) creates a state space object
%   with [min max] BOUNDS for x, y, and theta, specified as a
%   3-by-2 matrix.
%
%   stateSpaceReedsShepp properties:
%      Name              - Name of state space
%      NumStateVariables - Number of state variables in space
%      StateBounds       - Bounds of state variables
%      MinTurningRadius  - Minimum turning radius
%      ForwardCost       - Cost multiplier for forward motion
%      ReverseCost       - Cost multiplier for reverse motion
%
%   stateSpaceReedsShepp methods:
%      copy               - Create deep copy of object
%      distance           - Distance between two states
%      enforceStateBounds - Ensure state lies within state bounds
%      interpolate        - Interpolate between two states
%      sampleUniform      - Sample state using uniform distribution
%      sampleGaussian     - Sample state using Gaussian distribution
%
%   See also stateSpaceDubins, stateSpaceSE2.

%   Copyright 2019-2021 The MathWorks, Inc.

%#codegen
    properties
        %MinTurningRadius Minimum turning radius (in meters)
        %
        %   Default: 1
        MinTurningRadius = 1

        %ForwardCost - Cost multiplier for forward motion
        %   Increase to penalize forward motion.
        %
        %   Default: 1
        ForwardCost = 1

        %ReverseCost Cost multiplier for reverse motion
        %   Increase to penalize reverse motion.
        %
        %   Default: 1
        ReverseCost = 1
    end

    methods
        function obj = stateSpaceReedsShepp(varargin)
        %stateSpaceReedsShepp Construct ReedsShepp state space object

            obj@stateSpaceSE2(varargin{:});
            obj.Name = 'SE2 Reeds-Shepp';
        end
    end

    methods
        function dist = distance(obj, state1, state2)
        %DISTANCE Distance between two states
        %   DIST = DISTANCE(SPACE, STATE1, STATE2) computes the distance
        %   along the optimal ReedsShepp curve from STATE1 to STATE2.
        %   These states are M-by-3 matrices, where each row is a different
        %   state. The function calculates the distance between each row in
        %   the two matrices and returns a vector of M distances.
        %
        %   Example:
        %      space = stateSpaceReedsShepp
        %
        %      % Calculate ReedsShepp distance between 2 states
        %      DISTANCE(space, [2 10 -pi], [0 -2.5 -pi/4])
        %
        %      % Decrease the minimum turning radius and calculate again
        %      space.MinTurningRadius = 0.25;
        %      DISTANCE(space, [2 10 -pi; 2 10 0], [0 -2.5 -pi/4; 0 -2.5 -pi/2])

            if ~obj.SkipStateValidation
                narginchk(3,3);

                % If one of state1 and state2 is single state, skip checking rows for both
                if size(state1, 1) ==  1 || size(state2, 1) ==  1
                    nav.internal.validation.validateStateMatrix(state1, nan, 3, 'distance', 'state1');
                    nav.internal.validation.validateStateMatrix(state2, nan, 3, 'distance', 'state2');
                else %otherwise both states have to have the same number of rows
                    nav.internal.validation.validateStateMatrix(state1, nan, 3, 'distance', 'state1');
                    nav.internal.validation.validateStateMatrix(state2, size(state1,1), 3, 'distance', 'state2');
                end
            end

            % Call the builtins function directly. Only retrieve the
            % optimal ReedsShepp connection for distance calculation.
            distRow = matlabshared.planning.internal.ReedsSheppBuiltins.autonomousReedsSheppSegments(...
                double(state1), double(state2), obj.MinTurningRadius, obj.ForwardCost, ...
                obj.ReverseCost, 'optimal', {});
            dist = distRow.';
        end

        function interpState = interpolate(obj, state1, state2, ratios)
        %INTERPOLATE Interpolate between two states
        %   INTERPSTATE = INTERPOLATE(SPACE, STATE1, STATE2, RATIOS) computes
        %   interpolated states between two states at points given by RATIOS,
        %   which is specified as a vector of values between [0,1].
        %   The ratio values represent the distance along the Reeds-Shepp path segment
        %   that connects STATE1 and STATE2.
        %
        %   Example:
        %      space = stateSpaceReedsShepp
        %
        %      % Interpolate half-way between 2 states
        %      INTERPOLATE(space, [2 10 -pi], [0 -2.5 -pi/4], 0.5)
        %
        %      % Interpolate multiple points with a fixed interval
        %      INTERPOLATE(space, [2 10 -pi], [0 -2.5 -pi/4], [0:0.02:1])

            if ~obj.SkipStateValidation
                narginchk(4,4);

                % Validate and make sure that state vectors are rows, since the builtins
                % require that input format.
                [state1, state2, ratios] = obj.validateInterpolateInput(state1, state2, ratios);
            end

            % Use the builtins directly for maximum performance
            [pathLength,segmentLengths,segmentTypes] = ...
                matlabshared.planning.internal.ReedsSheppBuiltins.autonomousReedsSheppSegments(...
                    state1, state2, obj.MinTurningRadius, obj.ForwardCost, ...
                    obj.ReverseCost, 'optimal', {});

            % Extract motion directions
            segmentDirections = sign(segmentLengths);
            segmentDirections(segmentDirections == 0) = 1;

            % Extract motion lengths
            segmentLengths = abs(segmentLengths);

            % 'optimal' argument returns segmentLengths and segmentTypes as
            % vectors, and pathLength is a scalar
            interpState = matlabshared.planning.internal.ReedsSheppBuiltins.autonomousReedsSheppInterpolateSegments(...
                state1, state2, double(ratios) * pathLength(1), obj.MinTurningRadius, ...
                segmentLengths(:)', int32(segmentDirections(:)'), uint32(segmentTypes(:)'));

            % Make sure all angles are wrapped to [-pi,pi] interval
            interpState(:,3) = robotics.internal.wrapToPi(interpState(:,3));
        end

        function copyObj = copy(obj)
        %COPY Create deep copy of the state space object
        %   COPYSPACE = COPY(SPACE) creates a deep copy of the state space
        %   object, SPACE, and return the new object in COPYSPACE. All
        %   data of SPACE is also present in COPYSPACE.
        %
        %   Example:
        %      % Create Dubins object and set custom properties
        %      space = stateSpaceReedsShepp;
        %      space.MinTurningRadius = 0.5;
        %      space.ReverseCost = 0.25
        %
        %      % Make a deep copy
        %      space2 = COPY(space)
        %
        %      % Verify that property values are the same
        %      isequal(space.MinTurningRadius, space2.MinTurningRadius)

            copyObj = stateSpaceReedsShepp(obj.StateBounds);
            obj.copyProperties(copyObj);
        end
    end

    methods
        function set.MinTurningRadius(obj, radius)
        %set.MinTurningRadius Setter for MinTurningRadius property

            validateattributes(radius, {'double'}, ...
                               {'nonempty', 'scalar', 'real', 'nonnan', 'finite', 'positive'}, 'stateSpaceReedsShepp', 'MinTurningRadius');

            obj.MinTurningRadius = double(radius);
        end

        function set.ForwardCost(obj, cost)
        %set.ForwardCost Setter for ForwardCost property

            validateattributes(cost, {'double'}, ...
                               {'nonempty', 'scalar', 'real', 'nonnan', 'finite', 'nonnegative'}, 'stateSpaceReedsShepp', 'ForwardCost');

            obj.ForwardCost = double(cost);
        end

        function set.ReverseCost(obj, cost)
        %set.ReverseCost Setter for ReverseCost property

            validateattributes(cost, {'double'}, ...
                               {'nonempty', 'scalar', 'real', 'nonnan', 'finite', 'nonnegative'}, 'stateSpaceReedsShepp', 'ReverseCost');

            obj.ReverseCost = double(cost);
        end
    end

    methods (Access = protected)
        function propgrp = getPropertyGroups(obj)
        %getPropertyGroups Custom property group display
        %   This function is overrides the function in the
        %   CustomDisplay base class.

            propListSE2 = struct(...
                "Name", obj.Name,...
                "StateBounds", obj.StateBounds,...
                "NumStateVariables", obj.NumStateVariables);

            propgrpSE2 = matlab.mixin.util.PropertyGroup(propListSE2, ...
                                                         message("nav:navalgs:statespacese2:SE2Properties").getString);

            propListRS = struct(...
                "MinTurningRadius", obj.MinTurningRadius, ...
                "ForwardCost", obj.ForwardCost, ...
                "ReverseCost", obj.ReverseCost);
            propgrpRS = matlab.mixin.util.PropertyGroup(propListRS, ...
                                                        message("nav:navalgs:statespacereeds:ReedsSheppProperties").getString);

            propgrp = [propgrpSE2, propgrpRS];
        end

        function copyProperties(obj, copyObj)
        %copyProperties Copy property data from this object to new object

        % Copy all properties of SE2 base class
            copyProperties@stateSpaceSE2(copyObj);

            % Copy Reeds-Shepp properties
            copyObj.MinTurningRadius = obj.MinTurningRadius;
            copyObj.ForwardCost = obj.ForwardCost;
            copyObj.ReverseCost = obj.ReverseCost;
        end
    end
end

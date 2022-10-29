classdef reedsSheppConnection < matlabshared.planning.internal.ReedsSheppConnection
%reedsSheppConnection Reeds-Shepp path connection type
%   A reedsSheppConnection object holds information
%   for computing path segments. Use the connect function to compute a
%   path segment between two poses.
%
%   A Reeds-Shepp path segment that connects two poses as a sequence of
%   5 motions, each of which can be:
%   - Straight
%   - Left turn at maximum steer
%   - Right turn at maximum steer
%   - No operation
%
%   CONNECTIONOBJ = reedsSheppConnection, creates an object of
%   reedsSheppConnection with default parameters values of
%   'MinTurningRadius', 'DisabledPathTypes', 'ForwardCost' and
%   'ReverseCost'.
%
%   CONNECTIONOBJ = reedsSheppConnection(NAME-VALUE), creates
%   an object of reedsSheppConnection with specified NAME-VALUE pairs'
%   values of 'MinTurningRadius', 'DisabledPathTypes', 'ForwardCost'
%   and 'ReverseCost'.
%
%   reedsSheppConnection properties:
%   MinTurningRadius    - Minimum turning radius in meters.
%                         Default: 1
%   DisabledPathTypes   - ReedsShepp path types to disable. The
%                         disabled path types are ignored when
%                         calculating a path segment to connect poses.
%                         Specify any of the path types available in
%                         the AllPathTypes property.
%                         Default: {}.
%   AllPathTypes        - User can see all possible connections of
%                         ReedsShepp connection. This is constant
%                         property.
%   ForwardCost         - Cost multiplier to travel in forward
%                         direction. Increase to penalize forward
%                         motion.
%                         Default: 1
%   ReverseCost         - Cost multiplier to travel in reverse
%                         direction. Increase to penalize reverse
%                         motion.
%                         Default: 1
%
%   reedsSheppConnection methods:
%   connect             - Connect poses for a given connection type
%
%   Example:
%       % Create an object of reedsSheppConnection
%       rsConnObj = reedsSheppConnection;
%
%       % Define startPose and goalPose
%       startPose = [0 0 0];
%       goalPose  = [1 0 0];
%
%       % Compute the path segment between poses
%       [pathSegObj, pathCosts] = connect(rsConnObj, startPose, goalPose)
%
%   See also reedsSheppPathSegment, dubinsConnection.

% Copyright 2018-2019 The MathWorks, Inc.

%#codegen

    properties (Dependent)
        %DisabledPathTypes ReedsShepp path types to disable
        DisabledPathTypes;

        %ForwardCost Cost multiplier to travel in the forward direction
        ForwardCost;

        %ReverseCost Cost multiplier to travel in the reverse direction
        ReverseCost;
    end

    methods
        function obj = reedsSheppConnection(varargin)

            obj = obj@matlabshared.planning.internal.ReedsSheppConnection(varargin{:});
        end

        function [pathSegObjs, pathCosts] = connect(obj, startPoses, goalPoses, varargin)
        %connect Connect given start and goal poses for a given
        %   connection type
        %
        %   Start and goal poses can be anyone of the following
        %   combination:
        %   1. Single start pose and single goal pose (one-to-one mapping)
        %   2. Single start pose and multiple goal poses (one-to-all mapping)
        %   3. Multiple start poses and single goal pose (all-to-one mapping)
        %   4. Multiple start poses and multiple goal poses where the
        %   number of start poses should be equal to number of goal
        %   poses (one-to-one mapping).
        %   Each pose is a position and orientation specified as
        %   [x y theta], where x and y are positions on their
        %   respective axes and theta is the heading angle of the
        %   vehicle in radians.
        %
        %   [PATHSEGOBJ, PATHCOST] = connect(CONNECTIONOBJ,START,GOAL)
        %   connects the start and goal pose using the specified
        %   connection type object. Both a path segment object and the
        %   associated cost are returned.
        %
        %   [PATHSEGOBJ, PATHCOST] = connect(CONNECTIONOBJ,START,GOAL,
        %   'PathSegments','all')  returns all path segment types with
        %   their associated costs as a cell array. By default, the
        %   optimal path segment with the lowest cost is returned.
        %
        %   Example:
        %   % Create an object of reedsSheppConnection
        %   rsConnObj = reedsSheppConnection;
        %
        %   % Define startPose and goalPose
        %   startPose = [0 0 0];
        %   goalPose  = [0 0 pi];
        %
        %   % Compute the path segment between poses
        %   [pathSegObj, pathCosts] = connect(rsConnObj, startPose, goalPose)
        %
        %   See also reedsSheppPathSegment, dubinsConnection.

            [motionLengths, motionTypes, pathCosts, motionDirections] = ...
                connectInternal(obj, startPoses, goalPoses, varargin{:});

            nStartPoses = size(startPoses,1);
            nGoalPoses  = size(goalPoses,1);

            nStart      = 1;
            nGoal       = 1;

            nRows = size(pathCosts,1);
            nCols = size(pathCosts,2);

            pathSegObjs = cell(nRows, nCols);

            for i = 1:nRows
                for j = 1:nCols

                    numPrimtives = size(motionTypes, 1);
                    subsetMotionTypes = cell(1,numPrimtives);
                    for idTypes = coder.unroll(1:numPrimtives)
                        subsetMotionTypes{idTypes} = motionTypes{idTypes,i,j};
                    end

                    pathSegObjs{i,j} = reedsSheppPathSegment(obj, startPoses(nStart,:),...
                                                             goalPoses(nGoal,:), motionLengths(:,i,j)', subsetMotionTypes, motionDirections(:,i,j)');

                    % Update the counters
                    nStart  = mod(nStart, nStartPoses) + 1;
                    nGoal   = mod(nGoal, nGoalPoses) + 1;
                end
            end
        end

        function cpObj = copy(obj)
        %copy Create a deep copy of reedsSheppConnection object.

            cpObj = reedsSheppConnection(...
                'MinTurningRadius', obj.MinTurningRadius, ...
                'DisabledPathTypes', obj.DisabledPathTypes, ...
                'ForwardCost', obj.ForwardCost, ...
                'ReverseCost', obj.ReverseCost);
        end
    end

    methods
        function disabledPathTypes = get.DisabledPathTypes(obj)
        %get.DisabledPathTypes Getter for dependent property
        %   DisabledPathTypes
            disabledPathTypes = obj.DisabledPathTypesInternal;
        end

        function set.DisabledPathTypes(obj, disabledVal)
        %set.DisabledPathTypes Setter for dependent property
        %   DisabledPathTypes
            obj.DisabledPathTypesInternal = disabledVal;
        end

        function forwardCost = get.ForwardCost(obj)
        %get.ForwardCost Getter for dependent property ForwardCost
            forwardCost = obj.ForwardCostInternal;
        end

        function set.ForwardCost(obj, fCost)
        %set.ForwardCost Setter for dependent property ForwardCost
            obj.ForwardCostInternal = fCost;
        end

        function reverseCost = get.ReverseCost(obj)
        %get.ReverseCost Getter for dependent property ReverseCost
            reverseCost = obj.ReverseCostInternal;
        end

        function set.ReverseCost(obj, rCost)
        %set.ReverseCost Setter for dependent property ReverseCost
            obj.ReverseCostInternal = rCost;
        end
    end
end

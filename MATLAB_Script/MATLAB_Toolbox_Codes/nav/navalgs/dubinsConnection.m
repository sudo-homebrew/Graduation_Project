classdef dubinsConnection < matlabshared.planning.internal.DubinsConnection
%dubinsConnection Dubins path connection type
%   A dubinsConnection object holds information for
%   computing path segments. Use the connect function to compute a path
%   segment between two poses.
%
%   A Dubins path segment that connects two poses as a sequence of 3
%   motions, each of which can be:
%   - Straight
%   - Left turn at maximum steer
%   - Right turn at maximum steer
%
%   CONNECTIONOBJ = dubinsConnection, creates an object of
%   dubinsConnection with default parameters values of 'MinTurningRadius'
%   and 'DisabledPathTypes'.
%
%   CONNECTIONOBJ = dubinsConnection(NAME-VALUE), creates an
%   object of dubinsConnection with specified NAME-VALUE pair's values
%   of 'MinTurningRadius' and 'DisabledPathTypes'.
%
%   dubinsConnection properties:
%   MinTurningRadius    - Minimum turning radius in meters.
%                         Default: 1
%   DisabledPathTypes   - Dubins path types to disable. The disabled
%                         path types are ignored when calculating a
%                         path segment to connect poses. Specify any of
%                         the path types available in the AllPathTypes
%                         property.
%                         Default: {}.
%   AllPathTypes        - All possible connection path types.
%
%   dubinsConnection methods:
%   connect             - Connect poses for a given connection type
%
%   Example:
%       % Create an object of dubinsConnection
%       dubConnObj = dubinsConnection;
%
%       % Define startPose and goalPose
%       startPose = [0 0 0];
%       goalPose  = [0 0 pi];
%
%       % Compute the path segment between poses
%       [pathSegObj, pathCosts] = connect(dubConnObj, startPose, goalPose)
%
%   See also dubinsPathSegment, reedsSheppConnection

% Copyright 2018-2019 The MathWorks, Inc.

%#codegen

    properties (Dependent)
        %DisabledPathTypes Dubins path types to disable
        DisabledPathTypes;
    end

    methods
        function obj = dubinsConnection(varargin)

            obj = obj@matlabshared.planning.internal.DubinsConnection(varargin{:});
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
        %   % Create an object of dubinsConnection
        %   dubConnObj = dubinsConnection;
        %
        %   % Define startPose and goalPose
        %   startPose = [0 0 0];
        %   goalPose  = [1 0 0];
        %
        %   % Compute the path segment between poses
        %   [pathSegObj, pathCosts] = connect(dubConnObj, startPose, goalPose)
        %
        %   See also dubinsPathSegment, reedsSheppConnection.

            nStartPoses = size(startPoses,1);
            nGoalPoses  = size(goalPoses,1);

            [motionLengths, motionTypes, pathCosts] = ...
                connectInternal(obj, startPoses, goalPoses, varargin{:});

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

                    pathSegObjs{i,j} = dubinsPathSegment(obj, startPoses(nStart,:),...
                                                         goalPoses(nGoal,:), motionLengths(:,i,j)', subsetMotionTypes);

                    % Update the counters
                    nStart  = mod(nStart, nStartPoses) + 1;
                    nGoal   = mod(nGoal, nGoalPoses) + 1;
                end
            end
        end

        function cpObj = copy(obj)
        %copy Create a deep copy of dubinsConnection object.

            cpObj = dubinsConnection(...
                'MinTurningRadius', obj.MinTurningRadius, ...
                'DisabledPathTypes', obj.DisabledPathTypes);
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
    end
end

classdef StraightLineConnection < matlabshared.planning.internal.StraightLineConnection
%This class is for internal use only. It may be removed in the future.

%StraightLineConnection A StraightLineConnection object holds
%   information for computing straight-line path segments. Use the
%   connect function to compute a path segment between two poses.
%
%   CONNECTIONOBJ = nav.algs.internal.StraightLineConnection,
%   creates an object of StraightLineConnection.
%
%   StraightLineConnection methods:
%   connect             - Connect poses for a given connection type
%
%   Example:
%       % Create an object of StraightLineConnection
%       slConnObj = nav.algs.internal.StraightLineConnection;
%
%       % Define startPose & goalPose
%       startPose = [0 0 0];
%       goalPose  = [1 0 pi];
%
%       % Compute the path segment between poses
%       [pathSegObj, pathCosts] = connect(slConnObj, startPose, goalPose)
%
%   See also robotics.StraightLinePathSegment,
%   reedsSheppConnection, dubinsConnection

% Copyright 2018 The MathWorks, Inc.

    methods
        function obj = StraightLineConnection(varargin)

            obj = obj@matlabshared.planning.internal.StraightLineConnection;
        end

        function [pathSegObjs, pathCosts] = connect(obj, startPoses, goalPoses, varargin)
        %connect Connect poses for a given connection type
        %
        %   Start & goal poses can be anyone of the following
        %   combination:
        %   1. Single start pose & single goal pose (one-to-one mapping)
        %   2. Single start pose & multiple goal poses (one-to-all mapping)
        %   3. Multiple start poses & single goal pose (all-to-one mapping)
        %   4. Multiple start poses & multiple goal poses where the
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
        %   Example:
        %   % Create an object of StraightLineConnection
        %   slConnObj = nav.algs.internal.StraightLineConnection;
        %
        %   % Define startPose & goalPose
        %   startPose = [0 0 0];
        %   goalPose  = [1 0 0];
        %
        %   % Compute the path segment between poses
        %   [pathSegObj, pathCosts] = connect(slConnObj, startPose, goalPose)

            pathCosts = ...
                obj.connectInternal(startPoses, goalPoses, varargin{:});

            nStartPoses = size(startPoses,1);
            nGoalPoses  = size(goalPoses,1);

            nStart      = 1;
            nGoal       = 1;
            pathSegObjs = cell(size(pathCosts,1),size(pathCosts,2));
            for i = 1:size(pathCosts,2)
                pathSegObjs{i} = nav.algs.internal.StraightLinePathSegment(startPoses(nStart,:),...
                                                                  goalPoses(nGoal,:), pathCosts(:,i));

                % Update the counters
                nStart  = mod(nStart, nStartPoses) + 1;
                nGoal   = mod(nGoal, nGoalPoses) + 1;
            end
        end
    end
end

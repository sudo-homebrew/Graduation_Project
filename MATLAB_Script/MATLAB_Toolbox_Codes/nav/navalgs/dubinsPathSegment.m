classdef dubinsPathSegment < matlabshared.planning.internal.DubinsPathSegment
%dubinsPathSegment Dubins path segment connecting two poses
%
%   A dubinsPathSegment object represents a path segment connecting
%   two poses. A Dubins path segment is a sequence of 3 motions, each
%   of which can be:
%   - Straight
%   - Left turn at maximum steer
%   - Right turn at maximum steer
%
%   To generate a path segment between two poses, call the "connect"
%   function on a dubinsConnection object:
%   [PATHSEGOBJ, PATHCOST] = connect(CONNECTIONOBJ,START,GOAL) connects
%   the start and goal pose using the specified connection type object.
%   Both a path segment object and the associated cost are returned.
%
%   To specifically define a path segment:
%   DUBPATHSEGOBJ = dubinsPathSegment(DUBCONNOBJ,START,GOAL,...
%   MOTIONLENGTHS,MOTIONTYPES) creates a Dubins path segment object and
%   sets the MinTurningRadius, StartPose, GoalPose, MotionLengths, and
%   MotionTypes properties. MOTIONLENGTHS is a 3-element row vector
%   corresponding to the MOTIONTYPES given as a cell array of 3 strings
%   (e.g. {'L','S','R'}).
%
%   dubinsPathSegment properties:
%   MinTurningRadius    - Minimum turning radius (read-only)
%   StartPose           - Initial pose (read-only)
%   GoalPose            - Desired final pose (read-only)
%   MotionLengths       - Length of each motion (read-only)
%   MotionTypes         - Type of each motion (read-only)
%   Length              - Length of path segment (read-only)
%
%   DubinsPathSegments methods:
%   interpolate         - Interpolate poses along the path segment
%   show                - Visualize the path segment
%
%   Example:
%       % Create an object dubinsConnection
%       dubConn             = dubinsConnection;
%
%       % startPose and goalPose
%       startPose           = [0 0 0];
%       goalPose            = [1 0 0];
%
%       % Create an object of dubinsPathSegment
%       dubPathSegObj       = connect(dubConn,startPose, goalPose)
%
%       % Sample the poses
%       [poses, directions] = interpolate(dubPathSegObj{1})
%
%       % Visualize the path, start, goal and headings
%       show(dubPathSegObj{1});
%
%   See also reedsSheppPathSegment

% Copyright 2018-2020 The MathWorks, Inc.

%#codegen

    properties (Dependent, SetAccess = private)
        %StartPose Initial pose
        %   The initial pose specified as a [x,y,theta] vector in meters and
        %   radians.
        StartPose

        %GoalPose Desired final pose
        %   The desired final pose specified as an [x,y,theta] vector in
        %   meters and radians.
        GoalPose
    end

    methods
        function obj = dubinsPathSegment(varargin)

        % Check the number of inputs.
            narginchk(3,5);

            % Validate the connectionMethod
            validateattributes(varargin{1}, {'dubinsConnection'}, {'scalar'}, ...
                               mfilename, 'connectionMethod');

            obj = obj@matlabshared.planning.internal.DubinsPathSegment(varargin{:});
        end

        function [poses, directions] = interpolate(obj, varargin)
        %interpolate Interpolate poses along the path segment
        %
        %   POSES = interpolate(DUBPATHSEGOBJ) interpolates along the
        %   path segment at the transition between each motion type.
        %
        %   POSES = interpolate(DUBPATHSEGOBJ, LENGTHS) interpolates
        %   along the path segment at the specified path lengths,
        %   LENGTHS, specified as a vector of distances. Transitions
        %   between motion types are always included.
        %
        %   [POSES, DIRECTIONS] = interpolate(DUBPATHSEGOBJ)
        %   additionally returns the direction of motion along the path
        %   segment for each pose. A value of 1 indicates forward
        %   direction and a value of -1 indicates reverse direction.
        %
        %   Example:
        %       % Define a start and goal pose
        %       startPose = [4 4 0]; % [meters, meters, radian]
        %       goalPose  = [3 5 pi/4];
        %
        %       % Create a dubinsConnection object
        %       dubConnObj = dubinsConnection;
        %
        %       % Find the optimal path
        %       [dubPathSeg, pathCost] = connect(dubConnObj, startPose, ...
        %                            goalPose)
        %
        %       % Interpolate along the entire path every 0.1 m
        %       lengths = 0 : 0.1 : dubPathSeg{1}.Length;
        %       [poses, directions] = interpolate(dubPathSeg{1}, lengths)
        %
        %       % Visualize the path segment with transition poses
        %       show(dubPathSeg{1});
        %
        %   See also  reedsSheppPathSegment

            narginchk(1,2);
            [poses, directions] = interpolateInternal(obj, varargin{:});
        end

        function axHandle = show(obj, varargin)
        %show Visualize the path segment
        %
        %   AXHANDLE = show(DUBPATHSEGOBJ) plots the path segment with
        %   the start and goal positions and their headings.
        %
        %   AXHANDLE = show(DUBPATHSEGOBJ,NAME,VALUE) specifies
        %   additional name-value pair arguments as described below:
        %
        %   'Parent'            Handle to an axes on which to display
        %                       the path.
        %   'Headings'          Display specific headings in the path segment,
        %                       specified as a cell array of strings,
        %                       using any combination of "start","goal",
        %                       and "transitions". To disable heading
        %                       display, specify {}.
        %                       Default: {"start","goal","transitions"}
        %   'Positions'         Display specific positions in the path segment,
        %                       specified as a cell array of strings,
        %                       using "start","goal", or both. To disable
        %                       position display, specify {}.
        %                       Default: {"start","goal"}
        %   'HeadingLength'     Length of heading, 
        %                       specified as a positive numeric scalar.
        %                       
        %
        %   Example:
        %       % Define a start and goal pose
        %       startPose = [4 4 0]; % [meters, meters, radian]
        %       goalPose  = [3 5 pi/4];
        %
        %       % Create a dubinsConnection object
        %       dubConnObj = dubinsConnection;
        %
        %       % Find the optimal path
        %       [dubPathSeg, pathCost] = connect(dubConnObj, startPose, ...
        %                            goalPose)
        %
        %       % Visualize the path segment with transition poses
        %       show(dubPathSeg{1});
        %
        %       % Visualize the path segment with transition poses and
        %       % disable the headings at transitions and disable the
        %       % start position.
        %       axHandle = show(dubPathSeg{1}, "Positions", "goal", ...
        %                  "Headings", {"start", "goal"});
        %
        %       % Visualize the path segment with transition poses by
        %       % passing axes handle.
        %       show(dubPathSeg{1}, "Parent", axHandle);
        %
        %   See also  reedsSheppPathSegment.

        % Parse the input arguments
            inputparam = ...
                nav.algs.internal.parseMotionModelShowInput(varargin{:});

            % Create a new axes if not assigned
            if isempty(inputparam.axHandle)
                ax = newplot;
                inputparam.axHandle = ax;
            end

            % Get the hold status for given axes
            holdStatus = ishold(inputparam.axHandle);
            hold(inputparam.axHandle, 'on');

            % Initialize the colors and marker size
            showPathObj = nav.algs.internal.MotionModelShowPath;

            stepSize = 0.05;

            % Visualize the path
            h1 = pathPlot(showPathObj,...
                          interpolate(obj, 0:stepSize:obj.Length), inputparam.axHandle);

            % Visualize the heading at start, goal and transitions
            headingPlot(showPathObj, obj.interpolate, inputparam);

            % Visualize the start and goal
            [h2, h3] = startAndGoalPlot(showPathObj, obj.StartPose, ...
                                        obj.GoalPose, inputparam);


            %Filter legends
            legendHandles = [];
            legendNames   = {};

            counter = 1;
            if ~(isnan(h2.XData) || isnan(h2.YData))
                legendHandles(counter) = h2;
                legendNames{counter}   = message('nav:navalgs:motionModel:StartPosition').getString();
                counter = counter + 1;
            end

            if ~(isnan(h3.XData) || isnan(h3.YData))
                legendHandles(counter) = h3;
                legendNames{counter}   = message('nav:navalgs:motionModel:GoalPosition').getString();
                counter = counter + 1;
            end

            if ~isempty(h1)
                legendHandles(counter) = h1;
                legendNames{counter}   = message('nav:navalgs:motionModel:Path').getString();
            end

            % Legends
            legend(inputparam.axHandle, legendHandles,...
                   legendNames);

            % Restore the hold status of the original figure
            if ~holdStatus
                hold(inputparam.axHandle, 'off');
            end

            % Only return handle if user requested it
            if nargout > 0
                axHandle = inputparam.axHandle;
            end
        end
    end

    methods
        function startPose = get.StartPose(obj)
        %get.StartPose Getter for dependent property StartPose
            startPose = obj.StartPoseInternal;
        end

        function goalPose = get.GoalPose(obj)
        %get.GoalPose Getter for dependent property GoalPose
            goalPose = obj.GoalPoseInternal;
        end
    end
end

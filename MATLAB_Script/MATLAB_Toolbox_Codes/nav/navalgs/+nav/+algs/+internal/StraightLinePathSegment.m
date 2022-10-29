classdef StraightLinePathSegment < matlabshared.planning.internal.StraightLinePathSegment
%This class is for internal use only. It may be removed in the future.

%StraightLinePathSegment StraightLine path segment connecting two poses
%
%   A StraightLinePathSegment object represents a path segment connecting
%   two poses.
%
%   To generate a path segment between two poses, call the "connect"
%   function on a nav.algs.internal.StraightLineConnection object:
%   [PATHSEGOBJ, PATHCOST] = connect(CONNECTIONOBJ,START,GOAL) connects
%   the start and goal pose using the specified connection type object.
%   Both a path segment object and the associated cost are returned.
%
%   To specifically define a path segment:
%   SLPATHSEGOBJ = nav.algs.internal.StraightLinePathSegment(START,GOAL,...
%   LENGTH) creates a StraightLine path segment object and sets the
%   StartPose, GoalPose, Length and Direction properties. LENGTH is a
%   scalar double.
%
%   StraightLinePathSegment properties:
%   StartPose           - Initial pose (read-only)
%   GoalPose            - Desired final pose (read-only)
%   Length              - Length of path segment (read-only)
%   Direction           - Direction of path segment (read-only)
%
%   StraightLinePathSegments methods:
%   interpolate         - Interpolate poses along the path segment
%   show                - Visualize the path segment
%
%   Example:
%       % Create an object robotics.StraightLineConnection
%       slConn             = nav.algs.internal.StraightLineConnection;
%
%       % startPose & goalPose
%       startPose           = [0 0 0];
%       goalPose            = [1 0 0];
%
%       % Create an object of nav.algs.internal.StraightLinePathSegment
%       slPathSegObj        = connect(slConn,startPose, goalPose)
%
%       % Sample the poses
%       [poses, directions] = interpolate(slPathSegObj{1})
%
%       % Visualize the path, start, goal & headings
%       show(slPathSegObj{1});
%
%   See also reedsSheppPathSegment, dubinsPathSegment.

% Copyright 2018-2020 The MathWorks, Inc.

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
        function obj = StraightLinePathSegment(varargin)

        % Check the number of inputs.
            narginchk(2,3);

            if nargin == 3
                varargin{4} = 1;
            end

            obj = obj@matlabshared.planning.internal.StraightLinePathSegment(varargin{:});
        end

        function [poses, directions] = interpolate(obj, varargin)
        %interpolate Interpolate poses along the path segment
        %
        %   POSES = interpolate(SLPATHSEGOBJ) interpolates along the
        %   path segment at the transition between each motion type.
        %
        %   POSES = interpolate(SLPATHSEGOBJ, LENGTHS) interpolates
        %   along the path segment at the specified path lengths,
        %   LENGTHS, specified as a vector of distances. Transitions
        %   between motion types are always included.
        %
        %   [POSES, DIRECTIONS] = interpolate(SLPATHSEGOBJ)
        %   additionally returns the direction of motion along the path
        %   segment for each pose.
        %
        %   Example:
        %       % Define a start and goal pose
        %       startPose = [4 4 0]; % [meters, meters, radian]
        %       goalPose  = [3 5 pi/4];
        %
        %       % Create a StraightLineConnection object.
        %       slConnObj = nav.algs.internal.StraightLineConnection;
        %
        %       % Find the optimal path
        %       [slPathSeg, pathCost] = connect(slConnObj, startPose, ...
        %                            goalPose)
        %
        %       % Interpolate along the entire path every 0.1 m.
        %       lengths = 0 : 0.1 : slPathSeg{1}.Length;
        %       [poses, directions] = interpolate(slPathSeg{1}, lengths);
        %
        %       % Visualize the path segment with transition poses.
        %       show(slPathSeg{1}, "Headings", {"start", "goal"});
        %
        %   See also  reedsSheppPathSegment,
        %   dubinsPathSegment

            narginchk(1,2);
            [poses, directions] = interpolateInternal(obj, varargin{:});
        end

        function axHandle = show(obj, varargin)
        %show Visualize the path segment
        %
        %   AXHANDLE = show(SLPATHSEGOBJ) plots the path segment with
        %   the start and goal positions and their headings.
        %
        %   AXHANDLE = show(SLPATHSEGOBJ,NAME,VALUE) specifies
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
        %   Example:
%               % Define a start and goal pose
%               startPose = [4  4 0]; % [meters, meters, radian]
%               goalPose  = [3 5 pi/4];
%         
%               % Create a StraightLineConnection object.
%               slConnObj = nav.algs.internal.StraightLineConnection;
%         
%               % Find the optimal path
%               [slPathSeg, pathCost] = connect(slConnObj, startPose, ...
%                                    goalPose)
%         
        %       % Visualize the path segment with transition poses.
        %       show(slPathSeg{1});
        %
        %       % Visualize the path segment with transition poses &
        %       % disable the headings at transitions & disable the
        %       % start position.
        %       axHandle = show(slPathSeg{1}, "Positions", "goal", ...
        %                  "Headings", {"start", "goal"});
        %
        %       % Visualize the path segment with transition poses by
        %       % passing axes handle.
        %       show(slPathSeg{1}, "Parent", axHandle);
        %
        %   See also  reedsSheppPathSegment,
        %   dubinsPathSegment.

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

            % Initialize the colors & marker size.
            showPathObj = nav.algs.internal.MotionModelShowPath;

            % Visualize the path
            h1 = pathPlot(showPathObj,...
                          [obj.StartPose; obj.GoalPose], inputparam.axHandle);

            % Visualize the heading at start, goal & transitions
            headingPlot(showPathObj, obj.interpolate, inputparam);

            % Visualize the start & goal
            [h2, h3] = startAndGoalPlot(showPathObj, obj.StartPose, ...
                                        obj.GoalPose, inputparam);



            % Legends
            legend(inputparam.axHandle, [h2, h3, h1],...
                   {'Start Position', 'Goal Position', 'Path'});

            % Restore the hold status of the original figure
            if ~holdStatus
                hold(inputparam.axHandle, 'off');
            end

            % Only return handle if user requested it.
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

classdef reedsSheppPathSegment < matlabshared.planning.internal.ReedsSheppPathSegment
%reedsSheppPathSegment Reeds-Shepp path segment connecting two poses
%
%   A reedsSheppPathSegment object represents a path segment
%   connecting two poses. A Reeds-Shepp path segment is a sequence of 5
%   motions, each of which can be:
%   - Straight ('S')
%   - Left turn at maximum steer ('L')
%   - Right turn at maximum steer ('R')
%   - No Operation ('N')
%
%   To generate a path segment between two poses, call the "connect"
%   function on a reedsSheppConnection object:
%   [PATHSEGOBJ, PATHCOST] = connect(CONNECTIONOBJ,START,GOAL) connects
%   the start and goal pose using the specified connection type object.
%   Both a path segment object and the associated cost are returned.
%
%   To specifically define a path segment:
%   RSPATHSEGOBJ = reedsSheppPathSegment(RSCONNOBJ,START,GOAL,...
%   MOTIONLENGTHS,MOTIONTYPES,MOTIONDIRECTION) creates a Reeds-Shepp
%   path segment object and sets the MinTurningRadius, StartPose,
%   GoalPose, MotionLengths, MotionTypes and MotionDirections properties.
%   MOTIONLENGTHS is a 5-element row vector corresponding to the
%   MOTIONTYPES given as a cell array of 5 strings
%   (e.g. {'L','S','R','N', 'N'}) that cell of string should be one of
%   following cell strings
%   {'L', 'R', 'L', 'N', 'N'}, {'R', 'L', 'R', 'N', 'N'},
%   {'L', 'R', 'L', 'R', 'N'}, {'R', 'L', 'R', 'L', 'N'},
%   {'L', 'R', 'S', 'L', 'N'}, {'R', 'L', 'S', 'R', 'N'},
%   {'L', 'S', 'R', 'L', 'N'}, {'R', 'S', 'L', 'R', 'N'},
%   {'L', 'R', 'S', 'R', 'N'}, {'R', 'L', 'S', 'L', 'N'},
%   {'R', 'S', 'R', 'L', 'N'}, {'L', 'S', 'L', 'R', 'N'},
%   {'L', 'S', 'R', 'N', 'N'}, {'R', 'S', 'L', 'N', 'N'},
%   {'L', 'S', 'L', 'N', 'N'}, {'R', 'S', 'R', 'N', 'N'},
%   {'L', 'R', 'S', 'L', 'R'}, {'R', 'L', 'S', 'R', 'L'}.
%
%   reedsSheppPathSegment properties:
%   MinTurningRadius    - Minimum turning radius (read-only)
%   StartPose           - Initial pose (read-only)
%   GoalPose            - Desired final pose (read-only)
%   MotionLengths       - Length of each motion (read-only)
%   MotionTypes         - Type of each motion (read-only)
%   MotionDirections    - Moving direction of each motion (read-only)
%   Length              - Length of path segment (read-only)
%
%   ReedsSheppPathSegments methods:
%   interpolate         - Interpolate poses along the path segment
%   show                - Visualize the path segment
%
%   Example:
%       % Create an object reedsSheppConnection
%       rsConnMethod        = reedsSheppConnection;
%
%       % startPose and goalPose
%       startPose           = [0 0 0];
%       goalPose            = [1 0 0];
%
%       % Create an object of reedsSheppPathSegment
%       rsPathSegObj        = connect(rsConnMethod,startPose, goalPose);
%
%       % Sample the poses
%       [poses, directions] = interpolate(rsPathSegObj{1});
%
%       % Visualize the path, start, goal and headings
%       show(rsPathSegObj{1});
%
%   See also reedsSheppConnection

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
        function obj = reedsSheppPathSegment(varargin)

        % Check the number of inputs
            narginchk(3,6);

            % Validate the connectionMethod
            validateattributes(varargin{1}, {'reedsSheppConnection'}, {'scalar'}, ...
                               mfilename, 'connectionMethod');

            obj = obj@matlabshared.planning.internal.ReedsSheppPathSegment(varargin{:});
        end

        function [poses, directions] = interpolate(obj, varargin)
        %interpolate Interpolate poses along the path segment
        %
        %   POSES = interpolate(RSPATHSEGOBJ) interpolates along the
        %   path segment at the transition between each motion type.
        %
        %   POSES = interpolate(RSPATHSEGOBJ, LENGTHS) interpolates
        %   along the path segment at the specified path lengths,
        %   LENGTHS, specified as a vector of distances. Transitions
        %   between motion types are always included.
        %
        %   [POSES, DIRECTIONS] = interpolate(RSPATHSEGOBJ)
        %   additionally returns the direction of motion along the path
        %   segment for each pose. A value of 1 indicates forward
        %   direction and a value of -1 indicates reverse direction.
        %
        %   Example:
        %       % Define a start and goal pose
        %       startPose = [4 4 0]; % [meters, meters, radian]
        %       goalPose  = [3 5 pi/4];
        %
        %       % Create a reedsSheppConnection object.
        %       rsConnObj = reedsSheppConnection;
        %
        %       % Find the optimal path
        %       [rsPathSeg, pathCost] = connect(rsConnObj, startPose, ...
        %                            goalPose);
        %
        %       % Interpolate along the entire path every 0.1 m
        %       lengths = 0 : 0.1 : rsPathSeg{1}.Length;
        %       [poses, directions] = interpolate(rsPathSeg{1}, lengths);
        %
        %       % Visualize the path segment with transition poses
        %       show(rsPathSeg{1});
        %
        %   See also reedsSheppConnection

            narginchk(1,2);
            [poses, directions] = interpolateInternal(obj, varargin{:});
        end

        function axHandle = show(obj, varargin)
        %show Visualize the path segment
        %
        %   AXHANDLE = show(RSPATHSEGOBJ) plots the path segment with
        %   the start and goal positions and their headings.
        %
        %   AXHANDLE = show(RSPATHSEGOBJ,NAME,VALUE) specifies
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
        %       % Define a start and goal pose
        %       startPose = [4 4 0]; % [meters, meters, radian]
        %       goalPose  = [3 5 pi/4];
        %
        %       % Create a reedsSheppConnection object
        %       rsConnObj = reedsSheppConnection;
        %
        %       % Find the optimal path
        %       [rsPathSeg, pathCost] = connect(rsConnObj, startPose, ...
        %                            goalPose);
        %
        %       % Visualize the path segment with transition poses
        %       show(rsPathSeg{1});
        %
        %       % Visualize the path segment with transition poses and
        %       % disable the headings at transitions and disable the
        %       % start position.
        %       axHandle = show(rsPathSeg{1}, "Positions", "goal", ...
        %                  "Headings", {"start", "goal"});
        %
        %       % Visualize the path segment with transition poses by
        %       % passing axes handle.
        %       show(rsPathSeg{1}, "Parent", axHandle);
        %
        %   See also reedsSheppConnection

        % Parse the input arguments
            inputparam = ...
                nav.algs.internal.parseMotionModelShowInput(varargin{:});

            % Create a new axes if not assigned
            if isempty(inputparam.axHandle)

                ax = newplot;
                %                 if ~ishold(ax)
                %                     nav.algs.internal.MotionModelShowPath.clearAllArrows(ax);
                %                 end
                inputparam.axHandle = ax;
            end

            % Get the hold status for given axes
            holdStatus = ishold(inputparam.axHandle);
            hold(inputparam.axHandle, 'on');

            % Initialize the colors and marker size
            showPathObj = nav.algs.internal.MotionModelShowPath;

            [states, statesNeg] = reedsSheppForwardReverseStates(obj);

            % Visualize the forward path
            h1 = pathPlot(showPathObj, states, inputparam.axHandle);

            % Visualize the reverse path
            h2 = reversePathPlot(showPathObj, statesNeg, inputparam.axHandle);


            headingPlot(showPathObj, obj.interpolate, inputparam);

            % Visualize the start and goal
            [h3, h4] = startAndGoalPlot(showPathObj, obj.StartPose, ...
                                        obj.GoalPose, inputparam);

            %Filter legends
            legendHandles = [];
            legendNames   = {};

            counter = 1;
            if ~(isnan(h3.XData) || isnan(h3.YData))
                legendHandles(counter) = h3;
                legendNames{counter}   = message('nav:navalgs:motionModel:StartPosition').getString();
                counter = counter + 1;
            end

            if ~(isnan(h4.XData) || isnan(h4.YData))
                legendHandles(counter) = h4;
                legendNames{counter}   = message('nav:navalgs:motionModel:GoalPosition').getString();
                counter = counter + 1;
            end

            if ~isempty(h1)
                legendHandles(counter) = h1;
                legendNames{counter}   = message('nav:navalgs:motionModel:ForwardPath').getString();
                counter = counter + 1;
            end

            if ~isempty(h2)
                legendHandles(counter) = h2;
                legendNames{counter}   = message('nav:navalgs:motionModel:ReversePath').getString();
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

    methods (Access = {?reedsSheppPathSegment, ?nav.algs.internal.InternalAccess})

        function [states, statesNeg] = reedsSheppForwardReverseStates(obj)
        % Visualize the path, start, goal and headings for Reeds-Shepp
        % motion primitives

            stepSize = 0.05;

            samples = stepSize:stepSize:obj.Length;

            [allStates, directions] = obj.interpolate(samples);

            states = inf(numel(directions), 3);
            statesNeg = inf(numel(directions), 3);

            nPosStateCounter = 1;
            nNegStateCounter = 1;
            nStateCounter    = 1;

            previousFlag = directions(1);

            for directionsElement = directions'

                if previousFlag ~= directionsElement
                    if directionsElement == 1
                        % Changing the vehicle direction from reverse to forward
                        states(nPosStateCounter,:) = allStates(nStateCounter-1,:);
                        nPosStateCounter = nPosStateCounter + 1;
                        % To stop the reverse path.
                        statesNeg(nNegStateCounter:nNegStateCounter+1,:) = ...
                            [allStates(nStateCounter,:);nan(1,3)];
                        nNegStateCounter = nNegStateCounter + 2;
                    else
                        % Changing the vehicle direction from forward to reverse
                        statesNeg(nNegStateCounter,:) = allStates(nStateCounter-1,:);
                        nNegStateCounter = nNegStateCounter + 1;
                        % To stop the forward path.
                        states(nPosStateCounter:nPosStateCounter+1,:) = ...
                            [allStates(nStateCounter,:);nan(1,3)];
                        nPosStateCounter = nPosStateCounter + 2;
                    end
                end

                if directionsElement == 1
                    % Add forward move
                    states(nPosStateCounter,:) = allStates(nStateCounter,:);
                    nPosStateCounter = nPosStateCounter + 1;
                else
                    % Add reverse move
                    statesNeg(nNegStateCounter,:) = allStates(nStateCounter,:);
                    nNegStateCounter = nNegStateCounter + 1;
                end
                previousFlag = directionsElement;
                nStateCounter = nStateCounter + 1;
            end

            states      = states(~isinf(states(:,1)),:);
            statesNeg   = statesNeg(~isinf(statesNeg(:,1)),:);
        end
    end
end

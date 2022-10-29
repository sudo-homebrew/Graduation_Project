classdef navPath < handle & ...
        matlabshared.planning.internal.EnforceScalarHandle
    %NAVPATH Planned path
    %   The navPath object stores paths that are typically created by
    %   geometric path planners. Path points are stored as states in an
    %   associated state space.
    %
    %   PATHOBJ = navPath creates a path object, PATHOBJ, using the
    %   SE2 state space with default settings.
    %
    %   PATHOBJ = navPath(SPACE) creates a path object with the given state
    %   space, SPACE, specified as an object.
    %
    %   PATHOBJ = navPath(SPACE, STATES) initializes the path with the
    %   given state samples STATES. STATES is a matrix of state samples.
    %   States given outside the StateBounds of the state space object are
    %   clamped to the bounds.
    %
    %   navPath properties:
    %      StateSpace        - State space for path
    %      States            - Series of states for path
    %      NumStates         - Number of state samples in path
    %
    %   navPath methods:
    %      append       - Add states to end of path
    %      copy         - Create deep copy of object
    %      interpolate  - Interpolate points along path
    %      pathLength   - Length of path
    %
    %
    %   Example:
    %
    %     % Create navPath object based on multiple waypoints in Dubins space
    %     dubinsSpace = stateSpaceDubins([0 25; 0 25; -pi pi]);
    %     pathobj = navPath(dubinsSpace)
    %     waypoints = [...
    %         8 10 pi/2;
    %         10 12 pi/4;
    %         12 17 pi/2;
    %         11 10 -pi];
    %     append(pathobj, waypoints);
    %
    %     % Interpolate path, so it contains exactly 250 points
    %     interpolate(pathobj, 250)
    %
    %     % Visualize the interpolated path (and the original waypoints)
    %     figure;
    %     grid on;
    %     axis equal;
    %     hold on;
    %     plot(pathobj.States(:,1), pathobj.States(:,2), ".b");
    %     plot(waypoints(:,1), waypoints(:,2), "*r", "MarkerSize", 10)
    %
    %     % Calculate length of path
    %     len = pathLength(pathobj)
    %
    %   See also pathmetrics, stateSpaceSE2, stateSpaceDubins, stateSpaceReedsShepp.

    %   Copyright 2019 The MathWorks, Inc.

    %#codegen

    properties (SetAccess = immutable)
        %StateSpace - State space for path
        %   Specify the state space as an object during construction.
        %   You can use objects like stateSpaceSE2, stateSpaceDubins, or
        %   create a custom nav.StateSpace object.
        %   Each state in the path is a sample from this StateSpace.
        %
        %   Default: stateSpaceSE2
        StateSpace
    end

    properties (SetAccess = private)
        %States - Series of states for path
        %   The States property contains a series of states sampled within
        %   the state space bounds representing a path for navigation.
        %   Each row is a single state with the number of columns being the
        %   number of state variables. You can only set this property during
        %   object construction or through the append function.
        %
        %   Default: zeros(0, StateSpace.NumStateVariables)
        %
        %   See also append.
        States
    end

    properties (Dependent, SetAccess = private)
        %NumStates - Number of state samples in path
        %   The NumStates property is the number of samples for the path and
        %   is equal to the number of rows in the States property.
        NumStates
    end

    methods
        function obj = navPath(stateSpace, states)
        %NAVPATH Create geometric path object

            narginchk(0,2);

            switch nargin
              case 0
                % Syntax: navPath
                obj.StateSpace = stateSpaceSE2;
                statesInternal = zeros(0,3);
              case 1
                % Syntax: navPath(SPACE)
                nav.internal.validation.validateStateSpace(stateSpace, ...
                                                           'navPath', 'stateSpace');
                obj.StateSpace = stateSpace;
                statesInternal = zeros(0,obj.StateSpace.NumStateVariables);
              case 2
                % Syntax: navPath(SPACE, STATES)
                nav.internal.validation.validateStateSpace(stateSpace, ...
                                                           'navPath', 'stateSpace');
                obj.StateSpace = stateSpace;

                % Rely on validation of states in state space object
                statesInternal = obj.StateSpace.enforceStateBounds(states);
            end

            % Let States be a variably-sized property
            %NOTE: Only allowed if codegen if dynamic memory allocation
            % is enabled
            coder.varsize('statesInternal',[inf obj.StateSpace.NumStateVariables],[1 0]);
            obj.States = statesInternal;
        end

        function append(obj, state)
        %APPEND Add states to end of path
        %   APPEND(PATHOBJ, STATE) adds the state samples, STATE, to the end of
        %   the path. STATE is a matrix of state samples with each sample as
        %   a row. States outside the StateBounds of the state space are clamped
        %   to the bounds.
        %
        %   Example:
        %      pathobj1 = navPath(stateSpaceSE2, [0 0 0]);
        %
        %      % Add two state samples to path
        %      append(pathobj1, [1 1 pi/2; 2 2 pi])

            narginchk(2,2);

            % Syntax: APPEND(PATHOBJ, STATE)
            % State matrix is validated in enforceStateBounds

            obj.States = [obj.States; obj.StateSpace.enforceStateBounds(state)];
        end

        function interpolate(obj, numStates)
        %INTERPOLATE Interpolate points along path
        %   INTERPOLATE(PATHOBJ, NUMSTATES) inserts states in the path while
        %   ensuring the distribution of points is as uniform as possible.
        %   Existing points in the path are preserved and count towards
        %   the total number of points. As a result, NUMSTATES must be
        %   greater than or equal to the number of existing states in the path.
        %
        %   Example:
        %       % Create Dubins path with 3 states
        %       pathobj = navPath(stateSpaceDubins, [0 0 0; 3 5 pi/2; 8 3 0]);
        %
        %       % Interpolate to 100 states and plot
        %       interpolate(pathobj, 100)
        %       figure
        %       plot(pathobj.States(:,1), pathobj.States(:,2), ".b")

            narginchk(2,2);

            validateattributes(numStates, {'double'}, {'nonempty', 'scalar', 'real', 'nonnan', ...
                                'finite', 'integer', '>=', obj.NumStates}, 'interpolate', 'numStates');

            newNumStates = numStates;
            curNumStates = obj.NumStates;

            if curNumStates < 2
                coder.internal.error("nav:navalgs:navpath:MinNumStates", curNumStates);
            elseif newNumStates == curNumStates
                % If the number of requested states is the same as the
                % number of current states

                % Do nothing
            elseif curNumStates == 2 && isequal(obj.States(1,:),obj.States(end,:))
                % Special case where start/end state are the only states present and
                % are identical
                interval = linspace(0,1,newNumStates);
                obj.States = obj.StateSpace.interpolate(obj.States(1,:), obj.States(end,:), interval);
            else
                % Points are distributed in two steps:
                % Step 1: Assign points to each path based on its
                % contribution to the total path length. These values are
                % floored, which ensures we don't over-assign points.
                %
                % Step 2: Assign the remaining points to the K paths with
                % highest average point-to-point distance.

                % Retrieve lengths of path segments
                pLengths = obj.StateSpace.distance(obj.States(1:end-1,:), obj.States(2:end,:));
                ptsToAdd = numStates-curNumStates;

                % Get baseline number of pts needed to be added to each segment
                ptsToAddPerSegment = floor(pLengths/sum(pLengths)*ptsToAdd);
                ptsToAddPerSegment(isnan(ptsToAddPerSegment)) = 0;

                % Update remaining points
                ptsToAdd = ptsToAdd - sum(ptsToAddPerSegment);

                % Update the point-to-point distance for each segment using the
                % new distribution of points. For example, if segment 1
                % initially had length 2, and was assigned 2 points, its new
                % point-to-point distance is 2/(3+1) = 1/2
                pLengths = pLengths./(ptsToAddPerSegment+1);

                % Distribute the K remaining points based on the segments with
                % K-largest point-to-point distances
                [~, idxs] = maxk(pLengths, ptsToAdd);
                ptsToAddPerSegment(idxs) = ptsToAddPerSegment(idxs) + 1;

                % Pre-allocate new states
                interpStates = zeros(newNumStates, obj.StateSpace.NumStateVariables);

                % Loop through segments and add points where needed
                curPt = 1;
                for i = 1:numel(pLengths)
                    if ptsToAddPerSegment(i) > 0
                        % Interpolate extra points
                        interval = linspace(0,1,ptsToAddPerSegment(i)+2);
                        interpStates(curPt:curPt+ptsToAddPerSegment(i),:) = ...
                            obj.StateSpace.interpolate(obj.States(i,:), obj.States(i+1,:), ...
                                                       interval(1:end-1));
                        curPt = curPt + ptsToAddPerSegment(i) + 1;
                    else
                        % Just add starting pt in segment
                        interpStates(curPt,:) = obj.States(i,:);
                        curPt = curPt + 1;
                    end
                end

                % Add the last state
                interpStates(end,:) = obj.States(end,:);

                % Replace states with interpolated states
                obj.States = interpStates;
            end
        end

        function pLength = pathLength(obj)
        %pathLength Length of path
        %   LEN = pathLength(PATHOBJ) computes the total length of the path
        %   by summing the distance between every sequential pair of states
        %   in the States property. The distance of each path segment is
        %   computed by the StateSpace object associated with PATHOBJ.
        %
        %   Example:
        %       % Create SE2 path with 100 random points
        %       pathobj = navPath(stateSpaceSE2, rand(100,3));
        %
        %       length = pathLength(pathobj)

            narginchk(1,1);

            % Calculate individual distances between states in path and sum
            % them up
            pLength = sum(obj.StateSpace.distance(obj.States(1:end-1,:), obj.States(2:end,:)));
        end

        function copyObj = copy(obj)
        %COPY Create deep copy of object
        %   COPYPATHOBJ = COPY(PATHOBJ) creates a deep copy of the path
        %   object PATHOBJ and return the new object in COPYPATHOBJ. All
        %   data of PATHOBJ is also present in COPYPATHOBJ.
        %
        %   Example:
        %      % Create path object
        %      space = stateSpaceSE2([-100 0; -20 20; -pi pi]);
        %      pathobj = navPath(space);
        %
        %      % Make a deep copy
        %      pathobj2 = COPY(pathobj)

            states = obj.States;
            copiedSpace = copy(obj.StateSpace);

            % Set the state space through the constructor
            % This relies on the copy implementation in the state space.
            if isempty(states)
                copyObj = navPath(copiedSpace);
            else
                copyObj = navPath(copiedSpace, states);
            end
        end
    end

    methods
        function numStates = get.NumStates(obj)
        %get.NumStates - Getter for NumStates property
            numStates = size(obj.States,1);
        end
    end
end

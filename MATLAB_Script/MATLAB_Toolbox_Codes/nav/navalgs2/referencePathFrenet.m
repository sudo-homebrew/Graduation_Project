classdef referencePathFrenet < nav.algs.internal.FrenetReferencePath
%referencePathFrenet Smooth reference path fit to waypoints
%
%   The referencePathFrenet object fits a smooth, piecewise continuous
%   curve to the provided [x y] or [x y theta] waypoints. Points along the
%   curve are expressed as [x, y, theta, kappa, dkappa, s], where:
%
%       x,y,theta   - SE2 state relative to global coordinate
%       kappa       - Curvature
%       dkappa      - Derivative of curvature relative to arc length
%       s           - Arclength, or distance along path from path origin
%
%   Use methods on the object to convert trajectories between global and 
%   Frenet coordinate systems, interpolate states along the path based on 
%   arc length, and query the closest points on the path to global states.
%
%   Frenet states are [S dS ddS L dL ddL], where S is the arc
%   length, L is the deviation perpendicular to the path direction, 
%   derivatives of S are with respect to time and derivatives of L are with
%   respect to arclength, S.
%
%   Global states are [x y theta kappa speed accel], where speed and accel
%   are scalar magnitudes in the direction of theta.
%
%   SYNTAXES:
%
%       refPathObj = referencePathFrenet(WAYPOINTS) fits a piecewise
%       continuous set of curves to [x y] or [x y theta] waypoints. Provide
%       theta when reference path must approach waypoints with desired
%       orientation.
%
%       refPathObj = referencePathFrenet(__,Name,Value) set properties on
%       the object by specifying the property name and value as name-value
%       pairs. Name must appear inside single quotes (''). You can specify
%       several name-value pair arguments in any order as 
%       Name1,Value1,...,NameN,ValueN:
%
%           'DiscretizationDistance'    - Arclength between shown points
%                                           Default: 0.05 (m)
%
%           'MaxNumWaypoints'       	- Maximum waypoints allowed in path
%                                           Default: inf
%
%   referencePathFrenet properties:
%       SegmentParameters       - Clothoid parameters at start of segments
%       DiscretizationDistance  - Arclength between visualized points
%       MaxNumWaypoints         - Maximum waypoints allowed in path
%       PathLength              - Total arclength along the path
%       Waypoints               - Presampled points along the path
%
%   referencePathFrenet methods:
%       closestPoint            - Find state of path closest to global XY
%       closestPointsToSequence - Projects a sequence of points onto path
%       changeInCurvature       - Return change in curvature at arclength
%       closestProjections      - Find orthogonal projections between 
%                                 path tangent vector and query point
%       changeInCurvature       - Return change-in-curvature at arclength
%       copy                    - Create a deep copy of the object
%       curvature               - Return curvature at arclength
%       frenet2global           - Convert Frenet states to global states
%       global2frenet           - Convert global states to Frenet states
%       interpolate             - Evaluate path at provided arclength
%       position                - Return XY location at arclength
%       show                    - Show the reference path in a figure
%       tangentAngle            - Return tangent angle at arclength
%
%   Example:
%
%       % Create reference waypoints.
%       waypoints = [0 0; 100 50; 400 -50; 800 0];
%
%       % Initialize reference path object using the waypoints and default
%       % discretization interval.
%       refPath = referencePathFrenet(waypoints);
%
%       % Create a low fidelity trajectory in Frenet coordinates
%       dt = 0.5;
%       sInitial = [0 0 0];
%
%       % Define longitudinal position as the distance along the path from
%       % the path origin (arclength) and the first/second order
%       % longitudinal derivatives with respect to time.
%       ddS = [linspace(0,5,10) linspace(5,0,10)]';     % d(d(S))/dt/dt
%       dS  = cumsum(ddS*dt);                           % d(S)/dt
%       S   = cumsum(dS*dt);                            % arclength
%
%       % Define Lateral deviation as the distance projected along the 
%       % normal to the curve at S, and the first/second order lateral
%       % derivatives with respect to longitude.
%       latScale = 0.1;
%       Lpp = ddS*latScale;                             % d(d(L)/dS/dS
%       Lp  = cumsum(Lpp*dt);                           % d(L)/d(S)
%       L   = cumsum(Lp*dt);                            % lateral deviation
%       frenetTraj = [S dS ddS L Lp Lpp];
%
%       % Convert trajectory to global coordinates.
%       globalTraj = frenet2global(refPath, frenetTraj);
%
%       % Find the state of the path closest to a desired point.
%       xyDesired = waypoints(3,:) + [20 -50];
%       nearestPathPoint = closestPoint(refPath, xyDesired);
%
%       % Shift the Frenet trajectory so that it begins at this point.
%       shiftS = nearestPathPoint(1,6);
%       pathTangent = [cos(nearestPathPoint(3)) sin(nearestPathPoint(3))];
%       distVector = xyDesired-nearestPathPoint(1:2);
%       shiftL = norm(distVector)*sign(det([pathTangent' distVector']));
%
%       % Update Frenet trajectory.
%       frenetTrajShifted = frenetTraj + [shiftS 0 0 shiftL 0 0];
%
%       % Convert new trajectory back to global coordinates.
%       globalTrajShifted = frenet2global(refPath, frenetTrajShifted);
%       
%       % Display results
%       show(refPath);
%       axis equal;
%       hold on;
%       plot(globalTraj(:,1),globalTraj(:,2),'-b');
%       plot(xyDesired(1),xyDesired(2),'rX');
%       plot(globalTrajShifted(:,1),globalTrajShifted(:,2),'g-');
%       legend({'Waypoints','refPath','globalTrajShifted','xyDesired', ...
%           'globalTrajShifted'});
%
%       % Get path length
%       lastPathPoint = closestPoint(refPath, refPath.Waypoints(end,:));
%       pathLength = lastPathPoint(6);
%       
%       % Retrieve evenly spaced points along the original path.
%       resampleLengths = linspace(0, pathLength, 5);
%       pathStates = interpolate(refPath, resampleLengths);
%
%       % Create a new path, but force all points to alternate orientation
%       % between pi/2 and -pi/2.
%       waypointsNew = [pathStates(:,1:2), ...
%           pi/4*((-1).^(0:(numel(resampleLengths)-1)))'];
%       refPathNew = referencePathFrenet(waypointsNew);
%
%       % Display the updated path.
%       show(refPathNew);
%       legend({'Waypoints','refPath','globalTrajShifted','xyDesired', ...
%           'globalTrajShifted','waypointsNew','refPathNew'});
%
%   See also trajectoryOptimalFrenet, trajectoryGeneratorFrenet

%   Copyright 2020-2021 The MathWorks, Inc.

%#codegen
    
    properties (Dependent)
        %PathLength Total arclength along the path
        PathLength
        
        %SegmentParameters Clothoid parameters at start of segments
        %
        %   An Nx6 matrix where the i'th row contains parameters of the
        %   clothoidal segment, [x y theta k dk s], which connects 
        %   waypoint i to waypoint i+1. The final value, s, represents the
        %   arclength at which the segment begins (with respect to the
        %   origin of the path).
        SegmentParameters
    end
    
    methods
        function obj = referencePathFrenet(waypoints, varargin)
        %referencePathFrenet Represents waypoints using smooth continuous set of curves
            narginchk(1,5);
            obj = obj@nav.algs.internal.FrenetReferencePath(waypoints, varargin{:});
        end

        function cObj = copy(obj)
        %copy Create a deep copy of the object
            if isa(obj.PathManager,'nav.algs.internal.PathManagerFixed')
                maxSize = {'MaxNumWaypoints',obj.MaxNumWaypoints};
            else
                maxSize = {};
            end
            cObj = referencePathFrenet(obj.Waypoints, ...
                    'DiscretizationDistance', obj.DiscretizationDistance, ...
                    maxSize{:});
        end

        function [pathPoints, inWindow] = closestPoint(obj, points, searchWindow)
        %closestPoint Find state of path closest to global XY
        %
        %   PATHPOINTS = closestPoint(obj,POINTS) returns the closest 
        %   point on the path, PATHPOINTS, for each of the N points in 
        %   input, POINTS, specified as as an N-row matrix where the first
        %   two columns are assumed to contain X and Y positions. Each row
        %   in PATHPOINTS contains [x y theta kappa dkappa S].
        %
        %   [PATHPOINTS,INWINDOW] = closestPoint(obj,POINTS) optionally
        %   returns INWINDOW, an Nx1 boolean vector indicating indicating
        %   whether each of the N returned points lie inside the path 
        %   bounds.
        %
        %   [___] = closestPoint(obj,POINTS,SEARCHWINDOW) optionally 
        %   takes in SEARCHWINDOW, a 1x2 nondecreasing vector which defines
        %   the interval of the path that should be considered when finding
        %   the closest points.
        %
        %   Example:
        %   
        %       % Create a self-intersecting reference path
        %       refPath = referencePathFrenet([0 100 -pi/4; 50 50 -pi/4;...
        %           75 50 pi/2; 50 50 -3*pi/4; 0 0 -3*pi/4]);
        %
        %       % Show path
        %       figure;
        %       show(refPath); axis equal; hold on;
        %       title("Closest Points Around Intersection");
        %
        %       % Find the arclength at which the intersection occurs
        %       sIntersection = refPath.SegmentParameters(2,end);
        %
        %       % Generate Frenet states lying just before and after the intersection
        %       f0 = [sIntersection-20 5 0 10 0 0]; % [S dS ddS L Lp Lpp]
        %       f1 = [sIntersection+20 5 0 -5 0 0]; % [S dS ddS L Lp Lpp]
        %
        %       % Calculate time to travel with constant longitudinal velocity
        %       T = (f1(1)-f0(1))/f1(2);
        %
        %       % Create trajectory generator
        %       generator = trajectoryGeneratorFrenet(refPath);
        %
        %       % Generate trajectory between points
        %       [fTraj, gTraj] = connect(generator,f0,f1,T);
        %       pts = gTraj.Trajectory;
        %
        %       % Define plot helper function
        %       mergeFcn = @(v1,v2)reshape([v1 v2 nan(size(v1,1),size(v2,2))]',[],1);
        %       plotFcn = @(L1,L2,linespec)plot(mergeFcn(L1(:,1),L2(:,1:min(1,size(L2,2)))), ...
        %          mergeFcn(L1(:,2),L2(:,2:min(2,size(L2,2)))),linespec{:});
        %       plotInterval = @(bounds,nPt,linespec)plotFcn(interpolate(...
        %           refPath,linspace(bounds(1),bounds(2),nPt)'),[],linespec);
        %
        %       % Plot trajectory
        %       plotFcn(pts,[],{'k.-'});
        %
        %       % Calculate closest point on path to each global state
        %       closestPts = closestPoint(refPath,pts);
        %   
        %       % Plot closest points vectors
        %       plotFcn(pts,closestPts,{'b'});
        %
        %       % Define a window in which to search for closest points
        %       buffWindow = [f0(1)-5 f1(1)+5];
        %       plotInterval(buffWindow,100,{'Color',[.5 .5 .5],'LineWidth',5});
        %       
        %       % Find closest points within window
        %       closestPtsInWindow = closestPoint(refPath,pts,buffWindow);
        %
        %       % Display windowed results
        %       plotFcn(pts,closestPtsInWindow,{'g','LineWidth',3});
        %
        %       % Find closest points using a window that is too small
        %       smallWindow = [f0(1)+5 f1(1)-5];
        %       [closestPtsSmall,inWindow] = closestPoint(refPath,pts, ...
        %           smallWindow);
        %
        %       % Overlay small window results
        %       plotInterval(smallWindow,100,{'m','LineWidth',3});
        %       plotFcn(pts(inWindow,:),closestPtsSmall(inWindow,:),{'Color', [.5 1 .5]});
        %       plotFcn(pts(~inWindow,:),closestPtsSmall(~inWindow,:),{'r'});
        %       legend({'Waypoints','ReferencePath','Trajectory','ClosestPoints',...
        %           'BuffWindow','ClosestInsideBuffWindow','SmallWindow',...
        %           'ClosestInsideSmallWindow','ClosestOutsideSmallWindow'});
        %       
        %   See also closestPointsToSequence, closestProjections
            
            % Validate inputs
            narginchk(2,3);
            xy = obj.validateXYPoints(points,'closestPoint','points');
            
            if nargin == 2
                % Find closest points
                pathPoints = closestPoint@nav.algs.internal.FrenetReferencePath(obj,xy);
                clippedWindow = [0 obj.Length];
            else
                clippedWindow = obj.validateSearchWindow(searchWindow,'closestPoint','searchWindow');
                
                pathPoints = obj.searchInWindow(xy,clippedWindow,false);
            end
            
            if nargout == 2
                inWindow = obj.verifyInWindow(xy,pathPoints,clippedWindow);
            end
        end
        
        function [pathPoints, inWindow] = closestPointsToSequence(obj, points, initialWindow)
        %closestPointsToSequence Projects a sequence of points onto path
        %
        %   PATHPOINTS = closestPointsToSequence(obj,POINTS,INITIALWINDOW) 
        %   treats the input, POINTS, as a sequence of points. The closest
        %   point is initially required to be within INITIALWINDOW, a 1x2
        %   vector defining the valid search region. For each subsequent
        %   point in POINTS, the search window is recentered atop the 
        %   previous point.
        %
        %   [PATHPOINTS,INWINDOW] = ...
        %   closestPointsToSequence(obj,POINTS,INITIALWINDOW) optionally 
        %   returns INWINDOW, an Nx1 logical array where each element 
        %   signifies whether the nearest point to the corresponding
        %   xy coordinate in POINTS is projected within the search window
        %   (true), or lies at the end of the window, and is therefore not 
        %   a valid projection (false).
        %
        %   Example:
        %
        %       % Grab closestPoint help text
        %       x = help('referencePathFrenet.closestPoint');
        %         
        %       % Extract "Example" code:
        %       exampleCode = regexp(x,'((?<=Example:).*(?=See))','match');
        %         
        %       % Run example code
        %       for i = 1:numel(exampleCode)
        %           evalc(exampleCode{:});
        %       end
        %
        %       % Generate a trajectory which itself contains an intersection
        %       f2 = f1;
        %       f2(1) = refPath.SegmentParameters(4,end) + 20;
        %
        %       % Set the states such that it starts from rest and
        %       % terminates with a positive velocity and no acceleration
        %       f0(2) = 0;                          % Initial lon. speed set to 0
        %       f2(2) = 5;                          % Terminal lon. speed set to 5
        %       deltaS = (f2(1)-f0(1));             % Longitudinal distance traveled
        %       vAvgEstimate = (f2(2)-f0(2))/2;     % Rough average lon. velocity estimate
        %       T2 = deltaS/vAvgEstimate;           % Ballpark travel duration
        %
        %       % Generate trajectory
        %       [fIntersecting,gIntersecting] = connect(generator,f0,f2,T2);
        %       gXingPts = gIntersecting.Trajectory;
        %
        %       % Find closest points across trajectory
        %       closestPts = closestPoint(refPath, gXingPts);
        %
        %       % Find closest points in window that spans the full
        %       % trajectory length
        %       windowBuffer = 5;
        %       fXingPts = fIntersecting.Trajectory;
        %       fixedWindow = [min(fXingPts(:,1))-windowBuffer max(fXingPts(:,1))+windowBuffer];
        %       closestPtsFullWindow = closestPoint(refPath, gXingPts, fixedWindow);
        %
        %       % Calculate a sliding window size based on the max global
        %       % velocity. Note that this may not cover the trajectory
        %       % depending on the shape of the reference path.
        %       dsApprox = max(abs(gXingPts(:,5)))*generator.TimeResolution;
        %       initialWindow = fXingPts(1)+[-1 1]*dsApprox;
        %       [closestPtsSlidingWindow, inWindow] = closestPointsToSequence(refPath, gXingPts, initialWindow);
        %       
        %       % Calculate the region swept by the sliding window
        %       sweptRegion = [closestPtsSlidingWindow(end-1,end)-dsApprox, ...
        %           closestPtsSlidingWindow(end-1,end)+dsApprox];
        %       finalWindow = [closestPtsSlidingWindow(1,end)-dsApprox, ...
        %           closestPtsSlidingWindow(end-1,end)+dsApprox];
        %
        %       % Display results
        %       figure;
        %       show(refPath); axis equal; hold on;
        %       title("Closest Points Along Self-Intersecting Trajectory");
        %       plotInterval(fixedWindow,100,{'Color',[.5 .5 .5],'LineWidth',5});
        %       plotInterval(sweptRegion,100,{'k','LineWidth',5});
        %       plotInterval(finalWindow,100,{':','Color',[.25 .25 .25],'LineWidth',3});
        %       plotFcn(gXingPts,[],{'k.-'});
        %       plotFcn(gXingPts,closestPts,{'b','LineWidth',3});
        %       plotFcn(gXingPts,closestPtsFullWindow,{'m','LineWidth',2})
        %       plotFcn(gXingPts(inWindow,:),closestPtsSlidingWindow(inWindow,:),{'g'});
        %       plotFcn(gXingPts(~inWindow,:),closestPtsSlidingWindow(~inWindow,:),{'r'});
        %       legend({'Waypoints','ReferencePath','FixedWindow', ...
        %           'SlidingWindowSweptRegion','SlidingWindowFinalSpan', ...
        %           'Trajectory', 'ClosestPoint','ClosestInsideFixedWindow', ...
        %           'ClosestInsideSlidingWindow'});
        % 
        %   See also closestPoint, closestProjections
            
            % Validate inputs
            narginchk(3,3);
            xy = obj.validateXYPoints(points,'closestPointsToSequence','points');
            clippedWindow = obj.validateSearchWindow(initialWindow,'closestPointsToSequence','initialWindow');
            halfSpan = (initialWindow(2)-initialWindow(1))/2;
            
            % Allocate output
            n = size(xy,1);
            pathPoints = zeros(n,6);
            
            if nargout == 2
                inWindow = false(n,1);
            end
            
			arclimits = [0 obj.PathLength];

            for i = 1:size(xy,1)
                % Search in window
                if nargout == 2
                    [pathPoints(i,:), ~, inWindow(i)] = obj.searchInWindow(xy(i,:),clippedWindow,false);
                else
                    pathPoints(i,:) = obj.searchInWindow(xy(i,:),clippedWindow,false);
                end
                clippedWindow = obj.clipWindow(arclimits, pathPoints(i,end)+[-halfSpan halfSpan]);
            end
        end
        
        function [arclengths,distances,projPoints] = closestProjections(obj, points, varargin)
        %closestProjections Find orthogonal projections between path tangent vector and query point
        %
        %   NOTE: This method will only return path points whose normal 
        %   vector intersects the query point(s).
        %
        %   [ARCLENGTHS,DISTANCES] = closestNProjections(OBJ,POINTS) 
        %   attempts to project each XY point in the M-row POINTS matrix 
        %   onto each clothoid segment contained in OBJ such that the 
        %   projection vector is orthogonal to the path tangent-angle. 
        %   Returns two M-by-1 cell-arrays, ARCLENGTHS and DISTANCES, where
        %   each cell may contain a P-element column vector where 0<=P<=N
        %   (by default, N is the number of segments in the path). The
        %   i'th pair of cells contain the arclengths which form the closest
        %   orthogonal projection between the curve and query point in each
        %   segment (ARCLENGTHS{i}), and the corresponding distance 
        %   (DISTANCES{i}).
        %
        %   [___, PROJPOINTS] = closestNProjections(OBJ,POINTS) optionally
        %   returns the M-by-1 cell array, PROJPOINTS, where the i'th cell
        %   contains a P-by-[x y theta k dk s] matrix corresponding to the
        %   path evaluated at ARCLENGTHS{i}.
        %
        %   [___] = closestNProjections(OBJ,POINTS,BESTN) returns up to
        %   the nearest BESTN projections for each point, where BESTN is a 
        %   scalar 1<=BESTN<=N.
        %
        %   [___] = closestNProjections(OBJ,POINTS,INTERVALS) takes in an
        %   optional Nx2 matrix of arclengths, INTERVALS, where each row
        %   contains [min, max] arclengths. Each interval will return 
        %   at most one closest point if the segment contains valid 
        %   projections, 0<=P<=size(intervals,1).
        %
        %   [___] = closestNProjections(OBJ,POINTS,INTERVALS,BESTN) 
        %   returns up to the nearest BESTN projections for each XY point, 
        %   0<=P<=BESTN<=size(intervals,1).
        %
        %   Example:
        %
        %     % Create a reference path with multiple switch-backs
        %     leftSideAngles  = [linspace(-pi/6,pi/6,4) linspace(pi/6,-pi/6,4)]';
        %     rightSideAngles = [linspace(-pi/6,pi/6,4) linspace(-pi/6,pi/6,4)]';
        %     waypoints = zeros(numel(leftSideAngles)*2,3);
        %     width = 10;
        %     height = 20;
        %     waypoints(1:2:end,:) = [zeros(numel(leftSideAngles),1) linspace(0,height,numel(leftSideAngles))' leftSideAngles]; ...
        %     waypoints(2:2:end,:) = [width*ones(numel(leftSideAngles),1) linspace(0,height,numel(leftSideAngles))' rightSideAngles];
        %     refPath = referencePathFrenet(waypoints);
        %     
        %     % Create a set of random XY points around the path
        %     queryPoints = [width height]/2+(rand(10,2)-.5).*[width height]*1.5;
        %     
        %     % Retrieve the nearest valid projection of each query point on each segment
        %     % in the path.
        %     [allArclenth,allDistance,allProjection] = closestProjections(refPath,queryPoints);
        %     pLength = refPath.PathLength;
        %     breaks = [refPath.SegmentParameters(:,end); pLength];
        %     allInterval = [breaks(1:end) [breaks(2:end); pLength]];
        %     
        %     % Return the three best projections
        %     maxResult = 3;
        %     [best3Arclength,best3Distance,best3Projection] = closestProjections(refPath,queryPoints,maxResult);
        %     
        %     % Define a custom set of arclength-intervals.
        %     everyThreeMerged = [breaks(1:3:end-1) [breaks(4:3:end-1); breaks(end)]];
        %     
        %     % Find the best projection of each query-point onto each custom interval 
        %     % (if one exists).
        %     [allArclengthCustom,allDistanceCustom,allProjectionCustom] = ...
        %         closestProjections(refPath,queryPoints,everyThreeMerged);
        %     
        %     % Return the single best projection in the first and last quarter of the path
        %     endQuarterIntervals = [0 1/4; 3/4 1]*refPath.PathLength;
        %     [bestQuarterArclength,bestQuarterDistance,bestQuarterProjection] = ...
        %         closestProjections(refPath,queryPoints,endQuarterIntervals, 1);
        %     
        %     % Display results
        %     % Pack iterable containers
        %     intervalSets = {allInterval, allInterval, everyThreeMerged, endQuarterIntervals};
        %     S = {allArclenth best3Arclength allArclengthCustom bestQuarterArclength};
        %     D = {allDistance best3Distance allDistanceCustom bestQuarterDistance};
        %     PP = {allProjection best3Projection allProjectionCustom bestQuarterProjection};
        %     titles = ["All Projections","Best 3, All Segments","Best In Merged Segments","First vs Last Quarter"];
        %     cOrder = colororder;
        %     
        %     % Define helper functions
        %     mergeFcn = @(v1,v2)reshape([v1 v2 nan(size(v1,1),size(v2,2))]',[],1);
        %     plotFcn = @(L1,L2,linespec)plot(mergeFcn(L1(:,1),L2(:,1:min(1,size(L2,2)))), ...
        %         mergeFcn(L1(:,2),L2(:,2:min(2,size(L2,2)))),linespec{:});
        %     intervalPlotter = @(bounds,nPt,linespec)plotFcn(interpolate(refPath,linspace(bounds(1),bounds(2),nPt)'),[],linespec);
        %     
        %     % Create in-loop handles
        %     setupFcns = {};
        %     setupFcns{1} = @(figIdx)hold(show(refPath,'Parent',subplot(2,2,figIdx)),'on'); 
        %     setupFcns{2} = @(figIdx)axis(subplot(2,2,figIdx),'equal');
        %     setupFcns{3} = @(figIdx)title(subplot(2,2,figIdx),titles(figIdx));
        %     setupFcns{4} = @(figIdx)plotFcn(queryPoints,[],{'Xk','MarkerSize',5});
        %     setupFcns{5} = @(figIdx)arrayfun(@(i)intervalPlotter(intervalSets{figIdx}(i,:),100,{'Color',cOrder(mod(i,size(cOrder,1)-1)+1,:),'LineWidth',2'}),1:size(intervalSets{figIdx},1));
        %     setupFcns{6} = @(figIdx)cellfun(@(p,projPts)plotFcn(repmat(queryPoints(p,:),size(projPts,1),1),projPts,{'Color',cOrder(mod(p,size(cOrder,1)-1)+1,:)}),num2cell(1:size(queryPoints,1))',PP{figIdx});
        %     
        %     % Display results
        %     arrayfun(@(idx)cellfun(@(f)f(idx),setupFcns),1:4)
        %
        %   See also closestPoint, closestPointsToSequence

            % Parse and validate inputs
            narginchk(2,4);
            coder.internal.prefer_const(varargin);
            [xy, clippedIntervals, maxNumProj] = obj.parseClosestProjection(points, varargin{:});

            numPts = size(xy,1);
            projPoints_tmp  = repmat({zeros(maxNumProj,6)},numPts,1);
            S_tmp           = repmat({zeros(maxNumProj,1)},numPts,1);
            D_tmp           = repmat({zeros(maxNumProj,1)},numPts,1);
            dMax            = inf(numPts,1);
            curNumProj      = repmat({0},numPts,1);
            
            % Find nearest point within each section
            for seg = 1:size(clippedIntervals,1)
                sWindow = clippedIntervals(seg,:);
                [pp, dist, inRange] = obj.searchInWindow(xy,sWindow,true);
                
                for i = 1:numPts
                    % Update list of best projections for each point
                    if inRange(i)
                        [S_tmp{i},D_tmp{i},projPoints_tmp{i},curNumProj{i},dMax(i)] = obj.updateRecords(...
                            S_tmp{i}, D_tmp{i}, projPoints_tmp{i}, dMax(i), curNumProj{i}, dist(i), pp(i,:), maxNumProj);
                    end
                end
            end

            % Only return the valid projections
            arclengths = cell(numPts,1);
            distances = cell(numPts,1);
            projPoints = cell(numPts,1);
            
            for i = 1:numPts
                arclengths{i} = S_tmp{i}(1:curNumProj{i},1);
                distances{i} = D_tmp{i}(1:curNumProj{i},1);
                projPoints{i} = projPoints_tmp{i}(1:curNumProj{i},:);
            end
        end
        
        function pathPoint = interpolate(obj, arcLength)
        %interpolate Evaluate path at provided arclength
        %
        %   PATHPOINT = interpolate(OBJ,ARCLENGTH) samples the path at 
        %   the provided arc lengths. ARCLEGNTH is an N-element vector of
        %   distances along the reference path. PATHPOINT is in global
        %   coordinates, [x y theta kappa dkappa s].
        
            % Validate number of input arguments
            narginchk(2,2);
            validateattributes(arcLength,{'numeric'},{'nonempty','nonnan','finite','vector'},'interpolate','arcLength');
            pathPoint = obj.interpolate@nav.algs.internal.FrenetReferencePath(arcLength(:));
        end

        function xy = position(obj,s)
        %position Return XY location at arclength
        %
        %   XY = position(OBJ,S) Calculates the XY position of the 
        %   reference path at given arclength, S, where XY and S are Nx2 
        %   and Nx1 vectors, respectively.
            narginchk(2,2)
            idx = validateAndDiscretizeArclengths(obj, s, 'position');
            segStarts = obj.PathManager.SegStarts;
            xy = zeros(numel(s),2);
            for n = 1:numel(idx)
                i = idx(n);
                ds = s(n)-segStarts(i,end);
                [xy(n,1),xy(n,2)] = obj.clothoid(segStarts(i,1),segStarts(i,2),...
                    segStarts(i,3),segStarts(i,4),segStarts(i,5),ds);
            end
        end
        
        function theta = tangentAngle(obj,s)
        %tangentAngle Return tangent angle at arclength
        %
        %   THETA = tangentAngle(OBJ,S) Efficiently calculates the tangent 
        %   angle, THETA, of the reference path at a given arclength, S, 
        %   where THETA and S are Nx1 vectors.
            narginchk(2,2)
            idx = validateAndDiscretizeArclengths(obj, s, 'tangentAngle');
            TH0 = obj.PathManager.SegStarts(idx,3);
            K0 = obj.PathManager.SegStarts(idx,4);
            DK0 = obj.PathManager.SegStarts(idx,5);
            S0 = obj.PathManager.SegStarts(idx,6);
            n = numel(s);
            theta = zeros(n,1);
            
            for i = 1:n
                th0 = TH0(i);
                k0 = K0(i);
                dk = DK0(i);
                s0 = S0(i);
                L = s(i)-s0;
                theta(i) = dk/2*L*L + k0*L + th0;
            end
            theta = robotics.internal.wrapToPi(theta);
        end
        
        function k = curvature(obj,s)
        %curvature Return curvature at arclength
        %
        %   K = curvature(OBJ,S) Efficiently calculates the curvature, K,
        %   of the reference path at a given arclength, S, where K and S
        %   are Nx1 vectors.
            narginchk(2,2)
            idx = validateAndDiscretizeArclengths(obj, s, 'curvature');
            k0 = obj.PathManager.SegStarts(idx,4);
            dk = obj.PathManager.SegStarts(idx,5);
            s0 = obj.PathManager.SegStarts(idx,6);
            n = numel(s);
            k = zeros(n,1);
            for i = 1:n
                k(i) = k0(i)+dk(i).*(s(i)-s0(i));
            end
        end

        function dk = changeInCurvature(obj,s)
        %changeInCurvature Return change-in-curvature at arclength
        %
        %   DK = changeInCurvature(OBJ,S) Efficiently calculates the 
        %   change-in-curvature with respect to change-in-arclength, DK, 
        %   of the reference path at a given arclength, S, where DK and S
        %   are Nx1 vectors.
            narginchk(2,2)
            idx = validateAndDiscretizeArclengths(obj, s, 'changeInCurvature');
            dk = obj.PathManager.SegStarts(idx,5);
        end
        
        function S = get.PathLength(obj)
        %PathLength Total arclength of the path
            S = obj.Length;
        end
        
        function cParams = get.SegmentParameters(obj)
        %SegmentParameters Clothoid parameters at start of segments
            cParams = obj.PathManager.SegStarts(1:end-1,:);
        end

        function globalState = frenet2global(obj, varargin)
        %frenet2global Convert Frenet states to global states
        %
        %   GLOBALSTATE = frenet2global(OBJ,FRENETSTATE) converts Frenet
        %   trajectory states to global states.
        %
        %       FRENETSTATE - [S Sdot Sddot L dL/dS ddL/dS^2]
        %       GLOBALSTATE - [x y theta kappa speed accel]
        %
        %   Derivatives of S are with respect to time and derivatives of L
        %   are with respect to arclength, S.
        %
        %   GLOBALSTATE = frenet2global(OBJ,FRENETSTATE,LATERALTIMEDERIVATIVES)
        %   optionally accepts LATERALTIMEDERIVATIVES, an Nx3 matrix:
        % 
        %       LATERALTIMEDERIVATIVES - [dL/dt ddL/dt^2 invertHeading]
        % 
        %   Each row of LATERALTIMEDERIVATIVES contains the 1st/2nd order 
        %   derivatives of lateral deviation with respect to time and a 
        %   flag, invertHeading, indicating whether the heading should be 
        %   flipped when converting back to global coordinates (true).
        %
        %   NOTE: When defining LATERALTIMEDERIVATIVES without the use of
        %   global2frenet, the following rules should be followed:
        %       1) The invertHeading flag should be true when:
        %           a) The vehicle is moving in reverse (speed < 0)
        %           b) The vehicle is stationary (speed == 0) and the 
        %              vehicle is facing away from the path's tangent vector
        %              i.e cos(|tangentAngle(obj,S)-thetaExpected|) < 0
        %       2) If 1b is true, dL/dS must be negated
            
            % Validate number/type of input arguments
            narginchk(2,3);

            % Validate frenet states
            validateattributes(varargin{1},{'numeric'},{'nonnan','nonempty','finite','size',[nan 6]},'frenet2global','frenetState');
            
            if nargin == 3
                % Validate lateral time derivatives and heading invert
                validateattributes(varargin{2},{'numeric'},{'nonnan','finite','nonempty','size',[size(varargin{1},1) 3]},'frenet2global','latTimeDerivs');
                validateattributes(varargin{2}(:,3),{'numeric'},{'binary'},'frenet2global','invertHeadingTF');
            end

            % Convert frenet states to global coordinate frame
            globalState = obj.frenet2global@nav.algs.internal.FrenetReferencePath(varargin{:});
        end
        
        function [frenetState, lateralTimeDerivatives] = global2frenet(obj, varargin)
        %global2frenet Convert global states to Frenet states
        %
        %   FRENETSTATE = global2frenet(OBJ,GLOBALSTATE) converts global
        %   trajectory states to Frenet states.
        %
        %       GLOBALSTATE - [x y theta kappa speed accel]
        %       FRENETSTATE - [S Sdot Sddot L dL/dS ddL/dS^2]
        %
        %   Derivatives of S are with respect to time and derivatives of L
        %   are with respect to arclength, S.
        %
        %   [___,LATERALTIMEDERIVATIVES] = global2frenet(obj,GLOBALSTATE)
        %   returns LATERALTIMEDERIVATIVES, an Nx3 matrix:
        % 
        %       LATTIMEDERIVATIVES - [dL/dt ddL/dt^2 invertHeading]
        % 
        %   Each row of LATTIMEDERIVATIVES contains the 1st/2nd order 
        %   derivatives of lateral deviation with respect to time and a 
        %   flag, invertHeading, indicating whether the heading should be 
        %   flipped when converting back to global coordinates (true).
        %
        %   [___] = global2frenet(obj,GLOBALSTATE,SFRAME) Optionally 
        %   takes in an N-element vector of arclengths, SFRAME, at which the 
        %   Frenet frame should be situated. For correct results, the 
        %   vector between the frame and GLOBALSTATEs' XY location must be 
        %   orthogonal to the tangent angle.
            
            % Validate number/type of input arguments
            narginchk(2,3);

            % Validate globalState
            validateattributes(varargin{1},{'numeric'},{'nonnan','nonempty','finite','size',[nan 6]},'global2frenet','globalPoint');
            
            if nargin == 3
                % Validate arclengths at which the Frenet frame will be
                % placed.
                validateattributes(varargin{2},{'numeric'},{'nonnan','finite','nonnegative','vector','numel',size(varargin{1},1)},'global2frenet','sFrame');
            end

            % Convert global states to frenet coordinate frame
            if nargout == 2
                [frenetState, lateralTimeDerivatives] = obj.global2frenet@nav.algs.internal.FrenetReferencePath(varargin{:});
            else
                frenetState = obj.global2frenet@nav.algs.internal.FrenetReferencePath(varargin{:});
            end
        end
    end

    methods (Access = protected)
        function [pathPoints, dist, INWINDOW] = searchInWindow(obj, xy, searchWindow, mustBeProjection)
        %searchInWindow Find closest point or projection within search window
            
            % Determine initial and final indices
            i0 = discretize(searchWindow(:,1),[-inf;obj.PathManager.SegStarts(2:end-1,end);inf],'IncludedEdge','left');
            i1 = discretize(searchWindow(:,2),[-inf;obj.PathManager.SegStarts(2:end-1,end);inf],'IncludedEdge','left');
            
            % Grab path info for region of interest
            ss = obj.PathManager.SegStarts(i0:i1,:);
            bboxes = obj.PathManager.BoundingBoxes(:,i0:i1);
            
            % Recalculate lower bound
            [ss(1,1),ss(1,2),ss(1,3),ss(1,4)] = ...
                obj.clothoid(ss(1,1),ss(1,2),ss(1,3),ss(1,4),ss(1,5),...
                searchWindow(1)-ss(1,end));
            ss(1,end) = searchWindow(1);
            
            % Search within bounds
            if coder.target('MATLAB')
                [pathPoints,dist,~,~] = nav.algs.internal.mex.nearestPointIterative(...
                    xy, ss, bboxes, searchWindow(2), mustBeProjection);
            else
                [pathPoints,dist,~,~] = nav.algs.internal.impl.nearestPointIterative(...
                    xy, ss, bboxes, searchWindow(2), mustBeProjection);
            end

            if nargout == 3
                if mustBeProjection
                    % Any points returned by nearestPointIterative when
                    % mustBeProjection is true qualify as valid projections
                    INWINDOW = ~isnan(pathPoints(:,end));
                else
                    % Verify non-projection points
                    INWINDOW = verifyInWindow(obj,xy,pathPoints,searchWindow);
                end
            end
        end

        function INWINDOW = verifyInWindow(obj,xy,pathPoints,searchWindow)
        %verifyInWindow Verifies whether closest points returned also qualify as projections
        %
        %   First checks whether any points lie on the interval boundary.
        %   Any points close to the boundary are checked for orthogonality.
            
            % Point lies within bound
            INWINDOW = all(pathPoints(:,end) >= (searchWindow(1)+sqrt(eps)) & pathPoints(:,end) <= (searchWindow(2)-sqrt(eps)),2);
            if any(~INWINDOW)
                % Check perpendicularity
                pp = pathPoints(~INWINDOW,:);
                v = xy(~INWINDOW,:)-pp(:,1:2);
                t = [cos(pp(:,3)) sin(pp(:,3))];
                INWINDOW(~INWINDOW) = abs(dot(t,v./vecnorm(v,2,2),2)) > (1-sqrt(eps));
            end
        end
        
        function idx = validateAndDiscretizeArclengths(obj, s, methodName)
        %validateAndDiscretizeArclengths Validate arclengths and identify the segment to which they belong
            validateattributes(s,{'numeric'},{'column','nonnan','finite'},methodName,'s');
            idx = discretize(s,obj.PathManager.SegStarts(:,end),'IncludedEdge','left');
        end

        function [S,D,PTS,curNumProj,dMax] = updateRecords(obj, S, D, PTS, dMax, curNumProj, d, pp, maxNumProj)
        %updateRecords Add new projection to list or replace worst if list is full

            if curNumProj < maxNumProj
            % Add projection to results
                curNumProj = curNumProj+1;
                S(curNumProj,1) = pp(end);
                D(curNumProj,1) = d;
                PTS(curNumProj,:) = pp;
                if curNumProj == maxNumProj
                    dMax = max(D);
                end
            else
                if d < dMax
                % Find current max point and replace entry
                    [~,idx] = max(D);
                    D(idx,1) = d;
                    S(idx,1) = pp(end);
                    PTS(idx,:) = pp;
                end
                dMax = max(D);
            end
        end

        function xy = validateXYPoints(obj,points,fcnName,varName)
        %validateXYPoints Validate incoming points
            validateattributes(points, {'numeric'}, {'nonnan','finite','nonempty','2d'},fcnName,varName);
            sz = size(points,2);
            coder.internal.assert(sz>=2, 'nav:navalgs:referencepathfrenet:XYNumCol');

            if sz == 2
                xy = points;
            else
                xy = points(:,1:2);
            end
        end

        function searchWindow = validateSearchWindow(obj, searchWindow, fcnName, varName, numWindows)
        %validateSearchWindow Verify searchWindow attributes and clip window to path limits
            if nargin == 4
	            validateattributes(searchWindow, {'numeric'}, ...
		            {'row','numel',2,'finite','increasing'}, ...
		            fcnName,varName);
            else
	            validateattributes(searchWindow, {'numeric'}, ...
		            {'size',[numWindows 2],'finite'}, ...
		            fcnName,varName);
	            coder.internal.assert(all(diff(searchWindow,[],2)>=0),...
		            'nav:navalgs:referencepathfrenet:NonDecreasingIntervals');
            end
            searchWindow = obj.clipWindow([0 obj.PathLength],searchWindow);
        end

        function [xy, clippedIntervals, maxNumProj, defaultIntervals] = parseClosestProjection(obj, points, varargin)
            xy = obj.validateXYPoints(points,'closestProjections','points');
            
            switch nargin
                case 2
                    breaks = obj.PathManager.SegStarts(:,end);
                    intervals = [breaks(1:end-1) breaks(2:end)];
                    maxNumProj = size(intervals,1); % Subtract one due to extra line in segstarts
                    defaultIntervals = true;
                case 3
                    if coder.internal.isConstTrue(isscalar(varargin{1}))
                        % Treat 3rd input as number of possible distances
                        % returned
                        breaks = obj.PathManager.SegStarts(:,end);
                        intervals = [breaks(1:end-1) breaks(2:end)];
                        maxNumProj = varargin{1};
                        defaultIntervals = true;
                    else
                        intervals = varargin{1};
                        maxNumProj = size(intervals,1);
                        defaultIntervals = false;
                    end
                case 4
                    intervals = varargin{1};
                    maxNumProj = varargin{2};
                    defaultIntervals = false;
            end
            
            % Validate intervals
            clippedIntervals = obj.validateSearchWindow(intervals,'closestProjections','intervals',size(intervals,1));

            % Validate number of requested projections
            validateattributes(maxNumProj,{'numeric'},{'integer','positive','<=',size(intervals,1)},'closestProjections','N');
        end
    end

	methods (Static, Hidden)
		function searchWindow = clipWindow(arclimits,searchWindow)
		%clipWindow Constrain window to path limits
			searchWindow(searchWindow < arclimits(1)) = arclimits(1);
			searchWindow(searchWindow > arclimits(2)) = arclimits(2);
        end
    end
end

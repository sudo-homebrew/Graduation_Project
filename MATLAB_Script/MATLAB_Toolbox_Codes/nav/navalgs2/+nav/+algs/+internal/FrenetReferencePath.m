classdef FrenetReferencePath < nav.algs.internal.InternalAccess
%This class is for internal use only. It may be removed in the future.

%FrenetReferencePath Generate a reference path using waypoints
%   FrenetReferencePath It fits a parametric curve over the waypoints
%   and generate a discrete set of states which contains x, y, theta,
%   kappa, dkappa and s

%   Copyright 2019-2021 The MathWorks, Inc.

%#codegen
    properties(Access=?nav.algs.internal.InternalAccess)
        %PathPoints N-by-6 array of discretized states representing the path
        %   N-by-6 array of states on path in the following order [x, y,
        %   theta, kappa, dkappa, s]
        PathPoints
        
        %PointsDirty Dirty flag for visualization
        %
        %   This class only stores a discrete representation of the curve
        %   if the show method is called. Any time the waypoints are set, the
        %   flag is set to true.
        PointsDirty = true;
    end
    properties(Hidden, Constant)
        %DefaultStepSize Default distance between discretized points along path
        DefaultStepSize = 0.05;
        
        %NVPairDefaults Default values for name-value pair inputs
        NVPairDefaults = struct(...
            'DiscretizationDistance', 0.05, ...
            'MaxNumWaypoints', inf);
    end
    properties(SetAccess = immutable)
        %DiscretizationDistance Arclength used to sampled points during visualization
        DiscretizationDistance = nav.algs.internal.FrenetReferencePath.DefaultStepSize;
    end
    
    properties (Dependent)
        %Waypoints Presampled points along the path
        Waypoints
        
        %MaxNumWaypoints Maximum waypoints allowed in path
        %
        %   Accepts a scalar which determines the maximum number of 
        %   waypoints accepted by this object. 
        %
        %       Inf           - The path is explicitly resizeable.
        %       (Default)
        %
        %                       NOTE: Requires DynamicMemoryAllocation='On'
        %                             for codegen
        %
        %       VALUE         - The maximum number of waypoints in the
        %                       object is limited to VALUE. Use this to 
        %                       create a resizeable path when 
        %                       DynamicMemoryAllocation is not allowed 
        %                       during codegen.
        MaxNumWaypoints
    end
    
    properties(Access = ?nav.algs.internal.InternalAccess)
        %PathHandle Graphics handle to the patch that represents the reference path
        PathHandle
        
        %WaypointsHandle Graphics handle to the patch that represents the waypoints along refPath
        WaypointsHandle
        
        %Length Total length of the path
        Length
        
        %PathManager Class responsible for managing waypoint data
        PathManager
    end
    
    methods
        function obj = FrenetReferencePath(waypoints, varargin)
            % Validate number of input arguments
            narginchk(1,5);
            validateattributes(waypoints,{'numeric'},{},'FrenetReferencePath','waypoints');
            
            % Parse inputs
            nvPairs = nav.algs.internal.FrenetReferencePath.parseConstructor(varargin{:});
            
            % Set discretization step size
            obj.DiscretizationDistance = nvPairs.DiscretizationDistance;
            
            % Initialize path manager
            if coder.target('MATLAB')
                % Varsize always allowed in MATLAB
                obj.PathManager = nav.algs.internal.PathManagerFlexible(waypoints, nvPairs.MaxNumWaypoints);
            else
                % Check if varsize is allowed
                VARSIZE_ENABLED = coder.const(strcmp(eml_option('VariableSizing'),'DisableInInference'));
                
                if VARSIZE_ENABLED || ~isfinite(nvPairs.MaxNumWaypoints)
                    % The flexible manager will be used if:
                    %   1) Varsize is allowed
                    %   2) Varsize is not allowed, but the user hasn't
                    %   provided an upper bound on the size. If the size of
                    %   waypoints is compile-time constant, the manager 
                    %   will simply become a fixed-size path, otherwise
                    %   Coder will throw an error when attempting to assign
                    %   the varsize input to the object property.
                    obj.PathManager = nav.algs.internal.PathManagerFlexible(waypoints, nvPairs.MaxNumWaypoints);
                else
                    % The fixed-size manager is only used when
                    % DynamicMemoryAllocation is turned off and the user
                    % has provided an upper-bound to allocate.
                    obj.PathManager = nav.algs.internal.PathManagerFixed(waypoints, nvPairs.MaxNumWaypoints);
                end
            end
            
            % Fit path to waypoints
            obj.Waypoints = waypoints;
        end

        function pathPoints = interpolate(obj, arcLength)
        %interpolate Evaluate path at provided arclength
        %   Find the closest path point on the reference path at the
        %   input arc length and use previous index value for searching
        %   in a window from prevIdx
            
            if coder.target('MATLAB')
                pathPoints = nav.algs.internal.mex.pathInterpolate(obj.PathManager.SegStarts, arcLength(:));
            else
                pathPoints = nav.algs.internal.impl.pathInterpolate(obj.PathManager.SegStarts, arcLength(:));
            end
        end
        
        function pathPoint = closestPoint(obj, xyPoints)
        %closestPoint Find the closest point along path from (x,y) coordinate
        %   Find the closest path point on the reference path from the
        %   input point (x,y)

            pathPoint = obj.nearestPoint(xyPoints(:,1:2));
        end
        
        function [frenetState, latDerivs] = global2frenet(obj, globalStates, sFrame)
        %global2frenet Convert global states to Frenet states
            
            if nargin == 2
                % Get path point on reference path closest to given input state
                refPathPoints = obj.closestPoint(globalStates(:,1:2));
            else
                % Evaluate path at arclength provided
                refPathPoints = obj.interpolate(sFrame);
            end
            
            % Transform to Frenet states using path point as reference
            [frenetState, latDerivs] = matlabshared.planning.internal.CartesianFrenetConversions.cartesian2Frenet(refPathPoints, globalStates);
        end

        function globalStates = frenet2global(obj, frenetStates, latDerivs)
        %frenet2global Convert Frenet states to global states
            
            % Get path point on reference path closest to given input state
            refPathPoint = obj.interpolate(frenetStates(:,1));
            
            % Transform to global states using path point as reference
            if nargin == 3
                if coder.target('MATLAB')
                    globalStates = matlabshared.planning.internal.CartesianFrenetConversions.mex.frenet2Cartesian360(refPathPoint, frenetStates, latDerivs);
                else
                    globalStates = matlabshared.planning.internal.CartesianFrenetConversions.impl.frenet2Cartesian360(refPathPoint, frenetStates, latDerivs);
                end
            else
                globalStates = matlabshared.planning.internal.CartesianFrenetConversions.frenet2Cartesian(refPathPoint, frenetStates);
            end
        end

        function axisHandle = show(obj, varargin)
        %SHOW Show the reference path in a figure
        %   show(OBJ) shows the reference path and its lateral states
        %
        %   axisHandle = show(OBJ) returns the handle of the axes used
        %   by the show function.
        %
        %   show(___,Name,Value) provides additional options specified
        %   by one or more Name,Value pair arguments. Name must appear
        %   inside single quotes (''). You can specify several name-value
        %   pair arguments in any order as Name1,Value1,...,NameN,ValueN:
        %
        %       'Parent'        - Handle of an axes that specifies
        %                         the parent of the objects created by
        %                         show.
        %
        %   Example:
        %
        %     % Visualize the reference path
        %     show(OBJ);
        %
        %   See also trajectoryOptimalFrenet, trajectoryGeneratorFrenet

            % Check for number of input arguments
            narginchk(1,3);

            parser = robotics.core.internal.NameValueParser({'Parent'}, {[]});
            parse(parser, varargin{:});
            axisHandle = parameterValue(parser, 'Parent');

            % Create a new axes if not assigned
            if isempty(axisHandle)
                axisHandle = newplot;
            else
                robotics.internal.validation.validateAxesUIAxesHandle(axisHandle);
            end

            hold(axisHandle, 'on')

            % Get children of current plot
            children = axisHandle.Children;
            
            if obj.PointsDirty
                if mod(obj.Length,obj.DiscretizationDistance)==0
                    obj.PathPoints = obj.interpolate(0:obj.DiscretizationDistance:obj.Length);
                else
                    obj.PathPoints = obj.interpolate([0:obj.DiscretizationDistance:obj.Length obj.Length]);
                end
            end
            
            % Plot the reference path
            if isempty(obj.PathHandle) || ~any(obj.PathHandle == children)
                hold(axisHandle, 'on')
                % Plot the waypoints
                obj.WaypointsHandle = plot(axisHandle, obj.Waypoints(:,1),obj.Waypoints(:,2),'ro','MarkerFaceColor', 'r','Tag','Waypoints');
                % Plot the reference path
                obj.PathHandle = plot(axisHandle, obj.PathPoints(:,1),  obj.PathPoints(:,2),'r','Tag','Reference Path');
                axisHandle.Children(1).DataTipTemplate.DataTipRows(3) = dataTipTextRow('Curvature',obj.PathPoints(:,4));
                hold(axisHandle, 'off')
            else
                % Update Waypoints
                set(obj.WaypointsHandle, 'XData', obj.Waypoints(:,1), 'YData', obj.Waypoints(:,2), 'Visible', 'on');
                set(obj.PathHandle, 'XData', obj.PathPoints(:,1), 'YData', obj.PathPoints(:,2), 'Visible', 'on');
                obj.PathHandle.DataTipTemplate.DataTipRows(3) = dataTipTextRow('Curvature',obj.PathPoints(:,4));
            end
        end
        
        function waypoints = get.Waypoints(obj)
            waypoints = obj.PathManager.Waypoints;
        end
        function set.Waypoints(obj, waypoints)
            obj.PathManager.Waypoints = waypoints;
            obj.generateStates(waypoints);
        end
        function numWaypoints = get.MaxNumWaypoints(obj)
            numWaypoints = obj.PathManager.MaxNumWaypoints;
        end
        function set.MaxNumWaypoints(obj, numPts)
            obj.PathManager.MaxNumWaypoints = numPts;
        end
    end
    
    methods (Access = ?nav.algs.internal.FrenetReferencePath)
        function generateStates(obj, waypoints)
            %generateStates Generate path points from waypoints
            %   generateStates fits a clothoid on the
            %   waypoints and computes poses, curvature and curvature
            %   derivative
            
            % Fit a clothoid over waypoints
            switch size(waypoints,2)
                case 2
                    xy = waypoints;
                    obj.fitClothoid(xy);
                case 3
                    xy = waypoints(:,1:2);
                    th = waypoints(:,3);
                    obj.fitClothoid(xy, th);
                otherwise
                    coder.internal.error('nav:navalgs:referencepathfrenet:IncorrectNumCols')
            end
            
            % Segregate sampled states of fitted clothoid
            obj.PathManager.BoundingBoxes = obj.calculateBoundingBoxes(waypoints);
            
            % Mark PathPoints as being dirty for visualization
            obj.PointsDirty = true;
        end
        
        function fitClothoid(obj, waypoints, courseInput)
        %fitClothoid Fits a clothoid on the waypoints and computes states

            if nargin == 2
                % Find the heading at each waypoint
                course = matlabshared.tracking.internal.scenario.clothoidG2fitCourse(waypoints);
            else
                course = courseInput;
            end
        
            n = size(waypoints,1);
            
            % Obtain the initial positions
            initialPosition = complex(waypoints(:,1), waypoints(:,2));
            arcLengths = zeros(n,1);
                
            % Calculate starting curvature, final curvature, and length of each segment.
            [initialCurvature, finalCurvature, arcLengths(1:end-1)] = matlabshared.tracking.internal.scenario.clothoidG1fit2(initialPosition(1:n-1),course(1:n-1),initialPosition(2:n),course(2:n));
            
            % Store the path conditions at each waypoint and the arclength
            % of each clothoid segment.
            cumulativeLength = cumsum(circshift(arcLengths,1));
            segStarts = [real(initialPosition) imag(initialPosition) robotics.internal.wrapToPi(course) [initialCurvature; finalCurvature(end)] [(finalCurvature-initialCurvature)./arcLengths(1:end-1); (finalCurvature(end)-initialCurvature(end))/arcLengths(end-1)] cumulativeLength];
            obj.PathManager.Arclengths     = arcLengths;
            obj.PathManager.SegStarts      = segStarts;
            obj.Length         = cumulativeLength(end);
        end
        
        function bounds = calculateBoundingBoxes(obj, wp)
        %calculateBoundingBoxes Calculates the AABB limits for each section of the curve
            numSeg = size(wp,1)-1;
            bounds = zeros(4,numSeg);
            ss = obj.PathManager.SegStarts;
            for i = 1:numSeg
                bounds(:,i) = nav.algs.internal.findClothoidBoundingBox(ss(i,1),ss(i,2),ss(i,3),ss(i,4),ss(i,5),obj.PathManager.Arclengths(i));
            end
        end
    end
    
    methods(Access={?nav.algs.internal.InternalAccess})
        function [pts, dists, segIdx, ptIdx] = nearestPoint(obj, xyPathPoints, mustBeProjection)
        %nearestPoint Finds the nearest point along a discrete curve
        
            narginchk(2,3);
            if nargin == 2
                mustBeProjection = false;
            end

            if coder.target('MATLAB')
                [pts,dists,segIdx,ptIdx] = nav.algs.internal.mex.nearestPointIterative(xyPathPoints(:,1:2), ...
                obj.PathManager.SegStarts, obj.PathManager.BoundingBoxes, obj.Length, mustBeProjection);
            else
                [pts,dists,segIdx,ptIdx] = nav.algs.internal.impl.nearestPointIterative(xyPathPoints(:,1:2), ...
                obj.PathManager.SegStarts, obj.PathManager.BoundingBoxes, obj.Length, mustBeProjection);
            end
        end
    end
    
    methods (Hidden, Static)
        function [x1,y1,th1,k1] = clothoid(x0,y0,th0,k0,dk,L)
        %clothoid Construct terminal points for clothoid
            if isscalar(L) && L(1) == 0
                x1 = x0; y1 = y0; th1 = th0; k1 = k0;
            else
                k1  = k0+dk*L;
                th1 = dk/2*L.^2 + k0*L + th0;
                % Calc integrated xy
                xy = matlabshared.tracking.internal.scenario.fresnelg(L, dk, k0, th0);
                % Add to base location
                x1 = x0 + real(xy);
                y1 = y0 + imag(xy);
            end
        end
        
        function [minDist, minIdx, segDist] = nearestBoundingBox(bounds, xy)
        %nearestBoundingBox Find nearest AABB to point
            numSeg = size(bounds,2);
            segDist = inf(1,numSeg);
            
            % Find distance to each boundingbox
            for i = 1:numSeg
                if xy(1) < bounds(1,i)
                    % Left-side
                    xDist = xy(1)-bounds(1,i);
                elseif xy(1) > bounds(2,i)
                    % Right-side
                    xDist = xy(1)-bounds(2,i);
                else
                    % Between horizontal
                    xDist = 0;
                end
                if xy(2) < bounds(3,i)
                    % Bottom-side
                    yDist = xy(2)-bounds(3,i);
                elseif xy(2) > bounds(4,i)
                    % Top-side
                    yDist = xy(2)-bounds(4,i);
                else
                    % Between vertical
                    yDist = 0;
                end
                
                % Store distance to AABB
                segDist(i) = (xDist^2+yDist^2)^(1/2);
            end
            
            % Return nearest BB index
            [minDist,minIdx] = min(segDist);
        end
        
        function nvPairs = parseConstructor(varargin)
        %parseConstructor Parse optional inputs to constructor
            
            % Grab default values
            defaults = nav.algs.internal.FrenetReferencePath.NVPairDefaults;
            
            % Parse all NV-Pair inputs
            pstruct = coder.internal.parseParameterInputs(defaults,struct(),varargin{:});
            nvPairs = coder.internal.vararginToStruct(pstruct,defaults,varargin{:});
            
            % Validate MaxNumWaypoints
            numPts = nvPairs.MaxNumWaypoints;
            validateattributes(numPts,{'numeric'},{'positive','scalar','nonnan'},'FrenetReferencePath','MaxNumWaypoints');
            if isfinite(numPts)
                validateattributes(numPts,{'numeric'},{'integer','>=',2},'FrenetReferencePath','MaxNumWaypoints');
            end
            
            % Validate DiscretizationDistance
            validateattributes(nvPairs.DiscretizationDistance,{'numeric'},{'scalar','real','positive','nonnan','finite'});
        end
    end
end

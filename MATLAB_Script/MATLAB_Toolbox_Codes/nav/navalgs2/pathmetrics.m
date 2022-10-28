classdef pathmetrics < nav.algs.internal.InternalAccess
    %pathmetrics Information for path metrics
    %   The pathmetrics object holds information for computing path
    %   metrics. Calculate smoothness, clearance, and path validity based
    %   on a set of poses and the associated map environment.
    %
    %   PATHMETRICOBJ = pathmetrics(NAVPATH) creates an object based on the
    %   given navPath object. The state validator is assumed to be a
    %   validatorOccupancyMap object.
    %
    %   PATHMETRICOBJ = pathmetrics(NAVPATH, VALIDATOR) creates an object 
    %   based on the given navPath object and associated state validator 
    %   for checking the path validity.
    %
    %   pathmetrics properties:
    %   Path            - Path data structure
    %   StateValidator  - State validator based on 2-D grid map
    %                     validatorOccupancyMap
    %
    %   pathmetrics methods:
    %   smoothness    - Smoothness of path
    %   clearance     - Minimum clearance from obstacles in map
    %   isPathValid   - Determine if planned path is obstacle free
    %   show          - Visualize path metrics in map environment
    %
    %   Example:
    %       % Create binary occupancy map
    %       load exampleMaps.mat;
    %       map = binaryOccupancyMap(simpleMap, 2);
    %
    %       % Find a path using PRM planner
    %       prm = mobileRobotPRM(map);
    %       prm.ConnectionDistance = 4;
    %       prm.NumNodes = 200;
    %       path = findpath(prm, [2 2], [12 2]);
    %       states = [path, zeros(size(path, 1), 1)];
    %
    %       % Create navPath object
    %       navpath = navPath(stateSpaceSE2, states);
    %
    %       % Create validatorOccupancyMap object
    %       statevalidator = validatorOccupancyMap;
    %       statevalidator.Map = map;
    %
    %       % Create a path metrics object
    %       pmObj = pathmetrics(navpath, statevalidator);
    %
    %       % Evaluate the minimum clearance of the path
    %       clearancePathVal = clearance(pmObj);
    %
    %       % Evaluate the set of minimum distances for each
    %       % segment of the path
    %       clearanceStatesVals = clearance(pmObj, "Type", "states");
    %
    %   see also mobileRobotPRM, occupancyMap, occupancyMap3D.
    
    % Copyright 2019-2020 The MathWorks, Inc.
    
    properties

        %Path - Path data structure
        %   The Path property is the path whose metric is to be calculated,
        %   specified as a navPath object.
        Path

        %StateValidator - Validator for states on the path
        %   The StateValidator property is a validatorOccupancyMap
        %   object that validates the states and discretized motions based
        %   on the values in a 2-D occupancy map.
        %
        %   Default: validatorOccupancyMap(stateSpaceSE2, binaryOccupancyMap(10))
        StateValidator
    end

    properties (Access = private)

        %States's intersection points on obstacles at minimum distances.
        %   This is useful to plot clearance.
        XYPtsObstacle = [];
    end    

    properties(Access = {?tpathmetrics, ?pathmetrics})
        
        %IsStateValidatorDefault - State Validator default flag
        IsStateValidatorDefault;
    end
    
    methods
        
        function obj = pathmetrics(pathobj, validatorOccupMap)
            %pathmetrics constructor

            %Check number of input arguments
            narginchk(1,2);

            % Input will be validated in property setter
            obj.Path = pathobj;

            if nargin == 2
                % Validate state validator given as user input
                % Input will be validated in property setter
                obj.StateValidator = validatorOccupMap;
            else
                % Use default state validator
                obj.StateValidator = validatorOccupancyMap;
                obj.IsStateValidatorDefault = true;
                obj.StateValidator.ValidationDistance = ...
                    1/obj.StateValidator.Map.Resolution;
            end
        end
        
        function smoothVal = smoothness(obj, varargin)
            %smoothness Smoothness of path
            %
            %   SMOOTHVAL = smoothness(PATHMETRCSOBJ) evaluate the
            %   smoothness of the planned path. Values closer to 0 indicate
            %   a smoother path. Straight line paths return a value of 0.
            %
            %   SMOOTHVAL = smoothness(PATHMETRICSOBJ, "TYPE", "SEGMENTS")
            %   returns individual smoothness calculations between each set
            %   of three poses on the path. SMOOTHVAL is a (N-2)-element
            %   vector where N is the number of poses.
            %
            %   Example:
            %       % Create binary occupancy map
            %       load exampleMaps.mat;
            %       map = binaryOccupancyMap(simpleMap, 2);
            %
            %       % Find a path using PRM planner
            %       prm = mobileRobotPRM(map);
            %       prm.ConnectionDistance = 4;
            %       prm.NumNodes = 200;
            %       path = findpath(prm, [2 2], [12 2]);
            %       states = [path, zeros(size(path, 1), 1)];
            %
            %       % Create navPath object
            %       navpath = navPath(stateSpaceSE2, states);
            %
            %       % Create a path metrics object
            %       pmObj = pathmetrics(navpath);
            %
            %       % Evaluate the smoothness of the planned path
            %       smoothPathVal = smoothness(pmObj);
            %
            %       % Evaluate the smoothness between each set of three
            %       % poses on the path
            %       smoothSegmentsVals = smoothness(pmObj, "Type", "segments");
            %
            % see also pathmetrics, clearance, isPathValid, show.
            
            % Create parameter name cell array
            name = "Type";
            
            % Create default value cell array
            default = "sum";
            
            % Create a parser
            parser = robotics.core.internal.NameValueParser(name,default);
            
            % Parse name-value inputs (where the name-value inputs are
            % contained in varargin).
            parse(parser, varargin{:});
            
            % Access 'type' value
            type = parameterValue(parser, "Type");
            
            values = {'sum', 'segments'};
            
            % Validatestring error message was not clear to users g1885126
            % so using double validation
            validateattributes(type, {'string', 'char'}, {'nonempty'});
            
            validatestring(type, values, "smoothness", "pathmetrics");
            
            % Smoothness code
            smoothVal = nav.algs.internal.smoothness(obj.Path.States, ...
                @(state1,state2)obj.Path.StateSpace.distance(state1,state2));
            
            if strcmp(type, "sum")
                smoothVal = sum(smoothVal);
            end
        end
        
        function clearanceVal = clearance(obj, varargin)
            %clearance Minimum clearance of path
            %
            %   CLEARANCEVAL = clearance(PATHMETRICSOBJ) returns the
            %   minimum clearance of the path. Clearance is measured as the
            %   minimum distance between grid cell centers of states on the
            %   path and obstacles in the specified map environment. 
            %
            %   CLEARANCEVAL = clearance(PATHMETRICSOBJ, "TYPE", "STATES")
            %   returns the set of minimum distances for each states of the
            %   path. CLEARANCEVAL is returned as an n-by-1 vector, where n
            %   is the number of states
            %
            %   Note: The computed clearance is accurate up to sqrt(2)
            %   times grid map cell size
            %
            %   Example:
            %       % Create binary occupancy map
            %       load exampleMaps.mat;
            %       map = binaryOccupancyMap(simpleMap, 2);
            %
            %       % Find a path using PRM planner
            %       prm = mobileRobotPRM(map);
            %       prm.ConnectionDistance = 4;
            %       prm.NumNodes = 200;
            %       path = findpath(prm, [2 2], [12 2]);
            %       states = [path, zeros(size(path, 1), 1)];
            %
            %       % Create navPath object
            %       navpath = navPath(stateSpaceSE2, states);
            %
            %       % Create validatorOccupancyMap object
            %       statevalidator = validatorOccupancyMap;
            %       statevalidator.Map = map;
            %
            %       % Create a path metrics object
            %       pmObj = pathmetrics(navpath, statevalidator);
            %
            %       % Evaluate the minimum clearance of the path
            %       clearancePathVal = clearance(pmObj);
            %
            %       % Evaluate the set of minimum distances for each
            %       % states of path
            %       clearanceStatesVals = clearance(pmObj, "Type", "states");
            %
            %   See also pathmetrics, smoothness, isPathValid, show.
            
            % Create parameter name cell array
            name = "Type";
            
            % Create default value cell array
            default = "min";
            
            % Create a parser
            parser = robotics.core.internal.NameValueParser(name,default);
            
            % Parse name-value inputs (where the name-value inputs are
            % contained in varargin).
            parse(parser, varargin{:});
            
            % Access 'type' value
            type = parameterValue(parser, "Type");
            
            values = {'min', 'states'};
            
            % Validatestring error message was not clear to users g1885126
            % so using double validation
            validateattributes(type, {'string', 'char'}, {});
            
            validatestring(type, values, "clearance", "pathmetrics");
            
            poses = obj.Path.States;
            
            nStates = size(poses,1);
            clearanceVal = inf(1,nStates);
            obj.XYPtsObstacle = [];
            
            %Clearance will be INF if environment has not specified.
            if ~obj.IsStateValidatorDefault                                
                [clearanceVal, obj.XYPtsObstacle] = nearestObstacleDistance(...
                    obj.StateValidator, poses);
            end
            
            if strcmp(type, "min")
                clearanceVal = min(clearanceVal);
            end
        end
        
        function boolVal = isPathValid(obj)
            %isPathValid Determine if planned path is obstacle free
            %
            %   BOOLVAL = isPathValid(PATHMETRICSOBJ) returns true if
            %   planned path is obstacles free otherwise false.
            %
            %   Example:
            %       % Create binary occupancy map
            %       load exampleMaps.mat;
            %       map = binaryOccupancyMap(simpleMap, 2);
            %
            %       % Find a path using PRM planner
            %       prm = mobileRobotPRM(map);
            %       prm.ConnectionDistance = 4;
            %       prm.NumNodes = 200;
            %       path = findpath(prm, [2 2], [12 2]);
            %       states = [path, zeros(size(path, 1), 1)];
            %
            %       % Create navPath object
            %       navpath = navPath(stateSpaceSE2, states);
            %
            %       % Create validatorOccupancyMap object
            %       statevalidator = validatorOccupancyMap;
            %       statevalidator.Map = map;
            %
            %       % Create a path metrics object
            %       pmObj = pathmetrics(navpath, statevalidator);
            %
            %       % Determine if planned path is obstacle free
            %       boolVal = isPathValid(pmObj);
            %
            %   See also pathmetrics, smoothness, clearance, show.
            
            boolVal = true;
            
            if obj.IsStateValidatorDefault
                %Environment is obstacles free so is valid.
                return;
            end
            
            for i = 1:size(obj.Path.States,1)-1
                
                %Interpolate the segments at 0.1 meters
                segmentsPoses = obj.Path.StateSpace.interpolate(obj.Path.States(i,:), ...
                    obj.Path.States(i+1,:), 0:0.01:1);
                if any(isStateValid(obj.StateValidator, segmentsPoses) == false)
                    boolVal = false;
                    return;
                end
            end
        end
        
        function ax = show(obj, varargin)
            %Show path metrics in map environment
            %
            %   AXHANDLE = show(PATHMETRICSOBJ) plots the path in the map
            %   environment with minimum clearance shown. Use the "Metrics"
            %   name-value pair to see more metrics.
            %
            %   AXHANDLE = show(PATHMETRICSOBJ, NAME, VALUE) specifies
            %   additional name-value pair arguments as described below:
            %
            %   "Parent"            Handle to an axes on which to display
            %                       the metrics.
            %   "Metrics"           Display specific metrics of the path,
            %                       specified as a cell array of strings,
            %                       using any combination of "MinClearance",
            %                       "StatesClearance", and "Smoothness".
            %
            %   Example:
            %       % Create binary occupancy map
            %       load exampleMaps.mat;
            %       map = binaryOccupancyMap(simpleMap, 2);
            %
            %       % Find a path using PRM planner
            %       prm = mobileRobotPRM(map);
            %       prm.ConnectionDistance = 4;
            %       prm.NumNodes = 200;
            %       path = findpath(prm, [2 2], [12 2]);
            %       states = [path, zeros(size(path, 1), 1)];
            %
            %       % Create navPath object
            %       navpath = navPath(stateSpaceSE2, states);
            %
            %       % Create validatorOccupancyMap object
            %       statevalidator = validatorOccupancyMap;
            %       statevalidator.Map = map;
            %
            %       % Create a path metrics object
            %       pmObj = pathmetrics(navpath, statevalidator);
            %
            %       % Visualize the path minimum clearance
            %       show(pmObj);
            %
            %       % Visualize the path smoothness
            %       axHandle = show(pmObj, "Metrics", {"smoothness"});
            %
            %       % Visualize the path smoothness and clearance by
            %       % passing axes handle
            %       axHandle = show(pmObj, "Parent", axHandle, "Metrics", {"smoothness", "StatesClearance"});
            %
            %   See also pathmetrics, smoothness, clearance, isPathValid.
            
            %Parse the input parameters.
            parser = inputParser;
            
            addParameter(parser, "Parent", [], ...
                @(x)robotics.internal.validation.validateAxesUIAxesHandle(x));
            
            metricsDefaultValue = "MinClearance";
            addParameter(parser, "Metrics", metricsDefaultValue);
            
            parser.parse(varargin{:});
            axHandle = parser.Results.Parent;
            tempMetrics = parser.Results.Metrics;

            metrics = strings(1,numel(tempMetrics));
            
            % Validatestring error message was not clear to users g1885126
            % so using double validation
            validateattributes(tempMetrics, {'cell', 'string'}, {'nonempty'}, 'show', 'Metrics');

            % Validate positions
            for i=1:numel(tempMetrics)
                metrics(i) = validatestring(tempMetrics{i}, {'MinClearance', 'StatesClearance', 'Smoothness'}, 'show', 'Positions');
            end
            
            % Create a new axes if not assigned
            if isempty(axHandle)
                axHandle = newplot;
            end
            
            % Get the hold status for given axes
            holdStatus = ishold(axHandle);            
            
            hold(axHandle, 'on');
            
            if ~obj.IsStateValidatorDefault
                %Visualize the map
                if isa(obj.StateValidator.Map, "binaryOccupancyMap") || isa(obj.StateValidator.Map, "occupancyMap")
                    nav.algs.internal.MapUtils.showGrid(obj.StateValidator.Map, axHandle, 0, 0, false);
                else
                    plot(obj.StateValidator.Map, 'Parent', axHandle);                                       
                end
                hold(axHandle, "on");
            end
            
            %Take legend status;
            legendStatus = isempty(axHandle.Legend);           
            legend('off');                
            
            % Visualize the metrics
            if find(strcmp(metrics, "Smoothness"))
                smoothVals = obj.smoothness("Type", "segments");
                obj.showSmoothness(obj.Path.States, smoothVals, axHandle);
            else
                %visualize paths
                patch([obj.Path.States(:,1); nan],[obj.Path.States(:,2); nan],zeros(obj.Path.NumStates+1,1),'EdgeColor','interp','Parent',axHandle,'LineWidth',2);
            end
            
            if ~isempty(find(strcmp(metrics, "MinClearance"), 1)) ||...
                    ~isempty(find(strcmp(metrics, "StatesClearance"), 1))
                plotClearance(obj, axHandle, metrics);
            end
            
            % Restore the hold status of the original figure
            if ~holdStatus
                hold(axHandle, "off");
            end
            
            % Restore the legend hold status
            if ~legendStatus
                legend('on');
            end
            
            % Only return handle if user requested it.
            if nargout > 0
                ax = axHandle;
            end
        end
        
        function cpObj = copy(obj)
            %copy Create a deep copy of pathmetrics object.
            
            if isempty(obj)
                cpObj = pathmetrics.empty;
                return;
            end
            
            % construct new object
            cpObj = pathmetrics(obj.Path);
            
            % Copy public non-dependent properties with setters
            cpObj.StateValidator = obj.StateValidator;
            
            % Copy internal properties
            cpObj.IsStateValidatorDefault = obj.IsStateValidatorDefault;
        end
        
        function set.Path(obj, pathobj)
            
            % Validate navpath attribute
            
            nav.internal.validation.validateNavPath(pathobj, "", "Path");
            obj.Path = pathobj;
        end
        
        function set.StateValidator(obj, statevalidator)
            %set.StateValidator Setter for StateValidator property
            
            nav.internal.validation.validateStateValidators(statevalidator, "", "StateValidator");
            obj.StateValidator = statevalidator;
            obj.IsStateValidatorDefault = false; %#ok<MCSUP>                        
        end                
    end    
    
    methods(Access = private)
        
        function plotClearance(obj, axHandle, metrics)
            %plotClearance Visualize clearance of path and segments.
            
            if obj.IsStateValidatorDefault
                return;
            end
            
            %Compute clearance
            minDist = clearance(obj, "Type", "states");
            
            if isa(obj.StateValidator,'validatorVehicleCostmap')
                stateGridPts = obj.StateValidator.OccupancyMapInternal.world2grid(obj.Path.States(:,1:2));
                stateGridCellCenter = obj.StateValidator.OccupancyMapInternal.grid2world(stateGridPts);
                halfGridCellSize = (obj.StateValidator.Map.CellSize/2);
            else
                stateGridPts = obj.StateValidator.Map.world2grid(obj.Path.States(:,1:2));
                stateGridCellCenter = obj.StateValidator.Map.grid2world(stateGridPts);
                halfGridCellSize = 1/(2*obj.StateValidator.Map.Resolution);
            end
            
            if find(strcmp(metrics, "StatesClearance"))
                % compute unique query point and obstacle grid centers for patch plot
                if isa(obj.StateValidator,'validatorVehicleCostmap')
                    gridPtsObstacle = obj.StateValidator.OccupancyMapInternal.world2grid(obj.XYPtsObstacle);
                    gridSize = obj.StateValidator.Map.MapSize;
                else
                    gridPtsObstacle = obj.StateValidator.Map.world2grid(obj.XYPtsObstacle);
                    gridSize = obj.StateValidator.Map.GridSize;
                end
                % visualize square patches only if minDist is not zero
                squarePatchesToConsiderInd = find((minDist' > 0) & (~isinf(minDist')));
                [~,uniqueObstacleCellIndex] = unique(sub2ind(gridSize,gridPtsObstacle(squarePatchesToConsiderInd,1),gridPtsObstacle(squarePatchesToConsiderInd,2)));
                [~,uniqueStateCellIndex] = unique(sub2ind(gridSize,stateGridPts(squarePatchesToConsiderInd,1),stateGridPts(squarePatchesToConsiderInd,2)));
                xObst = obj.XYPtsObstacle(squarePatchesToConsiderInd(uniqueObstacleCellIndex),1)';
                yObst = obj.XYPtsObstacle(squarePatchesToConsiderInd(uniqueObstacleCellIndex),2)';
                xPt = stateGridCellCenter(squarePatchesToConsiderInd(uniqueStateCellIndex),1)';
                yPt = stateGridCellCenter(squarePatchesToConsiderInd(uniqueStateCellIndex),2)';
                
                for i = 1:size(obj.XYPtsObstacle,1)
                    plot([obj.XYPtsObstacle(i,1), stateGridCellCenter(i,1)],...
                        [obj.XYPtsObstacle(i,2), stateGridCellCenter(i,2)], "Parent", axHandle, "LineWidth", 3, "color", [0, 0.5, 0]);
                end
            end
            
            if find(strcmp(metrics, "MinClearance"))
                [d,idx] = min(minDist);
                
                % compute min clearance query state and obstacle grid centers for patch plot 
                if d > 0 && (~isinf(d))
                    xObst = obj.XYPtsObstacle(idx,1);
                    yObst = obj.XYPtsObstacle(idx,2);
                    xPt = stateGridCellCenter(idx,1);
                    yPt = stateGridCellCenter(idx,2);
                else
                    xObst = [];
                    yObst = [];
                    xPt = [];
                    yPt = [];
                end
                
                plot([obj.XYPtsObstacle(idx,1), stateGridCellCenter(idx,1)],...
                    [obj.XYPtsObstacle(idx,2), stateGridCellCenter(idx,2)], "Parent", axHandle, "LineWidth", 3, "color", [0.6350, 0.0780, 0.1840]);
            end
            
            % plot square patches on obstacle and query states
            patch('XData',[xObst-halfGridCellSize;xObst+halfGridCellSize;xObst+halfGridCellSize;xObst-halfGridCellSize],...
                'YData',[yObst-halfGridCellSize;yObst-halfGridCellSize;yObst+halfGridCellSize;yObst+halfGridCellSize],...
                'EdgeColor','none','FaceColor','red','FaceAlpha',0.5,'Parent',axHandle);
            patch('XData',[xPt-halfGridCellSize;xPt+halfGridCellSize;xPt+halfGridCellSize;xPt-halfGridCellSize],...
                'YData',[yPt-halfGridCellSize;yPt-halfGridCellSize;yPt+halfGridCellSize;yPt+halfGridCellSize],...
                'EdgeColor','none','FaceColor','blue','FaceAlpha',0.3,'Parent',axHandle);
        end
    end
    
    methods(Static, Access = private)
        
        function axHandle = showSmoothness(pathDiscrete, smoothVal, axHandle)
            % Find orientation change
            %orientationChange = angdiff(pathDiscrete(:,3));
            % Take care of precision issues
            %orientationChange = single(orientationChange);
            
            % Color transition: "g" -> "o" -> "r"
            colorCodeValues = [0 1 0; 0.91 0.41 0.17; 1 0 0];
            colorDiff = diff(colorCodeValues);
            
            % Generate points to blur colorspace over
            x = linspace(0,1,50)';
            
            % Create colormap by transitioning colors between
            % colorCodeValues
            cmapVals = [colorCodeValues(1,:) + x*colorDiff(1,:);  % Transition from low (green) to mid (orange)
                       colorCodeValues(2,:) + x*colorDiff(2,:)]; % Transition from mid (orange) to high (red)
                   
            % Plot the line as a patch so individual segments can be colorized
            patch([pathDiscrete(:,1); nan],[pathDiscrete(:,2); nan],[0; smoothVal(:); 0; 0],'EdgeColor','interp','Parent',axHandle,'LineWidth',2);

            % Set the colormap for the figure
            axHandle.Colormap = cmapVals;
            
            % Set limits and show colorbar
            if numel(smoothVal) > 1 && max(smoothVal) > 0
                % used 'ceil' to map colorbar max. limit to nearest
                % integer, which avoids visualization problem for very low
                % smooth values
                axHandle.CLim = [0 ceil(max(smoothVal))];
            end
            colorbar;
        end
    end
    
end

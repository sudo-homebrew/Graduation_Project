classdef geometricPlannerMetrics < nav.algs.internal.plannerMetricInterface
%This class is for internal use only. It may be removed in the future.

%geometricPlannerMetrics defines metrics for geometric planners.

% Copyright 2021 The MathWorks, Inc.

    properties(SetAccess = private, GetAccess = public)

        %MetricList list of metric names defined in geometricPlannerMetrics
        %   Methods which calculates the metrics should be public and
        %   have names which match the strings in MetricList. Each method
        %   should take geometricPlannerMetrics object as the only input.

        MetricList = {'pathLength', 'clearance', 'smoothness','isPathValid'};

        %MetricType Array representing type of metric in MetricList
        %   MetricType should be of same length as MetricList. Each value
        %   in MetricType represent the type of metric in corresponding
        %   position in MetricList. Value of 0 represent corresponding
        %   metric is of type double and 1 represent metric is of type
        %   boolean.
        MetricType = [0, 0, 0, 1];
    end

    properties(Access = {?nav.algs.internal.InternalAccess})

        %Environment Environment on which planners Benchmark
        Environment

        %PlanOutput Output of plan function
        PlanOutput

        %InitOutput Output of initialization function
        InitOutput
    end

    methods

        function obj = geometricPlannerMetrics(environment)
        %geometricPlannerMetrics Constructor
            validateattributes(environment,{'binaryOccupancyMap', ...
                                            'occupancyMap','validatorOccupancyMap'},"scalar", ...
                               'geometricPlannerMetrics');
            obj.Environment = environment;
        end

        function assignPlannerOutput(obj, planOutput,initializationOutput)
        %assignPlannerOutput Assign the planner outputs
        %   assignPlannerOutput(OBJ, planOutput,initializationOutput)
        %   is used to input the planOutput and initializationOutput
        %   to the geometricPlannerMetrics class. planOutput is the
        %   output of the plan function of the planner. initializationOutput
        %   is the output of the initialization function of the planner.

            validateattributes(planOutput,"cell","vector");
            path = planOutput{1};
            %check whether the path output from the planner is of navPath
            % or an array of size mx2 or mx3
            obj.validatePlannerOutput(path);

            obj.PlanOutput = planOutput;
            obj.InitOutput = initializationOutput;
        end

        function pLength = pathLength(obj)
        %pathLength Length of path
            pathObj = obj.getNavPath();
            if(isempty(pathObj.States) || size(pathObj.States,1) == 1)
                pLength = NaN;
            else
                pLength = pathLength(pathObj);
            end
        end

        function clearancePathVal = clearance(obj)
        %clearance Minimum clearance from obstacles in map
            pathObj = obj.getNavPath();
            if(isempty(pathObj.States))
                clearancePathVal = NaN;
            else
                pathMetricObj = obj.getPathMetricsObj(pathObj);
                clearancePathVal = clearance(pathMetricObj);
            end
        end

        function smoothPathVal = smoothness(obj)
        %smoothness Smoothness of path
            pathObj = obj.getNavPath();
            if(isempty(pathObj.States))
                smoothPathVal = NaN;
            else
                pathMetricObj = obj.getPathMetricsObj(pathObj);
                smoothPathVal = smoothness(pathMetricObj);
            end
        end

        function isPathValidVal = isPathValid(obj)
        %isPathValid Determine if planned path is obstacle free
            pathObj = obj.getNavPath();
            if(isempty(pathObj.States))
                isPathValidVal = NaN;
            else
                pathMetricObj = obj.getPathMetricsObj(pathObj);
                isPathValidVal = pathMetricObj.isPathValid;
            end
        end

        function cpObj = copy(obj)
        %copy Create a copy of the object
        %   cpObj = copy(obj) creates a deep copy of the geometricPlannerMetrics
        %   object with the same properties.

            environment = obj.Environment;

            cpObj = nav.algs.internal.geometricPlannerMetrics(environment);
            cpObj.PlanOutput = obj.PlanOutput;
            cpObj.InitOutput = obj.InitOutput;
        end
    end

    methods(Access = {?nav.algs.internal.InternalAccess})

        function outPath = convertNavPath(obj, path)
        %convertNavPath covert the input path to navPath assuming
        %   StateSpaceSE2
            if(isa(path, 'navPath'))
                outPath = path;
            else
                defaultStateSpace = stateSpaceSE2;
                if(size(path,2) == 2)
                    % when path is mx2, assume theta is 0
                    path(:,3) = 0;
                    defaultStateSpace.WeightTheta = 0;
                end
                environment = obj.Environment;
                % Assume StateSpaceSE2 when user provide path as mx2 or mx3
                if(isa(environment,'validatorOccupancyMap'))
                    stateBounds = [environment.Map.XWorldLimits;
                                   environment.Map.YWorldLimits;[-pi pi]];
                else
                    stateBounds = [environment.XWorldLimits;
                                   environment.YWorldLimits;[-pi pi]];
                end
                defaultStateSpace.StateBounds = stateBounds;
                if(isempty(path))
                    outPath = navPath(defaultStateSpace);
                else
                    outPath = navPath(defaultStateSpace, path);
                end
            end
        end

        function navPathOut = getNavPath(obj)
        %getNavPath return the navPath object
            planOutput = obj.PlanOutput;
            %Planner with multiple output require the first output to be
            %the path output
            path = planOutput{1};
            navPathOut = obj.convertNavPath(path);
        end

        function pathmetricsObj = getPathMetricsObj(obj, path)
        %getPathMetricObj return the pathmetrics object
            environment = obj.Environment;
            if(isa(environment,'validatorOccupancyMap'))
                sv = environment;
                pathmetricsObj = pathmetrics(path, sv);
            else
                ss = path.StateSpace;
                sv = validatorOccupancyMap(ss,"Map",environment);
                %Validation distance is assumed to be 0.1* (1/resolution)
                sv.ValidationDistance = (1/10)*(1/environment.Resolution);
                pathmetricsObj = pathmetrics(path, sv);
            end
        end

        function validatePlannerOutput(obj, path) %#ok<INUSL>
        %validatePlannerOutput check whether the planner output is of
        %   navPath or mx2 or mx3 array
            if(isa(path,'navPath'))
                validateattributes(path,{'navPath'},{'scalar'});
            elseif(size(path,2) == 3)
                validateattributes(path,{'double'},{'2d','finite','nonnan'});
            else
                validateattributes(path,{'double'},{'2d','finite','nonnan','ncols',2});
            end
        end
    end
end

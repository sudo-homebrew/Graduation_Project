classdef (Abstract) plannerMetricInterface < nav.algs.internal.InternalAccess
%This class is for internal use only. It may be removed in the future.

%plannerMetricInterface defines metrics for path planners
%   plannerMetricInterface is an interface class for defining metrics
%   of different planner types. Derived classes should specify the names of
%   metrics supported in the concrete implementation in MetricList property.
%   The derived classes should also implement the assignPlannerOutput
%   method which assign the output of the planner to the concrete implementation.

%   Copyright 2021 The MathWorks, Inc.
    properties(Abstract,SetAccess = private, GetAccess = public)
        %MetricList - The names of metrics that are defined for a planner type.
        %   The list should be a cell array of strings. The derived classes
        %   should have methods with names which match the strings in
        %   MetricList. These methods should be public and should take
        %   the class object as the only input and should return the metric value.
        MetricList

        %MetricType - Array representing type of metric in MetricList
        %   MetricType should be of same length as MetricList. Each value
        %   in MetricType represent the type of metric in corresponding
        %   position in MetricList. Value of 0 represent corresponding
        %   metric in metricList is of type double and 1 represent
        %   corresponding metric is of type boolean.
        MetricType


    end
    methods(Abstract)
        %assignPlannerOutput   Assign the output of the Planner
        %   assignPlannerOutput(planOutput, initOutput) is called by plannerBenchmark
        %   to pass the output of the initialization function and plan function
        %   to the classes which define metrics. Each time the method runPlanner
        %   of plannerBenchmark is executed, the assignPlannerOutput method is
        %   invoked to pass the latest planner output for calculating the
        %   metrics
        assignPlannerOutput(planOutput, initOutput)
    end

end

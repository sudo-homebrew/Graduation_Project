classdef SettingsModel < handle & ...
        matlab.mixin.SetGet & ...
        robotics.appscore.internal.mixin.SaveLoadTools
    %This class is for internal use only. It may be removed in the future.

    %SETTINGSMODEL Model to keep track of the SLAM problem settings

    % Copyright 2018 The MathWorks, Inc.

    properties (SetAccess = immutable)

        % default values

        DefaultMapResolution
        DefaultLidarRange
        DefaultLoopClosureThreshold
        DefaultLoopClosureSearchRadius
        DefaultLoopClosureMaxAttempts
        DefaultLoopClosureAutoRollback
        DefaultOptimizationInterval
        DefaultMovementThreshold

        DefaultMaxIterations
        DefaultMaxTime
        DefaultGradientTolerance
        DefaultFunctionTolerance
        DefaultStepTolerance
        DefaultFirstNodePose
    end

    properties % parameters related to SLAM problem, temporary values

        MapResolution
        LidarRange
        LoopClosureThreshold
        LoopClosureSearchRadius
        LoopClosureMaxAttempts
        LoopClosureAutoRollback
        OptimizationInterval
        MovementThreshold

        MaxIterations
        MaxTime
        GradientTolerance
        FunctionTolerance
        StepTolerance
        FirstNodePose

    end

    properties % parameters related to SLAM problem, applied values

        Applied

    end


    properties (Access = protected)
        MaxLevel

    end

    methods
        function obj = SettingsModel()
        %SETTINGSMODEL Constructor

            obj.MaxLevel = 5; % not exposed to user
            obj.DefaultLidarRange = [0.01, 8]; % min max
            obj.DefaultMapResolution = obj.estimateMapResolution(obj.DefaultLidarRange(2));

            tempLslam = lidarSLAM(obj.DefaultMapResolution, obj.DefaultLidarRange(2));
            obj.DefaultLoopClosureThreshold = 2*tempLslam.LoopClosureThreshold; % 2x
            obj.DefaultLoopClosureSearchRadius = tempLslam.LoopClosureSearchRadius;
            obj.DefaultLoopClosureMaxAttempts = tempLslam.LoopClosureMaxAttempts;
            obj.DefaultLoopClosureAutoRollback = tempLslam.LoopClosureAutoRollback;
            obj.DefaultOptimizationInterval = tempLslam.OptimizationInterval;
            obj.DefaultMovementThreshold = tempLslam.MovementThreshold;

            nlpSolver = nav.algs.internal.PoseGraphOptimizer.getDefaultSolver(2);
            params = nlpSolver.getSolverParams;

            obj.DefaultMaxIterations = params.MaxNumIteration;
            obj.DefaultMaxTime = params.MaxTime;
            obj.DefaultGradientTolerance = params.GradientTolerance;
            obj.DefaultFunctionTolerance = params.FunctionTolerance;
            obj.DefaultStepTolerance = params.StepTolerance;
            obj.DefaultFirstNodePose = [0 0 0];

            obj.applyDefaultValues();
        end

        function applyDefaultValues(obj)
        %applyDefaultValues
            propNames = properties(obj);

                % filter out those properties that start with "Default..."
                reResult = regexp(propNames, '^Default.*', 'match');
                defaultPropNames = propNames(~cellfun('isempty', reResult));

                % assign
                for i = 1:length(defaultPropNames)
                    defaultPropName = defaultPropNames{i};
                    propName = defaultPropName(8:end); % remove "Default" prefix
                    obj.(propName) = obj.(defaultPropName);
                    obj.Applied.(propName) = obj.(defaultPropName); % sync to Applied
                end

            end

            function readFromApplied(obj)
            %readFromApplied
                fnames = fieldnames(obj.Applied);
                for i = 1:length(fnames)
                    fn = fnames{i};
                    obj.(fn) = obj.Applied.(fn);
                end
            end

            function writeToApplied(obj)
            %writeToApplied
                fnames = fieldnames(obj.Applied);
                for i = 1:length(fnames)
                    fn = fnames{i};
                    obj.Applied.(fn) = obj.(fn);
                end
            end

            function val = settingsAreChanged(obj)
            %settingsAreChanged
                fnames = fieldnames(obj.Applied);
                val = false;
                for i = 1:length(fnames)
                    fn = fnames{i};
                    if any(obj.Applied.(fn) ~= obj.(fn)) % the input param might be a vector
                        val = true;
                        return;
                    end
                end
            end

            function output = extract(obj)
            %extract

            % filter
                propNames = properties(obj);
                    reResult = regexp(propNames, '(^Default.*)|(^Applied.*)', 'match');
                    currentPropNames = propNames(cellfun('isempty', reResult));

                    for i = 1:length(currentPropNames)
                        pn = currentPropNames{i};
                        if strcmp(pn, 'LoopClosureAutoRollback')
                            val = obj.(pn);
                            if val
                                str = 'on';
                            else
                                str = 'off';
                            end
                        else
                            str = num2str(obj.(pn));
                            val = 0;
                        end
                        output.(pn) = {str, val};
                    end
                end

    end

    methods % setters/getters
        function set.MapResolution(obj, val)
        %set.MapResolution
            if isscalar(val) && val > 0
                obj.MapResolution = ceil(val);
            end
        end

        function set.LidarRange(obj, val)
        %set.LidarRange
            if (numel(val) == 2) && all(val >= 0) && (val(1) < val(2))
                obj.LidarRange = val;
            end
        end


        function set.LoopClosureThreshold(obj, val)
        %set.LoopClosureThreshold
            if isscalar(val) && val > 0
                obj.LoopClosureThreshold = val;
            end
        end

        function set.LoopClosureSearchRadius(obj, val)
        %set.LoopClosureSearchRadius
            if isscalar(val) && val > 0
                obj.LoopClosureSearchRadius = val;
            end
        end

        function set.LoopClosureMaxAttempts(obj, val)
        %set.LoopClosureMaxAttempts
            if isscalar(val) && val > 0
                obj.LoopClosureMaxAttempts = ceil(val);
            end
        end

        function set.LoopClosureAutoRollback(obj, val)
        %set.LoopClosureMaxAttempts
            if isscalar(val)
                obj.LoopClosureAutoRollback = logical(val);
            end
        end

        function set.OptimizationInterval(obj, val)
        %set.OptimizationInterval
            if isscalar(val) && val > 0
                obj.OptimizationInterval = ceil(val);
            end

        end

        function set.MovementThreshold(obj, val)
        %set.MovementThreshold
            if numel(val) == 2
                obj.MovementThreshold = val;
            end
        end

        function set.FirstNodePose(obj, val)
        %set.FirstNodePose
            if numel(val) == 3
                obj.FirstNodePose = val;
            end
        end

        function set.MaxIterations(obj, val)
        %set.MaxIterations
            if isscalar(val) && val > 0
                obj.MaxIterations = round(val);
            end
        end

        function set.MaxTime(obj, val)
        %set.MaxTime
            if isscalar(val) && val > 0
                obj.MaxTime = val;
            end
        end

        function set.GradientTolerance(obj, val)
        %set.GradientTolerance
            if isscalar(val) && val > 0
                obj.GradientTolerance = val;
            end
        end

        function set.FunctionTolerance(obj, val)
        %set.FunctionTolerance
            if isscalar(val) && val > 0
                obj.FunctionTolerance = val;
            end
        end

        function set.StepTolerance(obj, val)
        %set.StepTolerance
            if isscalar(val) && val > 0
                obj.StepTolerance = val;
            end
        end
    end


    methods (Access = protected)

        function res = estimateMapResolution(obj, maxLidarRange)
        %estimateMapResolution
            finestGridSize = (maxLidarRange*2)/(10*power(2, obj.MaxLevel));
            res = round(1/finestGridSize);
        end

    end
end

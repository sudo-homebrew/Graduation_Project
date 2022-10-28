classdef ErrorMetrics < handle%
%

%   Copyright 2018 The MathWorks, Inc.
    
    properties
        ErrorFunctionFormat = 'built-in'
        MotionModel = 'constvel'
        EstimationErrorLabels = {'posMSE'}
        EstimationErrorFcn
    end
    
    properties (Dependent)
        TruthIdentifierFcn
        TrackIdentifierFcn
    end
    
    properties (Access = private)
        TruthError
        TrackError
        CumulativeMetrics
        CumulativeCount
        LastMetrics
        LastCount
    end
    
    properties (Constant, Access = private)
        %            model      est err fcn  labels
        Builtins = {'constvel', @errstatecv, {'posRMS','velRMS','posANEES','velANEES'} 
                    'constacc', @errstateca, {'posRMS','velRMS','accRMS','posANEES','velANEES','accANEES'}
                    'constturn',@errstatect, {'posRMS','velRMS','yawRateRMS','posANEES','velANEES','yawRateANEES'}
                    'singer',   @errstateca, {'posRMS','velRMS','accRMS','posANEES','velANEES','accANEES'}};
    end

    properties (Dependent, Access = private)
        NumResults
        MetricLabels
    end
        
    methods
        function obj = ErrorMetrics
            obj.TrackError = fusion.internal.metrics.TrackError;
            obj.TruthError = fusion.internal.metrics.TruthError;
            obj.EstimationErrorFcn = @errposcv;
        end
        
        function resetMetrics(obj)
            obj.LastMetrics = zeros(1,obj.NumResults);
            obj.LastCount = 0;
            obj.CumulativeMetrics = zeros(1,obj.NumResults);
            obj.CumulativeCount = 0;
            resetMetrics(obj.TrackError, obj.NumResults);
            resetMetrics(obj.TruthError, obj.NumResults);
        end
        
        function analyze(obj, tracks, currentTrackID, truths, currentTruthID)
            results = computeErrors(obj, tracks, currentTrackID, truths, currentTruthID);
            updateErrors(obj, results, currentTrackID, currentTruthID);
        end
        
        function results = lastMetrics(obj)
            results = obj.LastMetrics ./ obj.LastCount;
            if strcmp(obj.ErrorFunctionFormat,'built-in')
                results(1:end/2) = sqrt(results(1:end/2));
            end
            results = num2cell(results);
        end
        
        function results = cumulativeMetrics(obj)
            results = obj.CumulativeMetrics ./ obj.CumulativeCount;
            if strcmp(obj.ErrorFunctionFormat,'built-in')
                results(1:end/2) = sqrt(results(1:end/2));
            end
            results = num2cell(results);
        end            
        
        function tm = currentTrackMetricsTable(obj)
            tm = currentMetricsTable(obj.TrackError, obj.MetricLabels, strcmp(obj.ErrorFunctionFormat,'built-in'));
        end
        
        function tm = currentTruthMetricsTable(obj)
            tm = currentMetricsTable(obj.TruthError, obj.MetricLabels, strcmp(obj.ErrorFunctionFormat,'built-in'));
        end
        
        function tm = cumulativeTrackMetricsTable(obj)
            tm = cumulativeMetricsTable(obj.TrackError, obj.MetricLabels, strcmp(obj.ErrorFunctionFormat,'built-in'));
        end
        
        function tm = cumulativeTruthMetricsTable(obj)
            tm = cumulativeMetricsTable(obj.TruthError, obj.MetricLabels, strcmp(obj.ErrorFunctionFormat,'built-in'));
        end
        
        function setBuiltinErrorFunction(obj)
            r = strcmp(obj.MotionModel, obj.Builtins(:,1));
            obj.EstimationErrorFcn = obj.Builtins{r,2};
            obj.TrackError.IdentifierFcn = @defaultTrackIdentifier;
            obj.TruthError.IdentifierFcn = @defaultTruthIdentifier;
        end
        
        function n = get.NumResults(obj)
            if strcmp(obj.ErrorFunctionFormat,'built-in')
                r = strcmp(obj.MotionModel,obj.Builtins(:,1));
                n = numel(obj.Builtins{r,3});
            else
                n = numel(obj.EstimationErrorLabels);
            end
        end
        
        function labels = get.MetricLabels(obj)
            if strcmp(obj.ErrorFunctionFormat,'built-in')
                r = strcmp(obj.MotionModel,obj.Builtins(:,1));
                labels = obj.Builtins{r,3};
            else
                labels = obj.EstimationErrorLabels;
            end
        end
        
        function v = get.TruthIdentifierFcn(obj)
            v = obj.TruthError.IdentifierFcn;
        end
        function v = get.TrackIdentifierFcn(obj)
            v = obj.TrackError.IdentifierFcn;
        end
        function set.TruthIdentifierFcn(obj, v)
            obj.TruthError.IdentifierFcn = v;
        end
        
        function set.TrackIdentifierFcn(obj, v)
            obj.TrackError.IdentifierFcn = v;
        end
        
        function s = saveErrorMetrics(obj)
            s.TrackError = saveError(obj.TrackError);
            s.TruthError = saveError(obj.TruthError);
            s.CumulativeMetrics = obj.CumulativeMetrics;
            s.CumulativeCount = obj.CumulativeCount;
            s.LastMetrics = obj.LastMetrics;
            s.LastCount = obj.LastCount;
        end
        
        function loadErrorMetrics(obj,s)
            loadError(obj.TrackError, s.TrackError);
            loadError(obj.TruthError, s.TruthError);
            obj.CumulativeMetrics = s.CumulativeMetrics;
            obj.CumulativeCount = s.CumulativeCount;
            obj.LastMetrics = s.LastMetrics;
            obj.LastCount = s.LastCount;
        end
    end
    
    methods (Access = private)
        function results = computeErrors(obj, tracks, currentTrackID, truths, currentTruthID)
            currentTrack = selectCurrent(obj.TrackError, tracks, currentTrackID);
            currentTruth = selectCurrent(obj.TruthError, truths, currentTruthID);
            
            results = estimateError(obj, currentTrack, currentTruth);
        end
        
        function results = estimateError(obj, currentTrack, currentTruth)
            assert(numel(currentTrack)==numel(currentTruth));
            
            m = numel(currentTrack);
            n = obj.NumResults;
            results = zeros(m,n);
            outargs = cell(1,n);
            for i=1:numel(currentTrack)
                [outargs{:}] = obj.EstimationErrorFcn(currentTrack(i), currentTruth(i));
                results(i,:) = cell2mat(outargs);
            end
        end
        
        function updateErrors(obj, results, currentTrackID, currentTruthID)
            obj.LastMetrics = sum(results,1);
            obj.LastCount = size(results,1);
            obj.CumulativeMetrics = obj.CumulativeMetrics + obj.LastMetrics;
            obj.CumulativeCount = obj.CumulativeCount + size(results,1);
            
            updateErrors(obj.TrackError, results, currentTrackID);
            updateErrors(obj.TruthError, results, currentTruthID);
        end
    end
end

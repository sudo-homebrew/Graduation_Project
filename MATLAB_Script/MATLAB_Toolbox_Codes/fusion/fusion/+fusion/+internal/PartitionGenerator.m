classdef PartitionGenerator < matlab.System
    % PartitionGenerator Generate partitions from a detection set
    %
    % This is an internal class and may be removed or modified in a future
    % release.
    %
    % PartitionGenerator helps trackerPHD in two main tasks.
    % 1. Calculate partitions of a set of detections from the same sensor.
    % 2. Evalulate the partitions against a set of PHD filter.
    % pGenerator = PartitionGenerator('PartitioningFcn',@partitionDetections);
    % [detectionIndices,startID,endID] = pGenerator(detections);
    % [scaledLikelihood,partitionProbabilities,cellDensities] =
    % pGenerator(logLikelihood,detectionIndices,startID,endID,weights,detectionProbability,clutterDensity);
    %
    
    % Copyright 2018-2019 The MathWorks, Inc.
    
    %#codegen
    
    % PartitioningFcn
    properties (Nontunable)
        PartitioningFcn
    end
    
    % Data type
    properties (Nontunable)
        pDataType
    end
    
    methods
        function obj = PartitionGenerator(varargin)
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods(Access = protected)
        function setupImpl(obj,sensorDetections)
            % Validate the partitioning function and set the data type for
            % the partition generator.
            validatePartitioningFcn(obj,sensorDetections);
            if ~coder.internal.is_defined(obj.pDataType)
                obj.pDataType = class(sensorDetections{1}.Measurement);
            end
        end
        function [detectionIndices, startID, endID] = stepImpl(obj,sensorDetections,isPointSensor)
            % Compute partitions and convert them to logical detection
            % indices.
            % To keep track of which indices belong to which partition,
            % startID(i) is the column index of start of ith partition
            % endID(i) is the column index of end of ith partition
            % Therefore, detectionIndices(:,startID(i):endID(i)) are the
            % detection cells belonging to partition i.
            %
            M = numel(sensorDetections);
            if ~isPointSensor
                partitions = obj.PartitioningFcn(sensorDetections);
            else
                partitions = cast((1:M)','uint32');
            end
            numPartitions = size(partitions,2);
            numCells = max(partitions,[],1);
            startID = [0 cumsum(numCells(1:end-1))] + 1;
            endID = cumsum(numCells);
            totalNumCells = endID(end);
            detectionIndices = false(M,totalNumCells);
            for i = 1:numPartitions
                detectionIndices(:,startID(i):endID(i)) = bsxfun(@eq,partitions(:,i), 1:numCells(i));
            end
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            resetImpl@matlab.System(obj);
        end
        
        function validatePartitioningFcn(obj,sensorDetections)
            f = obj.PartitioningFcn;
            partitions = f(sensorDetections);
            % Throw meaningful error messages instead of using
            % validateattributes.
            coder.internal.assert(size(partitions,1) == numel(sensorDetections),'fusion:trackerPHD:invalidPartitionSize');
            coder.internal.assert(size(partitions,2) > 0,'fusion:trackerPHD:atleastOnePartition');
            % Verify that partitions are from 1 to max value and none of
            % the indices are missing.
            numPartitions = size(partitions,2);
            for i = 1:numPartitions
                thisPartition = partitions(:,i);
                uniqueCells = unique(thisPartition);
                expectation = (1:max(thisPartition))';
                coder.internal.assert(numel(uniqueCells) == max(thisPartition) && all(uniqueCells == expectation),'fusion:trackerPHD:expectedUniquePartition');
            end
        end
    end
    methods
        function [scaledLikelihoods,partitionProbabilities,cellDensities] = evaluatePartitions...
                (obj,logLikelihood,detectionIndices,startID,endID,weights,Pd,Kc)
            % evaluatePartitions: Scale the likelihoods for correcting the density
            %
            % logLikelihood is the log-likelihood of each column of
            % detectionIndices
            %
            % Pd is detection probability of each component
            % 
            % Kc is clutter density.
            %
            % weights are weights for each component
            %
            % startID column index for start of ith partition in
            % detectionIndices
            %
            % endID column index for end of ith partition in
            % detectionIndices
            %
            numComps = numel(weights);   
            numPartitions = numel(startID);
            numCells = size(detectionIndices,2);
            classToUse = obj.pDataType;
                    
            % Convert to limits. log(x) = -1e4 is below the log of realmin
            % and 1e4 is above the log of realmax and hence are "safe"
            % limits. If the maximum absolute of log-likelihood is below or
            % above these limits, 100 times that value is used.
            if ~isempty(logLikelihood)
                lhood = logLikelihood(:);
                maxValue = max(abs(lhood(isfinite(lhood))));
            else
                maxValue = zeros(1,classToUse);
            end
            minLog = min(-1e4*ones(1,classToUse),-100*maxValue);
            maxLog = max(1e4*ones(1,classToUse),100*maxValue);
            
            logLikelihood = max(minLog,min(maxLog,logLikelihood));
            
            % Allocate memory
            scaledLikelihoods = cast(exp(logLikelihood),classToUse);
            partitionProbabilities = 1/numPartitions*ones(numPartitions,1,classToUse);
            cellDensities = zeros(1,numCells,classToUse);
            if numComps == 0
                return;
            end
            
            numDetsPerCell = sum(detectionIndices,1);
            
            % A cell is treated for false alarm if it has only 1 detection.
            falseAlarmLikelihood = cast(numDetsPerCell == 1,classToUse);
            
            % Clutter log-likelihood for each cell
            clutterPart = numDetsPerCell.*log(Kc);
            
            % totalLi = (li*Pdi/Kc^W);
            % totalLiwi = (li*Pdi*weightsi)/Kc^W
            % totalLogLi = log(li) + log(Pi) + log(weightsi) - W*log(Kc)
            % dW = delta(W,1) + sigma(totalLi);
            %
            % Factorize by totalL1 and take log on both sides.
            % dW = totalL1 * (totalL1/totalL1 + totalL2/totalL1 + totalL3/totalL1 ... + delta(W,1)/totalL1)
            % log(dW) = log(totalLog1) + log(sigma(totalLi/totalL1) + delta(W,1)/totalL1)
            % log(dW) = log(totalLog1) + log(exp(totalLogL1 - totalLogL1) + exp(totalLog2 - totalLogL1) ... + delta(W,1)*exp(-totalLogL1);
            % dW: cellDensities
            % Pdi = Prob. of detection of ith component
            % Li: Likelihood of cell against ith component
            % delta(W,1): True if number of detections in cell is one.
            
            totalLogLikelihood = bsxfun(@plus,bsxfun(@minus,logLikelihood,clutterPart),log(Pd(:)));
            totalLogLikelihoodW = bsxfun(@plus,totalLogLikelihood,log(weights(:)));
            
            % A for-loop is required to find the maximum index cell of each
            % likelihood column to compute the log(sum(exp)). We want the
            % exp to converge to 0 and not inf and hence l2/l1 must be less
            % than 1.
            logdW = zeros(1,numCells,classToUse);
            for i = 1:numCells
                thisCellLogLikelihood = [log(falseAlarmLikelihood(i));totalLogLikelihoodW(:,i)];
                [~,maxID] = max(thisCellLogLikelihood);
                totalLogDiffs = thisCellLogLikelihood - thisCellLogLikelihood(maxID);
                logdW(i) = thisCellLogLikelihood(maxID) + log(sum(exp(totalLogDiffs)));
            end
            logdW = max(minLog,min(maxLog,logdW));
            
            % Return output for cellDensities.
            cellDensities = exp(logdW);
            
            % Evaluate probability of each partition
            logpartitionDensities = zeros(numPartitions,1,obj.pDataType);
            for i = 1:numPartitions
                logpartitionDensities(i) = sum(logdW(startID(i):endID(i)));
            end
            logpartitionDensities = max(minLog,min(maxLog,logpartitionDensities));
            
            % Pick a partition density to evaluate their log using the log
            % sum formula shown below:
            % log(x + y) = log(x) + log(1 + y/x);
            % log(x + y) = log(x) + log(1 + exp(log(y) - log(x)));
            [~,maxID] = max(logpartitionDensities);
            logPartDensitiesDiff = logpartitionDensities - logpartitionDensities(maxID);
            logSumPartitionDensities = logpartitionDensities(maxID) + log(sum(exp(logPartDensitiesDiff)));
            
            % wp
            logPartitionProbabilities = logpartitionDensities - logSumPartitionDensities;
            partitionProbabilities = exp(logPartitionProbabilities);
            
            % Scale likelihoods by partition probabilities and cell
            % densities
            for i = 1:numPartitions
                logwp = logPartitionProbabilities(i);
                theseIndices = startID(i):endID(i);
                theseCellLogLikelihoods = totalLogLikelihood(:,theseIndices);
                theseCellLogDensities = logdW(theseIndices);
                theseScaledLikelihoods = exp(logwp + bsxfun(@minus,theseCellLogLikelihoods,theseCellLogDensities));
                scaledLikelihoods(:,theseIndices) = theseScaledLikelihoods;
            end
        end
    end
    
    methods
        function set.PartitioningFcn(obj,val)
           validateattributes(val,{'function_handle'},{'scalar'},'','PartitioningFcn');
           obj.PartitioningFcn = val;
        end
    end
end




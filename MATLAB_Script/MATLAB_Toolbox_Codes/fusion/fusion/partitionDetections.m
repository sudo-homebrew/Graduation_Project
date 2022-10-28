function varargout = partitionDetections(detectionsIn,varargin)
% partitionDetections Partition detections based on distance
%
% partitions = partitionDetections(detections) calculates multiple
% partitions using Distance-partitioning algorithm. The function
% auto-calculates the different thresholds for partitioning by using the
% Mahalanobis distance between the detections. These auto-calculated
% thresholds are bounded between 0.5 and 6.25. The function returns a
% maximum of 100 partitions.
% 
% partitions = partitionDetections(detections,...,'Algorithm',algorithm)
% returns partitions of the detections using the specified algorithm. The
% choices for algorithm are 'Distance-Partitioning' and 'DBSCAN'. The
% default value of algorithm is 'Distance-Partitioning'.
% 
% partitions = partitionDetections(detections,...,'Distance',distance)
% allows you to control the distance metric between two detections. The
% choice for distance are 'Mahalanobis' and 'Euclidean'. The default value
% of distance is 'Mahalanobis'. 
%
% detections is either: 
% A cell array of objectDetection objects.
% An array of objectDetection objects.
% An array of struct with same fields as properties of objectDetection.
% 
% partitions is a N-by-Q matrix where N is the number of input detections
% and Q is the number of partitions.
%
% When algorithm is specified as 'Distance-Partitioning':
%
% partitions = partitionDetections(detections,'Algorithm','Distance-Partitioning') 
% is identical to partitionDetections(detections)
% 
% partitions = partitionDetections(detections,tLower,tUpper,'Algorithm','Distance-Partitioning') 
% allows specifying the lower and upper thresholds, tLower and tUpper,
% respectively. tLower and tUpper are scalar values which specify the lower
% and upper threshold for distance between detections belonging to the same
% subset of partition.
%
% partitions = partitionDetections(detections,tLower,tUpper,'MaxNumPartitions',maxNumber,'Algorithm','Distance-Partitioning') 
% allows specifying the maximum number of allowed partitions along with
% lower and upper thresholds.
%
% [partitions, idx] = partitionDetections(detections,allThresholds,'Algorithm','Distance-Partitioning')
% calculates multiple partitions at each threshold specified by
% allThresholds. allThresholds is a vector of length P. As multiple
% thresholds can correspond to the same partition of detections, partitions
% is a N-by-Q matrix of unique partitions, where Q is less than or equal to
% P. detections is a cell array of objectDetection objects. The second
% output, idx specifies the mapping between allThresholds and the generated
% partitions. partitions(:,idx(i)) represents the partition generated from
% distance threshold equal to allThresholds(i). 
%
% When algorithm is specified as 'DBSCAN':
% 
% partitions = partitionDetections(detections,'Algorithm','DBSCAN')
% calculates multiple partitions of the detections by using 10 distance
% threshold (epsilon or neighbor search radius) values linearly spaced
% between 0.25 and 6.25. The function uses a minimum of 3 points per
% cluster.
% 
% partitions = partitionDetections(detections, epsilon, minNumPts, 'Algorithm', 'DBSCAN')
% calculates the partitions of detections by using the specified values of
% thresholds, epsilon, and minimum number of points per cluster, minNumPts.
% epsilon is an array defining the thresholds. minNumPts is either a scalar
% value or an array with the number of elements same as that of epsilon.
%
% [partitions, idx] = partitionDetections(detections,epsilon, minNumPts,'Algorithm','DBSCAN')
% provides an additional output, idx, specifying the mapping between unique
% partitions and the input epsilon values. partitions(:,idx(i)) represents
% the partition generated from epsilon equal to epsilon(i).
%
%   % Example: Generate multiple partitions from detection using Distance-Partitioning
%   % --------------------------------------------------------------
%   % Randomly generate some detections clustered around 5 points  
%   rng(2018); % For reproducible results
%   detections = cell(10,1);
%   for i = 1:numel(detections)
%       id = randi([1 5]);
%       detections{i} = objectDetection(0,[id;id] + 0.1*randn(2,1));
%       detections{i}.MeasurementNoise = 0.01*eye(2);
%   end
% 
%   % Partition data
%   partitions = partitionDetections(detections);
%   numPartitions = size(partitions,2);
%   
%   % Visualize partitions. Each color represents a cluster.
%   for i = 1:numPartitions
%     numClusters = max(partitions(:,i));
%     p = [detections{:}];
%     measurements = [p.Measurement];
%     subplot(3,ceil(numPartitions/3),i);
%     for k = 1:numClusters
%         ids = partitions(:,i) == k;
%         plot(measurements(1,ids),measurements(2,ids),'.','MarkerSize',30);
%         hold on;    
%     end
%     title(['Partition ',num2str(i)]);
%   end
%
% References:
%   [1] Granstrom, Karl, Christian Lundquist, and Omut Orguner. "Extended
%       target tracking using a Gaussian-mixture PHD filter." IEEE
%       Transaction on Aerospace and Electronic Systems 48.4 (2012):
%       3268-3286
%   [2] Ester M., Kriegel H.-P., Sander J., and Xu X. "A Density-Based
%       Algorithm for Discovering Clusters in Large Spatial Databases with
%       Noise." Proc. 2nd Int. Conf. on Knowledge Discovery and Data Mining,
%       Portland, OR, AAAI Press, 1996, pp. 226-231.
%
% See also: objectDetection, mergeDetections, trackerPHD

% Copyright 2018-2021 The MathWorks, Inc.

%#codegen

% Validate detections input
validateattributes(detectionsIn,{'cell','struct','objectDetection'},{'vector'},mfilename,'Detections',1);

% Validate the cell or struct and convert to cell array of detections
if ~iscell(detectionsIn)
    detections = matlabshared.tracking.internal.fusion.makeDetectionCells(detectionsIn); 
else
    detections = detectionsIn;
end

% Parse algorithm input
algIdx = fusion.internal.findProp('Algorithm',varargin{:});
inputs = {varargin{1:algIdx-1},varargin{algIdx + 2:end}};
if algIdx < numel(varargin)
    % user-provided algorithm
    algIn = varargin{algIdx + 1};
    if coder.target('MATLAB')
        alg = localValidateString(lower(algIn));
    else
        alg = validatestring(algIn,{'Distance-Partitioning','DBSCAN'},'Algorithm');
    end
else
    % Use default
    alg = 'Distance-Partitioning';
end

% Divert all inputs to respective algorithms
switch lower(alg)
    case 'distance-partitioning'
        [varargout{1:nargout}] = fusion.internal.partitionDP(detections, inputs{:});
    case 'dbscan'
        [varargout{1:nargout}] = fusion.internal.partitionDBSCAN(detections, inputs{:});
    otherwise
        assert(false);
end
end

function algorithm = localValidateString(algorithm)
arguments %#ok<EMFIV> 
    algorithm {mustBeMember(algorithm,{'dbscan','distance-partitioning'})}
end
end


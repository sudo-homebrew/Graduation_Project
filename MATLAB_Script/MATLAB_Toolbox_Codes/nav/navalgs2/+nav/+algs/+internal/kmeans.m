function [clusterId, centers, exitFlag] = kmeans(pts, K, centers)
%This function is for internal use only. It may be removed in the future.

% KMEANS Simple K-Means clustering.
% [IDX, C] = kmeans(POINTS, K) clusters the POINTS into K clusters using
% the simple k-means clustering algorithm. POINTS is a N-by-2 matrix, where
% N is the number of points. K is a positive scalar. kmeans++ is used to
% initialize cluster centers. IDX is a N-by-1 vector where each element
% contains the cluster id for the point in corresponding row in POINTS, all
% value in IDX are between 1 and K. C is a K-by-2 matrix containing cluster
% centers. C(IDX(1),:) will give the cluster center the first point belongs
% to.
%
% [...] = kmeans(POINTS, K, CENTERS) uses the CENTERS as initialization
% of cluster centers. CENTERS should be a K-by-2 matrix.
%
% [..., EXITFLAG] = kmeans(...) additionally returns an exit flag
% indicating the reason for exit. FLAG is a numeric vector containing
% status of execution. Exit codes are as follows:
%   0 - Exit after convergence.
%   1 - Exit after Maximum number of iterations are executed.
%   2 - Error because number of points are less than number of clusters.
%   3 - More than 10% iterations required reseeding on encountering empty
%       cluster.
%
% References:
%   [1] Wikipedia contributors, "K-means clustering," Wikipedia, The Free
%       Encyclopedia,
%       https://en.wikipedia.org/w/index.php?title=K-means_clustering&oldid=947989652
%       (accessed March 31, 2020).
%
%   [2] Wikipedia contributors, "K-means++," Wikipedia, The Free
%       Encyclopedia,
%       https://en.wikipedia.org/w/index.php?title=K-means%2B%2B&oldid=940216119
%       (accessed March 31, 2020).
%
%#codegen

% Copyright 2020 The MathWorks, Inc.

narginchk(2,3);

ptsDimAllowed = 2;

validateattributes(pts, {'numeric'},...
    {'nonempty', '2d', 'ncols', ptsDimAllowed, 'nonnan', 'finite', 'real',...
        'nonsparse'},...
    mfilename, 'Points', 1);

validateattributes(K, {'numeric'},...
    {'nonempty', 'scalar', 'nonnan', 'finite', 'real',...
        'integer', 'positive'}, ...
    mfilename, 'K', 2);

numExitCodes = 4;
% Order: convergence, max iter, num PMs>K, reseeds > 10% of iter.
status = zeros(1,numExitCodes);

if(size(pts,1) < K)
    % Not using the status here to throw error there is no other possible
    % value.
    clusterId = [];
    centers = [];
    exitFlag = 2;
    return;
end

% 1. Get initial center points.
if nargin <= 2
    centers = getInitialClusterCenters(pts, K);
else
    % Check if data in center consists of normal numbers, it has K
    % number of rows and same number of columns as points.
    validateattributes(centers, {'numeric'},...
        {'nonempty', '2d', 'ncols', size(pts,2), 'nrows', K,...
            'nonnan', 'finite', 'real', 'nonsparse'},...
        mfilename, 'center', 3);
end

% Assign clusters for the initial centers.
clusterId = assignPointsToCluster(pts, centers);

% 2. Loop till convergence or max number of iterations.
maxIter = 100;
converged = false;

% To keep track of number of attempts.
iter=1;
emptyClusterRetry = 0;
while(~converged && iter < maxIter)
    % 2.0 Store and assign all required data for the loop
    prevClusterId = clusterId;
    % 2.1 Recompute cluster centers
    centers = computeNewCenters(pts, clusterId);
    % 2.2 Compute distance of each point to each center.
    % 2.3 Assign each point to a cluster.
    % Do both in one shot.
    [clusterId, distToCenter] = assignPointsToCluster(pts, centers);
    % 2.4 Check if there are empty cluster.
    uniqueClusterIds = unique(clusterId(:));
    if numel(uniqueClusterIds) < K
        % Randomly seed centers for empty clusters.

        % 2.4.1 Find which clusters are missing.
        missingIds = find(ismember(1:K, uniqueClusterIds)==0);

        % 2.4.2 Seed new cluster centers for the empty clusters.
        % Add ||x||^2 to relative distance used by assignPointsToCluster to
        % get the squared distance.
        sqDistToCenter = distToCenter + sum(pts.^2,2);
        % Compute new centers for the empty clusters.
        centers(missingIds,:) = seedClusterCenters(pts, sqDistToCenter,...
            numel(missingIds));

        % 2.4.3 Reassign label for the new centers.
        % Although distToCenter is not used in the following code, it is a
        % potential silent hard to reproduce sporadic bug to not update it.
        % If it's used for reporting or decision making in the future then
        % centers would have changed but not distances.
        [clusterId, distToCenter] = assignPointsToCluster(pts, centers); %#ok<ASGLU>

        % To track how many attempts were made to reseed the empty
        % clusters.
        emptyClusterRetry = emptyClusterRetry + 1;

        % Can add a continue here, but then there is a theoretical chance
        % of an infinite loop and the below operations are not that heavy,
        % so letting it be.
    end

    % 2.5 Check if any point moved clusters.
    if all(prevClusterId == clusterId)
        % 2.4.1 If no point moved the set converged to true.
        converged = true;
        status(1) = 1;
    end

    % 2.6 Update data for the next loop
    iter = iter + 1;
end

% Indicate 2nd element as 1 to return exitflag 1 if maximum number of
% iterations were reached.
status(2) = (iter == maxIter);

areSomeClustersEmpty = (numel(unique(clusterId(:))) < K);
isEmptyRetryFrequent = (emptyClusterRetry > 0.1*iter);
% Indicate 4th element as 1 to return exit flag 3 if reseed of empty
% cluster was required for more than 10% of iteration.
status(4) = areSomeClustersEmpty && isEmptyRetryFrequent;

% subtract -1 for 0 indexing.
exitFlag = find(status) - 1;

end


function [clusterId, distToCenter] = assignPointsToCluster(pts, centers)
% Dist between each point can be computed using sqrt(sum((X-Y(i)).^2,2)),
% but it requires 1 for loop to iterate over all the cluster centers as
% there is a mismatch in the number of points and number of clusters. There
% are couple of "hacks" which can be used to use vectorization.
% 1. We don't need distance, squared distance does the job as distance is
%    only used for ranking, hence relative values are important.
% 2. Expanding ||x-y||^2 as ||x||^2 + ||y||^2 - 2x'y can allow use of
%    functions which can be easily vectorized, like sum or dot for ||x||^2
%    and ||y||^2, and matrix multiplication for 2x'y
% 3. Since, we are ranking each point's distance to each cluster,
%    computation of ||x||^2 is redundant as it's a constant quantity for
%    each point.
% So, compute ||y||^2 - 2x'y and find the minimum value to assign a cluster
% to points.
y2 = sum(centers.^2, 2)';
% Since points and centers each point is row vector x'y becomes xy'
xty2 = (pts*(centers'*2));
distToCenter = repmat(y2, size(pts,1), 1) - xty2;

% Get the id of the closest cluster. If points is a N-by-2 matrix, clusterId is
% n-by-1 column vector with a 1 to 1 mapping for point to cluster id.
[~,clusterId] = min(distToCenter, [], 2);
end

function newCenters = computeNewCenters(pts, clusterId)
% We want to sum x and y of each point and then divide by number of points.
% summation with if and else can be replaced by multiplication with 0 and 1
% ClusterId right now are represented as linear indexing, converting it to
% logical indexing and multiplying would give us only the points belonging
% to the cluster. This requires clusterId to be converted into a matrix
% where each id is represented by a vector of 0's and 1's. Since in the
% vector most elements will be zero and few 1's we can use sparse matrix.
numPoints = size(pts,1);
% In the full matrix, each row represents cluster and each column a point.
% repmat might required for codegen.
clusterIdMat = sparse(clusterId, (1:numPoints)', ones(numPoints,1));

% ClusterIdMat is K-by-n, points is n-by-2, sumXY wold be K-by-2.
sumXY = clusterIdMat*pts;
% numPointInCluster is K-by-1
numPointInCluster = sum(clusterIdMat,2);

% Mean points of cluster are the new centers.
newCenters = sumXY./repmat(numPointInCluster, 1, 2);

end

function centers = getInitialClusterCenters(pts, K)
% Use k-means++ to initialize the clusters.

centers = zeros(K,size(pts,2));
numPoints = size(pts,1);

% choose first center randomly from the points.
centers(1,:) = pts(randi(numPoints,1,1),:);
% Same size as points
repeatedCenter = repmat(centers(1,:), size(pts, 1), 1);
squaredDist = sum((pts - repeatedCenter).^2, 2);
centers(2:K,:) = seedClusterCenters(pts, squaredDist, K-1);

end

function centers = seedClusterCenters(pts, squaredDist, numCenters)
% Iterate over all the missing cluster ids.
% Needs to be done in a loop.
centers = zeros(numCenters, size(pts,2));

for idx = 1:numCenters
    % 1. Generate new center.
    newIdx = weightedRandomIntegerSample(squaredDist);
    centers(idx,:) = pts(newIdx(1,1),:);

    % 2. Squared distance computation for the new cluster.
    repeatedCenter = repmat(centers(idx,:), size(pts,1), 1);
    squaredDist = min(sum((pts - repeatedCenter).^2, 2), squaredDist);
end

end

function randomSample = weightedRandomIntegerSample(weights)
    % Normalize distance to generate proportional probability values.
    sampleProbability = weights/sum(weights,1);
    % Compute probability weights based on distance.
    probabilityWeights = cumsum(sampleProbability);
    % Uniformly sample data point
    randomVal = rand();
    % Use the random sample to choose the index bin. Larger the distance,
    % larger the sampleProbability, larger the gap with previous
    % element. A larger gap leads to a higher probability of a uniformly
    % sampled point to fall in the range. Take the first value larger than
    % sampled value as the new index of the center.
    randomSample = find( randomVal < probabilityWeights, 1);

    if isempty(randomSample)
        randomSample = randi(numel(weights), 1);
        return
    end
end

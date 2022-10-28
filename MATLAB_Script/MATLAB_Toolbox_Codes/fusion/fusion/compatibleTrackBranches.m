function [hyps, hypScores] = compatibleTrackBranches(clu,incompatibleBranches,branchScores,maxNumHypotheses)
%compatibleTrackBranches - Formulate global hypotheses from clusters
%   [hyps, hypScores] = compatibleTrackBranches(clusters,incompatibleBranches,scores,maxNumHypotheses) 
%   returns the list of hypotheses, hyps, and their scores, hypScores.
%   Hypotheses are sets of compatible track branches, i.e., branches that
%   do not belong to the same track or share a detection in their history.
%   The hypotheses list, hyps, is an M-by-H logical matrix, where M is the
%   number of branches and H is defined by the input maxNumHypotheses. The
%   hypotheses scores, hypScores, is a 1-by-H scores vector. The score of
%   each hypothesis is the sum of scores of all the branches included in
%   the hypothesis.
%
%   The inputs are clusters, a mapping of individual branches to clusters
%   (see below), an M-by-M logical matrix incompatibleBranches, where the
%   (i,j) element is true if branches i and j are pairwise-incompatible,
%   an M-by-1 or M-by-2 matrix listing the branch scores, branchScores, and
%   maxNumHypotheses, a scalar that defined the maximum number of
%   hypotheses.
%
%   The input clusters can be in any of the following formats returned by
%   clusterTrackBranches.
%     * An M-by-P logical matrix, where the (i,j) element is true if branch
%       j is contained in cluster i. Each cluster is a column vector.
%     * A vector, where the i-th element gives the index of the cluster 
%       that contains branch i.  
%     * A cell array c, with c{j} containing the IDs of all the branches in
%       cluster j.
%
%   Example:
%   --------
%   % Formulate 6 global hypotheses from the clusters, incompatible tracks,
%   % and scores
%   clusters = logical([...
%       0 0 1 0 0 0 1 0 0 0 0 0
%       0 0 0 0 0 0 0 1 0 0 0 0
%       1 1 0 1 1 1 0 0 1 1 1 1]')
%   incompatibleBranches = logical([...
%        1   0   0   1   0   1   0   0   0   0   0   0
%        0   1   0   0   1   1   0   0   0   0   0   0
%        0   0   1   0   0   0   1   0   0   0   0   0
%        1   0   0   1   1   1   0   0   0   0   0   0
%        0   1   0   1   1   1   0   0   0   0   0   0
%        1   1   0   1   1   1   0   0   0   0   0   0
%        0   0   1   0   0   0   1   0   0   0   0   0
%        0   0   0   0   0   0   0   1   0   0   0   0
%        0   0   0   0   0   0   0   0   1   1   0   1
%        0   0   0   0   0   0   0   0   1   1   1   1
%        0   0   0   0   0   0   0   0   0   1   1   1
%        0   0   0   0   0   0   0   0   1   1   1   1])
%   scores = [81.4; 90.5; 12.7; 91.3; 63.2; 9.7; 27.8; 54.6; 95.7; 96.4; 15.7; 97.1]
%   maxNumHypotheses = 6;
%
%   % Get the hypotheses
%   [hyps, hypScores] = compatibleTrackBranches(clusters,incompatibleBranches,scores,maxNumHypotheses)
%
%   See also: trackerTOMHT, clusterTrackBranches, pruneTrackBranches

%   References:
%   [1] J.R. Werthmann, "A Step-by-Step Description of a Computationally
%       Efficient Version of Multiple Hypothesis Tracking", SPIE Vol. 1698
%       Signal and Processing of Small Targets, pp 288-300, 1992.

% Copyright 2018 The MathWorks, Inc.

%#codegen

% The hypotheses management algorithm contains the following
% steps:
%   1. Formulate cluster hypotheses
%   2. Merge cluster hypotheses to global hypotheses
%   3. Score the hypotheses by adding track scores

% Input validation
narginchk(4,4);

% incompatibleBranches
validateattributes(incompatibleBranches,{'logical','numeric'},...
    {'nonsparse','binary','2d','square'},...
    mfilename,'incompatibleBranches');
coder.internal.assert(isequal(incompatibleBranches,incompatibleBranches'),'fusion:compatibleTrackBranches:expectedSymmetric','incompatibleBranches');
coder.internal.assert(trace(~incompatibleBranches)==0,'fusion:compatibleTrackBranches:expectedSelfEdges','incompatibleBranches');
numBranches = size(incompatibleBranches,1);

% Parse the clusters input in various formats
clusters = localParseClusters(clu,numBranches);

% clusters
numClusters = size(clusters,2);
% branchScores
validateattributes(branchScores,{'double','single'},...
    {'nonsparse','real','nrows',numBranches},mfilename,'branchScores');
% maxNumHypotheses
validateattributes(maxNumHypotheses,{'numeric'},...
    {'nonsparse','real','finite','positive','integer','scalar'},mfilename,'maxNumHypotheses');

% Allocate memory
hyps = false(numBranches,maxNumHypotheses);
hypScores = zeros(1,maxNumHypotheses,'like',branchScores);
nHyps = 1;

% Edge case, when no branches exist, clusters is empty
if isempty(clusters)
    return
end

% For the rest of the clusters, find the best combinations of hypotheses
% and merge them into bigger hypotheses
scores = branchScores(:,1);
for i = 1:numClusters
    clusterHyps = false(numBranches,maxNumHypotheses);
    clusterScores = -inf(1,maxNumHypotheses,'like',branchScores);
    scoredTrackInCluster = scores(clusters(:,i),:);
    incompClusterTracks = incompatibleBranches(clusters(:,i),clusters(:,i));
    [nCHyps,cHyps,cScores] = localCluster2Hypotheses(scoredTrackInCluster,incompClusterTracks,maxNumHypotheses);
    clusterHyps(clusters(:,i),1:nCHyps) = cHyps(:,1:nCHyps);
    clusterScores(1:nCHyps) = cScores(1:nCHyps);
    bestCombs = localFindBestCombinations(nHyps,hypScores,nCHyps,clusterScores,maxNumHypotheses);
    [nHyps,hyps,hypScores] = localMergeHypotheses(hyps,hypScores,clusterHyps,clusterScores,bestCombs);
end
end

function clusters = localParseClusters(clu,numBranches)
%localParseClusters Parses input clusters from various formats to logical
%   The input clusters can be in any of the following formats returned by
%   clusterTrackBranches.
%     * An M-by-P logical matrix, where the (i,j) element is true if branch
%       j is contained in cluster i. Each cluster is a column vector.
%     * A vector, where the i-th element gives the index of the cluster 
%       that contains branch i.  
%     * A cell array c, with c{j} containing the IDs of all the branches in
%       cluster j.
if iscell(clu)
    numClusters = numel(clu);
    branchCount = uint32(0);
    for i = 1:numClusters
        validateattributes(clu{i},{'numeric'},...
            {'real','nonsparse','vector','positive','integer'},...
            mfilename,'each element of the cell ''clusters''');
        branchCount = branchCount + numel(clu{i});
    end
    coder.internal.assert(branchCount==numBranches,'fusion:compatibleTrackBranches:incorrectNumBranches','incompatibleBranches');
    clusters = false(branchCount,numClusters);
    branchIDs = zeros(branchCount,1,'uint32');
    currentInd = 0;
    for i = 1:numClusters
        branchIDs(currentInd + (1:numel(clu{i}))) = clu{i}(:);
        clusters(currentInd + (1:numel(clu{i})), i) = true;
        currentInd = currentInd + numel(clu{i});
    end
    [~,I] = sortrows(branchIDs);
    clusters = clusters(I,:); % Rearrange by branchID
elseif isnumeric(clu) && isvector(clu) && numel(clu)==numBranches
    validateattributes(clu,{'numeric'},...
            {'real','nonsparse','vector','positive','integer'},...
            mfilename,'the vector ''clusters''');
    clusterIDs = unique(clu);
    numClusters = numel(clusterIDs);
    clusters = false(numel(clu),numClusters);
    for i = 1:numClusters
        inCluster = (clu == clusterIDs(i));
        clusters(inCluster,i) = true;
    end
else
    validateattributes(clu,{'logical','numeric'},...
        {'nonsparse','binary','2d'},mfilename,'clusters');
    coder.internal.assert(size(clu,2)<=numBranches,'fusion:compatibleTrackBranches:expectedMoreRows');
    clusters = clu;
end
end

function [numHyps,hyps,hypScores] = localCluster2Hypotheses(scores,incompatibilityList,maxNumHypotheses)
%localCluster2Hypotheses  Creates consistent cluster hypotheses
%  [numHyps, hyps] = cluster2hypotheses(cluster,incompatibilityList)
%  returns the number of consistent hypotheses, numHyps, and hypotheses,
%  hyps. Hypotheses are created such that the tracks in each hypothesis are
%  consistent. Cluster is a group of inconsistent tracks and incompatbility
%  list provides the information of pairs of tracks are inconsistent.
%
%  Inputs:
%    scores              - An N-by-1 matrix of track scores.
%    incompatibilityList - An N-by-N logical matrix. If the (i,j) element
%                          of the matrix is true, the i-th and j-th tracks
%                          in cluster are inconsistent.
%    maxNumHypotheses    - Maximum number of hypotheses to return.
%
%  Outputs:
%    hyps       - An N-by-maxNumHypotheses logical matrix.
%    hypScores  - A 1-by-maxNumHypotheses hypothesis scores.
%
%  Example:
%  --------
% % cluster contains a list of track scores
%   clusterList = [97;94;92;90;88;80;79;79];
% % incompatibilityList:
%   incompatibilityList = [
%     1   1   0   0   0   0   0   0
%     1   1   1   1   0   0   0   1
%     0   1   1   1   0   0   0   1
%     0   1   1   1   1   1   0   1
%     0   0   0   1   1   1   0   0
%     0   0   0   1   1   1   1   1
%     0   0   0   0   0   1   1   1
%     0   1   1   1   0   1   1   1];
%
%   [numHyps, hyps] = cluster2hypotheses(clusterList,incompatibilityList)

% Allocate memory
numClusterTracks = size(incompatibilityList,1);
hyps = false(numClusterTracks,maxNumHypotheses);
hypScores = zeros(1,maxNumHypotheses,'like',scores);

% Find hypotheses
cliques = fusion.internal.findMaxCliques(~incompatibilityList);
numHyps = min(size(cliques,2),maxNumHypotheses);
unsortedHypScores = scores' * cliques;
[~,I] = sort(unsortedHypScores,2,'descend');

hyps(:,1:numHyps) = cliques(:,I(1:numHyps));
hypScores(1:numHyps) = unsortedHypScores(I(1:numHyps));
end

function combs = localFindBestCombinations(nS1,scores1,nS2,scores2,maxNumHypotheses)
%localFindBestCombinations Finds the best combinations of scores
%   combs = localFindBestCombinations(scores1,scores2,maxNumHypotheses) finds
%   the best maxNumHypotheses combinations of scores from the two sets of
%   scores: scores1, scores2. 

% scores are already sorted
allCombs = [zeros(maxNumHypotheses^2,2,'like',scores1),-inf(maxNumHypotheses^2,1,'like',scores1)];
for i = 1:nS1
    for j = 1:nS2
        allCombs(j+(i-1)*maxNumHypotheses,:) = [i,j,scores1(i)+scores2(j)];
    end    
end
allCombs = sortrows(allCombs,3,'descend');
combs = allCombs(1:maxNumHypotheses,1:2);
end

function [numCombs,mergedHyps,mergedHypScores] = localMergeHypotheses(hyps,hypScores,clusterHyps,clusterScores,bestCombs)
%localMergeHypotheses Returns the merged hypotheses from two cluster hypotheses
mergedHyps = false(size(hyps));
mergedHypScores = zeros(size(hypScores),'like',hypScores);
numCombs = size(bestCombs,1);
for i = 1:size(bestCombs,1)
    if bestCombs(i,1)==0
        numCombs = i-1;
        break
    end
    mergedHyps(:,i) = or(hyps(:,bestCombs(i,1)),clusterHyps(:,bestCombs(i,2)));
    mergedHypScores(i) = hypScores(bestCombs(i,1)) + clusterScores(bestCombs(i,2));
end
end
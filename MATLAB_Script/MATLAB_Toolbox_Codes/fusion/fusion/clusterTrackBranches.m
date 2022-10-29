function [clusters,incompatibleBranches] = clusterTrackBranches(branchHistory,varargin)
%clusterTrackBranches Cluster track-oriented multi-hypothesis history
%   [clusters,incompatibleBranches] = clusterTrackBranches(branchHistory)
%   computes the clusters and incompatibility matrix for a set of branches.
%   clusters is an M-by-P logical matrix (P<=M), and clusters(i,j)=true if
%   branch j belongs to cluster i. M is the number of rows (branches) in
%   branchHistory.
%   Branches i, j, and k belong in the same cluster if at least branch i
%   and branch j are pairwise-incompatible and branch j and branch k are
%   pairwise-incompatible.
%   incompatibleBranches is an M-by-M logical matrix, where the (i,j)
%   element is true if branches i and j are incompatible. Two branches are
%   considered to be pairwise-incompatible if they share a track ID (first 
%   column of branchHistory) or detections that fall in their gates in the 
%   last D scans (from the 4th column to the last column).
%   branchHistory is an M-by-(3+S*D) matrix that provides the history of
%   branches (track hypotheses), where M is the number of branches, S is
%   the number of sensors and D is the number of scans (history depth). The
%   format of branchHistory is the same as defined in the output from the
%   step method in trackBranchHistory.
%
%   [...] = clusterTrackBranches(..., 'OutputForm', OUT) specifies the
%   format in which the clusters output is returned. OUT can be:
%       'logical' - Return an M-by-P logical matrix as described above.
%                   This is the default output and the only one supported
%                   for code generation.
%       'vector'  - Return a vector clusters. clusters(i) gives the index
%                   of the cluster that contains branch i.
%       'cell'    - Return a cell array c, with c{j} containing the branch
%                   IDs (3rd column of branchHistory) of all the nodes in
%                   cluster j.
%
%   Example:
%   --------
%   % Define branch history.
%   branchHistory = uint32([     
%      4     9     9     0     0     1     0     0     0     0     0
%      5    10    10     0     0     0     2     0     0     0     0
%      6    11    11     0     0     3     0     0     0     0     0
%      1    12    12     0     0     1     0     1     0     0     0
%      1    13    13     0     0     0     2     1     0     0     0
%      1    14    14     0     0     1     2     1     0     0     0
%      2    15    15     0     0     3     0     3     0     0     0
%      3    16    16     0     0     0     4     0     4     0     0
%      7     0    17     1     0     0     0     0     0     0     0
%      1     5    18     1     0     0     0     0     2     0     0
%      1     5    19     0     2     0     0     0     2     0     0
%      1     5    20     1     2     0     0     0     2     0     0]);
%
%   % Get the list of clusters and the list of incompatible branches
%   [clusters, incompBranches] = clusterTrackBranches(branchHistory)
%
%   % Show incompBranches as a graph and see the clusters. You can see the 
%   % three distinct clusters.
%   branchIDs = cellstr(num2str(branchHistory(:,3)));
%   g = graph(incompBranches,branchIDs,'omitselfloops');
%   plot(g)
%
%   See also: trackerTOMHT, trackBranchHistory, compatibleTrackBranches

%   References:
%   [1] J.R. Werthmann, "A Step-by-Step Description of a Computationally
%       Efficient Version of Multiple Hypothesis Tracking", SPIE Vol. 1698
%       Signal and Processing of Small Targets, pp 288-300, 1992.

% Copyright 2018 The MathWorks, Inc.

%#codegen

% The process of clustering:
%   1. Find incompatible branches within each cluster
%   2. Formulate clusters of branches

narginchk(1,3)

validateattributes(branchHistory,{'numeric'},...
    {'2d','real','nonsparse','nonnegative','integer'},...
    mfilename,'branchHistory')
coder.internal.assert(size(branchHistory,2)>3,'fusion:clusterTrackBranches:expected4Columns')

if numel(varargin)>0
    coder.internal.assert(numel(varargin)==2,'fusion:clusterTrackBranches:insufficentNargin');
    validatestring(varargin{1},{'OutputForm'},mfilename);
    out = char(validatestring(varargin{2},{'logical','vector','cell'},mfilename,'out'));
else
    out = 'logical';
end
coder.internal.assert(coder.target('MATLAB') || strcmpi(out,'logical'), 'fusion:clusterTrackBranches:outputNotSupportedInCodegen','''logical'' output format')

% Find incompatible tracks
incompatibleBranches = localTrackIncompatibility(branchHistory);

% Cluster the branches
clu = localClusterTracks(incompatibleBranches);
numClusters = size(clu,2);

if strcmpi(out,'logical') || ~coder.target('MATLAB')
    clusters = clu;
elseif strcmpi(out,'vector')
    % Output as a vector
    clusters = uint32(clu*(1:numClusters)');
else
    clusters = cell(1,numClusters);
    for i = 1:numClusters
        clusters{i} = branchHistory(clu(:,i),3)';
    end
end
end

function incompatibilityList = localTrackIncompatibility(branchHistory)
%localTrackIncompatibility  Calculate list of incompatible cluster branches
%   incompatibilityList = incompatibleClusterTracks(cluster) returns the
%   N-by-N logical matrix incompatibilityList for the list of N branches in
%   cluster. A branch is incompatible with another branch if they directly
%   share a detection in their branch history, branchHistory.
% 
%   Inputs:
%       branchHistory - M-by-(3+S*D) matrix that provides the history of 
%                       branches (track hypotheses), where M is the number
%                       of branches, S is the number of sensors and D is
%                       the number of scans (history depth).
%
%   Output:
%       incompatibilityList - an N-by-N logical matrix. The (i,j) value is
%                             true if the i-th and j-th branches directly
%                             share the same detection in their history. By
%                             definition, the values along the diagonal are
%                             true.
%
% Example:
% --------
%   branchHistory = uint32([     
%      4     9     9     0     0     1     0     0     0     0     0
%      5    10    10     0     0     0     2     0     0     0     0
%      6    11    11     0     0     3     0     0     0     0     0
%      1    12    12     0     0     1     0     1     0     0     0
%      1    13    13     0     0     0     2     1     0     0     0
%      1    14    14     0     0     1     2     1     0     0     0
%      2    15    15     0     0     3     0     3     0     0     0
%      3    16    16     0     0     0     4     0     4     0     0
%      7     0    17     1     0     0     0     0     0     0     0
%      1     5    18     1     0     0     0     0     2     0     0
%      1     5    19     0     2     0     0     0     2     0     0
%      1     5    20     1     2     0     0     0     2     0     0]);
%
% incompatibilityList = localTrackIncompatibility(branchHistory)

numTracks = size(branchHistory,1); % Number of branches
incompatibilityList = true(numTracks,numTracks);

for i=1:numTracks
    for j=i+1:numTracks
        incompatibilityList(i,j) = any(fusion.internal.isequalNZ(branchHistory(i,[1 4:end]),branchHistory(j,[1 4:end])));
        incompatibilityList(j,i) = incompatibilityList(i,j);
    end
end
end

function outputClusters = localClusterTracks(incompatibleTracks)
%localClusterTracks Return the clusters from an incompatible tracks matrix
%   clusters = localClusterTracks(incompatibleTracks) returns the list of
%   clusters as an N-by-C logical matrix, where N is the number of tracks
%   and C is the number of clusters, C<=N. incompatibleTracks is an N-by-N
%   logical matrix, where the (i,j) element indicates if track i and track
%   j are pairwise-incompatible. 
%   If track i and track j are pairwise-incompatible and track j and track
%   k are pairwise-incompatible, then tracks i, j, and k all belong in the
%   same cluster.
% Example:
% --------
% incompatibleTracks = logical([
%    1   0   0   1   0   1   0   0   0   0   0   0
%    0   1   0   0   1   1   0   0   0   0   0   0
%    0   0   1   0   0   0   1   0   0   0   0   0
%    1   0   0   1   1   1   0   0   0   1   1   1
%    0   1   0   1   1   1   0   0   0   1   1   1
%    1   1   0   1   1   1   0   0   0   1   1   1
%    0   0   1   0   0   0   1   0   0   0   0   0
%    0   0   0   0   0   0   0   1   0   0   0   0
%    0   0   0   0   0   0   0   0   1   1   0   1
%    0   0   0   1   1   1   0   0   1   1   1   1
%    0   0   0   1   1   1   0   0   0   1   1   1
%    0   0   0   1   1   1   0   0   1   1   1   1]);
% outputClusters = localClusterTracks(incompatibleTracks)

% Initialize clusters
numTracks = size(incompatibleTracks,1);
clusters = incompatibleTracks;
branch2cluster = (1:numTracks)';

% Compare branches pairwise. Tracks that share the same detection should be
% clustered in the same cluster so merge the 2 clusters in which the
% branches are
for i = 1:numTracks
    for j = i+1:numTracks
        if branch2cluster(i)==branch2cluster(j) % Has already been merged
            continue
        end
        if incompatibleTracks(i,j)
            toClusterIdx = branch2cluster(i);
            fromClusterIdx = branch2cluster(j);
            clusters(:,toClusterIdx) = or(clusters(:,toClusterIdx),clusters(:,fromClusterIdx));
            clusters(:,fromClusterIdx) = false;
            branch2cluster(branch2cluster == fromClusterIdx) = toClusterIdx;
        end
    end
end
clusterNums = unique(branch2cluster);
outputClusters = clusters(:,clusterNums);
end
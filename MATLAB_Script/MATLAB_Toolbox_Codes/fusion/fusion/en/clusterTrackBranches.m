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

 
% Copyright 2018 The MathWorks, Inc.


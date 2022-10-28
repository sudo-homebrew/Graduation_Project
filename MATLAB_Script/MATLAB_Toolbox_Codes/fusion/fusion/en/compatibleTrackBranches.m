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

 
% Copyright 2018 The MathWorks, Inc.


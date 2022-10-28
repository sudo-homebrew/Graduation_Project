%pruneTrackBranches  Prune track branches with low likelihood
%   [toPrune,globalProbability] = pruneTrackBranches(history,scores,hypotheses)
%   returns the M-element logical array, toPrune, of branches that should
%   be pruned and the M-element array of global (posterior) branch
%   probabilities, globalProbability. M is the number of branches. The
%   inputs are an M-by-(3+S*D) branch history matrix, history (see
%   trackBranchHistory for more details), an M-element array of branch
%   scores, scores, and an M-by-H logical matrix hypotheses. S is the
%   number of sensors, D is the history depth (number of scans), and H is
%   the number of global hypotheses.
%
%   [...] = pruneTrackBranches(...,'MinBranchProbability', minProb) allows
%   you to specify the value of the threshold for minimum branch
%   probability as a value in the range [0,1). Typical values are between
%   1e-3 and 5e-3. Branches with global probability lower than the
%   threshold are pruned. 
%   If unspecified, the default value for MinBranchProbability is 1e-3.
%
%   [...] = pruneTrackBranches(...,'MaxNumTrackBranches', maxNumBranches)
%   allows you to specify the maximum number of track branches to keep per
%   track as a positive integer scalar. Typical values are between 2 and 6.
%   If a track has more than this number of branches, the branches with the
%   lowest initial score are pruned. 
%   If unspecified, the default value for MaxNumTrackBranches is 3. 
%
%   [...] = pruneTrackBranches(...,'NScanPruning', nScanPruning) allows you
%   to specify the way you want to use N-Scan pruning. N-scan pruning
%   allows you to prune branches that, if you look N-scan backwards, are
%   incompatible with the current most likely branch. Choose an N-scan
%   method from the following:
%     'None'        - No N-scan pruning is done.
%     'Hypothesis'  - Prune branches that are incompatible with branches in
%                     the most likely global hypothesis.
%   If unspecified, the default value is 'None'.
%
%   [...] = pruneTrackBranches(...,'NumSensors', S) allows you to specify
%   the number of sensors in history, S, as a positive integer scalar. The
%   number of scans in history, D, is calculated based on this input. If
%   unspecified, the default value for NumSensors is 20.
%
%   [..., info] = pruneTrackBranches(...) returns the optional output,
%   info, as a struct with the following fields:
%     BranchID           - Branch identifier, the 3rd column of the history
%     PriorProbability   - Branch prior probability, from the branch score
%     GlobalProbability  - Branch global probability, taking into account
%                          the hypotheses that contain the branch and their
%                          scores.
%     PruneByProbability - A logical value, true if the branch is pruned by
%                          MinBranchProbability.
%     PruneByNScan       - A logical value, true if the branch is pruned by
%                          N-scan pruning.
%     PruneByNumBranches - A logical value, true if the branch is pruned by
%                          MaxNumTrackBranches.
%
%   EXAMPLE: prune branches
%   % Create branch history, scores, and hypotheses
%   history = [     
%      8    14    14     0     0     2     0
%      1    23    23     0     0     2     1
%      2    24    24     0     0     1     2
%      9    25    25     0     1     0     0
%     10    26    26     0     2     0     0
%      1    28    28     0     1     0     1
%      4    33    33     0     1     2     1
%      1    34    34     0     1     2     1
%      2    35    35     0     2     1     2
%     11     0    36     1     0     0     0
%     12     0    37     2     0     0     0
%      8    14    38     2     0     2     0
%      1    23    39     2     0     2     1
%      2    24    40     1     0     1     2
%      9    25    41     2     1     0     0
%     10    26    42     1     2     0     0
%      1    28    43     2     1     0     1
%      4    33    44     2     1     2     1
%      1    34    45     2     1     2     1
%      2    35    46     1     2     1     2];
%   scores = [4.5 44.9 47.4 6.8 6.8 43.5 50.5 61.9 64.7 9.1 9.1 19 61.7 ...
%       63.5 21.2 20.5 60.7 67.3 79.2 81.5]';
%   [clusters,incompBranches] = clusterTrackBranches(history);
%   hypotheses = compatibleTrackBranches(clusters,incompBranches,scores,10);
%
%   [toPrune,probs,info] = pruneTrackBranches(history,scores,hypotheses,'NumSensors',1,'NScanPruning','Hypothesis')
%   struct2table(info)
%
%   See also: trackerTOMHT, trackBranchHistory, clusterTrackBranches,
%   compatibleTrackBranches

 
% Copyright 2018 The MathWorks, Inc.


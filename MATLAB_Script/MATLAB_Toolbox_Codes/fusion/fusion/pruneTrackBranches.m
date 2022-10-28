function [toPrune,posteriorProbs,info] = pruneTrackBranches(history,scores,hypotheses,varargin)
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

%   References:
%   [1] J.R. Werthmann, "A Step-by-Step Description of a Computationally
%       Efficient Version of Multiple Hypothesis Tracking", SPIE Vol. 1698
%       Signal and Processing of Small Targets, pp 288-300, 1992.
%   [2] Samuel Blackman and Robert Popoli, "Design and Analysis of Modern
%       Tracking Systems", Artech House, 1999.

% Copyright 2018 The MathWorks, Inc.

%#codegen

% Input parsing and validation
narginchk(3,11)
validateattributes(history,{'numeric'},...
    {'real','nonnegative','nonsparse','2d','integer'},mfilename,'history');
[numBranches,numCols] = size(history);
coder.internal.assert(numCols>3,'fusion:clusterTrackBranches:expected4Columns')
validateattributes(scores,{'single','double'},...
    {'real','nonsparse','2d','nrows',numBranches},mfilename,'scores');
validateattributes(hypotheses,{'numeric','logical'},...
    {'real','nonsparse','binary','2d','nrows',numBranches},mfilename,'hypotheses');
[minProb,maxNumBranches,nScan,numSensors] = localParseAndValidate(varargin{:});

% If hypotheses is not logical, cast it to be
hyps = cast(hypotheses,'logical');

% Low probability pruning: prune tracks that are not part of any hypothesis
% or that have a very low probability
rangelimit = cast(50,'like',scores); % Limit range to avoid Inf
trackScores = localLimit(scores(:,1),rangelimit);
posteriorProbs = localCalcPosteriorProbability(trackScores, hyps);
toPruneByProbability = (posteriorProbs < minProb);

% N-Scan pruning
if strcmpi(nScan,'None')
    toPruneByNScan = false(size(history,1),1);
else
    toPruneByNScan = localHypothesisPruning(history,hyps,numSensors);
end  

% Prune branches that have low score. From all the branches that have the
% same target tree, prune the ones that have lowest scores, if the number
% of branches exceeds MaxNumTrackBranches
survivingSoFar = find((toPruneByProbability | toPruneByNScan)==false);
toPrune3 = localPruneBranchesByNumBranches(history(survivingSoFar,:),trackScores(survivingSoFar),maxNumBranches);
toPruneByNumBranches = false(size(toPruneByProbability));
toPruneByNumBranches(survivingSoFar(toPrune3)) = true;

% Branches to prune are branches that should be pruned by any method
toPrune = (toPruneByNumBranches | toPruneByProbability | toPruneByNScan);

% Output additional information if required
if nargout == 3
    priorProbs = exp(trackScores) ./ (1+exp(trackScores));
    info = struct(...
        'BranchID',history(:,3),...
        'PriorProbability',priorProbs,...
        'GlobalProbability',posteriorProbs,...
        'PrunedByProbability',toPruneByProbability, ...
        'PrunedByNScan',toPruneByNScan,...
        'PrunedByNumBranches',toPruneByNumBranches);
end
end

%----------------------------------------------------------------------
% Local pruning functions
%----------------------------------------------------------------------
function toPruneByNumLeaves = localPruneBranchesByNumBranches(history,scores,maxNumBranches)
    numBranches = size(history,1);
    toPruneByNumLeaves = false(numBranches,1);
    targetTrees = history(:,1);
    uniqueTargetTrees = unique(targetTrees);
    numUniqueTT = numel(uniqueTargetTrees);
    for i = 1:numUniqueTT
        thisBranch = find(targetTrees==uniqueTargetTrees(i));
        if numel(thisBranch) > maxNumBranches
            branchScores = scores(thisBranch);
            [~,I] = sort(branchScores,'descend');
            toPruneByNumLeaves(thisBranch(I(maxNumBranches+1:end)))=true;
        end
    end
end
        
        
function toPrune = localHypothesisPruning(history,hyps,numSensors)
    % N-scan pruning by most-likely hypothesis
    likelyTracks = hyps(:,1);
    numLikely = sum(likelyTracks);
    a = find(likelyTracks == true);
    numTracks = size(history,1);
    toPrune = false(numTracks,1);
    for i = 1:numLikely
        thisTree = history(a(i),1);
        earliestScan = history(a(i),(end-numSensors+1):end);
        secondScan = history(a(i),(end-2*numSensors+1):(end-numSensors));

        if any(~(earliestScan)==0) && any(~(secondScan==0))
            for j = 1:numTracks
                if history(j,1) == thisTree && ... %Same tree
                        all(history(j,end-numSensors+1:end)==earliestScan) && ...
                        all(history(j,end-2*numSensors+1:end-numSensors) ~= secondScan)
                    toPrune(j) = true;
                end
            end
        end
    end
end

function probs = localCalcPosteriorProbability(scores, hypotheses)
    %localCalcPosteriorProbability  Track glocal probability based on its prior
    %probability and the scores of the hypotheses it is in
    %   probs = calcPosteriorProbability(scores, hypotheses, hypScores)
    %   returns the global probability of each track specified by the list of
    %   trackIDs based on the prior probability and its existence in the list
    %   of hypotheses.
    %
    %   Inputs:
    %       scores      - An M element array of track scores.
    %       hypotheses  - An M-by-H logical matrix that maps
    %                     branches to hypotheses.
    %
    %   Outputs:
    %       probs       - An M-by-1 array of global track probabilities based
    %                     on their existence in the hypotheses.
    %   Example:
    %   --------
    %   hypotheses = logical([...
    %       1 0 0 1;
    %       0 1 0 0;  % Will be pruned because it has low support in hypotheses
    %       0 0 0 0;  % Will be pruned because does not appear in hypotheses
    %       1 0 1 0;
    %       0 1 0 1]);
    %
    %   scores = [9;15;2;19;1];
    %
    %   probs = calcPosteriorProbability(scores, hypotheses)
    
    % Memory allocation
    numTracks = numel(scores);
    probs = zeros(numTracks,1,'like',scores);
    hypScores = scores'*hypotheses;
    
    % Limits the range over which we calculate exp. The actual value of n 
    % where exp(single(n)) becomes Inf is close to 88. Then
    % exp(n)/(1+exp(n) becomes NaN. To avoid that, limit the range.
    rangelimit = cast(50,'like',scores);
    scores(:,1) = localLimit(scores(:,1), rangelimit);
    priorProbs = exp(scores(:,1)) ./ (1+exp(scores(:,1)));
    
    hypsWithTracks = any(hypotheses,1);
    
    if ~any(hypsWithTracks) % Edge case: no hypothesis with tracks
        return
    end
    
    hypScores = localLimit(hypScores, rangelimit);
    expHypScores = exp(hypScores(hypsWithTracks));
    sumExpHypScores = sum(expHypScores);
    
    for i = 1:numTracks
        isInHyps = hypotheses(i,hypsWithTracks);
        probs(i) = priorProbs(i)*sum(expHypScores(isInHyps))/sumExpHypScores;
    end
end

function scores = localLimit(scores, range)
%localLimit returns scores limited to the range, range. Helps avoid
%overflow of the exponent function which causes exp(s)/(1+exp(s)) to become
%NaN
scores = min(max(scores,-range),range);
end

%----------------------------------------------------------------------
% Local parsing functions
%----------------------------------------------------------------------
function [minProb,maxNumBranches,nScan,numSensors] = localParseAndValidate(varargin)
%localParseAndValidate Local function for parsing and validting name-value pairs
    if coder.target('MATLAB')
        [minProb,maxNumBranches,nScanPartial,numSensors] = localParseMATLAB(varargin{:});
    else
        [minProb,maxNumBranches,nScanPartial,numSensors] = localParseCodegen(varargin{:});
    end
    validateattributes(minProb,{'single','double'},...
        {'nonnegative','real','nonsparse','scalar','<=',1}, mfilename,'MinBranchProbability')
    validateattributes(maxNumBranches,{'numeric'},...
        {'positive','real','nonsparse','finite','scalar'}, mfilename,'MaxNumBranches')
    nScan = validatestring(lower(nScanPartial),...
        {'none','hypothesis'}, mfilename,'NScanPruning');
    validateattributes(numSensors,{'numeric'},...
        {'positive','real','nonsparse','finite','scalar'}, mfilename,'NumSensors')
end

function [minProb,maxNumBranches,nScanPartial,numSensors] = localParseMATLAB(varargin)
%localParseMATLAB   Local function for parsing name-value pairs in MATLAB
    % Define parser
    parser = inputParser;
    parser.addParameter('MinBranchProbability',1e-3);
    parser.addParameter('MaxNumTrackBranches',3);
    parser.addParameter('NScanPruning','None');
    parser.addParameter('NumSensors',20);
    
    % Parse
    parser.parse(varargin{:});
    
    % Provide outputs
    minProb         = parser.Results.MinBranchProbability;
    maxNumBranches  = parser.Results.MaxNumTrackBranches;
    nScanPartial    = parser.Results.NScanPruning;
    numSensors      = parser.Results.NumSensors;
end

function [minProb,maxNumBranches,nScanPartial,numSensors] = localParseCodegen(varargin)
%localParseCodegen   Local function for parsing name-value pairs in Codegen
    % Define parser
    parms = struct( ...
        'MinBranchProbability',  uint32(0), ...                
        'MaxNumTrackBranches',  uint32(0), ...
        'NScanPruning',         uint32(0), ...
        'NumSensors',           uint32(0) ...
        );
            
    popt = struct( ...
        'CaseSensitivity', false, ...
        'StructExpand',    true, ...
        'PartialMatching', false);
            
    % Parse
    optarg = eml_parse_parameter_inputs(parms, popt, varargin{:});            
            
    % Provide outputs
    minProb         = eml_get_parameter_value(optarg.MinBranchProbability, 1e-3, varargin{:});            
    maxNumBranches  = eml_get_parameter_value(optarg.MaxNumTrackBranches, 3, varargin{:});
    nScanPartial    = eml_get_parameter_value(optarg.NScanPruning, 'None', varargin{:});
    numSensors      = eml_get_parameter_value(optarg.NumSensors, 20, varargin{:});
end
function [pgNew, trimInfo, debugInfo] = trimLoopClosures(pg, trimmerParams, solverOptions)
%TRIMLOOPCLOSURES Optimize pose graph and remove bad loop closures
%   POSEGRAPHUPDATED = trimLoopClosures(POSEGRAPH, TRIMMERPARAMS, 
%                                       SOLVEROPTIONS)
%   optimizes POSEGRAPH to best satisfy its edge constraints and also
%   removes any bad loop closure edges that cause the edge residual errors 
%   in the optimized pose graph to exceed certain threshold. POSEGRAPH
%   can either be a poseGraph or poseGraph3D object. TRIMMERPARAMS includes
%   the parameters regarding to loop closure trimming, it is a struct with
%   the following fields:
%
%      MaxIterations       - Max number of iterations allowed for loop
%                            closure trimming. It is a positive integer. 
%                            In one trimming iteration, the pose graph is
%                            optimized once with a new set of weighted
%                            information matrices using a solver of choice
%                            from optimizePoseGraph.
% 
%      TruncationThreshold - Maximum allowed residual error for an edge.
%                            This value varies between pose graphs.  
%                            To find a proper value for truncation 
%                            threshold, inspect the result from 
%                            edgeResidualErrors of POSEGRAPH. 
%                            The threshold is a positive number.
%
%   SOLVEROPTIONS is an object that defines the parameters for the 
%   underlying pose graph solver. Generate the object using the
%   poseGraphSolverOptions function. The output, POSEGRAPHUPDATED, is the 
%   optimized pose graph with bad loop closures trimmed.
%
%   [POSEGRAPHUPDATED, TRIMINFO] = trimLoopClosures(POSEGRAPH, ___) also
%   returns additional information related to the trimming process as the 
%   second output, TRIMINFO. TRIMINFO is a struct with the following 
%   fields:
%
%      LoopClosuresToRemove - Loop closure edge IDs to be removed from the
%                             input POSEGRAPH. These loop closures are no
%                             longer to be found in the output
%                             POSEGRAPHUPDATED.
%
%      Iterations           - Number of trimming iterations performed.
%
%   trimLoopClosures implements the graduated non-convexity (GNC) method 
%   with truncated least squares (TLS) robust cost in combination with a 
%   non-minimal pose graph solver, as introduced in [1].
%
%   Example:
%      % Create a minimalistic pose graph.
%      pg = poseGraph;
% 
%      % Add incremental edge
%      for i = 1:4
%          addRelativePose(pg, [0, 0.95, -pi/2-0.01*pi]);
%      end
% 
%      % Add a loop closure edge that represents a good measurement.
%      addRelativePose(pg, [0, 0, 0], [1, 0, 0, 1, 0, 1], 1, 5);
% 
%      % Add a loop closure edge that represents a bad measurement.
%      addRelativePose(pg, [0.5, -0.5, pi/4], [1, 0, 0, 1, 0, 1], 4, 2);
% 
%      % Visualize the pose graph before running optimization.
%      figure; subplot(1,3,1); box on;
%      show(pg);
% 
%      % Run pose graph optimization.
%      pg1 = optimizePoseGraph(pg);
% 
%      % Examine the optimized poseGraph.
%      subplot(1,3,2); box on;
%      show(pg1);
% 
%      % The "normal" pose graph should look like a square, but the bad 
%      % loop closure that is just added has skewed the pose graph shape 
%      % after optimization.
% 
%      % Take a look at the residual errors of pg1.
%      edgeResidualErrors(pg1)
% 
%      % Assume it has been known that a reasonable edge measurement falls
%      % within +/-[0.05 m, 0.05 m, 0.05*pi rad] range of the true value,
%      % we can ten set the truncation threshold to 0.05^2*2 + (0.05*pi)^2,
%      % which is around 0.0297.
%      trimParams.TruncationThreshold = 0.0297;
%      trimParams.MaxIterations = 100;
% 
%      % Get the default pose graph solver options
%      solverOptions = poseGraphSolverOptions;
% 
%      % Trim the loop closures.
%      [pg2, trimInfo] = trimLoopClosures(pg, trimParams, solverOptions);
% 
%      % Examine the trimming result.
%      subplot(1,3,3); box on;
%      show(pg2);
% 
%      % See which loop closures should be removed from pg. The true bad 
%      % loop closure edge ID is 6. 
%      trimInfo.LoopClosuresToRemove
%
%   Reference:
%   [1] H. Yang et al., "Graduated Non-Convexity for Robust Spatial 
%       Perception: From Non-Minimal Solvers to Global Outlier Rejection", 
%       IEEE Robotics and Automation Letters (RA-L), 5(2):1127-1134, 2020.
%   
%   See also poseGraph, poseGraph3D, optimizePoseGraph, 
%   poseGraphSolverOptions

%   Copyright 2020-2021 The MathWorks, Inc.
    
%#codegen

% validations and data extraction
validateattributes(pg, {'poseGraph', 'poseGraph3D'},...
                   {'nonempty', 'scalar'}, 'trimLoopClosures', 'poseGraph');

validateattributes(solverOptions, {'nav.solveroptions.TrustRegion', 'nav.solveroptions.G2oLevenbergMarquardt'},...
                   {'nonempty', 'scalar'}, 'trimLoopClosures', 'solverOptions');
               
validateattributes(trimmerParams, {'struct'},{'nonempty', 'scalar'}, 'trimLoopClosures', 'TrimmerParams')
            
if ~isfield(trimmerParams, 'MaxIterations')
    coder.internal.error('nav:navalgs:trimloopclosures:TrimmerParamMissing', 'MaxIterations');
end
if ~isfield(trimmerParams, 'TruncationThreshold')
    coder.internal.error('nav:navalgs:trimloopclosures:TrimmerParamMissing', 'TruncationThreshold');
end

maxTrimIter = robotics.internal.validation.validatePositiveIntegerScalar(trimmerParams.MaxIterations, 'trimLoopClosures', 'trimmerParams.MaxIterations');

truncationThreshold = robotics.internal.validation.validatePositiveNumericScalar(trimmerParams.TruncationThreshold, 'trimLoopClosures', 'trimmerParams.TruncationThreshold');

solverParams = nav.solveroptions.internal.SolverOptionsExtractor.dump(solverOptions);
isVerbose = solverParams.IsVerbose;
solverParams.IsVerbose = false; % do not print out inner loop iterations

weightsIni = ones(1, pg.NumEdges);



[pgUpd, solnInfo] = weightedOptimize(pg, solverParams, weightsIni);

% collect debug info
residualErrorHistoryInternal = zeros(maxTrimIter + 1, 1);
weightsHistoryInternal = zeros(maxTrimIter + 1, pg.NumEdges);
residualErrorHistoryInternal(1) = solnInfo.Error;

res = edgeResidualErrors(pgUpd);

% initial mu
mu = truncationThreshold/(2*max(res) - truncationThreshold);

lcIds = pg.LoopClosureEdgeIDs;

weightsPre = weightsIni;
weights = computeNewWeights(res, truncationThreshold, mu, lcIds);
weightsHistoryInternal(1,:) = weights; 

iter = 1;
if coder.target('MATLAB')
    if isVerbose
        fprintf('\n%s\n', message('nav:navalgs:trimloopclosures:VerbosePrintOutStart').getString());
        fprintf('%s %f\n', message('nav:navalgs:trimloopclosures:VerbosePrintOutIter', iter).getString(), solnInfo.Error);
    end
end

while any(abs(weights - weightsPre) > 1e-2) ... % stop iterating when weights converge
         && (iter < maxTrimIter)

    weightsPre = weights;
    [pgUpdTmp, solnInfo] = weightedOptimize(pgUpd, solverParams, weights);
    
    nav.algs.internal.PoseGraphOptimizer.updateNodeEstimates(pgUpd, pgUpdTmp); % for codegen, we cannot assign handle object inside while loop

    res = edgeResidualErrors(pgUpd);
    weights = computeNewWeights(res, truncationThreshold, mu, lcIds);
    
    mu = 1.4*mu; % see [1]
    
    iter = iter + 1;
    
    weightsHistoryInternal(iter,:) = weights;
    residualErrorHistoryInternal(iter) = solnInfo.Error;
    
    if coder.target('MATLAB')
        if isVerbose
            fprintf('%s %f\n', message('nav:navalgs:trimloopclosures:VerbosePrintOutIter', iter).getString(), solnInfo.Error);
        end
    end
end


lcIdsToRemove = find(weights < 0.3);

% populate trimInfo
trimInfo = struct('Iterations', iter, 'LoopClosuresToRemove', lcIdsToRemove);


% optimize with only good loop closures 
pgNew = copy(pg);
if ~isempty(lcIdsToRemove)
    pgNew.removeEdges(lcIdsToRemove);
end

[pgNew, solnInfo] = weightedOptimize(pgNew, solverParams, ones(1, pgNew.NumEdges));

weightsFinal = ones(1, pg.NumEdges);
weightsFinal(lcIdsToRemove) = 0;
weightsHistoryInternal(iter+1,:) = weightsFinal;
residualErrorHistoryInternal(iter+1) = solnInfo.Error;

% populate debugInfo
debugInfo = struct('ResidualErrorHistory', residualErrorHistoryInternal(1:iter+1,:), ...
                   'WeightsHistory', weightsHistoryInternal(1:iter+1,:) );

if coder.target('MATLAB')
    if isVerbose
        fprintf('%s\n\n', message('nav:navalgs:trimloopclosures:VerbosePrintOutDone').getString());
    end
end

end % trimLoopClosures


function weights = computeNewWeights(residualVec, csq, mu, loopClosureIds)
%computeNewWeights Perform GNC-TLS closed-form weight update.
%   See Proposition 6 in [1].

    thresh2 = ((mu+1)/mu)*csq;
    thresh1 = (mu/(mu+1))*csq;
    c = sqrt(csq);
    A = sqrt(mu*(mu+1));
    nr = numel(residualVec);
    weights = ones(1,nr);
    for k = 1:nr
        if ismember(k, loopClosureIds)
            rsq = residualVec(k);
            if rsq > thresh2
                weights(k) = 0;
            elseif rsq < thresh1
                weights(k) = 1;
            else
                weights(k) = (c/sqrt(rsq))*A - mu;
            end
        end
    end

end

function [pgUpd, solnInfo] = weightedOptimize(pg, solverParams, weights) 
%weightedOptimize
    if coder.target('MATLAB')
        if solverParams.SolverID == -1
            [pgUpd, solnInfo] = nav.algs.internal.PoseGraphOptimizer.optimizeWeighted_tr(pg, solverParams, weights);
        else
            [pgUpd, solnInfo] = nav.algs.internal.PoseGraphOptimizer.optimizeWeighted_g2olm(pg, solverParams, weights);
        end
    else
        % g2o solver doesn't support codegen
        coder.internal.errorIf(solverParams.SolverID == 0,  'nav:navalgs:optimizeposegraph:NoCodegenSupportForG2O');
        [pgUpd, solnInfo] = nav.algs.internal.PoseGraphOptimizer.optimizeWeighted_tr(pg, solverParams, weights);
    end
end

function [poseGraphUpdated, solutionInfo, hessian] = optimizePoseGraph(poseGraph, varargin)
%optimizePoseGraph Refine poses in a pose graph to best satisfy constraints
%   POSEGRAPHUPDATED = optimizePoseGraph(POSEGRAPH) adjusts node poses in
%   POSEGRAPH so that they comply with the underlying edge constraints as
%   much as possible. POSEGRAPH can be a poseGraph object, a poseGraph3D
%   object, or a MATLAB digraph object. The returned POSEGRAPHUPDATED
%   has the same object type and graph topology as POSEGRAPH, but with the
%   updated node poses.
%
%   Note: Computer Vision Toolbox is required if the pose graph is
%   represented as a digraph object. Use the createPoseGraph method on
%   imageviewset or pcviewset to create the digraph of the required format,
%   where node estimates are represented by rigid3d and edge constraints 
%   are represented by either rigid3d or affine3d. If digraph edge 
%   information matrix is not supplied its assumed to be identity by 
%   default.
%   
%   See the comparison chart below for more details on different
%   representations of pose graph.
%
%    ------------------------------------------------------
%    Pose Graph        | Pose Formats
%    Representations   | 
%    ------------------------------------------------------
%    poseGraph object  | Poses are 1-by-3 vectors, 
%                      | [x, y, yaw].
%    ------------------|-----------------------------------
%    poseGraph3D object| Poses are 1-by-7 vectors, 
%                      | [x, y, z, qw, qx, qy, qz].
%                      | Note the sequence in the
%                      | quaternion part.
%    ------------------|-----------------------------------
%    digraph object    | Poses are rigid3d or affine3d objects.
%                      | This representation requires
%                      | Computer Vision Toolbox.
%    ------------------------------------------------------
%
%   [POSEGRAPHUPDATED, SOLUTIONINFO] = optimizePoseGraph(POSEGRAPH) returns
%   additional statistics about the optimization process in SOLUTIONINFO 
%   struct, which has the following fields:
%
%      Iterations - Number of iterations used in optimization.
%
%      ResidualError - The value of the cost function when the optimizer 
%         exits. Within the same problem, lower residual error means that 
%         the poses in the graph are more consistent with the edge 
%         constraints.
%
%      ExitFlag - Exit condition for the optimizer.
%
%   ___ = optimizePoseGraph(POSEGRAPH, SOLVER, Name, Value) optimizes the 
%   POSEGRAPH using the specified SOLVER. SOLVER can be either 
%   'builtin-trust-region' (default solver, see exception for digraph pose
%   graph below) or 'g2o-levenberg-marquardt'. Provides additional options
%   to set SOLVER specific parameters specified by one or more Name,Value
%   pair arguments. Name must appear inside single quotes (''). You can 
%   specify several Name,Value pair arguments in any order as  
%   Name1, Value1, ..., NameN, ValueN:
%
%   The Name, Value pairs common for both 'builtin-trust-region' and
%    'g2o-levenberg-marquardt' solvers are:
%
%      'MaxIterations' - Maximum number of iterations allowed for 
%                        optimization.
%         Default: 300
%
%      'MaxTime' - Maximum time allowed for optimization.
%         Default: 500
%
%      'FunctionTolerance' - Lower bound on the change in the cost function
%         If the cost function value (a scalar) changes less than this 
%         number between optimization steps, the iterations end.
%         Default: 1e-8 if the Solver is 'builtin-trust-region';
%                  1e-9 if the Solver is 'g2o-levenberg-marquardt'.
%
%      'VerboseOutput' - Display intermediate iteration information.
%         Possible values: {'on', 'off'}
%         Default: 'off'
%
%      'LoopClosuresToIgnore' - IDs of loop closure edges in POSEGRAPH to 
%         ignore during optimization. The IDs are given in a vector.
%         Default: 1x0 empty double row vector
%
%      'FirstNodePose' - The pose of the first node. The pose of the first
%         node can be used to anchor and orient the entire pose graph.
%         Default: [0 0 0] if POSEGRAPH is a poseGraph object;
%                  [0 0 0 1 0 0 0] if POSEGRAPH is a poseGraph3D
%                  object.
%
%   The Name, Value pairs specific to 'builtin-trust-region' solver are:
%
%      'GradientTolerance' - Lower bound on the norm of the gradient. If 
%         the norm of the gradient of the cost function falls below this 
%         number, the iterations end.
%         Default: 0.5e-8
%
%      'StepTolerance' - Lower bound on the step size. If the norm of the
%         optimization step falls below this number, the iterations end.
%         Default: 1e-12
%
%      'InitialTrustRegionRadius' - The initial trust region radius.
%         Default: 100
%
%   
%   Note: if the pose graph is a MATLAB digraph object, then
%   'g2o-levenberg-marquardt' must be designated as the solver, and the 
%   'LoopClosuresToIgnore' and 'FirstNodePose' parameters are ignored
%   during optimization.
%
%
% See also poseGraph, poseGraph3D, digraph, imageviewset/createPoseGraph,
% pcviewset/createPoseGraph

%   Copyright 2017-2021 The MathWorks, Inc.

%#codegen

    if ~isempty(varargin)
        validateattributes(varargin{1},{'char','string'},{'nonempty','scalartext'},'optimizePoseGraph','second argument');
        if (mod(nargin,2)==0)
            % If an even number of arguments are passed to the optimizePoseGraph
            % function, then it is assumed that the syntax specifies a Solver
            % option as the second input. All subsequent inputs are assumed to
            % be name value pairs. The Syntax used is:
            % optimizePoseGraph(POSEGRAPH,SOLVER,Name,Value)
            solver = validatestring(varargin{1}, nav.algs.internal.PoseGraphOptimizer.AvailableSolvers, 'optimizePoseGraph', 'Solver');
            nar = length(varargin);
            var = cell(1,nar-1);
            for k = 1:(nar-1)
                var{k} = varargin{k+1};
            end
        else
            % Otherwise, if the number of inputs is odd, it is assumed that no
            % solver is specified and the default is used. The syntax used is:
            % optimizePoseGraph(POSEGRAPH, Name, Value)
            solver = nav.algs.internal.PoseGraphOptimizer.AvailableSolvers{1};
            var = varargin;
        end
    else
        % All default parameters will be used if only POSEGRAPH is passed
        solver = nav.algs.internal.PoseGraphOptimizer.AvailableSolvers{1};
        var = varargin;
    end
    
    validateattributes(poseGraph, {'poseGraph', 'poseGraph3D', 'digraph'},...
                       {'nonempty', 'scalar'}, 'optimizePoseGraph', 'poseGraph');
                   
    if isa(poseGraph, 'digraph')
        
        if strcmp(solver, 'builtin-trust-region')
            nav.algs.internal.error('nav:navalgs', 'optimizeposegraph:DigraphOnlySupportsG2OLM');
        end
        paramStruct = nav.algs.internal.PoseGraphOptimizer.parseG2oSolver(7, 2, var{:}); %7 - size of the 3D pose, 2 - g2o solver internal id 
        
        [poseTable, solInfo] = vision.internal.optimizePoses(poseGraph, 'MaxIterations', paramStruct.MaxNumIteration, ...
                                                             'MaxTime', paramStruct.MaxTime, ...
                                                             'Verbose', paramStruct.IsVerbose, ...
                                                             'Tolerance', paramStruct.FunctionTolerance);
        
        % create a digraph with same edges and optimized nodes
        poseGraphUpdated = digraph(poseGraph.Edges, poseTable);
        
        hessian = speye(poseGraph.numnodes*6);
    else

        coder.internal.assert(poseGraph.NumEdges > 0,  'nav:navalgs:optimizeposegraph:NoEdgesInGraph');
        coder.internal.assert(isempty(poseGraph.LandmarkNodeIDs) || ~strcmp(solver, 'g2o-levenberg-marquardt'), ...
                                             'nav:navalgs:optimizeposegraph:G2oDoesNotSupportLandmarkPoseGraph');

        paramStruct = nav.algs.internal.PoseGraphOptimizer.parseOptimizePoseGraphInputs(poseGraph, solver, var{:});
        

        [poseGraphUpdated, solInfo, hessian] = nav.algs.internal.PoseGraphOptimizer.optimize(poseGraph, paramStruct);
    end

    solutionInfo.Iterations = solInfo.Iterations;
    solutionInfo.ResidualError = solInfo.Error;
    solutionInfo.ExitFlag = solInfo.ExitFlag;

end

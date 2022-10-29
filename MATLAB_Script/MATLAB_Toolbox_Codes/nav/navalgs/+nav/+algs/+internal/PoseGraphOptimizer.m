classdef PoseGraphOptimizer < nav.algs.internal.InternalAccess
%This class is for internal use only. It may be removed in the future.

%POSEGRAPHOPTIMIZER Internal implementation for optimizePoseGraph
%   function

%   Copyright 2017-2020 The MathWorks, Inc.

%#codegen

    properties (Constant)
        % All available solvers
        AvailableSolvers = {'builtin-trust-region',... Trust Region Dogleg Builtin Solver
                            'g2o-levenberg-marquardt'... g2o Levenberg Marquardt Sparse Cholesky Solver
                           };
    end

    methods (Static)
        function updateNodeEstimates(pg1, pg2)
            %updateNodeEstimates Update node estimates in pg1 with those in pg2
            %   pg1 and pg2 must have the same number of nodes
            pg1.quickUpdateNodeEstimates(pg2.NodeEstimates.Matrix);
            
        end
        
        function paramStruct = parseOptimizePoseGraphInputs(poseGraph, solverName, varargin)
        %parseOptimizePoseGraphInput Parse inputs to "optimizePoseGraph" function
        %solverName: can be either 
        %  'g2o-levenberg-marquardt' OR
        %   'builtin-trust-region' (default).
        %
        % varargin can be Name-Value pairs corresponding to the chosen
        % solver.
        %
        %   If the solver is 'builtin-trust-region', the possible
        %   Name,Value pairs are:
        %
        %   'MaxIterations' - Maximum number of iterations allowed for optimization.
        %      Default: 300
        %
        %   'MaxTime' - Maximum time allowed for optimization
        %      Default: 500
        %
        %   'GradientTolerance' - Lower bound on the norm of the gradient. If the
        %      norm of the gradient of the cost function falls below this number,
        %      the iterations end.
        %      Default: 0.5e-8
        %
        %   'FunctionTolerance' - Lower bound on the change in the cost function.
        %      If the cost function value (a scalar) changes less than this number
        %      between optimization steps, the iterations end.
        %      Default: 1e-8
        %
        %   'StepTolerance' - Lower bound on the step size. If the norm of the
        %      optimization step falls below this number, the iterations end.
        %      Default: 1e-12
        %
        %   'InitialTrustRegionRadius' - The initial trust region radius.
        %      Default: 100
        %
        %   'VerboseOutput' - Display intermediate iteration information
        %      Possible values: {'on', 'off'}
        %      Default: 'off'
        %
        %   'LoopClosuresToIgnore' - IDs of loop closure edges in POSEGRAPH to ignore
        %      during optimization. The IDs are given in a vector.
        %      Default: 1x0 empty double row vector
        %
        %   'FirstNodePose' - The pose of the first node. The pose of the first
        %      node can be used to anchor and orient the entire pose graph.
        %      Default: [0 0 0] if POSEGRAPH is a poseGraph object;
        %               [0 0 0 1 0 0 0] if POSEGRAPH is a poseGraph3D
        %               object.
        %
        %    If the solver is 'g2o-levenberg-marquardt', the possible
        %    Name,Value pairs are:
        %
        %   'MaxIterations' - Maximum number of iterations allowed for optimization.
        %      Default: 300
        %
        %   'MaxTime' - Maximum time allowed for optimization
        %      Default: 500
        %
        %   'FunctionTolerance' - Lower bound on the relative change in the cost function.
        %      If the cost function value (a scalar) changes less than this number
        %      between optimization steps, the iterations end.
        %      Default: 1e-9
        %
        %   'VerboseOutput' - Display intermediate iteration information
        %      Possible values: {'on', 'off'}
        %      Default: 'off'
        %
        %   'LoopClosuresToIgnore' - IDs of loop closure edges in POSEGRAPH to ignore
        %      during optimization. The IDs are given in a vector.
        %      Default: 1x0 empty double row vector
        %
        %   'FirstNodePose' - The pose of the first node. The pose of the first
        %      node can be used to anchor and orient the entire pose graph.
        %      Default: [0 0 0] if POSEGRAPH is a poseGraph object;
        %               [0 0 0 1 0 0 0] if POSEGRAPH is a poseGraph3D
        %               object.
        %
        % Representing solver with numeric ID. Each solver option has a
        % unique id. For all available solvers and their ids see help
        % for getSolverId method.
            solverID = nav.algs.internal.PoseGraphOptimizer.getSolverId(solverName);

            switch solverID

              case 0
                if coder.target('MATLAB')
                    paramStruct = nav.algs.internal.PoseGraphOptimizer.parseG2oSolver(poseGraph.PoseLength, solverID, varargin{:});
                else
                    % Filling dummy values for codegen
                    paramStruct.MaxNumIteration = 0;
                    paramStruct.MaxTime = 0;
                    paramStruct.FunctionTolerance = 0;
                    paramStruct.IsVerbose = false;
                    paramStruct.GradientTolerance = 0;
                    paramStruct.StepTolerance = 0;
                    paramStruct.InitialTrustRegionRadius = 0;
                    paramStruct.LoopClosuresToIgnore = zeros(1,4);
                    paramStruct.FirstNodePose = zeros(1,4);
                    paramStruct.TrustRegionRadiusTolerance = 1e-7;
                    paramStruct.SolverID = solverID;
                    coder.internal.assert(~(solverID==0),'nav:navalgs:optimizeposegraph:NoCodegenSupportForG2O');
                end
              otherwise
                paramStruct = nav.algs.internal.PoseGraphOptimizer.parseDefaultSolver(poseGraph.PoseLength,varargin{:});
            end

        end


        function [pgUpdated, solutionInfo] = optimizeWeighted_g2olm(pg, paramStruct, weights)
            %optimizeWeighted_g2olm Weighted optimization using g2o
            %   Levenberg-Marquardt solver
            if pg.PoseLength == 3
                paramStruct.OptimizationType = 0; %SE2
            else
                paramStruct.OptimizationType = 1; %SE3
            end
            
            solver = nav.algs.internal.PoseGraphOptimizer.configureG2oSolver(paramStruct);

            pgUpdated = copy(pg); % needs to keep pg's infoMat intact
            
            for i = 1:pgUpdated.NumEdges
                pgUpdated.scaleInfoMat(i, weights(i));
            end
            
            nodePoses = pgUpdated.nodeEstimates;
            edgeNodePairs = pgUpdated.edgeNodePairs;
            [relPoses,infoMats] = pgUpdated.edgeConstraints;
                    
            solution = solver.solve(nodePoses',[edgeNodePairs-1,relPoses]',infoMats');

            % update the pose graph with optimized poses and realign
            T1 = pgUpdated.poseToTform(solution.optimizedPoses(1,:));
            T1Offset = pgUpdated.tforminv(T1);
            for i = 1:pgUpdated.NumNodes
                pgUpdated.NodeEstimates.replaceBlock(i,1,T1Offset * pgUpdated.poseToTform(solution.optimizedPoses(i,:)));
            end

            % Update solutionInfo
            solutionInfo.Iterations = solution.NumIterations;
            solutionInfo.Error = solution.FinalChi;
            solutionInfo.ExitFlag = solution.ExitFlag;

        end        
        
        function [pgUpdated, solutionInfo] = optimizeWeighted_tr(pg, paramStruct, weights)
            %optimizeWeighted_tr Weight optimization using builtin trust
            %   region solver
            
            solver = nav.algs.internal.PoseGraphOptimizer.getDefaultSolver(pg.PoseLength);
            solver.setSolverParams(paramStruct);

            pgUpdated = copy(pg); % needs to keep pg's infoMat intact

            args = struct;
            args.edgeNodePairs = pgUpdated.edgeNodePairs;

            for i = 1:pgUpdated.NumEdges
                pgUpdated.scaleInfoMat(i, weights(i));
            end

            [P, In, E] = pgUpdated.dump;
            args.edgeMeasurements = E;
            args.edgeInfoMats = In;
            args.tformSize = pgUpdated.TformSize;
            args.infoMatSize = pgUpdated.PoseInfoMatSize;
            args.poseDeltaLength = pgUpdated.PoseDeltaLength;
            args.nodeMap = pgUpdated.NodeMap;
            args.nodeDims = pgUpdated.NodeDims;
            args.IsLandmarkNode = pgUpdated.IsLandmarkNode;

            solver.ExtraArgs = args;
            solver.CostFcn = @nav.algs.internal.PoseGraphHelpers.poseGraphCost;

            initialGuess = P;

            [posesUpdated, solutionInfo] = solver.solve(initialGuess);

            % re-align the optimized node poses
            T1 = posesUpdated.extractBlock(1,1);

            T1Offset = pg.tforminv(T1);
            for i = 1:pgUpdated.NumNodes
                posesUpdated.replaceBlock(i,1, T1Offset * posesUpdated.extractBlock(i,1));
            end

            pgUpdated.quickUpdateNodeEstimates(posesUpdated.Matrix);

        end
        
        
        function [poseGraphUpdated, solutionInfo, hessian] = optimize(poseGraph, paramStruct)
        %optimize
            poseGraphUpdated = copy(poseGraph);
            if ~isempty(paramStruct.LoopClosuresToIgnore)
                poseGraphUpdated.removeEdges(paramStruct.LoopClosuresToIgnore);
            end
            T = poseGraphUpdated.poseToTform(paramStruct.FirstNodePose);

            hessian = speye(poseGraph.NumNodes*poseGraph.PoseLength);
            switch paramStruct.SolverID
              case 0 %g2o
                if coder.target('MATLAB')
                    
                    solver = nav.algs.internal.PoseGraphOptimizer.configureG2oSolver(paramStruct);
                    nodePoses = poseGraphUpdated.nodeEstimates;
                    edgeIds = poseGraphUpdated.edgeNodePairs;
                    [relPoses,infoMats] = poseGraphUpdated.edgeConstraints;
                    solution = solver.solve(nodePoses',[edgeIds-1,relPoses]',infoMats');

                    % update the pose graph with optimizedPoses
                    T1 = poseGraphUpdated.poseToTform(solution.optimizedPoses(1,:));
                    T1Offset = T*poseGraphUpdated.tforminv(T1);
                    for i = 1:poseGraphUpdated.NumNodes
                        poseGraphUpdated.NodeEstimates.replaceBlock(i,1, ...
                                                            T1Offset * poseGraphUpdated.poseToTform(solution.optimizedPoses(i,:)));
                    end

                    % Update solutionInfo
                    solutionInfo.Iterations = solution.NumIterations;
                    solutionInfo.Error = solution.FinalChi;
                    solutionInfo.ExitFlag = solution.ExitFlag;

                else
                    % Update solutionInfo for codegeneration
                    solutionInfo.Iterations = 0;
                    solutionInfo.Error = 0;
                    solutionInfo.ExitFlag = 0;
                    % g2o solver doesn't support codegen
                    coder.internal.assert(~(paramStruct.SolverID==0),'nav:navalgs:optimizeposegraph:NoCodegenSupportForG2O');
                end
              otherwise
                solver = nav.algs.internal.PoseGraphOptimizer.getDefaultSolver(poseGraphUpdated.PoseLength);

                solver.setSolverParams(paramStruct);

                args = struct;
                args.edgeNodePairs = poseGraphUpdated.edgeNodePairs;

                [P, In, E] = poseGraphUpdated.dump;
                args.edgeMeasurements = E;
                args.edgeInfoMats = In;
                args.tformSize = poseGraphUpdated.TformSize;
                args.infoMatSize = poseGraphUpdated.PoseInfoMatSize;
                args.poseDeltaLength = poseGraphUpdated.PoseDeltaLength;
                args.nodeMap = poseGraphUpdated.NodeMap;
                args.nodeDims = poseGraphUpdated.NodeDims;
                args.IsLandmarkNode = poseGraphUpdated.IsLandmarkNode;

                solver.ExtraArgs = args;
                solver.CostFcn = @nav.algs.internal.PoseGraphHelpers.poseGraphCost;

                initialGuess = P;

                [posesUpdated, solutionInfo, hessian] = solver.solve(initialGuess);

                % re-align the optimized node poses
                T1 = posesUpdated.extractBlock(1,1);

                T1Offset = T*poseGraphUpdated.tforminv(T1);
                for i = 1:poseGraphUpdated.NumNodes
                    posesUpdated.replaceBlock(i,1, T1Offset * posesUpdated.extractBlock(i,1));
                end
                poseGraphUpdated.quickUpdateNodeEstimates(posesUpdated.Matrix);
            end


        end

        function solver = getDefaultSolver(poseLength)
        %getDefaultSolver
            if poseLength == 3
                solver = robotics.core.internal.TrustRegionIndefiniteDogLegSE2();
            else
                solver = robotics.core.internal.TrustRegionIndefiniteDogLegSE3();
            end
        end

        function solver = configureG2oSolver(paramStruct)
        %configureG2oSolver
            solver = matlabshared.g2o.internal.SLAMBackendInterface();
            solver.setParams(...
                paramStruct.OptimizationType, ...
                paramStruct.MaxNumIteration,...
                paramStruct.MaxTime, ...
                paramStruct.FunctionTolerance,...
                paramStruct.IsVerbose, ...
                paramStruct.SolverID);
        end

        function paramStruct = parseDefaultSolver(poseLength,varargin)
        % prepare default values
            solver = nav.algs.internal.PoseGraphOptimizer.getDefaultSolver(poseLength);

            defaultSolverParams = solver.getSolverParams;
            % Parse input
            parameterNames = {'MaxIterations', 'MaxTime', 'GradientTolerance', 'FunctionTolerance', 'StepTolerance', 'InitialTrustRegionRadius', 'VerboseOutput', 'LoopClosuresToIgnore', 'FirstNodePose'};

            defaultVerboseOutput = 'off';
            if defaultSolverParams.IsVerbose
                defaultVerboseOutput = 'on';
            end
            defaultPose = [0 0 0];
            if poseLength == 7
                defaultPose = [0 0 0 1 0 0 0];

            end
            defaultValuesForParameters = {defaultSolverParams.MaxNumIteration, ...
                                defaultSolverParams.MaxTime, ...
                                defaultSolverParams.GradientTolerance, ...
                                defaultSolverParams.FunctionTolerance, ...
                                defaultSolverParams.StepTolerance, ...
                                defaultSolverParams.InitialTrustRegionRadius, ...
                                defaultVerboseOutput, ...
                                zeros(1,0), ...
                                defaultPose};

            parser = robotics.core.internal.NameValueParser(parameterNames, defaultValuesForParameters);
            parse(parser, varargin{:});


            % Extract parsed inputs
            maxIterations     = parameterValue(parser, parameterNames{1});
            maxTime           = parameterValue(parser, parameterNames{2});
            gradTol           = parameterValue(parser, parameterNames{3});
            funTol            = parameterValue(parser, parameterNames{4});
            stepTol           = parameterValue(parser, parameterNames{5});
            initialTRRadius   = parameterValue(parser, parameterNames{6});
            verboseOutput     = parameterValue(parser, parameterNames{7});
            lcToIgnore        = parameterValue(parser, parameterNames{8});
            firstNodePose     = parameterValue(parser, parameterNames{9});

            % validate parsed inputs

            % MaxIterations
            validateattributes(maxIterations, {'numeric'}, {'nonempty', 'scalar', 'real', ...
                                'nonnan', 'finite', 'integer', 'positive'}, 'optimizePoseGraph', 'MaxIterations');
            paramStruct.MaxNumIteration = double(maxIterations);

            % MaxTime
            paramStruct.MaxTime = robotics.internal.validation.validatePositiveNumericScalar(maxTime, 'optimizePoseGraph', 'MaxTime');


            % FunTol
            paramStruct.FunctionTolerance = robotics.internal.validation.validatePositiveNumericScalar(funTol, 'optimizePoseGraph', 'FunctionTolerance');

            % VerboseOutput
            % Validatestring error message was not clear to users g1885126
            % so using double validation
            validateattributes(verboseOutput,{'char','string'},{'nonempty','scalartext'},'optimizePoseGraph','VerboseOutput');
            verboseOutput = validatestring(verboseOutput, {'on', 'off'}, 'optimizePoseGraph', 'VerboseOutput');
            if strcmp(verboseOutput, 'on')
                paramStruct.IsVerbose = true;
            else
                paramStruct.IsVerbose = false;
            end

            % GradTol
            paramStruct.GradientTolerance = robotics.internal.validation.validatePositiveNumericScalar(gradTol, 'optimizePoseGraph', 'GradientTolerance');

            % StepTol
            paramStruct.StepTolerance = robotics.internal.validation.validatePositiveNumericScalar(stepTol, 'optimizePoseGraph', 'StepTolerance');

            % InitialTRRadius
            paramStruct.InitialTrustRegionRadius = robotics.internal.validation.validatePositiveNumericScalar(initialTRRadius, 'optimizePoseGraph', 'InitialTrustRegionRadius');



            % LoopClosuresToIgnore
            validateattributes(lcToIgnore, {'numeric'}, {'vector', 'real', ...
                                'nonnan', 'integer', 'finite'}, 'optimizePoseGraph', 'LoopClosuresToIgnore');
            paramStruct.LoopClosuresToIgnore = double(lcToIgnore);

            % FirstNodePose
            paramStruct.FirstNodePose = robotics.internal.validation.validateVectorNumElements(firstNodePose, poseLength, 'optimizePoseGraph', 'FirstNodePose');

            paramStruct.TrustRegionRadiusTolerance = defaultSolverParams.TrustRegionRadiusTolerance;

            % SolverID
            paramStruct.SolverID = -1;
        end

        function paramStruct = parseG2oSolver(poseLength, solverID, varargin)
            %parseG2oSolver
            parameterNames = {'MaxIterations', 'MaxTime', 'FunctionTolerance', 'LoopClosuresToIgnore','VerboseOutput','FirstNodePose'};
            defaultPose = [0 0 0];
            if poseLength == 7
                defaultPose = [0 0 0 1 0 0 0];

            end
            defaultValuesForParameters = {300,500,1e-6,zeros(1,0),'off',defaultPose};

            parser = inputParser();
            parser.KeepUnmatched = true;
            for i = 1:numel(parameterNames)
                parser.addParameter(parameterNames{i}, defaultValuesForParameters{i});
            end
            parse(parser, varargin{:});
            wrongParams = fieldnames(parser.Unmatched);

            if ~isempty(wrongParams)
                for k = 1:length(wrongParams)
                    switch wrongParams{k}
                      case {'GradientTolerance', 'StepTolerance', 'InitialTrustRegionRadius'}
                        warning(message('nav:navalgs:optimizeposegraph:WrongParametersPassed',wrongParams{k}));
                      otherwise
                        error(message('MATLAB:InputParser:UnmatchedParameter', wrongParams{k}, 'For a list of valid name-value pair arguments, see the documentation for this function.'));

                    end
                end
            end

            % Extract parsed inputs
            maxIterations     = parser.Results.(parameterNames{1});
            maxTime           = parser.Results.(parameterNames{2});
            funTol            = parser.Results.(parameterNames{3});
            lcToIgnore        = parser.Results.(parameterNames{4});
            verboseOutput     = parser.Results.(parameterNames{5});
            firstNodePose     = parser.Results.(parameterNames{6});

            % validate parsed inputs

            % MaxIterations
            validateattributes(maxIterations, {'numeric'}, {'nonempty', 'scalar', 'real', ...
                                'nonnan', 'finite', 'integer', 'positive'}, 'optimizePoseGraph', 'MaxIterations');
            paramStruct.MaxNumIteration = double(maxIterations);

            % MaxTime
            paramStruct.MaxTime = robotics.internal.validation.validatePositiveNumericScalar(maxTime, 'optimizePoseGraph', 'MaxTime');

            % FunTol
            paramStruct.FunctionTolerance = robotics.internal.validation.validatePositiveNumericScalar(funTol, 'optimizePoseGraph', 'FunctionTolerance');

            % LoopClosuresToIgnore
            validateattributes(lcToIgnore, {'numeric'}, {'vector', 'real', ...
                                'nonnan', 'integer', 'finite'}, 'optimizePoseGraph', 'LoopClosuresToIgnore');
            paramStruct.LoopClosuresToIgnore = double(lcToIgnore);

            % VerboseOutput
            % Validatestring error message was not clear to users g1885126
            % so using double validation
            validateattributes(verboseOutput,{'char','string'},{'nonempty','scalartext'},'optimizePoseGraph','VerboseOutput');
            verboseOutput = validatestring(verboseOutput, {'on', 'off'}, 'optimizePoseGraph', 'VerboseOutput');
            if strcmp(verboseOutput, 'on')
                paramStruct.IsVerbose = true;
            else
                paramStruct.IsVerbose = false;
            end

            % FirstNodePose
            paramStruct.FirstNodePose = robotics.internal.validation.validateVectorNumElements(firstNodePose, poseLength, 'optimizePoseGraph', 'FirstNodePose');

            % OptimizationType
            if poseLength == 3
                % SE2 optimization
                paramStruct.OptimizationType = 0;
            else
                %SE3 optimization
                paramStruct.OptimizationType = 1;
            end

            % SolverID
            paramStruct.SolverID = solverID;
        end

        function solverID = getSolverId(solver)
        % Select g2o solver for optimization
        % Available Solvers and their ids:
        % solver - solverID
        % builtin-trust-region - (-1)
        % g2o-levenberg-marquardt - (0)
            switch solver
              case nav.algs.internal.PoseGraphOptimizer.AvailableSolvers{2}
                solverID = 0;
              otherwise
                solverID = -1;
            end
        end

    end
end

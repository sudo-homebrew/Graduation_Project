classdef(StrictDefaults) inverseKinematics < matlab.System & ...
                               robotics.manip.internal.InternalAccess
    %inverseKinematics Create inverse kinematics solver
    %   Given the desired SE3 pose of an end-effector, the inverse  
    %   kinematics solver computes the joint configurations that realize
    %   the desired end-effector pose.
    %
    %   IK = robotics.INVERSEKINEMATICS returns an inverse kinematics 
    %   system object, IK, with the rigid body tree model unset. The solver
    %   algorithm is default to 'BFGSGradientProjection'
    %
    %   IK = robotics.INVERSEKINEMATICS('PropertyName', PropertyValue, ...) 
    %   returns an inverse kinematics system object, IK, with each specified
    %   property set to the specified value.
    %
    %   Step method syntax:
    %
    %   [QSOL, SOLINFO] = step(IK, ENDEFFECTORNAME, TFORM, WEIGHTS, INITIALGUESS) 
    %   finds the joint configuration, QSOL (a structure array),
    %   given the end-effector (body) with name ENDEFFECTORNAME,
    %   the desired SE3 pose of that end-effector, TFORM (in the
    %   form of 4x4 a homogeneous transformation), the weight on each
    %   component of the desired pose, WEIGHTS, and an initial guess of the
    %   solution, INITIALGUESS (a structure array). A second output,  
    %   SOLINFO, provides more detailed information about the solving process.
    %
    %   System objects may be called directly like a function instead of 
    %   using the step method. For example, 
    %   QSol = step(ik, endEffectorName, tform, weights, initialGuess) and 
    %   QSol = ik(endEffectorName, tform, weights, initialGuess) are
    %   equivalent.
    %
    %   inverseKinematics properties:
    %
    %   RigidBodyTree          - Rigid body tree model 
    %   SolverAlgorithm        - Solver algorithm name
    %   SolverParameters       - Parameters for specific algorithm 
    %
    %   inverseKinematics methods:
    %
    %   step        - Compute IK solution
    %   release     - Allow property value changes
    %
    %   Example
    %
    %       % Load rigid body tree model
    %       load exampleRobots.mat
    %
    %       % Create an inverse kinematics object
    %       ik = inverseKinematics('RigidBodyTree', puma1);
    %
    %       % Set up desired end-effector pose, weights and initial guess
    %       tform = [  0.7071         0   -0.7071    0.1408
    %                       0         1         0   -0.1500
    %                  0.7071         0    0.7071    0.3197
    %                       0         0         0    1     ];
    %       weights = [0.25 0.25 0.25 1 1 1];
    %       initialGuess = puma1.homeConfiguration;
    %
    %       % Call IK solver, the end-effector body name is 'L6'
    %       [QSol, solnInfo] = ik('L6', tform, weights, initialGuess);
    %      
    %   See also rigidBodyTree.
    
    %   Copyright 2016-2019 The MathWorks, Inc.
    
    %#codegen
    
    properties (Nontunable, Dependent)
        %RigidBodyTree The rigid body tree model
        %
        %   Default: []
        RigidBodyTree 

        %SolverAlgorithm The algorithm used to solve the IK problem
        %   Possible choices are 'BFGSGradientProjection' and 'LevenbergMarquardt'
        %
        %   Default: 'BFGSGradientProjection'
        SolverAlgorithm 
    end
    
    properties (Dependent)
        %SolverParameters Parameters associated with the specified algorithm
        %   Solver parameters vary depending on the solver. 
        SolverParameters
        
    end
    
    properties (Access = {?robotics.manip.internal.InternalAccess, ...
                          ?inverseKinematics})
        %Solver Solver object
        Solver = []
        
        %SolverAlgorithmInternal
        SolverAlgorithmInternal
        
        %Limits Joint limit matrix extracted from the rigid body tree model
        Limits
        
        %LastGoal
        LastGoal
        
        %RigidBodyTreeInternal
        RigidBodyTreeInternal
        
        %UseTimerInternal Flag that explicitly control whether underlying
        %solvers reference SystemTimeProvider (Platform-specific timer
        %object) or not. The SystemTimeProvider will cause compilation
        %issues in cross-platform deployment.
        UseTimerInternal
    end
    
    methods (Access = protected)
        
        function setupImpl(obj)
            %setupImpl Assemble IK problem 
            
            obj.assembleProblem();
            
            % Load variable bounds, optional for solvers
            obj.Limits = obj.RigidBodyTreeInternal.JointPositionLimits;
            
            % Configure extra arguments to be passed to callbacks
            obj.setupExtraArgs();
        end
        
        function setupExtraArgs(obj)
            %setupExtraArgs
            % Initialize extra arguments
            obj.Solver.ExtraArgs = robotics.manip.internal.IKExtraArgs();
            obj.Solver.ExtraArgs.WeightMatrix = zeros(6);
            obj.Solver.ExtraArgs.Robot = obj.RigidBodyTreeInternal; %This is called in robotics.manip.internal.IKHelpers
            obj.Solver.ExtraArgs.Limits = obj.Limits;
            obj.Solver.ExtraArgs.Tform = eye(4);
            
            coder.varsize('bname', [1, obj.RigidBodyTreeInternal.MaxNameLength], [false, true]);
            bname = '';
            obj.Solver.ExtraArgs.BodyName = bname;
            
            coder.varsize('errTmp', [6,1], [true, false]);
            errTmp = zeros(6,1);
            obj.Solver.ExtraArgs.ErrTemp = errTmp;
            obj.Solver.ExtraArgs.CostTemp = 0;
            
            coder.varsize('gradTmp', [obj.RigidBodyTreeInternal.MaxNumBodies*obj.RigidBodyTreeInternal.MaxJointPositionNumber, 1], [true, false]);
            gradTmp = zeros(obj.RigidBodyTreeInternal.PositionNumber,1);
            obj.Solver.ExtraArgs.GradTemp = gradTmp;
            
        end

        function releaseImpl(obj) %#ok<MANU>
            % Release resources, such as file handles
        end
        
        function [QSol, solutionInfo] = stepImpl(obj, endEffectorName, tform, weights, initialGuess)
            %stepImpl Solve IK
            
            setPoseGoal(obj, endEffectorName, tform, weights);
            [QSol, solutionInfo] = solve(obj, initialGuess);            
        end



        function num = getNumInputsImpl(obj) %#ok<MANU>
            % Define total number of inputs for system with optional inputs
            
            num = 4;
        end

        function num = getNumOutputsImpl(obj) %#ok<MANU>
            % Define total number of outputs 
            
            num = 2;

        end                

        function loadObjectImpl(obj,s,wasLocked)
            %loadObjectImpl

            loadObjectImpl@matlab.System(obj,s,wasLocked);

            obj.RigidBodyTreeInternal = s.RigidBodyTreeInternal;
            obj.SolverAlgorithmInternal = s.SolverAlgorithmInternal;
            obj.Solver = copy(s.Solver);

            obj.Limits = s.Limits;
            obj.LastGoal = s.LastGoal;
        end

        function s = saveObjectImpl(obj)
            %saveObjectImpl
            
            s = saveObjectImpl@matlab.System(obj);
            
            s.RigidBodyTreeInternal = obj.RigidBodyTreeInternal;
            s.SolverAlgorithmInternal = obj.SolverAlgorithmInternal;
            s.Solver = copy(obj.Solver);
            
            s.Limits = obj.Limits;
            s.LastGoal = obj.LastGoal;
            
        end
        
        function flag = isInputSizeMutableImpl(obj,~)            
            flag = true;
        end
    end
    
    methods 
        function obj = inverseKinematics(varargin)
            %inverseKinematics Inverse kinematics Constructor
            %   The first six possible inputs consist of the three
            %   Name-Value pairs specified in the documentation and help
            %   text above (RigidBodyTree, SolverAlgorithm, and
            %   SolverParameters, and their associated values).
            %
            %   The following input is for internal use only, and may not
            %   be supported in a future release:
            %      The constructor can also accept 8 inputs. These include
            %      the six inputs above, as well as an additional
            %      Name-Value pair:
            %         UseTimer -   Indicates whether to use the 
            %                      SystemTimeProvider or a mock version
            %                      that does not call platform-specific
            %                      time functions. 
            %                      Values: true (default) | false
            
            narginchk(0,8);
            if nargin > 6
                obj.UseTimerInternal = varargin{8};
            else
                obj.UseTimerInternal = true;
            end
            
            if ~coder.target('MATLAB')
                hasTree = containsName('RigidBodyTree', varargin{:});
                coder.internal.assert(hasTree, 'robotics:robotmanip:inversekinematics:RBTRequiredAtConstructionForCodegen');
                
                hasAlg = containsName('SolverAlgorithm', varargin{:});
                if ~hasAlg
                    obj.SolverAlgorithm = 'BFGSGradientProjection'; 
                end
                
            else
                obj.SolverAlgorithm = 'BFGSGradientProjection'; 
                defaultRBT = rigidBodyTree();
                obj.RigidBodyTreeInternal = defaultRBT.TreeInternal;
            end
            
            if nargin > 6
                setProperties(obj,6,varargin{1:6});
            else
                setProperties(obj,nargin,varargin{:});
            end

        end
    end
    
    methods (Access = private)
        function setPoseGoal(obj, endEffectorName, tform, weights)
            %setPoseGoal Create cost and gradient function handles 
            if strcmp(obj.RigidBodyTreeInternal.BaseName, endEffectorName)
                robotics.manip.internal.error('inversekinematics:EndEffectorIsBase'); 
            end
            validateInputBodyName(obj.RigidBodyTreeInternal, endEffectorName);
            
            validateattributes(tform, {'double'}, {'nonempty','real', ...
                'size',[4, 4]}, 'setPoseGoal', 'tform'); 
            
            R = tform(1:3,1:3);
            lastrow = tform(4,:);
            % a rudimentary check on the validity of tform matrix and will
            % provide a warning if the check fails
            if max(max(abs(inv(R) - R')))>1e-4 || norm(lastrow - [0 0 0 1])>1e-7
               robotics.manip.internal.warning(...
                 'inversekinematics:HomogeneousTransformInvalid'); 
            end
            
            validateattributes(weights, {'double'}, ...
              {'nonempty', 'real', 'vector', 'nonnan', 'nonnegative','finite', 'numel', 6},...
                    'setPoseGoal', 'weights');
            weightMatrix = diag(weights);
            
            args = obj.Solver.ExtraArgs;
            args.WeightMatrix = weightMatrix;
            
            args.BodyName = endEffectorName;
            args.Tform = tform;


            obj.Solver.CostFcn = @robotics.manip.internal.IKHelpers.computeCost;
            obj.Solver.GradientFcn = @robotics.manip.internal.IKHelpers.computeGradient;
            
            obj.Solver.SolutionEvaluationFcn = @robotics.manip.internal.IKHelpers.evaluateSolution;
            
            obj.Solver.BoundHandlingFcn = @robotics.manip.internal.IKHelpers.clamping;
            
           
            obj.Solver.RandomSeedFcn = @robotics.manip.internal.IKHelpers.randomConfig;
            
            goal = struct();
            goal.EE = endEffectorName;
            goal.Tf = tform;
            goal.W = weights;
            
            obj.LastGoal = goal;
        end
        
        function [QSol, solutionInfo] = solve(obj, initialGuess)
            %solve
            iniGuessVec = obj.RigidBodyTreeInternal.validateConfigurationWithLimits(initialGuess, true); % check joint limits, autoAdjust

            [qvSolRaw, sol] = obj.Solver.solve(iniGuessVec);
            
            % for joints not on the shortest path from end-effector to
            % base, their position values are kept the same as initial
            % guess.
            qvSol = iniGuessVec;
            bodyIndices = obj.RigidBodyTreeInternal.kinematicPathToBase(obj.Solver.ExtraArgs.BodyName);
            positionIndices = bodyIndicesToPositionIndices( ...
                obj.RigidBodyTreeInternal, bodyIndices);
            qvSol(positionIndices) = qvSolRaw(positionIndices);
            
            QSol = formatConfiguration(obj.RigidBodyTreeInternal, qvSol);

            solutionInfo.Iterations = sol.Iterations;
            solutionInfo.NumRandomRestarts = sol.RRAttempts;
            solutionInfo.PoseErrorNorm = sol.Error;
            solutionInfo.ExitFlag = sol.ExitFlag;
            solutionInfo.Status = sol.Status;
        end
        
    end
    
    methods 
        function solverparams = get.SolverParameters(obj)
            %get.SolverParameters
            params = obj.Solver.getSolverParams;
            
            solverparams.MaxIterations = params.MaxNumIteration;
            solverparams.MaxTime = params.MaxTime;
            solverparams.GradientTolerance = params.GradientTolerance;
            solverparams.SolutionTolerance = params.SolutionTolerance;
            solverparams.EnforceJointLimits = params.ConstraintsOn;
            solverparams.AllowRandomRestart = params.RandomRestart;
            solverparams.StepTolerance = params.StepTolerance;            
            
            switch (obj.SolverAlgorithmInternal)
                case 'BFGSGradientProjection'

                case 'LevenbergMarquardt'  
                    solverparams.ErrorChangeTolerance = params.ErrorChangeTolerance;
                    solverparams.DampingBias = params.DampingBias;
                    solverparams.UseErrorDamping = params.UseErrorDamping;
            end
            
        end
        
        function set.SolverParameters(obj, solverparams)
            %set.SolverParameters Validate and set solver parameters

            params = obj.Solver.getSolverParams;
            
            validateattributes(solverparams, {'struct'},{'scalar'},...
                'inverseKinematics', 'SolverParameters')
            
            if isfield(solverparams, 'MaxIterations')
                validateattributes(solverparams.MaxIterations, {'double'}, ...
                     {'nonempty',  'nonnan', 'finite', 'integer', 'scalar', '>',1}, ...
                      'SolverParameters','MaxIterations');
                params.MaxNumIteration = solverparams.MaxIterations;
            end

            if isfield(solverparams, 'MaxTime')
                if obj.UseTimerInternal
                    %When a timer is used, the MaxTime value must be
                    %finite.
                    validateattributes(solverparams.MaxTime, {'double'}, ...
                         {'nonempty', 'nonnan', 'finite', 'real', 'scalar', '>', 0}, 'SolverParameters','MaxTime');                   
                    params.MaxTime = solverparams.MaxTime;
                else
                    %When UseTimerInternal is false, the MaxTime value is
                    %mostly a placeholder and may be infinite.
                    validateattributes(solverparams.MaxTime, {'double'}, ...
                         {'nonempty', 'nonnan', 'real', 'scalar', '>', 0}, 'SolverParameters','MaxTime');   
                    params.MaxTime = solverparams.MaxTime;
                end
            end

            if isfield(solverparams, 'GradientTolerance')
                validateattributes(solverparams.GradientTolerance, {'double'}, ...
                     {'nonempty', 'nonnan', 'finite', 'real', 'scalar', '>', 1e-14}, 'SolverParameters','GradientTolerance');
                params.GradientTolerance = solverparams.GradientTolerance;
            end

            if isfield(solverparams, 'SolutionTolerance')
                validateattributes(solverparams.SolutionTolerance, {'double'}, ...
                     {'nonempty', 'nonnan', 'finite', 'real', 'scalar', '>', 1e-14}, 'SolverParameters','SolutionTolerance');
                params.SolutionTolerance = solverparams.SolutionTolerance;
            end

            if isfield(solverparams, 'EnforceJointLimits') 
                validateattributes(solverparams.EnforceJointLimits, {'logical', 'numeric'}, ...
                     {'nonempty', 'scalar', 'nonnan', 'finite', 'real'}, 'SolverParameters','EnforceJointLimits');
                params.ConstraintsOn = logical(solverparams.EnforceJointLimits);
            end

            if isfield(solverparams, 'AllowRandomRestart')
                validateattributes(solverparams.AllowRandomRestart, {'logical','numeric',}, ...
                     {'nonempty','scalar', 'nonnan', 'finite', 'real'}, 'SolverParameters','AllowRandomRestart');
                params.RandomRestart = logical(solverparams.AllowRandomRestart);
            end

            if isfield(solverparams, 'StepTolerance')
                validateattributes(solverparams.StepTolerance, {'double'}, ...
                     {'nonempty','scalar', 'nonnan', 'finite', 'real', '>', 1e-18}, 'SolverParameters','StepTolerance');
                params.StepTolerance = solverparams.StepTolerance;
            end
            
            switch (obj.Solver.Name)
                case 'BFGSGradientProjection'
                    
                case 'LevenbergMarquardt' 
                    if isfield(solverparams, 'ErrorChangeTolerance')
                        validateattributes(solverparams.ErrorChangeTolerance, {'double'}, ...
                            {'nonempty','scalar', 'nonnan', 'finite', 'real', '>', 1e-14}, 'SolverParameters', 'ErrorChangeTolerance');
                        params.ErrorChangeTolerance = solverparams.ErrorChangeTolerance;
                    end
                    
                    if isfield(solverparams, 'DampingBias')
                        validateattributes(solverparams.DampingBias, {'double'}, ...
                            {'nonempty','scalar', 'nonnan', 'finite', 'real', '>', 1e-14}, 'SolverParameters', 'DampingBias');
                        params.DampingBias = solverparams.DampingBias;
                    end
                    
                    if isfield(solverparams, 'UseErrorDamping')
                        validateattributes(solverparams.UseErrorDamping, {'logical','numeric',}, ...
                            {'nonempty','scalar', 'nonnan', 'finite', 'real'}, 'SolverParameters','UseErrorDamping');
                        params.UseErrorDamping = logical(solverparams.UseErrorDamping);
                    end
            end
            
            obj.Solver.setSolverParams(params);
        end
        
        function set.SolverAlgorithm(obj, salgorithm)
            %set.SolverAlgorithm
            
            obj.SolverAlgorithmInternal = validatestring(salgorithm, ...
                {'BFGSGradientProjection','LevenbergMarquardt'}, ...
                'inverseKinematics', 'SolverAlgorithm');
            
            switch (obj.SolverAlgorithmInternal)
                case 'BFGSGradientProjection'
                    obj.Solver = robotics.core.internal.DampedBFGSwGradientProjection(obj.UseTimerInternal);
                case 'LevenbergMarquardt'
                    obj.Solver = robotics.core.internal.ErrorDampedLevenbergMarquardt(obj.UseTimerInternal);
            end
            
        end
        
        function salgorithm = get.SolverAlgorithm(obj)
            %get.SolverAlgorithm
            salgorithm = obj.SolverAlgorithmInternal;
        end
        
        function set.RigidBodyTree(obj, rigidbodytree)
            %set.RigidBodyTree 
            
            if isa(rigidbodytree, 'robotics.manip.internal.RigidBodyTree')
            % For internal use only: provide robotics.manip.internal.RigidBodyTree as an input
                obj.RigidBodyTreeInternal = copy(rigidbodytree); 
            else
            
                validateattributes(rigidbodytree, {'rigidBodyTree'},...
                    {'nonempty','scalar'},'inverseKinematics', 'rigidbodytree');

                if rigidbodytree.NumNonFixedBodies == 0
                    robotics.manip.internal.error(...
                      'inversekinematics:RigidBodyTreeFixed'); 
                end           

                rbt = copy(rigidbodytree);
                obj.RigidBodyTreeInternal = rbt.TreeInternal;   
            end
        end
        
        function rigidbodytree = get.RigidBodyTree(obj)
            %get.RigidBodyTree
            rbtInternal = copy(obj.RigidBodyTreeInternal);
            
            %Create a RigidBodyTree from the robotics.manip.internal.RigidBodyTree
            rigidbodytree = rigidBodyTree(rbtInternal);
        end

    end
    
    methods (Access = private) 
        function assembleProblem(obj)
            % To be optimized (to only include the support set of end-eff)
            n = obj.RigidBodyTreeInternal.PositionNumber;
            A = zeros(n, 2*n);
            b = zeros(2*n,1);
            k = 1; m = 1;
            
            % Form constraints (to be double-checked)  A'*x = b
            for i = 1 : obj.RigidBodyTreeInternal.NumBodies
                joint = obj.RigidBodyTreeInternal.Bodies{i}.Joint;
                pnum = joint.PositionNumber; 
                if ~strcmp(joint.Type, 'fixed')
                    A(k:k+pnum-1, m:m+pnum-1) = eye(pnum);
                    A(k:k+pnum-1, m+pnum:m+2*pnum-1) = -eye(pnum);
                    b(m) = joint.PositionLimits(2);
                    b(m+1) = -joint.PositionLimits(1);
                    m = m + 2*pnum;
                end
                k = k + pnum;
            end
            
            obj.Solver.ConstraintMatrix = A;
            obj.Solver.ConstraintBound = b;            
        end

    end

    methods (Static, Hidden)
        function props = matlabCodegenNontunableProperties(~)
            % In codegen, IK solver can only be specified once
            props = {'SolverAlgorithmInternal','UseTimerInternal'};% 'RigidBodyTreeInternal'
        end
    end
    
end


function hasName = containsName(argName, varargin)
%containsName During codegen, check if the given Name-Value pairs contain a
%   certain ARGNAME. To be called before SetProperties.
    hasName = false;
    for i = 1:length(varargin)
        if strcmp(varargin{i}, argName)
            hasName = true;
            return;
        end
    end
end



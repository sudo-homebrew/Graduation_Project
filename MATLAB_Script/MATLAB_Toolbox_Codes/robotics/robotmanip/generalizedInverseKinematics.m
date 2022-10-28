classdef(StrictDefaults) generalizedInverseKinematics < matlab.System & ...
                                        robotics.manip.internal.InternalAccess
    %generalizedInverseKinematics Create multiconstraint inverse kinematics solver
    %   Given a set of kinematic constraints, the inverse kinematics solver
    %   computes a joint configuration that satisfies those constraints.
    %
    %   GIK = generalizedInverseKinematics returns a generalized
    %   inverse kinematics system object, GIK, with the rigid body tree
    %   model unset. The solver algorithm is default to
    %   'BFGSGradientProjection'
    %
    %   GIK = generalizedInverseKinematics('RigidBodyTree',
    %   rigidbodytree, 'ConstraintInputs', inputs) returns a
    %   generalized inverse kinematics system object, GIK, with the rigid
    %   body tree model, RIGIDBODYTREE, and the expected constraint
    %   inputs, INPUTS.
    %
    %   GIK = generalizedInverseKinematics('PropertyName',
    %   PropertyValue, ...) returns a generalized inverse kinematics system
    %   object, GIK, with each specified property set to the specified
    %   value.
    %
    %   Step method syntax:
    %
    %   [QSOL, SOLINFO] = step(GIK, INITIALGUESS, CONSTRAINT1, CONSTRAINT2,
    %   ...) finds the joint configuration, QSOL, based on INITIALGUESS, and
    %   a comma-separated list of constraint descriptions. Each description
    %   must match the corresponding entry in the ConstraintInputs property
    %   of GIK. SOLINFO is an optional output which provides more detailed
    %   information about the solution process.
    %
    %   System objects may be called directly like a function instead of 
    %   using the step method. For example, 
    %   QSol = step(gik, initialGuess, constraint1, constraint2) and 
    %   QSol = gik(initialGuess, constraint1, constraint2) are
    %   equivalent.
    %
    %   generalizedInverseKinematics properties:
    %
    %   NumConstraints         - Number of constraint inputs
    %   ConstraintInputs       - Constraint type of each constraint input
    %   RigidBodyTree          - Rigid body tree model 
    %   SolverAlgorithm        - Solver algorithm name
    %   SolverParameters       - Parameters for specific algorithm 
    %
    %   generalizedInverseKinematics methods:
    %
    %   step        - Compute solution
    %   release     - Allow property value changes
    %
    %   Constraint inputs:
    %
    %   Specify the constraints on the robot using the following constraint
    %   types, which correspond to MATLAB objects. You must specify the
    %   MATLAB objects as inputs to the GIK object when you call it. The
    %   ConstraintInput property should be a cell array consisting of any
    %   combination of these character vectors:
    %       - 'orientation': constraintOrientationTarget
    %       - 'position': constraintPositionTarget
    %       - 'pose': constraintPoseTarget
    %       - 'aiming': constraintAiming
    %       - 'cartesian': constraintCartesianBounds
    %       - 'jointbounds': constraintJointBounds
    %       - 'distance': constraintDistanceBounds
    %
    %       Joint constraints 
    %       - 'revolutejoint': constraintRevoluteJoint
    %       - 'prismaticjoint': constraintPrismaticJoint
    %       - 'fixedjoint': constraintFixedJoint
    %
    %   Example:
    %
    %       % Load pre-defined robot models
    %       load exampleRobots.mat
    %
    %       % Create system object 
    %       gik = generalizedInverseKinematics();
    %
    %       % Configure the system object to use the KUKA LBR robot
    %       gik.RigidBodyTree = lbr;
    %
    %       % Tell the solver to expect a constraintPositionTarget and a
    %       % constraintAiming as the constraint inputs.
    %       gik.ConstraintInputs = {'position','aiming'};
    %
    %       % Create constraint objects
    %       % Constraint 1: The origin of the body named "tool0" should be
    %       % located at [0.0, 0.5, 0.5] relative to the robot's base.
    %       posTgt = constraintPositionTarget('tool0');
    %       posTgt.TargetPosition = [0.0, 0.5, 0.5];
    %
    %       % Constraint 2: The z-axis of the body named "tool0" should
    %       % point towards the origin of the robot's base frame.
    %       aimCon = constraintAiming('tool0');
    %       aimCon.TargetPoint = [0.0, 0.0, 0.0];
    %
    %       % Find a configuration that satisfies the constraints. Note
    %       % that the constraint objects are passed to the system object
    %       % in the order specified by the ConstraintInputs property.
    %       q0 = homeConfiguration(lbr); % Initial guess for solver
    %       [q, solutionInfo] = gik(q0, posTgt, aimCon);
    %
    %       % Visualize the configuration returned by the solver.
    %       show(lbr, q);
    %       display(['Solver status: ' solutionInfo.Status]);
    %
    %       % Plot the line segment from the target position to the origin
    %       % of the base. The origin of the "tool0" frame should coincide
    %       % with one end of the segment, and its z-axis should be aligned
    %       % with the segment.
    %       hold on;
    %       plot3([0.0, 0.0], [0.5, 0.0], [0.5, 0.0], '--o')
    %      
    %   See also rigidBodyTree, constraintOrientationTarget,
    %   constraintPositionTarget, constraintPoseTarget,
    %   constraintPrismaticJoint, constraintAiming,
    %   constraintCartesianBounds, constraintRevoluteJoint,
    %   constraintJointBounds, inverseKinematics
    
    %   Copyright 2016-2021 The MathWorks, Inc.

    %#codegen
    
    properties(Dependent, SetAccess = private)
        %NumConstraints Number of constraint inputs
        NumConstraints
    end
    
    properties(Nontunable)
        %ConstraintInputs Cell array of character vectors
        %   Each element of ConstraintInputs indicates the type of
        %   constraint object expected by the corresponding constraint
        %   input to the step method.
        ConstraintInputs = cell(1,0);
    end
    
    properties(Nontunable, Dependent)
        %RigidBodyTree The rigid body tree model
        RigidBodyTree

        %SolverAlgorithm The algorithm used to solve the IK problem
        %   Possible choices are 'BFGSGradientProjection' and 'LevenbergMarquardt'
        %
        %   Default: 'BFGSGradientProjection'
        SolverAlgorithm
    end
    
    properties(Dependent)
        %SolverParameters Parameters associated with the specified algorithm
        %   Solver parameters vary depending on the solver.   
        SolverParameters
    end
    
    properties(Access = {?robotics.manip.internal.InternalAccess, ...
                          ?generalizedInverseKinematics})
        %Solver Instance of a class that implements
        %robotics.internal.manip.NLPSolverInterface
        Solver
        
        %SolverAlgorithmInternal
        SolverAlgorithmInternal
        
        %Tree Instance of robotics.manip.internal.RigidBodyTree
        Tree
        
        %Problem Instance of robotics.manip.internal.GIKProblem
        %   Defines the generalized inverse kinematics problem. Note that
        %   this property is initialized in SETUP and should only be
        %   accessed after SETUP has been run (e.g. in STEP or if the
        %   system object is locked).
        Problem
        
        %EnforceJointLimits
        EnforceJointLimits = true;
    end
    
    properties (Access = private, Hidden)
        %RigidBodyTreeHasBeenSet
        RigidBodyTreeHasBeenSet = false
    end
    
    methods
        function obj = generalizedInverseKinematics(varargin)
            if ~coder.target('MATLAB')
                hasTree = containsName('RigidBodyTree', varargin{:});
                coder.internal.assert(hasTree, 'robotics:robotmanip:inversekinematics:RBTRequiredAtConstructionForCodegen');
                
                hasAlg = containsName('SolverAlgorithm', varargin{:});
                if ~hasAlg
                    obj.SolverAlgorithm = 'BFGSGradientProjection'; 
                end
                
            else
                obj.SolverAlgorithm = 'BFGSGradientProjection'; 
                obj.Tree = robotics.manip.internal.RigidBodyTree();
            end
            
            setProperties(obj,nargin,varargin{:});
        end
        
    end

    methods(Access = protected)
        function [QSol, solutionInfo] = stepImpl(obj,initialGuess, varargin)
            %stepImpl Solve generalized inverse kinematics problem
            % Validate inputs
            q0 = obj.Tree.validateConfigurationWithLimits(initialGuess,true);
            validateConstraintInputs(obj, varargin{:});
            
            % Update problem description
            % This is safe to do because SETUP is guaranteed to run before
            % STEP and therefore obj.Problem will be initialized.
            obj.Problem.update(varargin{:})
            obj.Problem.EnforceJointLimits = obj.EnforceJointLimits;
            obj.Solver.ConstraintMatrix = obj.Problem.ConstraintMatrix;
            obj.Solver.ConstraintBound = obj.Problem.ConstraintBound;
            
            % Solve optimization problem
            x0 = [q0; obj.Problem.defaultSlackValues(q0)];
            [xSol, solutionInfoRaw] = obj.Solver.solve(x0);
            qSol = xSol(1:obj.Tree.PositionNumber);
            
            % Process optimization results
            [QSol, solutionInfo] = ...
                obj.processResults(qSol, solutionInfoRaw, q0, ...
                                   obj.Problem.KinematicPath, ...
                                   obj.Problem.constraintViolations(qSol));
        end

        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            if ~obj.RigidBodyTreeHasBeenSet
                robotics.manip.internal.error( ...
                    'generalizedinversekinematics:RigidBodyTreeUnset');
            end
            obj.Problem = robotics.manip.internal.GIKProblem(obj.Tree, ...
                                                         obj.ConstraintInputs);
            obj.Problem.EnforceJointLimits = obj.SolverParameters.EnforceJointLimits;
            obj.setupExtraArgs(obj.Problem);
        end

        function num = getNumInputsImpl(obj)
            % Define total number of inputs for system with optional inputs
            num = numel(obj.ConstraintInputs) + 1;
        end

        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s

            % Set private and protected properties
            if wasLocked
                % Only copy obj.Problem if SETUP has run
                obj.Problem = copy(s.Problem);
            end
            obj.Tree = s.Tree;
            obj.RigidBodyTreeHasBeenSet = s.RigidBodyTreeHasBeenSet;
            obj.Solver = copy(s.Solver);
            obj.SolverAlgorithmInternal = s.SolverAlgorithmInternal;
            obj.EnforceJointLimits=s.EnforceJointLimits;

            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end

        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj

            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);

            % Set private and protected properties
            s.Tree = obj.Tree;
            s.RigidBodyTreeHasBeenSet = obj.RigidBodyTreeHasBeenSet;
            if obj.isLocked
                % Only copy obj.Problem if SETUP has run
                s.Problem = copy(obj.Problem);
            end
            s.Solver = copy(obj.Solver);
            s.SolverAlgorithmInternal = obj.SolverAlgorithmInternal;
            s.EnforceJointLimits=obj.EnforceJointLimits;
        end

        function flag = isInputSizeMutableImpl(~,~)
            flag = true;
        end
    end
    
    methods (Access = private)
        
        function [QSol, solInfo] = processResults(obj, qSolRaw, ...
                                                  solInfoRaw, qInitial, ...
                                                  kinematicPath, violations)
            % for joints not on the union of the kinematic paths of all
            % constraints, their position values are kept the same as
            % initial guess.
            qSol = qInitial;
            positionIndices = bodyIndicesToPositionIndices(obj.Tree, ...
                                                           kinematicPath);
            qSol(positionIndices) = qSolRaw(positionIndices);
            QSol = formatConfiguration(obj.Tree, qSol);
            solInfo = struct( ...
                'Iterations', solInfoRaw.Iterations, ...
            	'NumRandomRestarts', solInfoRaw.RRAttempts, ...
            	'ConstraintViolations', violations, ...
            	'ExitFlag', solInfoRaw.ExitFlag, ...
            	'Status', solInfoRaw.Status);
        end
        
        function validateConstraintInputs(obj, varargin)
            %validateConstraintInputTypes Basic validation
            % Checks that each constraint input is an instance of the
            % appropriate class.
            for i = 1:nargin-1
                input = varargin{i};
                switch obj.ConstraintInputs{i}
                    case 'pose'
                        validateattributes(input, {'constraintPoseTarget'}, ...
                            {},'step', '', i+2);
                    case 'orientation'
                        validateattributes(input, {'constraintOrientationTarget'}, ...
                            {},'step', '', i+2);
                    case 'position'
                        validateattributes(input, {'constraintPositionTarget'}, ...
                            {},'step', '', i+2);
                    case 'cartesian'
                        validateattributes(input, {'constraintCartesianBounds'}, ...
                            {},'step', '', i+2);
                    case 'aiming'
                        validateattributes(input, {'constraintAiming'}, ...
                            {},'step', '', i+2);
                    case 'joint'
                        validateattributes(input, {'constraintJointBounds'}, ...
                            {},'step', '', i+2);
                    case 'jointbounds'
                        validateattributes(input, {'constraintJointBounds'}, ...
                            {},'step', '', i+2);
                    case 'distance'
                        validateattributes(input, {'constraintDistanceBounds'}, ...
                            {},'step', '', i+2);
                    case 'prismaticjoint'
                        validateattributes(input, {'constraintPrismaticJoint'}, ...
                            {},'step', '', i+2);
                    case 'revolutejoint'
                        validateattributes(input, {'constraintRevoluteJoint'}, ...
                            {},'step', '', i+2);
                    case 'fixedjoint'
                        validateattributes(input, {'constraintFixedJoint'}, ...
                            {},'step', '', i+2);
                    % No "otherwise" needed. "set.ConstraintInputs"
                    % guarantees that obj.ConstraintInputs{i} is one of the
                    % options handled above.
                end
            end
        end
        
    end
    
    methods % Property access methods
        
        function set.ConstraintInputs(obj, value)
            validateattributes(value, {'cell','string'}, {'row'}, ...
                'set.ConstraintInputs', 'ConstraintInputs');
            for i = 1:numel(value)
                value{i} = validatestring(value{i}, {'orientation', ...
                                                    'position', ...
                                                    'pose', ...
                                                    'cartesian', ...
                                                    'aiming', ...
                                                    'joint', ...
                                                    'jointbounds', ...
                                                    'distance', ...
                                                    'prismaticjoint', ...
                                                    'revolutejoint', ...
                                                    'fixedjoint'}, ...
                    'generalizedInverseKinematics', 'ConstraintInputs');
            end
            obj.ConstraintInputs = value;
        end
        
        function rigidbodytree = get.RigidBodyTree(obj)
            %get.RigidBodyTree
            rigidbodytree = ...
                rigidBodyTree.fromTreeInternal(copy(obj.Tree));
        end
        
        function set.RigidBodyTree(obj, rigidbodytree)
            %set.RigidBodyTree 
            
            validateattributes(rigidbodytree, {'rigidBodyTree'},...
                {'nonempty','scalar'},'generalizedInverseKinematics', 'rigidbodytree');

            if rigidbodytree.NumNonFixedBodies == 0
                robotics.manip.internal.error(...
                  'inversekinematics:RigidBodyTreeFixed'); 
            end           
            
            obj.Tree = copy(rigidbodytree.TreeInternal);   
            obj.RigidBodyTreeHasBeenSet = true;
        end
        
        function solverparams = get.SolverParameters(obj)
            %get.SolverParameters
            params = obj.Solver.getSolverParams;
            
            solverparams.MaxIterations = params.MaxNumIteration;
            solverparams.MaxTime = params.MaxTime;
            solverparams.GradientTolerance = params.GradientTolerance;
            solverparams.SolutionTolerance = params.SolutionTolerance;
            solverparams.EnforceJointLimits = obj.EnforceJointLimits;
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
                'generalizedInverseKinematics', 'SolverParameters')
            
            if isfield(solverparams, 'MaxIterations')
                validateattributes(solverparams.MaxIterations, {'double'}, ...
                     {'nonempty',  'nonnan', 'finite', 'integer', 'scalar', '>',1}, ...
                      'SolverParameters','MaxIterations');
                params.MaxNumIteration = solverparams.MaxIterations;
            end

            if isfield(solverparams, 'MaxTime')
                validateattributes(solverparams.MaxTime, {'double'}, ...
                     {'nonempty', 'nonnan', 'finite', 'real', 'scalar', '>', 0}, 'SolverParameters','MaxTime');                   
                params.MaxTime = solverparams.MaxTime;
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
                obj.EnforceJointLimits = logical(solverparams.EnforceJointLimits);
            end
            
            % The formulation of the GIK problem relies on having bounds on
            % decision variables
            params.ConstraintsOn = true;

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
        
        function value = get.SolverAlgorithm(obj)
            %get.SolverAlgorithm
            value = obj.SolverAlgorithmInternal;
        end
        
        function set.SolverAlgorithm(obj, value)
            %set.SolverAlgorithm
            
            obj.SolverAlgorithmInternal = validatestring(value, ...
                     {'BFGSGradientProjection','LevenbergMarquardt'}, ...
                     'generalizedInverseKinematics', 'SolverAlgorithm');
                 
            switch (obj.SolverAlgorithmInternal)
                case 'BFGSGradientProjection'
                    solver = robotics.core.internal.DampedBFGSwGradientProjection;
                case 'LevenbergMarquardt'
                    solver = robotics.core.internal.ErrorDampedLevenbergMarquardt();
            end
            
            %Set callbacks for generalized inverse kinematics
            solver.CostFcn = ...
                @robotics.manip.internal.GIKHelpers.computeCost;
            solver.GradientFcn = ...
                @robotics.manip.internal.GIKHelpers.computeGradient;
            solver.SolutionEvaluationFcn = ...
                @robotics.manip.internal.GIKHelpers.evaluateSolution;
            solver.BoundHandlingFcn = ...
                @robotics.manip.internal.GIKHelpers.clamping;
            solver.RandomSeedFcn = ...
                @robotics.manip.internal.GIKHelpers.randomConfig;
            
            obj.Solver = solver;
        end
        
        function value = get.NumConstraints(obj)
            value = numel(obj.ConstraintInputs);
        end
        
        function setupExtraArgs(obj,value)
            obj.Solver.ExtraArgs = value;
        end
        
    end
    methods (Static, Hidden)
        function props = matlabCodegenNontunableProperties(~)
            % In codegen, GIK solver can only be specified once
            props = {'SolverAlgorithmInternal', 'ConstraintInputs'};
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

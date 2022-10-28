classdef SolverOptions < nav.algs.internal.InternalAccess
    %SOLVEROPTIONS All possible options for solvers used in Navigation
    %   Toolbox, along with their validation functions
    
    %   Copyright 2020 The MathWorks, Inc.
    
    %#codegen
    
    properties (GetAccess = ?nav.algs.internal.InternalAccess)
        %MaxIterations
        MaxIterations
        
        %MaxTime
        MaxTime
        
        %FunctionTolerance
        FunctionTolerance
        
        %GradientTolerance
        GradientTolerance
        
        %StepTolerance
        StepTolerance
        
        %IsVerbose
        IsVerbose
        
        %InitialTrustRegionRadius
        InitialTrustRegionRadius
        
    end
    
    properties (Access = protected)
        SolverName
    end
       
    methods (Access = ?nav.algs.internal.InternalAccess)
        function obj = SolverOptions(solverName)
            %SOLVEROPTIONS
            obj.SolverName = solverName;
            obj.MaxIterations = 300;
            obj.MaxTime = 10;
            obj.FunctionTolerance = 1e-8;
            obj.GradientTolerance = 1e-9;
            obj.StepTolerance = 1e-12;
            obj.IsVerbose = false;
            obj.InitialTrustRegionRadius = 100;
        end
        
        function paramStruct = dump(obj)
            %dump
            paramStruct.MaxIterations = obj.MaxIterations;
            paramStruct.MaxNumIteration = obj.MaxIterations;
            paramStruct.MaxTime = obj.MaxTime;
            paramStruct.FunctionTolerance = obj.FunctionTolerance;
            paramStruct.GradientTolerance = obj.GradientTolerance;
            paramStruct.StepTolerance = obj.StepTolerance;
            paramStruct.IsVerbose = obj.IsVerbose;
            paramStruct.TrustRegionRadiusTolerance = 1e-7;
            paramStruct.InitialTrustRegionRadius = obj.InitialTrustRegionRadius;
            if strcmp(obj.SolverName, 'TrustRegion')
                paramStruct.SolverID = -1; % builtin-trust-region - (-1)
            else
                paramStruct.SolverID = 0; % g2o-levenberg-marquardt - (0)
            end
            paramStruct.OptimizationType = 0; % 0 - SE2, 1 - SE3
        end
    end
    
    % setters
    methods (Access = ?nav.algs.internal.InternalAccess)
        
        function validateMaxIterations(obj, maxIter)
            %validateMaxIterations
            validateattributes(maxIter, {'numeric'}, {'nonempty', 'scalar', 'real', ...
                                'nonnan', 'finite', 'integer', 'positive'}, obj.SolverName, 'MaxIterations');
            obj.MaxIterations = double(maxIter);
        end
        
        function validateMaxTime(obj, maxT)
            %validateMaxTime
            obj.MaxTime = robotics.internal.validation.validatePositiveNumericScalar(maxT, obj.SolverName, 'MaxTime');
        end
        
        function validateFunctionTolerance(obj, funTol)
            %validateFunctionTolerance
            obj.FunctionTolerance = robotics.internal.validation.validatePositiveNumericScalar(funTol, obj.SolverName, 'FunctionTolerance');
        end
        
        function validateGradientTolerance(obj, gradTol)
            %validateGradientTolerance
            obj.GradientTolerance = robotics.internal.validation.validatePositiveNumericScalar(gradTol, obj.SolverName, 'GradientTolerance');
        end
        
        function validateStepTolerance(obj, stepTol)
            %validateStepTolerance
            obj.StepTolerance = robotics.internal.validation.validatePositiveNumericScalar(stepTol, obj.SolverName, 'StepTolerance');
        end
        
        function validateInitialTrustRegionRadius(obj, initialTRRadius)
            %validateInitialTrustRegionRadius
            obj.InitialTrustRegionRadius = robotics.internal.validation.validatePositiveNumericScalar(initialTRRadius, obj.SolverName, 'InitialTrustRegionRadius');
        end
        
        function validateVerboseOutput(obj, verboseOutput)
            %validateVerboseOutput
            validateattributes(verboseOutput,{'char','string'}, {'nonempty','scalartext'}, obj.SolverName, 'VerboseOutput');
            verboseOutput = validatestring(verboseOutput, {'on', 'off'}, obj.SolverName, 'VerboseOutput');
            if strcmp(verboseOutput, 'on')
                obj.IsVerbose = true;
            else
                obj.IsVerbose = false;
            end
        end
    end
    
end


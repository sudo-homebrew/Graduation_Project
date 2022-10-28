classdef TrustRegion < nav.solveroptions.internal.mixin.SolverOptionCustomDisplay
    %TRUSTREGION Tunable parameters available for the builtin trust-region NLP solver.
    %   Use poseGraphSolverOptions to generate an instance of this class
    %   with default parameter values.
    %
    %   TRUSTREGION properties:
    %   
    %   MaxIterations            - Maximum number of iterations
    %   MaxTime                  - Maximum time
    %   FunctionTolerance        - Lower bound on the change in the cost function
    %   GradientTolerance        - Lower bound on the norm of the gradient    
    %   StepTolerance            - Lower bound on the step size
    %   InitialTrustRegionRadius - The initial trust region radius
    %   VerboseOutput            - Display intermediate iteration information
    %
    %   See also poseGraphSolverOptions
    
    %   Copyright 2020 The MathWorks, Inc.
    
    %#codegen
    
    properties (Constant, Access = private)
        %SolverName
        SolverName = 'TrustRegion';
        
        %DetailedSolverName
        DetailedSolverName = 'builtin-trust-region-dogleg';
    end
    
    properties (Dependent)
        %MaxIterations Maximum number of iterations allowed for optimization
        %   Default: 300
        MaxIterations
        
        %MaxTime Maximum time allowed for optimization
        %   Default: 500
        MaxTime
        
        %FunctionTolerance Lower bound on the change in the cost function
        %   If the scalar cost function value changes less than this number
        %   between two iterations, the optimization concludes.
        %   Default: 1e-8
        FunctionTolerance
        
        %GradientTolerance Lower bound on the norm of the gradient
        %   If the norm of the gradient of the cost function falls below
        %   this number, the optimization concludes.
        %   Default: 0.5e-8
        GradientTolerance
        
        %StepTolerance Lower bound on the step size
        %   If the norm of the optimization step falls below this number,
        %   the iterations end.
        %   Default: 1e-12
        StepTolerance
        
        %InitialTrustRegionRadius The initial trust region radius
        %   Default: 100
        InitialTrustRegionRadius
        
        %VerboseOutput Display intermediate iteration information
        %   Possible values: {'on', 'off'}
        %   Default: 'off'
        VerboseOutput
    end
    
    properties (Access = ?nav.algs.internal.InternalAccess)
        OptionsInternal
    end
    
    methods
        function obj = TrustRegion()
            %TRUSTREGION Constructor
            obj.OptionsInternal = nav.solveroptions.internal.SolverOptions(obj.SolverName);
            obj.OptionsInternal.GradientTolerance = 0.5e-8;
        end
        
        function newObj = copy(obj)
            %copy
            newObj = nav.solveroptions.TrustRegion();
            newObj.OptionsInternal = nav.solveroptions.internal.SolverOptions(obj.SolverName);
            newObj.MaxIterations            = obj.MaxIterations;
            newObj.MaxTime                  = obj.MaxTime;
            newObj.FunctionTolerance        = obj.FunctionTolerance;
            newObj.GradientTolerance        = obj.GradientTolerance;
            newObj.StepTolerance            = obj.StepTolerance;
            newObj.InitialTrustRegionRadius = obj.InitialTrustRegionRadius;
            newObj.VerboseOutput            = obj.VerboseOutput;
        end
    end
    
    methods (Access = protected)
                
        function list = getDisplayOptions(obj)
            %getDisplayOptions
            list.MaxIterations            = obj.MaxIterations;
            list.MaxTime                  = obj.MaxTime;
            list.FunctionTolerance        = obj.FunctionTolerance;
            list.GradientTolerance        = obj.GradientTolerance;
            list.StepTolerance            = obj.StepTolerance;
            list.InitialTrustRegionRadius = obj.InitialTrustRegionRadius;
            list.VerboseOutput            = obj.VerboseOutput;
        end
        
        function sn = getDetailedSolverName(obj)
            %getDetailedSolverName
            sn = obj.DetailedSolverName;
        end
    end
    

    methods % getters
        
        function maxIter = get.MaxIterations(obj)
            %get.MaxIterations
            maxIter = obj.OptionsInternal.MaxIterations;
        end
        
        function maxT = get.MaxTime(obj)
            %get.MaxTime
            maxT = obj.OptionsInternal.MaxTime;
        end
        
        function funTol = get.FunctionTolerance(obj)
            %get.FunctionTolerance
            funTol = obj.OptionsInternal.FunctionTolerance;
        end
        
        function gradTol = get.GradientTolerance(obj)
            %get.GradientTolerance
            gradTol = obj.OptionsInternal.GradientTolerance;
        end
        
        function stepTol = get.StepTolerance(obj)
            %get.StepTolerance
            stepTol = obj.OptionsInternal.StepTolerance;
        end
        
        function iniTRRadius = get.InitialTrustRegionRadius(obj)
            %get.InitialTrustRegionRadius
            iniTRRadius = obj.OptionsInternal.InitialTrustRegionRadius;
        end
        
        function vb = get.VerboseOutput(obj)
            %get.VerboseOutput
            isVerbose = obj.OptionsInternal.IsVerbose;
            if isVerbose
                vb = 'on';
            else
                vb = 'off';
            end
        end
    end
    

    methods % setters
        
        function set.MaxIterations(obj, maxIter)
            %set.MaxIterations
            validateMaxIterations(obj.OptionsInternal, maxIter);
        end
        
        function set.MaxTime(obj, maxT)
            %set.MaxTime
            validateMaxTime(obj.OptionsInternal, maxT);
        end
        
        function set.FunctionTolerance(obj, funTol)
            %set.FunctionTolerance
            validateFunctionTolerance(obj.OptionsInternal, funTol);
        end
        
        function set.GradientTolerance(obj, gradTol)
            %set.GradientTolerance
            validateGradientTolerance(obj.OptionsInternal, gradTol)
        end
        
        function set.StepTolerance(obj, stepTol)
            %set.StepTolerance
            validateStepTolerance(obj.OptionsInternal, stepTol);
        end
        
        function set.InitialTrustRegionRadius(obj, iniTRRadius)
            %set.InitialTrustRegionRadius
            validateInitialTrustRegionRadius(obj.OptionsInternal, iniTRRadius);
        end
        
        function set.VerboseOutput(obj, vb)
            %set.VerboseOutput
            validateVerboseOutput(obj.OptionsInternal, vb)
        end
    end
    
end


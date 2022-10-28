classdef G2oLevenbergMarquardt < nav.solveroptions.internal.mixin.SolverOptionCustomDisplay
    %G2oLevenbergMarquardt Tunable parameters exposed for the Levenberg-Marquardt NLP solver
    %   as implemented in g2o package.
    %   Use poseGraphSolverOptions('g2o-levenberg-marquardt') to generate 
    %   an instance of this class with default parameter values.
    %
    %   G2oLevenbergMarquardt properties:
    %
    %   MaxIterations            - Maximum number of iterations
    %   MaxTime                  - Maximum time
    %   FunctionTolerance        - Lower bound on the change in the cost function
    %   VerboseOutput            - Display intermediate iteration information
    %
    %   See also poseGraphSolverOptions
    
    %   Copyright 2020 The MathWorks, Inc.
    
    %#codegen
    
    properties (Constant, Access = private)
        %SolverName
        SolverName = 'G2oLevenbergMarquardt';
        
        %DetailedSolverName
        DetailedSolverName = 'g2o-levenberg-marquardt';
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
        %   Default: 1e-9
        FunctionTolerance
        
        %VerboseOutput Display intermediate iteration information
        %   Possible values: {'on', 'off'}
        %   Default: 'off'
        VerboseOutput
    end
    
    properties (Access = ?nav.algs.internal.InternalAccess)
        OptionsInternal
    end
    
    methods
        function obj = G2oLevenbergMarquardt()
            %G2oLevenbergMarquardt Constructor
            obj.OptionsInternal = nav.solveroptions.internal.SolverOptions(obj.SolverName);
            obj.OptionsInternal.FunctionTolerance = 1e-9;
        end
        
        function newObj = copy(obj)
            %copy
            newObj = nav.solveroptions.G2oLevenbergMarquardt();
            newObj.OptionsInternal = nav.solveroptions.internal.SolverOptions(obj.SolverName);
            newObj.MaxIterations            = obj.MaxIterations;
            newObj.MaxTime                  = obj.MaxTime;
            newObj.FunctionTolerance        = obj.FunctionTolerance;
            newObj.VerboseOutput            = obj.VerboseOutput;
        end
    end
    
    methods (Access = protected)
                
        function list = getDisplayOptions(obj)
            %getDisplayOptions
            list.MaxIterations            = obj.MaxIterations;
            list.MaxTime                  = obj.MaxTime;
            list.FunctionTolerance        = obj.FunctionTolerance;
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
        
        function set.VerboseOutput(obj, vb)
            %set.VerboseOutput
            validateVerboseOutput(obj.OptionsInternal, vb)
        end
    end
end


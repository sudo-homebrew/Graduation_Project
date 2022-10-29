classdef factorGraphSolverOptions
%FACTORGRAPHSOLVEROPTIONS Create solver options for optimizing factor graph
%
%   OPTS = FACTORGRAPHSOLVEROPTIONS returns a factor graph solver options
%   object, OPTS.
%
%   OPTS = FACTORGRAPHSOLVEROPTIONS(Name=Value,...) returns a
%   factorGraphSolverOptions object, OPTS, with each specified property
%   name set to the specified value. You can specify additional name-value
%   pair arguments in any order as (Name1=Value1,...,NameN=ValueN).
%
%   FACTORGRAPHSOLVEROPTIONS properties:
%       MaxIterations           - Max number of solver iterations
%       FunctionTolerance       - Lower bound of change in cost function
%       GradientTolerance       - Lower bound of norm of gradient
%       StepTolerance           - Lower bound of step size
%       VerbosityLevel          - Flag to change command line verbosity
%       TrustRegionStrategyType - Flag to change trust region step
%                                 computation algorithm
%
%   Example:
%       % Optimize a factor graph with custom options.
%       G = factorGraph;
%       opts = factorGraphSolverOptions(MaxIterations=100);
%       f = factorGPS(1,ReferenceFrame="NED");
%       addFactor(G,f);
%       optimize(G,opts);
%
%   See also factorGraph, importFactorGraph, factorGPS, factorIMU

%   Copyright 2021 The MathWorks, Inc.    

    properties
        %MaxIterations Maximum number of solver iterations allowed
        %   Default: 200
        MaxIterations = 200

        %FunctionTolerance Lower bound on the change in cost function
        %   |newCost - oldCost| < FunctionTolerance * oldCost 
        %   (costs are always > 0)
        %
        %   Default: 1e-6
        FunctionTolerance = 1e-6

        %GradientTolerance Lower bound on the norm of gradient
        %   max_norm{ x - [x Oplus -g(x)] } <= GradientTolerance, where
        %   Oplus is the manifold version of the plus operation, and g(x)
        %   is the gradient at x. 
        %
        %   Default: 1e-10
        GradientTolerance = 1e-10

        %StepTolerance Lower bound on the step size
        %   |deltaX| <= (|x| + StepTolerance) * StepTolerance, where
        %   deltaX is the step computed by the linear solver
        %   Default: 1e-8
        StepTolerance = 1e-8

        %VerbosityLevel Controls commandline message verbosity
        %   0 - No printing
        %   1 - With solver summary
        %   2 - Per-iteration update + solver summary
        %   
        %   Default: 0
        VerbosityLevel = 0

        %TrustRegionStrategyType The trust region step computation algorithm
        %   0 - Levenberg Marquardt
        %   1 - Dogleg
        %
        %   Default: 1
        TrustRegionStrategyType = 1
    end
    
    methods
        function obj = factorGraphSolverOptions(varargin)
            %FACTORGRAPHSOLVEROPTIONS Constructor
            obj = matlabshared.fusionutils.internal.setProperties(obj, nargin, varargin{:});
        end
        
        function obj = set.MaxIterations(obj, maxIter)
            %set.MaxIterations
            validateattributes(maxIter, 'numeric', ...
                {'scalar', 'integer', '>=', 1, 'nonsparse'}, 'factorGraphSolverOptions', 'MaxIteration');
            obj.MaxIterations = double(maxIter);
        end

        function obj = set.FunctionTolerance(obj, funTol)
            %set.MaxIterations
            validateattributes(funTol, 'numeric', ...
                {'scalar', 'real', 'finite','positive', 'nonsparse'}, 'factorGraphSolverOptions', 'FunctionTolerance');
            obj.FunctionTolerance = double(funTol);
        end

        function obj = set.GradientTolerance(obj, gradTol)
            %set.GradientTolerance
            validateattributes(gradTol, 'numeric', ...
                {'scalar', 'real', 'finite','positive', 'nonsparse'}, 'factorGraphSolverOptions', 'GradientTolerance');
            obj.GradientTolerance = double(gradTol);
        end

        function obj = set.StepTolerance(obj, stepTol)
            %set.StepTolerance
            validateattributes(stepTol, 'numeric', ...
                {'scalar', 'real', 'finite','positive', 'nonsparse'}, 'factorGraphSolverOptions', 'StepTolerance');
            obj.StepTolerance = double(stepTol);
        end

        function obj = set.VerbosityLevel(obj, vbLevel)
            %set.TrustRegionStrategyType
            validateattributes(vbLevel, 'numeric', ...
                {'scalar', 'integer', '>=', 0, '<=', 2, 'nonsparse'}, 'factorGraphSolverOptions', 'VerbosityLevel');
            obj.VerbosityLevel = double(vbLevel);
        end

        function obj = set.TrustRegionStrategyType(obj, trType)
            %set.TrustRegionStrategyType
            validateattributes(trType, 'numeric', ...
                {'scalar', 'integer', '>=', 0, '<=', 1, 'nonsparse'}, 'factorGraphSolverOptions', 'TrustRegionStrategyType');
            obj.TrustRegionStrategyType = double(trType);
        end

    end

    methods (Hidden)
        function S = toStruct(obj)
            S = struct("MaxNumIterations", obj.MaxIterations, ...
                        "FunctionTolerance", obj.FunctionTolerance, ...
                        "GradientTolerance", obj.GradientTolerance, ...
                        "StepTolerance", obj.StepTolerance, ...
                        "VerbosityLevel", obj.VerbosityLevel, ...
                        "TrustRegionStrategyType", obj.TrustRegionStrategyType);
        end
    end
end


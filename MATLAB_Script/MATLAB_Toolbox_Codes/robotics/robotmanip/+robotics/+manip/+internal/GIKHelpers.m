% function handles 
classdef GIKHelpers < robotics.manip.internal.InternalAccess
    %This class is for internal use only. It may be removed in the future.
    
    %   Copyright 2016 The MathWorks, Inc.
    
    %#codegen

    %GIKHELPERS Defines IK-specific function handles required by NLP solver
    methods(Static)
        
        function [cost, W, Jac, problem] = computeCost(x, problem)
            %computeCost 
            [~,Jac] = problem.residuals(x);
            cost = problem.objective(x);
            W = problem.WeightMatrix;
        end

        function grad = computeGradient(x, problem)
            %computeGradient
            [~,grad] = problem.objective(x);
        end  

        function [en, evec] = evaluateSolution(x, problem)
            %evaluateSolution
            [en, evec] = problem.evaluateSolution(x);
        end 

        function rc = randomConfig(problem)
            %randomConfig
            rc = problem.randomSeed();
        end

        function xc = clamping(x, problem)
            %clamping
            xc = problem.enforceBounds(x);
        end
        
    end

end
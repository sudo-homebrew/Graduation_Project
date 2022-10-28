classdef NLPProblem < handle
    %This class is for internal use only. It may be removed in the future.
    
    %   Copyright 2016 The MathWorks, Inc.
    
    %#codegen
    
    properties (Abstract)
        
        %ConstraintMatrix
        ConstraintMatrix
        
        %ConstraintBounds
        ConstraintBound
        
    end
    
    methods (Abstract)
        
        [value, grad] = objective(obj, x)
        x = randomSeed(obj, x)
        
    end
    
    methods
        
        function en = evaluateSolution(obj, x)
            %evaluateSolution Default implementation
            en = objective(obj,x);
        end
    end

end
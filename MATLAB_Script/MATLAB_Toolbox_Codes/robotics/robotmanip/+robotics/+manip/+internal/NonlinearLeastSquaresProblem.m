classdef NonlinearLeastSquaresProblem < robotics.manip.internal.NLPProblem
    %This class is for internal use only. It may be removed in the future.
    
    %   Copyright 2016 The MathWorks, Inc.
    
    %#codegen
    
    properties (Abstract)
        
        %WeightMatrix
        WeightMatrix
        
    end
    
    properties (Dependent)
        
        %DesignVariableBounds
        DesignVariableBounds
        
        %ConstraintMatrix
        ConstraintMatrix
        
        %ConstraintBounds
        ConstraintBound
        
    end
    
    properties (Access = private)
        
        %DesignVariableBoundsInternal
        DesignVariableBoundsInternal
        
        %ConstraintMatrixInternal
        ConstraintMatrixInternal
        
        %ConstraintBoundsInternal
        ConstraintBoundInternal
        
    end
    
    methods (Abstract)
        
        [f, J] = residuals(obj, x)
        
    end
    
    methods
        
        function [value, gradient] = objective(obj, x)
            %objective Compute the weighted least-squares cost
            W = obj.WeightMatrix;
            computeGradient = (nargout > 1);
            if computeGradient
                [f,J] = obj.residuals(x);
                gradient = (f'*W*J)';
            else
                f = obj.residuals(x);
            end
            value = 0.5*f'*W*f;
        end
        
        function [en, ev] = evaluateSolution(obj, x) 
            %evaluateSolution Compute scalar and vector error metrics
            ev = obj.residuals(x);
            en = norm(obj.WeightMatrix * ev);
        end
        
        function x = enforceBounds(obj, x)
            x = min(obj.DesignVariableBounds(:,2), ...
                    max(obj.DesignVariableBounds(:,1), x) );
        end
        
        function value = get.DesignVariableBounds(obj)
            value = obj.DesignVariableBoundsInternal;
        end
        
        function set.DesignVariableBounds(obj, value)
            obj.DesignVariableBoundsInternal = value;
            n = size(value, 1);
            A = zeros(2*n, n);
            b = zeros(2*n,1);
            
            % Form constraints (to be double-checked)
            for i = 1:n
                A(2*i-1, i) = -1;
                A(2*i, i) = 1;
                b(2*i-1) = -value(i,1);
                b(2*i) = value(i,2);
            end
            
            obj.ConstraintMatrixInternal = A';
            obj.ConstraintBoundInternal = b;
        end
        
        function value = get.ConstraintMatrix(obj)
            value = obj.ConstraintMatrixInternal;
        end
        
        function value = get.ConstraintBound(obj)
            value = obj.ConstraintBoundInternal;
        end
        
    end

end
classdef (Abstract) KinematicConstraint < robotics.manip.internal.InternalAccess
    %This class is for internal use only. It may be removed in the future.

    %KinematicConstraint Abstract class representing a kinematic constraint
    % This class represents constraints given by the element-wise
    % inequalities
    %
    %   obj.BoundsInternal(:,1) <= obj.evaluate(q) <= obj.BoundsInternal(:,2)
    
    %   Copyright 2016-2020 The MathWorks, Inc.
    
    %#codegen
    
    properties (Abstract, SetAccess = protected)
        
        %KinematicPath Vector of body indices describing which bodies are
        %affected by this constraint
        KinematicPath
        
    end
    
    properties (SetAccess = immutable)
        
        %NumElements Length of the vector returned by obj.evaluate
        NumElements
        
        %Tree Instance of internal.RigidBodyTree
        Tree
        
    end
    
    properties (SetAccess = {?robotics.manip.internal.KinematicConstraint, ...
                             ?robotics.manip.internal.InternalAccess})
        
        %BoundsInternal [NumElements x 2] array containing lower and upper bounds
        %on each element of the vector returned by obj.evaluate
        BoundsInternal
        
        %Weights NumElements vector containing weights on violations of
        %each constraint value
        Weights
        
    end
    
    methods (Abstract)
        
        %evaluate Returns a vector of constraint values computed with the
        %joint positions specified by q. Also returns the Jacobian of that
        %vector.
        [g, J] = evaluate(obj, q);
        
        %copy Returns a deep copy of the object
        newobj = copy(obj)
        
    end
    
    methods
        
        function obj = KinematicConstraint(tree, numElements)
            %KinematicConstraint
            obj.Tree = tree;
            obj.NumElements = numElements;
            obj.BoundsInternal = zeros(obj.NumElements,2);
            obj.Weights = ones(1,obj.NumElements);
        end
        
        function update(obj, other)
            %UPDATE Modify internal properties to match those of OTHER
            %   ASSUMES that OTHER is either an object with the appropriate
            %   properties or a struct with the appropriate fields. 
            %   ASSUMES that all values in OTHER are valid
            obj.Weights = other.Weights;
        end
        
        function cv = violation(obj, q)
            %violation Returns the vector of constraint violations at the
            %specified configuration, q.
            g = obj.evaluate(q);
            cv_min = min(0, g(:) - obj.BoundsInternal(:,1));
            cv_max = max(0, g(:) - obj.BoundsInternal(:,2));
            % Since all(obj.BoundsInternal(:,1) <= obj.BoundsInternal(:,2)), 
            % cv_min(i) ~= 0 implies cv_max(i) == 0 and cv_max(i) ~= 0
            % implies cv_min(i) == 0. Therefore, we can get the overall
            % violation vector by adding the two.
            cv = (cv_min + cv_max)';
        end
        
    end
    
end
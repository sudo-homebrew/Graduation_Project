classdef PositionTarget < robotics.manip.internal.TwoBodyConstraint
    %This class is for internal use only. It may be removed in the future.
    
    %PositionTarget Constraint on the relative position of a body
    %   The PositionTarget object describes a constraint on the position of
    %   one body (the end effector) relative to the body frame of another
    %   body (the reference body). This constraint is satisfied if the
    %   position of the end-effector origin matches the target position to
    %   within a specified tolerance. The target position is defined
    %   relative to the specified reference body.
    
    %   Copyright 2016 The MathWorks, Inc.
    
    %#codegen
    
    properties
        
        %TargetPosition Desired position of the end effector
        %   Expressed in the frame of the reference body
        TargetPosition = zeros(1,3)
        
    end
    
    properties (Dependent)
        
        %PositionTolerance Maximum allowed position error
        %   Upper bound on the distance between the end effector and the
        %   target position
        PositionTolerance
        
    end
    
    methods
        
        function obj = PositionTarget(tree)
            obj = obj@robotics.manip.internal.TwoBodyConstraint(tree, 1);
        end
        
        function newobj = copy(obj)
            %copy Create a deep copy of this object
            newobj = robotics.manip.internal.PositionTarget(obj.Tree);
            update(newobj, obj);
        end
        
        function update(obj, other)
            update@robotics.manip.internal.TwoBodyConstraint(obj, other);
            obj.TargetPosition = other.TargetPosition;
            obj.PositionTolerance = other.PositionTolerance;
        end
        
        function value = get.PositionTolerance(obj)
            value = obj.BoundsInternal(1,2);
        end
        
        function set.PositionTolerance(obj, value)
            obj.BoundsInternal(1,2) = value;
        end
        
    end
    
    methods (Access = {?robotics.manip.internal.TwoBodyConstraint, ...
                       ?robotics.manip.internal.InternalAccess})
        
        function [g, JTwist] = evaluateFromTransform(obj, T)
            %evaluateFromTransform Compute constraint values and twist Jacobian
            %   This method returns the vector of constraint values, G, and
            %   the Jacobian, JTwist, that converts a spatial velocity of
            %   the end effector relative to the reference body into the
            %   time-derivative of G. The input, T, is the transform that
            %   converts points in the end-effector frame to points in the
            %   reference body frame.
            
            posErr = obj.TargetPosition' - T(1:3,4);
            g = sqrt(posErr'*posErr + eps);
            dgderr = [zeros(1,3), posErr'/g];
            % Note the geometric Jacobian of the robot and the Jacobian of
            % the cost error differ by a negative sign.
            JTwist = -dgderr;
        end
        
    end
        
end
classdef CartesianBounds < robotics.manip.internal.TwoBodyConstraint
    %This class is for internal use only. It may be removed in the future.
    
    %CartesianBounds Constraint to keep a body origin inside Cartesian bounds
    %   The CartesianBounds object describes a constraint on the position
    %   of one body (the end effector) relative to a target frame fixed on
    %   another body (the reference body). This constraint is satisfied if
    %   the position , p, of the end-effector origin relative to the target
    %   frame satisfies
    %
    %       Bounds(i,1) <= p(i) <= Bounds(i,2) for i = 1, 2, 3.
    %
    %   The target frame is defined by the TargetTransform, which is the
    %   homogeneous transformation that converts points in the Target frame
    %   to points in the ReferenceBody frame.
    
    %   Copyright 2016 The MathWorks, Inc.
    
    %#codegen
    
    properties
        
        %TargetTransform Transform from the target frame to the ReferenceBody
        TargetTransform = eye(4)
        
    end
    
    properties (Dependent)
        
        %Bounds Bounds on end-effector position expressed in the target frame
        Bounds
        
    end
    
    methods
        
        function obj = CartesianBounds(tree)
            obj = obj@robotics.manip.internal.TwoBodyConstraint(tree, 3);
        end
        
        function newobj = copy(obj)
            %copy Create a deep copy of this object
            newobj = robotics.manip.internal.CartesianBounds(obj.Tree);
            update(newobj, obj);
        end
        
        function update(obj, other)
            update@robotics.manip.internal.TwoBodyConstraint(obj, other);
            obj.TargetTransform = other.TargetTransform;
            obj.Bounds = other.Bounds;
        end
        
        function value = get.Bounds(obj)
            value = obj.BoundsInternal;
        end
        
        function set.Bounds(obj, value)
            obj.BoundsInternal = value;
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
            
            endEffectorToTargetTransform = ...
                robotics.manip.internal.tforminv(obj.TargetTransform)*T;
            g = endEffectorToTargetTransform(1:3,4);
            JTwist = [zeros(3), eye(3)];
        end
        
    end
end
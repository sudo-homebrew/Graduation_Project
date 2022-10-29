classdef OrientationTarget < robotics.manip.internal.TwoBodyConstraint
    %This class is for internal use only. It may be removed in the future.

    %OrientationTarget Constraint on the relative orientation of a body
    %   The OrientationTarget object describes a constraint that requires
    %   the orientation of one body (the end effector) to match a target
    %   orientation to within an angular tolerance in any direction. The
    %   target orientation is specified relative to the body frame of the
    %   reference body.
    
    %   Copyright 2016 The MathWorks, Inc.
    
    %#codegen

    
    properties (Access = private)
        
        %TargetTransform Transform from the target frame to the ReferenceBody
        TargetTransform = eye(4);
        
    end
    
    properties (Dependent)
        
        %TargetOrientation Desired orientation relative to reference body
        TargetOrientation
        
        %OrientationTolerance Maximum allowed rotation angle (in radians)
        OrientationTolerance
    
    end
    
    methods
        
        function obj = OrientationTarget(tree)
            obj = obj@robotics.manip.internal.TwoBodyConstraint(tree,1);
        end
        
        function newobj = copy(obj)
            %copy Create a deep copy of this object
            newobj = robotics.manip.internal.OrientationTarget(obj.Tree);
            update(newobj, obj);
        end
        
        function update(obj, other)
            update@robotics.manip.internal.TwoBodyConstraint(obj, other);
            obj.TargetOrientation = other.TargetOrientation;
            obj.OrientationTolerance = other.OrientationTolerance;
        end
        
        function value = get.TargetOrientation(obj)
            value = tform2quat(obj.TargetTransform);
        end
        
        function set.TargetOrientation(obj, value)
            obj.TargetTransform = quat2tform(value);
        end
        
        function value = get.OrientationTolerance(obj)
            value = obj.BoundsInternal(1,2);
        end
        
        function set.OrientationTolerance(obj, value)
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
            err = robotics.manip.internal.IKHelpers.poseError(obj.TargetTransform, T);
            angErr = err(1:3);
            g = sqrt(angErr'*angErr + eps);
            % Note the geometric Jacobian of the robot and the Jacobian of
            % the cost error differ by a negative sign.
            JTwist = -[angErr'/g, zeros(1,3)];
        end
        
    end
        
end
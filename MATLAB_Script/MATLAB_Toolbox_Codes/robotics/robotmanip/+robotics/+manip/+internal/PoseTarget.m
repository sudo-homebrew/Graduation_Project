classdef PoseTarget < robotics.manip.internal.TwoBodyConstraint
    %This class is for internal use only. It may be removed in the future.
    
    %PoseTarget Constraint on the relative pose of a body
    %   The PoseTarget object describes a constraint on the pose of one
    %   body (the end effector) relative to another body (the reference
    %   body). This constraint is satisfied if the orientation of the end
    %   effector matches the target pose to within an angular tolerance
    %   specified by OrientationTolerance and the position of the end
    %   effector matches the target pose to within a positional tolerance
    %   specified by PositionTolerance.
    
    %   Copyright 2016 The MathWorks, Inc.
    
    %#codegen
    
    properties
        
        %TargetTransform Transform from the target frame to the ReferenceBody
        TargetTransform = eye(4)
        
    end
    
    properties (Dependent)
        
        %OrientationTolerance Maximum allowed rotation angle (in radians)
        OrientationTolerance
        
        %PositionTolerance Maximum allowed position error
        %   Upper bound on the distance between the end effector and the
        %   target position
        PositionTolerance
        
    end
    
    methods
        
        function obj = PoseTarget(tree)
            obj = obj@robotics.manip.internal.TwoBodyConstraint(tree, 2);
        end
        
        function newobj = copy(obj)
            %copy Create a deep copy of this object
            newobj = robotics.manip.internal.PoseTarget(obj.Tree);
            update(newobj, obj);
        end
        
        function update(obj, other)
            update@robotics.manip.internal.TwoBodyConstraint(obj, other);
            obj.TargetTransform = other.TargetTransform;
            obj.OrientationTolerance = other.OrientationTolerance;
            obj.PositionTolerance = other.PositionTolerance;
        end
        
        function value = get.OrientationTolerance(obj)
            value = obj.BoundsInternal(1,2);
        end
        
        function set.OrientationTolerance(obj, value)
            obj.BoundsInternal(1,2) = value;
        end
        
        function value = get.PositionTolerance(obj)
            value = obj.BoundsInternal(2,2);
        end
        
        function set.PositionTolerance(obj, value)
            obj.BoundsInternal(2,2) = value;
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
            posErr = err(4:6);
            g = sqrt([angErr'*angErr; posErr'*posErr] + eps);
            % Note the geometric Jacobian of the robot and the Jacobian of
            % the cost error differ by a negative sign.
            JTwist = -[angErr'/g(1), zeros(1,3); zeros(1,3), posErr'/g(2)];
        end
        
    end
    
end

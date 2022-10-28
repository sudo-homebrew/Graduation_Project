classdef AimingConstraint < robotics.manip.internal.TwoBodyConstraint
    %This class is for internal use only. It may be removed in the future.

    %AimingConstraint Aiming constraint for pointing at a target location
    %   The AimingConstraint object describes a constraint that requires
    %   the z-axis of one body (the end effector) to aim at a target point
    %   on another body (the reference body). This constraint is satisfied
    %   if the aiming angle, a, between the z-axis of the end-effector
    %   frame and the line connecting the end-effector origin to the target
    %   point satisfies
    %
    %       abs(a) <= AngularTolerance
    %
    %   The position of the target point is defined relative to the
    %   reference body.
    
    %   Copyright 2016 The MathWorks, Inc.
    
    %#codegen
    
    properties (SetAccess = protected)
        
        %CosAngularTolerance Maximum allowed rotation angle (in radians)
        CosAngularTolerance = 1;
        
    end
    
    properties (Dependent)
        
        %AngularTolerance Maximum allowed rotation angle (in radians)
        AngularTolerance
        
    end
    
    properties
         
        %TargetPoint Position of the target relative to reference body
        TargetPoint = zeros(1,3);
        
    end
        
    
    methods
        
        function obj = AimingConstraint(tree)
            obj = obj@robotics.manip.internal.TwoBodyConstraint(tree,1);
            obj.BoundsInternal = [0, Inf];
        end
        
        function newobj = copy(obj)
            %copy Create a deep copy of this object
            newobj = robotics.manip.internal.AimingConstraint(obj.Tree);
            update(newobj, obj);
        end
        
        function update(obj, other)
            update@robotics.manip.internal.TwoBodyConstraint(obj, other);
            obj.TargetPoint = other.TargetPoint;
            obj.AngularTolerance = other.AngularTolerance;
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
            
            % r - Vector from the origin of the end-effector frame to the
            % target point, expressed in the reference body frame
            r = obj.TargetPoint(:) - T(1:3,4);
            
            % rdot = JTwistV*[wE, vE] where wE and vE are the angular and
            % linear rates of change of T.
            JTwistR = [zeros(3), -eye(3)];
            
            % z - Unit vector along z-axis of end-effector frame, expressed
            % in the reference body frame
            z = T(1:3,3);
            
            % zdot = JTwistZ*[wE, vE] where wE and vE are the angular and
            % linear rates of change of T.
            JTwistZ = [-robotics.manip.internal.skew(z), zeros(3)];
            
            % The cosine of the angle, a,  between the z-axis of the end
            % effector and the line between the end-effector origin and
            % target is given by:
            %
            %   cos(a) = (v'*z)/norm(v)
            %
            % we want to ensure that 
            %
            %   cos(obj.AngularTolerance) <= cos(a)
            %
            % This is equivalent to
            %
            %   0 <= g
            %
            % where
            %
            %   g = v'*z - norm(v)*cos(obj.AngularTolerance)
            %
            smoothNormV = sqrt(r'*r + eps);
            g = r'*z - smoothNormV*obj.CosAngularTolerance;
            dgdr = z' - (obj.CosAngularTolerance/smoothNormV)*r';
            dgdz = r';
            % gdot = JTwist*[wE, vE] where wE and vE are the angular and
            % linear rates of change of T.
            JTwist = dgdr*JTwistR + dgdz*JTwistZ;
        end
        
    end
    
     
    %Property access methods
    methods
        
        function value = get.AngularTolerance(obj)
            value = acos(obj.CosAngularTolerance);
        end
        
        function set.AngularTolerance(obj, value)
            obj.CosAngularTolerance = cos(value);
        end
        
    end
        
end

classdef (Abstract) TwoBodyConstraint < robotics.manip.internal.KinematicConstraint
    %This class is for internal use only. It may be removed in the future.

    %TwoBodyConstraint Parent class for constraints between bodies
    %This class represents constraints based on a geometric relationship
    %between an end-effector body and a reference body in a rigid body
    %tree.
    
    %   Copyright 2016-2020 The MathWorks, Inc.
    
    %#codegen
    
    properties (Dependent)
        
        %ReferenceBody Name of the body relative to which the target frame
        %is specified
        ReferenceBody
        
        %EndEffector Name of the RigidBody representing the end effector
        EndEffector
        
    end
    
    properties (Dependent, SetAccess = protected)
        
        %KinematicPath
        KinematicPath
        
    end
    
    properties (SetAccess = ?robotics.manip.internal.TwoBodyConstraint)
        
        %ReferenceBodyIndex
        ReferenceBodyIndex
        
        %EndEffectorIndex
        EndEffectorIndex
        
    end
    
    methods (Abstract, Access = {?robotics.manip.internal.TwoBodyConstraint, ...
                                 ?robotics.manip.internal.InternalAccess})
        
        %evaluateFromTransform Compute constraint values and twist Jacobian
        %   This method returns the vector of constraint values, G, and the
        %   Jacobian, JTwist, that converts a spatial velocity for the pose
        %   T into the time-derivative of G. The input, T, is the transform
        %   that converts points in the end-effector frame to points in the
        %   reference body frame.
        [g, JTwist] = evaluateFromTransform(obj, T)
        
    end
    
    methods
        
        function obj = TwoBodyConstraint(varargin)
            obj = obj@robotics.manip.internal.KinematicConstraint(varargin{:});
            obj.EndEffectorIndex = 1;
            obj.ReferenceBodyIndex = 0;
        end
        
        function update(obj, other)
            update@robotics.manip.internal.KinematicConstraint(obj, other);
            obj.EndEffector = other.EndEffector;
            obj.ReferenceBody = other.ReferenceBody;
        end
        
        function value = get.ReferenceBody(obj)
            if obj.ReferenceBodyIndex > 0
                value = obj.Tree.Bodies{obj.ReferenceBodyIndex}.Name;
            else
                value = obj.Tree.BaseName;
            end
        end
        
        function set.ReferenceBody(obj, value)
            if ~strcmp(obj.ReferenceBody, value)
                if isempty(value)
                    obj.ReferenceBodyIndex = 0;
                else
                    obj.ReferenceBodyIndex = validateInputBodyName(obj.Tree, value);
                end
            end
        end
        
        function value = get.EndEffector(obj)
            if obj.EndEffectorIndex > 0
                value = obj.Tree.Bodies{obj.EndEffectorIndex}.Name;
            else
                value = obj.Tree.BaseName;
            end
        end
        
        function set.EndEffector(obj, value)
            if ~strcmp(obj.EndEffector, value)
                obj.EndEffectorIndex = validateInputBodyName(obj.Tree, value);
            end
        end
        
        function value = get.KinematicPath(obj) 
            value = obj.Tree.kinematicPath(obj.EndEffector, ...
                                           obj.ReferenceBody);
        end
        
    end
    
    methods (Sealed)
        
        function [g, J] = evaluate(obj, q)
            [T, Jrobot] = efficientFKAndJacobianForIK(obj.Tree, q, obj.EndEffector, obj.ReferenceBody);
            coder.varsize('J',[6,obj.Tree.MaxNumBodies],[1,1]);
            [g, JTwist] = evaluateFromTransform(obj, T);
            J = JTwist*Jrobot;
        end
        
    end
        
end
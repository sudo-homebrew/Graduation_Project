classdef JointPositionBounds < robotics.manip.internal.KinematicConstraint
    %This class is for internal use only. It may be removed in the future..
    
    %JointPositionBounds Bounds on joint positions of a rigidBodyTree
    %   The JointPositionBounds object describes a constraint on the joint
    %   positions of a rigid body tree. This constraint is satisfied if
    %
    %       Bounds(:,1) <= q <= Bounds(:,2)
    %
    %   where q is a configuration vector for the rigid body tree.
    
    %   Copyright 2016-2019 The MathWorks, Inc.
    
    %#codegen
    
    properties (Dependent, SetAccess = protected)
        
        KinematicPath
        
    end
    
    properties (Dependent)
        
        Bounds
        
    end
    
    methods
        
        function obj = JointPositionBounds(tree)
            obj = obj@robotics.manip.internal.KinematicConstraint(tree,tree.PositionNumber);
            obj.BoundsInternal = obj.Tree.JointPositionLimits;
            obj.Weights = ones(1, obj.NumElements);
        end
        
        function newobj = copy(obj)
            %copy Create a deep copy of this object
            newobj = robotics.manip.internal.JointPositionBounds(obj.Tree);
            update(newobj, obj);
        end
        
        function [g, J] = evaluate(obj, q)
            g = q;
            J = eye(obj.NumElements);
        end
        
        function update(obj, params)
            update@robotics.manip.internal.KinematicConstraint(obj, params);
            obj.Bounds = params.Bounds;
        end
        
        function value = get.KinematicPath(obj)
            relevantPositionIndices = find(any(~isinf(obj.Bounds),2));
            [~,bodyIndices] = ismember(relevantPositionIndices(:),obj.Tree.PositionDoFMap);
            value = fliplr(unique(sort([0;bodyIndices]))');
        end
        
        function value = get.Bounds(obj)
            value = obj.BoundsInternal;
        end
        
        function set.Bounds(obj, value)
            obj.BoundsInternal = value;
        end
        
    end
        
end

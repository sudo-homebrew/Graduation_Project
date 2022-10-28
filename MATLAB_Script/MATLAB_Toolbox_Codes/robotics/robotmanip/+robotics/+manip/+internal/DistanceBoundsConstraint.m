classdef DistanceBoundsConstraint < robotics.manip.internal.TwoBodyConstraint
%This class is for internal use only and may be removed in the future

%DistanceBoundsConstraint Constraint the distance between two bodies
%   The constraint ensures that the distance("d") between two bodies is within the
%   user-specified distance bounds("Bounds")
%                   Bounds(1) <= d <= Bounds(2)

%   Copyright 2021 The MathWorks, Inc.

%#codegen

    properties(Dependent)
        %Bounds Bounds on the distance between the two bodies
        Bounds
    end

    methods
        function set.Bounds(obj, value)
            obj.BoundsInternal = value;
        end

        function value = get.Bounds(obj)
            value = obj.BoundsInternal;
        end
    end

    methods
        function obj = DistanceBoundsConstraint(tree)
        %DistanceBoundsConstraint Constructor
            obj = obj@robotics.manip.internal.TwoBodyConstraint(tree,1);
        end

        function newobj = copy(obj)
        %copy Create a deep copy of this object
            newobj = robotics.manip.internal.DistanceBoundsConstraint(obj.Tree);
            update(newobj, obj);
        end

        function update(obj, other)
        %update Update the constraint
            update@robotics.manip.internal.TwoBodyConstraint(obj, other);
            obj.Bounds = other.Bounds;
        end
    end

    methods (Access = {?robotics.manip.internal.TwoBodyConstraint, ...
                       ?robotics.manip.internal.InternalAccess})
        function [g, JTwist] = evaluateFromTransform(~, T)
        %evaluateFromTransform Evaluate the constraint error and its Jacobian
        %   Given the transform "T", compute the distance of the EndEffector
        %   with respect to the ReferenceBody.
            r = T(1:3,end);

            % Add "eps" to prevent divide-by-zero errors while computing the
            % unit vector of vector joining the EndEffector and ReferenceBody.
            g = sqrt(r'*r+eps);
            JTwist = [zeros(1,3) r'/g];
        end
    end
end

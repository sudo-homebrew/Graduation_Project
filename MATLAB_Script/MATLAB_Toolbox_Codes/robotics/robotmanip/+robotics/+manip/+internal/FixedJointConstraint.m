classdef FixedJointConstraint < ...
        robotics.manip.internal.LoopClosureJointConstraint
%This class is for internal use only and may be removed in the future

%FixedJointConstraint A fixed loop closure joint constraint
%   The constraint ensures that the relative orientation and position between the
%   successor and predecessor intermediate frames is within some tolerance. The
%   constraint is evaluated by:
%       1) Evaluating the distance of the origin of the successor's intermediate
%       frame from the predecessor's intermediate frame.
%       2) Evaluating the orientation of the successor's intermediate relative
%       to the predecessor's intermediate frame.

%   Copyright 2021 The MathWorks, Inc.

%#codegen

    properties(Dependent)
        % PositionTolerance Tolerance on the position of the constraint
        PositionTolerance

        % OrientationTolerance Tolerance on the orientation of the constraint
        OrientationTolerance
    end

    properties(Constant, Access=private)
        %PositionToleranceRowIdx Index of PositionTolerance in BoundsInternal
        PositionToleranceRowIdx = 1

        %OrientationToleranceRowIdx Index of OrientationTolerance in BoundsInternal
        OrientationToleranceRowIdx = 2

        %BoundsMaxColIdx Column index corresponding to maximum bounds in BoundsInternal
        BoundsMaxColIdx = 2;
    end

    methods
        function set.PositionTolerance(obj, value)
        %set.PositionTolerance
            obj.BoundsInternal(obj.PositionToleranceRowIdx, obj.BoundsMaxColIdx) = value;
        end

        function value = get.PositionTolerance(obj)
        %get.PositionTolerance
            value = obj.BoundsInternal(obj.PositionToleranceRowIdx, obj.BoundsMaxColIdx);
        end

        function set.OrientationTolerance(obj, value)
        %set.OrientationTolerance
            obj.BoundsInternal(obj.OrientationToleranceRowIdx, obj.BoundsMaxColIdx) = value;
        end

        function value = get.OrientationTolerance(obj)
        %get.OrientationTolerance
            value = obj.BoundsInternal(obj.OrientationToleranceRowIdx, obj.BoundsMaxColIdx);
        end

        function obj = FixedJointConstraint(tree)
        %FixedJointConstraint
            obj@robotics.manip.internal.LoopClosureJointConstraint(tree, 2);
        end

        function [g, JTwist] = evaluateConstraintFromIntermediateTransform(obj, T)
        %evaluateConstraintFromIntermediateTransform Fixed constraint's costs
        %   The costs are evaluated for the Successor's intermediate frame with
        %   respect to the Predecessor's intermediate frame.
            [positionCost, Jposition] = obj.evaluatePosition(T);
            [orientationCost, Jorientation] = obj.evaluateOrientation(T);
            g = [positionCost;
                 orientationCost];
            JTwist = [Jposition;
                      Jorientation];
        end

        function newObj = copy(obj)
        %copy
            newObj = robotics.manip.internal.FixedJointConstraint(obj.Tree);
            update(newObj, obj);
        end

        function update(obj, other)
        %update Update additional fixed joint specific properties
            obj.update@robotics.manip.internal.LoopClosureJointConstraint(other);
            obj.PositionTolerance = other.PositionTolerance;
            obj.OrientationTolerance = other.OrientationTolerance;
        end
    end

    methods(Access=private)
        function [positionCost, JPosition] = evaluatePosition(~, T)
        %evaluatePosition Evaluate the distance of the intermediate frame's origin
            pos = T(1:3,end);

            % positionCost is 0 if pos is 0. Add "eps" to avoid divide by zero
            % errors when JPosition is computed.
            positionCost = sqrt(pos'*pos + eps);
            JPosition = [zeros(1,3) (pos'/positionCost)];
        end

        function [orientationCost, JOrientation] = evaluateOrientation(~, T)
        %evaluateOrientation Evaluate the orientation of the frame
            axang = tform2axang(T);
            orientationCost = abs(axang(4));
            JOrientation = [axang(1:3),zeros(1,3)];
        end
    end
end

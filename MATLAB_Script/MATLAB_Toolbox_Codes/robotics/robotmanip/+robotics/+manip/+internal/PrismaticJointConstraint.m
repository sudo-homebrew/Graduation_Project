classdef PrismaticJointConstraint < ...
        robotics.manip.internal.LoopClosureJointConstraint
%This class is for internal use only and may be removed in the future

%PrismaticJointConstraint A prismatic loop-closure joint constraint
%   The prismatic joint ensures that the distance of the successor intermediate
%   frame's origin from the Z-axis of the predecessor's intermediate frame is
%   within some tolerance(PositionTolerance), and that the relative orientation
%   between the predecessor and the successor intermediate frame is within some
%   tolerance (OrientationTolerance). Thus, the constraint is satisfied by:
%       1) Evaluating the distance of the successor intermediate frame's origin
%       from the Z-axis of the predecessor intermediate frame
%       2) Evaluating the orientation between the successor intermediate frame
%       and the predecessor intermediate frame.
%       3) Evaluating the joint position as the Z-coordinate of the
%       successor intermediate frame's origin with respect to the predecessor's
%       intermediate frame.

%   Copyright 2021 The MathWorks, Inc.

%#codegen

    properties(Dependent)
        % PositionTolerance Tolerance on the position of the constraint
        PositionTolerance

        %JointPositionLimits Position limits of the prismatic joint.
        %   The joint position is defined as the position between the two loop
        %   frames given the axis-angle orientation between them.
        JointPositionLimits

        % OrientationTolerance Tolerance on the orientation of the constraint
        OrientationTolerance
    end

    properties(Constant, Access=private)
        %PositionToleranceRowIdx Index of PositionTolerance in BoundsInternal
        PositionToleranceRowIdx = 1

        %OrientationToleranceRowIdx Index of OrientationTolerance in BoundsInternal
        OrientationToleranceRowIdx = 2

        %JointPositionLimitsRowIdx Index of JointPositionLimits in BoundsInternal
        JointPositionLimitsRowIdx = 3

        %BoundsMaxColIdx Column index corresponding to maximum bounds in BoundsInternal
        BoundsMaxColIdx = 2;
    end

    methods
        function value = get.JointPositionLimits(obj)
        %get.JointPositionLimits
            value = obj.BoundsInternal(obj.JointPositionLimitsRowIdx, :);
        end

        function set.JointPositionLimits(obj, value)
        %set.JointPositionLimits
            obj.BoundsInternal(obj.JointPositionLimitsRowIdx, :) = value;
        end

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

        function obj = PrismaticJointConstraint(tree)
        %PrismaticJointConstraint
            obj@robotics.manip.internal.LoopClosureJointConstraint(tree, 3);
        end

        function [g, JTwist] = evaluateConstraintFromIntermediateTransform(obj, T)
        %evaluateConstraintFromIntermediateTransform Prismatic constraint's costs
        %   The costs are evaluated for the Successor's intermediate frame with
        %   respect to the Predecessor's intermediate frame.
        %   The evaluation comprises of three costs:
        %       1) Position of the successor intermediate frame from the Z-axis
        %       2) Orientation of the successor intermediate frame
        %       3) Joint position defined in terms of the Z-coordinate of the
        %       successor intermediate frame's origin
            [positionCost, Jposition] = obj.evaluatePosition(T);
            [orientationCost, Jorientation] = obj.evaluateOrientation(T);
            [positionLimits, JpositionLimits] = obj.evaluateJointPosition(T);
            g = [positionCost;
                 orientationCost;
                 positionLimits];
            JTwist = [Jposition;
                      Jorientation;
                      JpositionLimits];
        end

        function newObj = copy(obj)
        %copy
            newObj = robotics.manip.internal.PrismaticJointConstraint(obj.Tree);
            update(newObj, obj);
        end

        function update(obj, other)
        %update Update additional prismatic joint specific properties
            obj.update@robotics.manip.internal.LoopClosureJointConstraint(other);
            obj.PositionTolerance = other.PositionTolerance;
            obj.OrientationTolerance = other.OrientationTolerance;
            obj.JointPositionLimits = other.JointPositionLimits;
        end
    end

    methods(Access=private)
        function [positionCost, JPosition] = evaluatePosition(~, T)
        %evaluatePosition Evaluate the frame's origin's distance from the Z-axis
        %   The routine below finds the distance of a point at position vector
        %   "r" from the Z-axis(a unit vector "n")
        %
        %    (r.n)n
        %   \----->-------------------> n
        %    \    |
        %     \   |
        %      \  |
        %     r \ | d = r-(r.n)n
        %        \|
        %         v
        %
        %   This is done by computing the projection of vector "r" on vector
        %   "n". Thus, the displacement vector "d" is given as:
        %       _   _    _ _  _
        %       d = r - (r.n) n
        %
        %   Thus,
        %            _
        %       g = |d|
        %                        _  _    _
        %       JTwist:= dg/dT = d/|d| = d/g

            n = [0,0,1];
            r = T(1:3,4)';

            % If "r" coincides with the origin [0,0,0], then g is 0. Add
            % sqrt(eps) to avoid divide-by-zero errors.
            g = norm(r-dot(r,n)*n) + sqrt(eps);
            JTwist = [zeros(1,3) (r-dot(r,n)*n)/g];
            positionCost = g;
            JPosition = JTwist;
        end

        function [orientationCost, JOrientation] = evaluateOrientation(~, T)
        %evaluateOrientation Evaluate the orientation of the frame
            axang = tform2axang(T);
            orientationCost = abs(axang(4));
            JOrientation = [axang(1:3),zeros(1,3)];
        end

        function [jointPositionCost, JJointPosition] = evaluateJointPosition(~, T)
        %evaluateJointPosition Evaluate the joint position
        %   This corresponds to the Z-coordinate of the successor's intermediate
        %   frame.
            jointPositionCost = T(3,end);

            % JJointPosition is given as [wx,wy,wz,vx,vy,vz]. Since this is change in
            % the Z-coordinate, all except vz are 0
            JJointPosition = [zeros(1,5),1];
        end
    end
end

classdef RevoluteJointConstraint < robotics.manip.internal.LoopClosureJointConstraint
%This class is for internal use only. It may be removed in the future.

%RevoluteJointConstraint A revolute loop closure joint constraint
%   The constraint is a combination of three constraints:
%       1) The origins of the loop frames must coincide and be within some
%       position tolerance
%       2) The angle between the loop frame's Z axes must be within some
%       orientation tolerance
%       3) The relative orientation between the loop frames must be within the
%       joint's position limits


%   Copyright 2021 The MathWorks, Inc.

%#codegen

    properties(Dependent)
        % PositionTolerance Tolerance on the position of the constraint
        PositionTolerance

        %JointPositionLimits Position limits of the revolute joint.
        %   The joint position is defined as the angle between the two loop
        %   frames given the axis-angle orientation between them.
        JointPositionLimits
    end

    properties
        % OrientationTolerance Tolerance on the orientation of the constraint
        OrientationTolerance = 0
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
        function obj = RevoluteJointConstraint(tree)
        %RevoluteJointConstraint Constructor
            obj@robotics.manip.internal.LoopClosureJointConstraint(tree, 3);
            obj.BoundsInternal(obj.OrientationToleranceRowIdx, :) = [0, inf];
        end
    end

    methods
        function set.PositionTolerance(obj, value)
            obj.BoundsInternal(obj.PositionToleranceRowIdx, obj.BoundsMaxColIdx) = value;
        end

        function value = get.PositionTolerance(obj)
            value = obj.BoundsInternal(obj.PositionToleranceRowIdx, obj.BoundsMaxColIdx);
        end

        function value = get.JointPositionLimits(obj)
            value = obj.BoundsInternal(obj.JointPositionLimitsRowIdx, :);
        end

        function set.JointPositionLimits(obj, value)
        %set.JointPositionLimits Set the joint position limits based on the posLimits 
        %   This function always assumes that posLimits(1) <= posLimits(2) i.e., the
        %   row vector, posLimits, is non-decreasing.
            obj.BoundsInternal(obj.JointPositionLimitsRowIdx, :) = value;
        end
    end

    methods(Access=private)
        function [positionCost, JPosition] = evaluatePosition(~, T)
        %evaluatePosition Evaluate the distance of the intermediate frame's origin
            pos = T(1:3,end);

            % positionCost is 0 if pos is 0. Add "eps" to avoid divide by zero errors
            % when JPosition is computed.
            positionCost = sqrt(pos'*pos + eps);
            JPosition = [zeros(1,3) (pos'/positionCost)];
        end

        function [orientationCost, JOrientation] = evaluateZAlignment(obj, T)
        %evaluateZAlignment Evaluate the difference in alignment of the intermediate frame's Z-axis
            v = [0,0,1]';

            % Z-axis of the intermediate frame. T is a homogeneous transform,
            % hence T(1:3,1:3) is a valid rotation matrix. Thus, T(1:3,end) is a
            % unit vector.
            z = T(1:3,3);

            orientationCost = v'*z - cos(obj.OrientationTolerance);
            omega = robotics.manip.internal.skew(z)*v;
            JOrientation = [omega', zeros(1,3)];
        end

        function [jointPositionCost, JJointPosition] = evaluateJointPosition(obj, T)
        %evaluateJointPosition Evaluate the angle of the intermediate frame
        %   If the intermediate frame's Z-axis aligns and the origin is
        %   coincident, the joint position is the angle formed along the common
        %   Z-axis. In order for this cost to be meaningful, this assumes that
        %   the Z-axes are aligned.
            axang = tform2axang(T);
            jointPositionCost_ = axang(4);
            rotax_ = axang(1:3);
            rotsign = sign(rotax_(3));
            jointPositionCost = rotsign * jointPositionCost_;
            rotax = rotsign * rotax_;
            JJointPosition = [rotax,0,0,0];
        end
    end

    methods
        function [g, JTwist] = evaluateConstraintFromIntermediateTransform(obj, T)
        %evaluateConstraintFromIntermediateTransform Revolute constraint's costs
        %   The costs are evaluated for the Successor's intermediate frame with
        %   respect to the Predecessor's intermediate frame.
            [positionCost, JPosition] = evaluatePosition(obj, T);
            [orientationCost, JOrientation] = evaluateZAlignment(obj, T);
            [jointPositionCost, JJointPosition] = evaluateJointPosition(obj, T);
            g = [positionCost;
                 orientationCost;
                 jointPositionCost];
            JTwist = [JPosition;
                      JOrientation;
                      JJointPosition];

        end

        function update(obj, other)
        %update Update additional revolute joint specific properties
            obj.update@robotics.manip.internal.LoopClosureJointConstraint(other);
            obj.PositionTolerance = other.PositionTolerance;
            obj.OrientationTolerance = other.OrientationTolerance;
            obj.JointPositionLimits = other.JointPositionLimits;
        end
    end

    methods
        function newObj = copy(obj)
        %copy Create a deep copy of the constraint
            newObj = robotics.manip.internal.RevoluteJointConstraint(obj.Tree);
            update(newObj, obj);
        end
    end
end

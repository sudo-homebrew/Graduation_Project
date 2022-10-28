classdef ConstraintType < uint32
%This class is for internal use only and may be removed in a future release

%ConstraintType Enumeration class for the IK solver constraint type

%   Copyright 2021 The MathWorks, Inc.

    enumeration
        %Default Used when no constraint is specified
        Default       (0)

        %Aiming Aiming constraint
        Aiming        (1)

        %Pose Pose constraint
        Pose          (2)

        %Cartesian Cartesian bounds constraint
        Cartesian     (3)

        %JointBounds Joint bounds constraint
        JointBounds   (4)
    end

    methods (Static)
        function str = getUserFacingName(constraintEnum)
        %getUserFacingName Helper function to get a user-facing constraint type from the enum
        %   This method is used to generate text in the user-facing
        %   language that describes the constraint type.

            switch constraintEnum
              case robotics.ikdesigner.internal.toolstrip.ConstraintType.Aiming
                str = string(message('robotics:ikdesigner:constraintsbrowser:AimingConstraintTypeString'));
              case robotics.ikdesigner.internal.toolstrip.ConstraintType.Pose
                str = string(message('robotics:ikdesigner:constraintsbrowser:PoseConstraintTypeString'));
              case robotics.ikdesigner.internal.toolstrip.ConstraintType.Cartesian
                str = string(message('robotics:ikdesigner:constraintsbrowser:CartesianConstraintTypeString'));
              case robotics.ikdesigner.internal.toolstrip.ConstraintType.JointBounds
                str = string(message('robotics:ikdesigner:constraintsbrowser:JointBoundsConstraintTypeString'));
            end
        end
    end
end

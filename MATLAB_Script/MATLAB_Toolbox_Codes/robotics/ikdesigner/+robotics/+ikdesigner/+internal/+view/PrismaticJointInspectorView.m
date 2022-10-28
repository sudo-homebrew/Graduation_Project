classdef PrismaticJointInspectorView < robotics.ikdesigner.internal.view.JointInspectorView
%This class is for internal use only. It may be removed in the future.

%   PrismaticJointInspectorView Inspector for prismatic joints

%   Copyright 2021 The MathWorks, Inc.

    properties (Constant)
        %UNITSUFFIX Text to indicate units for the joint
        UNITSUFFIX = string(message('robotics:ikdesigner:sceneinspector:PrisJtUnitSuffix'))

        %INFVALUEBOUND Value to use as a slider bound when the actual value is infinite
        %   Use a large value for prismatic joints but keep it small enough
        %   that the user can reasonably still use it to set a range of
        %   values near zero.
        INFVALUEBOUND = 1000
    end

    properties (Dependent)
        JointValue
    end

    methods
        function jointVal = get.JointValue(obj)
        %JointValue Returns an internal joint value
        %   For prismatic joints, the stored value is contained in the
        %   edit field. Since internal and user-facing values are both
        %   computed in meters, no conversion is required.

            jointVal = obj.JointPositionEditField.Value;
        end

        function populateConfigState(obj, config)
        %populateConfigState Populate the joint state UI components given robot configuration
        %   Given a joint configuration, populate values related to
        %   current joint position, including edit fields and slider.

        % In the case of a prismatic joint, everything is enabled
            obj.JointPositionSlider.Enable = 'on';
            obj.JointPositionEditField.Enable = 'on';
            obj.populateJointPositionFields(config);
        end
    end

    methods (Static)
        function positionLimits = computeUserFacingPositionLimits(rawPosLimits)
        %computeUserFacingPositionLimits Return user-facing position limits given the internal value
        %   For prismatic joints, the user-facing and raw joint limits
        %   are identical.

            positionLimits = rawPosLimits;
        end

        function jointVal = computeUserFacingJointValue(rawJointValue)
        %computeUserFacingJointValue Return user-facing joint position given the internal value
        %   For prismatic joints, the user-facing and raw joint values
        %   are identical.

            jointVal = rawJointValue;
        end
    end
end

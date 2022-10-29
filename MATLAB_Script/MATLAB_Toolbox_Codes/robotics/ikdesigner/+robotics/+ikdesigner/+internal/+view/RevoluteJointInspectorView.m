classdef RevoluteJointInspectorView < robotics.ikdesigner.internal.view.JointInspectorView
%This class is for internal use only. It may be removed in the future.

%   RevoluteJointInspectorView Inspector for revolute joints

%   Copyright 2021 The MathWorks, Inc.

    properties (Constant)
        %UNITSUFFIX Text to indicate units for the joint
        UNITSUFFIX = string(message('robotics:ikdesigner:sceneinspector:RevJtUnitSuffix'))

        %INFVALUEBOUND Value to use as a slider bound when the actual value is infinite
        %   Use a value that will guarantee at least 2*pi range of motion
        %   in each direction. This value is added to the mid-range.
        INFVALUEBOUND = 2*180
    end

    properties (Dependent)
        JointValue
    end

    methods
        function jointVal = get.JointValue(obj)
        %JointValue Returns an internal joint value
        %   For revolute joints, the internal joint value is stored in
        %   radians, but user-facing values are set in degrees, so a
        %   conversion required when reading from an edit field.

            jointVal = deg2rad(obj.JointPositionEditField.Value);
        end

        function populateConfigState(obj, config)
        %populateConfigState Populate the joint state UI components given robot configuration
        %   Given a joint configuration, populate values related to
        %   current joint position, including edit fields and slider.

        % In the case of a revolute joint, everything is enabled
            obj.JointPositionSlider.Enable = 'on';
            obj.JointPositionEditField.Enable = 'on';
            obj.populateJointPositionFields(config);
        end
    end

    methods (Static)
        function positionLimits = computeUserFacingPositionLimits(rawPosLimits)
        %computeUserFacingPositionLimits Return user-facing position limits given the internal value
        %   For revolute joints, the raw position limits are stored in
        %   radians, but the user-facing values are shown in degrees.

            positionLimits = rad2deg(rawPosLimits);
        end

        function jointVal = computeUserFacingJointValue(rawJointValue)
        %computeUserFacingJointValue Return user-facing joint position given the internal value
        %   For revolute joints, the raw joint values are stored in
        %   radians, but the user-facing values are shown in degrees.

            jointVal = rad2deg(rawJointValue);
        end
    end
end

classdef FixedJointInspectorView < robotics.ikdesigner.internal.view.JointInspectorView
%This class is for internal use only. It may be removed in the future.

%   FixedJointInspectorView Inspector for fixed joints

%   Copyright 2021 The MathWorks, Inc.

    properties (Constant)
        %UNITSUFFIX Text to indicate units for the joint
        UNITSUFFIX = ""

        %INFVALUEBOUND Value to use as a slider bound when the actual value is infinite
        %   For fixed joints, this is meaningless
        INFVALUEBOUND = 1000
    end

    properties (Dependent)
        JointValue
    end

    methods
        function jointVal = get.JointValue(obj)
        %JointValue Returns an internal joint value
        %   For fixed joints, this value is exactly the displayed
        %   value, which is anyway zero.

            jointVal = obj.JointPositionEditField.Value;
        end

        function populateConfigState(obj, ~)
        %populateConfigState Populate the joint state UI components given robot configuration
        %   Given a joint configuration, populate values related to
        %   current joint position, including edit fields and slider.

        % In the case of a fixed joint, the position cannot be
        % varied
            obj.JointPositionSlider.Enable = 'off';
            obj.JointPositionEditField.Enable = 'off';
            obj.JointPositionSlider.MajorTickLabels = {'0','0'};
            obj.JointPositionEditField.Value = 0;
        end
    end

    methods (Static)
        function positionLimits = computeUserFacingPositionLimits(~)
        %computeUserFacingPositionLimits Return user-facing position limits given the internal value
        %   For fixed joints, the position limits are meaningless and
        %   are returned as zeros.

            positionLimits = [0 0];
        end

        function jointVal = computeUserFacingJointValue(rawJointValue)
        %computeUserFacingJointValue Return user-facing joint position given the internal value
        %   For fixed joints, this value is exactly the displayed
        %   value, which is anyway zero.

            jointVal = rawJointValue;
        end
    end
end

classdef PoseInspectionView < handle
%This class is for internal use only. It may be removed in the future.

%   PoseInspectionView Object to add pose inspection to a panel view

%   Copyright 2021 The MathWorks, Inc.

    properties (SetAccess = private, GetAccess = ?matlab.unittest.TestCase)

        %SendNotificationCB Callback to send a notification via a controller-facing view
        SendNotificationCB
    end

    % Inspector gadgets
    properties (Access = {?matlab.unittest.TestCase, ?robotics_tests.app.ikdesigner.InspectorPoseViewTester})

        %PositionXField X Position value
        PositionXField

        %PositionYField Y Position value
        PositionYField

        %PositionZField Z Position value
        PositionZField

        %OrientationXField Pose euler(x) value
        OrientationXField

        %OrientationYField Pose euler(y) value
        OrientationYField

        %OrientationZField Pose euler(z) value
        OrientationZField
    end

    properties (Constant)
        %Pose X position label and tooltip
        XLABEL = string(message('robotics:ikdesigner:sceneinspector:PoseXLabel'))
        XTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:PoseXTooltip'))

        %Pose Y position label and tooltip
        YLABEL = string(message('robotics:ikdesigner:sceneinspector:PoseYLabel'))
        YTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:PoseYTooltip'))

        %Pose Z position label and tooltip
        ZLABEL = string(message('robotics:ikdesigner:sceneinspector:PoseZLabel'))
        ZTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:PoseZTooltip'))

        %Pose Euler X Rotation label and tooltip
        EULERXLABEL = string(message('robotics:ikdesigner:sceneinspector:PoseEulerXLabel'))
        EULERXTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:PoseEulerXTooltip'))

        %Pose Euler Y Rotation label and tooltip
        EULERYLABEL = string(message('robotics:ikdesigner:sceneinspector:PoseEulerYLabel'))
        EULERYTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:PoseEulerYTooltip'))

        %Pose Euler Z Rotation label and tooltip
        EULERZLABEL = string(message('robotics:ikdesigner:sceneinspector:PoseEulerZLabel'))
        EULERZTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:PoseEulerZTooltip'))
    end

    methods
        function obj = PoseInspectionView(parentGrid, notificationCallback)
        %RigidBodyInspectorView Constructor

        % Add child panels to the view
            obj.addPositionStateFields(parentGrid);
            obj.addOrientationStateFields(parentGrid);
            obj.SendNotificationCB = notificationCallback;
        end

        function setup(obj)
        %setup Set up for new session

        % Reset to a default value
            obj.populatePoseView(eye(4));
        end
    end

    methods
        function populatePoseView(obj, bodyPose)
        %populateBodyPoseState Populate the inspected body's pose
        %   Given a transform tree, this method populates just the
        %   pieces of the state related to body pose. The method makes
        %   use of the stored body index to extract this information
        %   form the transform tree cell array.

            obj.PositionXField.Value = bodyPose(1,4);
            obj.PositionYField.Value = bodyPose(2,4);
            obj.PositionZField.Value = bodyPose(3,4);

            orientations = tform2eul(bodyPose, 'XYZ');
            obj.OrientationXField.Value = rad2deg(orientations(1));
            obj.OrientationYField.Value = rad2deg(orientations(2));
            obj.OrientationZField.Value = rad2deg(orientations(3));

        end
    end

    methods (Access = private)
        function addPositionStateFields(obj, statesGrid)
        %addPositionStateFields Add the position state components

        % Use a local grid to lay out the fields
            posStateGrid = uigridlayout(statesGrid);
            posStateGrid.Layout.Row = 1;
            posStateGrid.Layout.Column = 1;
            posStateGrid.ColumnWidth = {'1x','1x','1x'};
            posStateGrid.RowHeight = {'fit','fit'};

            % Add the X Position
            xPositionLabel = uilabel(posStateGrid);
            xPositionLabel.Text = obj.XLABEL;
            xPositionLabel.Tooltip = obj.XTOOLTIP;
            xPositionLabel.Layout.Row = 1;
            xPositionLabel.Layout.Column = 1;
            obj.PositionXField = uieditfield(posStateGrid, 'numeric','Editable', 'off','Enable','on');
            obj.PositionXField.Layout.Row = 2;
            obj.PositionXField.Layout.Column = 1;

            % Add the Y Position
            yPositionLabel = uilabel(posStateGrid);
            yPositionLabel.Text = obj.YLABEL;
            yPositionLabel.Tooltip = obj.YTOOLTIP;
            yPositionLabel.Layout.Row = 1;
            yPositionLabel.Layout.Column = 2;
            obj.PositionYField = uieditfield(posStateGrid, 'numeric','Editable', 'off','Enable','on');
            obj.PositionYField.Layout.Row = 2;
            obj.PositionYField.Layout.Column = 2;

            % Add the Z Position
            zPositionLabel = uilabel(posStateGrid);
            zPositionLabel.Text = obj.ZLABEL;
            zPositionLabel.Tooltip = obj.ZTOOLTIP;
            zPositionLabel.Layout.Row = 1;
            zPositionLabel.Layout.Column = 3;
            obj.PositionZField = uieditfield(posStateGrid, 'numeric','Editable', 'off','Enable','on');
            obj.PositionZField.Layout.Row = 2;
            obj.PositionZField.Layout.Column = 3;
        end

        function addOrientationStateFields(obj, statesGrid)
        %addOrientationStateFields Add the orientation state components

        % Use a local grid to lay out the fields
            rotStateGrid = uigridlayout(statesGrid);
            rotStateGrid.Layout.Row = 2;
            rotStateGrid.Layout.Column = 1;
            rotStateGrid.ColumnWidth = {'1x','1x'};
            rotStateGrid.RowHeight = {'fit','fit','fit'};

            % Add the X Euler rotation
            xOrientationLabel = uilabel(rotStateGrid);
            xOrientationLabel.Text = obj.EULERXLABEL;
            xOrientationLabel.Tooltip = obj.EULERXTOOLTIP;
            xOrientationLabel.Layout.Row = 1;
            xOrientationLabel.Layout.Column = 1;
            obj.OrientationXField = uieditfield(rotStateGrid, 'numeric','Editable', 'off');
            obj.OrientationXField.Layout.Row = 1;
            obj.OrientationXField.Layout.Column = 2;

            % Add the Y Euler rotation
            yOrientationLabel = uilabel(rotStateGrid);
            yOrientationLabel.Text = obj.EULERYLABEL;
            yOrientationLabel.Tooltip = obj.EULERYTOOLTIP;
            yOrientationLabel.Layout.Row = 2;
            yOrientationLabel.Layout.Column = 1;
            obj.OrientationYField = uieditfield(rotStateGrid, 'numeric','Editable', 'off');
            obj.OrientationYField.Layout.Row = 2;
            obj.OrientationYField.Layout.Column = 2;

            % Add the Z Euler rotation
            zOrientationLabel = uilabel(rotStateGrid);
            zOrientationLabel.Text = obj.EULERZLABEL;
            zOrientationLabel.Tooltip = obj.EULERZTOOLTIP;
            zOrientationLabel.Layout.Row = 3;
            zOrientationLabel.Layout.Column = 1;
            obj.OrientationZField = uieditfield(rotStateGrid, 'numeric','Editable', 'off');
            obj.OrientationZField.Layout.Row = 3;
            obj.OrientationZField.Layout.Column = 2;
        end
    end
end

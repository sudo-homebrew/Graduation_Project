classdef JointInspectorView < robotics.ikdesigner.internal.view.InspectorViewTemplate ...
        & robotics.manip.internal.InternalAccess
%This class is for internal use only. It may be removed in the future.

%   JointInspectorView Inspector for joint objects

%   Copyright 2021 The MathWorks, Inc.

    properties (SetAccess = private, GetAccess = ?matlab.unittest.TestCase)

        %JointIndex Index of the joint currently under inspection
        JointIndex

        %JointType Character array indicating the type of joint (revolute, prismatic, fixed)
        JointType
    end

    % Abstract properties are dependent on the joint type
    properties (Constant, Abstract)
        %UNITSUFFIX Text to indicate units for the joint
        UNITSUFFIX

        %INFVALUEBOUND Value to use as a slider bound when the actual value is infinite
        INFVALUEBOUND
    end

    properties (Constant)
        %Joint name label and tooltip in the inspector view
        JOINTNAMELABEL = string(message('robotics:ikdesigner:sceneinspector:JointNameLabel'))
        JOINTNAMETOOLTIP = string(message('robotics:ikdesigner:sceneinspector:JointNameTooltip'))

        %Joint type label and tooltip in the inspector view
        JOINTTYPELABEL = string(message('robotics:ikdesigner:sceneinspector:JointTypeLabel'))
        JOINTTYPETOOLTIP = string(message('robotics:ikdesigner:sceneinspector:JointTypeTooltip'))

        %Joint axis label and tooltip in the inspector view
        JOINTAXISLABEL = string(message('robotics:ikdesigner:sceneinspector:JointAxisLabel'))
        JOINTAXISTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:JointAxisTooltip'))

        %Home position label and tooltip in the inspector view
        HOMEPOSLABEL = string(message('robotics:ikdesigner:sceneinspector:HomePositionLabel'))
        HOMEPOSTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:HomePositionTooltip'))

        %Parent body label and tooltip in the inspector view
        PARENTLABEL = string(message('robotics:ikdesigner:sceneinspector:ParentLabel'))
        PARENTTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:JointParentTooltip'))

        %Child body label and tooltip in the inspector view
        CHILDLABEL = string(message('robotics:ikdesigner:sceneinspector:ChildLabel'))
        CHILDTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:ChildTooltip'))

        %Joint-To-Parent transform label and tooltip in the inspector view
        JOINTTOPARENTLABEL = string(message('robotics:ikdesigner:sceneinspector:JointToParentLabel'))
        JOINTTOPARENTTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:JointToParentTooltip'))

        %Child-To-Joint transform label and tooltip in the inspector view
        CHILDTOJOINTLABEL = string(message('robotics:ikdesigner:sceneinspector:ChildToJointLabel'))
        CHILDTOJOINTTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:ChildToJointTooltip'))

        %Lower joint limit label and tooltip in the inspector view
        LOWERJTLIMLABEL = string(message('robotics:ikdesigner:sceneinspector:JtLowerLimitLabel'))
        LOWERJTLIMTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:JtLowerLimitTooltip'))

        %Body name label and tooltip in the inspector view
        UPPERJTLIMLABEL = string(message('robotics:ikdesigner:sceneinspector:JtUpperLimitLabel'))
        UPPERJTLIMTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:JtUpperLimitTooltip'))

        %Upper joint limit label and tooltip in the inspector view
        JOINTPOSITIONLABEL = string(message('robotics:ikdesigner:sceneinspector:JointPositionLabel'))
        JOINTPOSITIONTOOLTIP = string(message('robotics:ikdesigner:sceneinspector:JointPositionTooltip'))

        %PROPSLABEL Properties panel header in the inspector view
        PROPSLABEL = string(message('robotics:ikdesigner:sceneinspector:PropertiesLabel'))

        %STATESLABEL States panel header in the inspector view
        STATESLABEL = string(message('robotics:ikdesigner:sceneinspector:StatesLabel'))
    end

    % Abstract properties are dependent on the joint type
    properties (Dependent, Abstract)
        JointValue
    end

    % Abstract methods are dependent on the joint type
    methods (Abstract)
        populateConfigState(obj, config)
        %populateConfigState Populate the joint state UI components given robot configuration
        %   Given a joint configuration, populate values related to current
        %   joint position, including edit fields and slider.
    end

    % Abstract methods are dependent on the joint type
    methods (Static, Abstract)
        computeUserFacingPositionLimits(obj, rawPosLimits)
        %computeUserFacingPositionLimits Return user-facing position limits given the internal value

        computeUserFacingJointValue(obj, rawJointValue)
        %computeUserFacingJointValue Return user-facing joint position given the internal value
    end

    % Inspector gadgets
    properties (SetAccess = protected)

        %JointNameText Joint name
        JointNameText

        %JointTypeText Joint type
        JointTypeText

        %JointAxisText Joint axis
        JointAxisText

        %HomePositionText Home position
        HomePositionText

        %ParentBodyText Parent body name
        ParentBodyText

        %ChildBodyText Child body name
        ChildBodyText

        %JointToParentTransformTable Joint-to-parent transform matrix
        JointToParentTransformTable

        %ChildToJointTransformTable Child-to-joint transform matrix
        ChildToJointTransformTable

        %MinJointPosField Minimum joint limit value
        MinJointPosField

        %MaxJointPosField Maximum joint limit value
        MaxJointPosField

        %MinJointPosLabel Minimum joint position limit label
        %   The label has differing values depending whether the joint is
        %   revolute or fixed
        MinJointPosLabel

        %MaxJointPosLabel Maximum joint position limit label
        %   The label has differing values depending whether the joint is
        %   revolute or fixed
        MaxJointPosLabel

        %JointPosLabel Joint position label
        %   The label has differing values depending whether the joint is
        %   revolute or fixed
        JointPosLabel

        %JointPositionSlider Joint position slider
        JointPositionSlider

        %JointPositionEditField Joint position edit field
        JointPositionEditField
    end

    % Template methods
    methods
        function createInspectorPanels(obj)
        %createInspectorPanels Create data to populate the inspector view

        % Add panels to display joint properties and state
            obj.addPropertiesPanel();
            obj.addStatesPanel();

        end

        function setup(obj)
        %setup Set up data members for next use

        % Start with nothing inspected
            obj.InspectedObjectKey = string.empty;
            obj.JointIndex = [];
        end

        function populateSelectionProperties(obj, objKey, sceneObjMap, sceneModel)
        %populateSelectionProperties Populate view with properties and states for a selected object
        %   When the joint is first selected, this method populates the
        %   inspector views with properties and states.

        % Get additional state information from the scene model
            config = sceneModel.Config;

            % Get the joint handle and associated body names from the map
            mapEntry = sceneObjMap(objKey);
            jointHandle = mapEntry.Handle;
            associatedRigidBody = sceneObjMap(mapEntry.RigidBodyKey).Handle;
            parentBodyName = associatedRigidBody.Parent.Name;
            childBodyName = associatedRigidBody.Name;

            % Store key properties so that state views can be separately updated later
            obj.InspectedObjectKey = objKey;
            obj.JointType = jointHandle.Type;
            obj.JointIndex = sceneModel.RigidBodyTree.TreeInternal.PositionDoFMap(associatedRigidBody.BodyInternal.Index,1);

            % Assign a joint
            % Populate the different elements of the view
            obj.populateJointProperties(jointHandle, parentBodyName, childBodyName);
            obj.populateConfigState(config);
        end

        function updateViewGivenNewRBTConfig(obj, evt)
        %updateViewGivenNewRBTConfig Update the view given a new joint configuration

        % Populate the view from the joint configuration
            jointConfiguration = evt.Config;
            obj.populateConfigState(jointConfiguration);
        end

        function populateCollisionState(~, ~)
        %populateCollisionState Populate the displayed collision state fields for the currently selected object
        %   This is a no-op for this view since joints cannot be in
        %   collision (only the associated bodies are shown to be in
        %   collision).
        end

        function resetViewCollisionState(~)
        %resetViewCollisionState Reset the collision state display
        %   This is a no-op for this view since joints cannot be in
        %   collision (only the associated bodies are shown to be in
        %   collision).
        end
    end

    % Non template methods that must be public (get/set and button
    % callbacks)
    methods
        function updateJointPositionField(obj, value)
        %updateJointPositionField Change the displayed joint edit field value

            obj.JointPositionEditField.Value = value;
        end

        function updateSliderPosition(obj, value)
        %updateSliderPosition Change the displayed joint slider value

        % The slider has to be inside its limits, even if the value
        % field is larger
            sliderValue = max(value, obj.JointPositionSlider.Limits(1));
            sliderValue = min(sliderValue, obj.JointPositionSlider.Limits(2));
            obj.JointPositionSlider.Value = sliderValue;
        end

        function updateRBTJointPosition(obj)
        %updateRBTJointPosition Update the rigid body tree position
        %   This method is called to update the rigid body tree
        %   position after an individual joint value has been changed

        % Since the edit field is used as the ground truth, update the
        % slider in case the values differ
            obj.updateSliderPosition(obj.JointPositionEditField.Value);

            % Send a notification through the correct controller-facing
            % object
            evt = robotics.ikdesigner.internal.event.JointMotionEvent(obj.JointIndex, obj.JointValue);
            obj.SendNotificationCB('RequestMoveJoint', evt);
        end
    end

    methods (Access = protected)
        function populateJointPositionFields(obj, config)
        %populateJointPositionFields Populate the joint position slider and edit field

        % Update the slider limits and tick marks
            obj.assignSliderLimits(obj.MinJointPosField.Value, obj.MaxJointPosField.Value);
            obj.JointPositionSlider.MajorTicks = obj.JointPositionSlider.Limits;
            obj.JointPositionSlider.MajorTickLabelsMode = 'auto';

            % The joint value is computed from the configuration given the
            % joint's index
            jointValue = config(obj.JointIndex);
            obj.JointPositionEditField.Value = obj.computeUserFacingJointValue(jointValue);

            % Update the slider to reflect the field stored in the edit field
            obj.updateSliderPosition(obj.JointPositionEditField.Value);
        end
    end

    methods (Access = private)
        function assignSliderLimits(obj, minJtPos, maxJtPos)
        %assignSliderLimits Slider limits must be finite
        %   If either slider limit is infinite, set it to an
        %   appropriate replacement value that has meaning but doesn't
        %   render the slider useless. The user can always set the
        %   value directly if the slider covers too great a range,
        %   though.

        % Check if either limit is infinite
            hasInfLimits = isinf([minJtPos, maxJtPos]);

            if all(hasInfLimits)
                % If the limits cover an infinite range, set up the slider
                % so that the values are equivalent or close and meaningful
                minJtPos = sign(minJtPos)*obj.INFVALUEBOUND;
                maxJtPos = sign(maxJtPos)*obj.INFVALUEBOUND;
            elseif hasInfLimits(1) && ~hasInfLimits(2)
                % If the range is infinite in only the minimum direction,
                % set up the slider so that the range is meaningfully large
                % but the upper bound is still enforced.
                minJtPos = maxJtPos - obj.INFVALUEBOUND;
            elseif ~hasInfLimits(1) && hasInfLimits(2)
                % If the range is infinite only in the maximum direction,
                % set up the slider so that the range is meaningfully large
                % but the lower bound is still enforced
                maxJtPos = minJtPos + obj.INFVALUEBOUND;
            else
                % If neither value is infinite, can just use the joint
                % limits as given.
            end

            obj.JointPositionSlider.Limits = [minJtPos maxJtPos];

        end

        function populateJointProperties(obj, jointHandle, parentName, childName)
        %populateJointProperties Populate the inspector with joint property values

            obj.JointNameText.Text = jointHandle.Name;
            obj.JointTypeText.Text = jointHandle.Type;
            obj.JointAxisText.Text = obj.printVector(jointHandle.JointAxis);
            obj.HomePositionText.Text = obj.printNumber(jointHandle.HomePosition);
            obj.ParentBodyText.Text = parentName;
            obj.ChildBodyText.Text = childName;

            obj.JointToParentTransformTable.Data = jointHandle.JointToParentTransform;
            obj.ChildToJointTransformTable.Data = jointHandle.ChildToJointTransform;

            % Process joint limits differently depending on the joint type
            positionLimits = obj.computeUserFacingPositionLimits(jointHandle.PositionLimits);
            obj.MinJointPosLabel.Text = obj.LOWERJTLIMLABEL + " " + obj.UNITSUFFIX;
            obj.MaxJointPosLabel.Text = obj.UPPERJTLIMLABEL + " " + obj.UNITSUFFIX;
            obj.MinJointPosField.Value = positionLimits(1);
            obj.MaxJointPosField.Value = positionLimits(2);
        end

        function addPropertiesPanel(obj)
        %addPropertiesPanel Add the properties panel

        % Add the properties panel structure
            propPanel = uipanel(obj.Grid, 'Title', obj.PROPSLABEL);
            propGrid = uigridlayout(propPanel);
            propGrid.ColumnWidth = {'1x'};
            propGrid.RowHeight = {'fit'};

            listedPropsGrid = uigridlayout(propGrid);
            listedPropsGrid.ColumnWidth = {'1x','1x'};
            listedPropsGrid.RowHeight = repmat("fit",1,7);

            % Add the joint name
            jointNameLabel = uilabel(listedPropsGrid);
            jointNameLabel.Text = obj.JOINTNAMELABEL;
            jointNameLabel.Tooltip = obj.JOINTNAMETOOLTIP;
            jointNameLabel.Layout.Row = 1;
            jointNameLabel.Layout.Column = 1;
            obj.JointNameText = uilabel(listedPropsGrid);
            obj.JointNameText.Layout.Row = 1;
            obj.JointNameText.Layout.Column = 2;

            % Add the joint type
            jointTypeLabel = uilabel(listedPropsGrid);
            jointTypeLabel.Text = obj.JOINTTYPELABEL;
            jointTypeLabel.Tooltip = obj.JOINTTYPETOOLTIP;
            jointTypeLabel.Layout.Row = 2;
            jointTypeLabel.Layout.Column = 1;
            obj.JointTypeText = uilabel(listedPropsGrid);
            obj.JointTypeText.Layout.Row = 2;
            obj.JointTypeText.Layout.Column = 2;

            % Add the joint axis
            jointAxisLabel = uilabel(listedPropsGrid);
            jointAxisLabel.Text = obj.JOINTAXISLABEL;
            jointAxisLabel.Tooltip = obj.JOINTAXISTOOLTIP;
            jointAxisLabel.Layout.Row = 3;
            jointAxisLabel.Layout.Column = 1;
            obj.JointAxisText = uilabel(listedPropsGrid);
            obj.JointAxisText.Layout.Row = 3;
            obj.JointAxisText.Layout.Column = 2;

            % Add the home position
            homePositionLabel = uilabel(listedPropsGrid);
            homePositionLabel.Text = obj.HOMEPOSLABEL;
            homePositionLabel.Tooltip = obj.HOMEPOSTOOLTIP;
            homePositionLabel.Layout.Row = 4;
            homePositionLabel.Layout.Column = 1;
            obj.HomePositionText = uilabel(listedPropsGrid);
            obj.HomePositionText.Layout.Row = 4;
            obj.HomePositionText.Layout.Column = 2;

            % Add the parent body name
            parentNameLabel = uilabel(listedPropsGrid);
            parentNameLabel.Text = obj.PARENTLABEL;
            parentNameLabel.Tooltip = obj.PARENTTOOLTIP;
            parentNameLabel.Layout.Row = 5;
            parentNameLabel.Layout.Column = 1;
            obj.ParentBodyText = uilabel(listedPropsGrid);
            obj.ParentBodyText.Layout.Row = 5;
            obj.ParentBodyText.Layout.Column = 2;

            % Add the child body name
            childNameLabel = uilabel(listedPropsGrid);
            childNameLabel.Text = obj.CHILDLABEL;
            childNameLabel.Tooltip = obj.CHILDTOOLTIP;
            childNameLabel.Layout.Row = 6;
            childNameLabel.Layout.Column = 1;
            obj.ChildBodyText = uilabel(listedPropsGrid);
            obj.ChildBodyText.Layout.Row = 6;
            obj.ChildBodyText.Layout.Column = 2;

            % Add the joint-to-parent grid, label, and table
            jointToParentGrid = uigridlayout(propGrid);
            jointToParentGrid.ColumnWidth = {'1x'};
            jointToParentGrid.RowHeight = {"fit",92.5};

            jointToParentLabel = uilabel(jointToParentGrid);
            jointToParentLabel.Text = obj.JOINTTOPARENTLABEL;
            jointToParentLabel.Tooltip = obj.JOINTTOPARENTTOOLTIP;
            jointToParentLabel.Layout.Row = 1;
            jointToParentLabel.Layout.Column = 1;

            obj.JointToParentTransformTable = uitable(jointToParentGrid, 'Data', eye(4));
            obj.JointToParentTransformTable.Layout.Column = 1;
            obj.JointToParentTransformTable.Layout.Row = 2;
            obj.JointToParentTransformTable.ColumnEditable=[false, false, false,false];
            obj.JointToParentTransformTable.ColumnWidth = {'1x','1x','1x','1x'};
            obj.JointToParentTransformTable.ColumnFormat={'numeric','numeric','numeric','numeric'};
            obj.JointToParentTransformTable.RowName = [];
            obj.JointToParentTransformTable.ColumnName = [];

            % Add the child-to-joint grid, label, and table
            childToJointGrid = uigridlayout(propGrid);
            childToJointGrid.ColumnWidth = {'1x'};
            childToJointGrid.RowHeight = {'1x',92.5};

            childToJointLabel = uilabel(childToJointGrid);
            childToJointLabel.Text = obj.CHILDTOJOINTLABEL;
            childToJointLabel.Tooltip = obj.CHILDTOJOINTTOOLTIP;
            childToJointLabel.Layout.Row = 1;
            childToJointLabel.Layout.Column = 1;

            obj.ChildToJointTransformTable = uitable(childToJointGrid, 'Data', eye(4));
            obj.ChildToJointTransformTable.Layout.Column = 1;
            obj.ChildToJointTransformTable.Layout.Row = 2;
            obj.ChildToJointTransformTable.ColumnEditable=[false, false, false,false];
            obj.ChildToJointTransformTable.ColumnWidth = {'1x','1x','1x','1x'};
            obj.ChildToJointTransformTable.ColumnFormat={'numeric','numeric','numeric','numeric'};
            obj.ChildToJointTransformTable.RowName = [];
            obj.ChildToJointTransformTable.ColumnName = [];

            % Use a local grid to lay out the fields
            jointLimitGrid = uigridlayout(propGrid);
            jointLimitGrid.ColumnWidth = {'fit'};
            jointLimitGrid.RowHeight = {'fit','fit','fit','fit'};

            % Add the lower joint limit
            obj.MinJointPosLabel = uilabel(jointLimitGrid);
            obj.MinJointPosLabel.Text = obj.LOWERJTLIMLABEL;
            obj.MaxJointPosLabel.Tooltip = obj.LOWERJTLIMTOOLTIP;
            obj.MinJointPosLabel.Layout.Row = 1;
            obj.MinJointPosLabel.Layout.Column = 1;
            obj.MinJointPosField = uieditfield(jointLimitGrid, 'numeric','Editable', 'off','Enable','on');
            obj.MinJointPosField.Layout.Row = 2;
            obj.MinJointPosField.Layout.Column = 1;

            % Add the upper joint limit
            obj.MaxJointPosLabel = uilabel(jointLimitGrid);
            obj.MaxJointPosLabel.Text = obj.UPPERJTLIMLABEL;
            obj.MaxJointPosLabel.Tooltip = obj.UPPERJTLIMTOOLTIP;
            obj.MaxJointPosLabel.Layout.Row = 3;
            obj.MaxJointPosLabel.Layout.Column = 1;
            obj.MaxJointPosField = uieditfield(jointLimitGrid, 'numeric','Editable', 'off','Enable','on');
            obj.MaxJointPosField.Layout.Row = 4;
            obj.MaxJointPosField.Layout.Column = 1;
        end

        function addStatesPanel(obj)
        %addStatesPanel Add panel containing pose and collision state data

            statesPanel = uipanel(obj.Grid, 'Title', obj.STATESLABEL);

            % There is one large grid containing all the states
            statesGrid = uigridlayout(statesPanel);
            statesGrid.ColumnWidth = {'1x','fit'};
            statesGrid.RowHeight = repmat("fit",1,5);

            obj.JointPosLabel = uilabel(statesGrid);
            obj.JointPosLabel.Text = obj.JOINTPOSITIONLABEL;
            obj.JointPosLabel.Tooltip = obj.JOINTPOSITIONTOOLTIP;
            obj.JointPosLabel.Layout.Row = 1;
            obj.JointPosLabel.Layout.Column = 1;

            obj.JointPositionSlider = uislider(statesGrid);
            obj.JointPositionSlider.Layout.Row = 2;
            obj.JointPositionSlider.Layout.Column = 1;
            obj.JointPositionSlider.MajorTicks =[0 100];
            obj.JointPositionSlider.MinorTicks =[];
            obj.JointPositionSlider.ValueChangingFcn = @(src,evt)obj.updateJointPositionField(evt.Value);
            obj.JointPositionSlider.ValueChangedFcn = @(src,evt)obj.updateRBTJointPosition();

            obj.JointPositionEditField = uieditfield(statesGrid, 'numeric','Editable', 'on','Enable','on');
            obj.JointPositionEditField.Layout.Row = 2;
            obj.JointPositionEditField.Layout.Column = 2;
            obj.JointPositionEditField.ValueChangedFcn = @(src,evt)obj.updateRBTJointPosition();

        end
    end
end

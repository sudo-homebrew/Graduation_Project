classdef AimingConstraintTab < robotics.ikdesigner.internal.toolstrip.ConstraintTab
%This class is for internal use only. It may be removed in the future.

%AimingConstraintTab Tab view that for the aiming constraint in the inverse kinematics designer toolstrip

%   Copyright 2021-2022 The MathWorks, Inc.

% Use str2num on edit field interpretation for ease of validation and
% broader acceptance of user inputs
%#ok<*ST2NM> 

    properties (SetAccess = {?robotics.ikdesigner.internal.view.View, ?matlab.unittest.TestCase})

        %EndEffectorBodyDropdown Dropdown for selecting the end effector body
        EndEffectorBodyDropdown

        %ReferenceBodyDropdown Dropdown for selecting the reference body
        ReferenceBodyDropdown

        %XField Field for entering the target point X Data
        XField

        %YField Field for entering the target point Y Data
        YField

        %ZField Field for entering the target point Z Data
        ZField

        %AngularToleranceField Field for entering the angular tolerance
        AngularToleranceField

        %ConstraintWeightField Field for entering the constraint weight
        ConstraintWeightField

    end

    properties (Dependent)
        %TargetPoint Numerical target point value
        TargetPoint

        %AngularTolerance Numerical angular tolerance value
        AngularTolerance

        %ConstraintWeight Numerical constraint weight value
        ConstraintWeight
    end

    properties (Constant)
        %DEFAULTTARGETPOINT Default target point values
        DEFAULTTARGETPOINT = [0 0 0]

        %DEFAULTANGULARTOLERANCE Default angular tolerance values
        DEFAULTANGULARTOLERANCE = pi/180

        %getConstraintData Default constraint weight values
        DEFAULTCONSTRAINTWEIGHT = 1
    end

    properties (Constant)
        SETTINGSTITLE = string(message('robotics:ikdesigner:constrainttabviews:AimingConstraintSettingsTitle'))

        XTARGETLABEL = string(message('robotics:ikdesigner:constrainttabviews:AimingConstraintXTargetLabel'))
        XTARGETTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:AimingConstraintXTargetTooltip'))

        YTARGETLABEL = string(message('robotics:ikdesigner:constrainttabviews:AimingConstraintYTargetLabel'))
        YTARGETTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:AimingConstraintYTargetTooltip'))

        ZTARGETLABEL = string(message('robotics:ikdesigner:constrainttabviews:AimingConstraintZTargetLabel'))
        ZTARGETTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:AimingConstraintZTargetTooltip'))

        ANGTOLERANCELABEL = string(message('robotics:ikdesigner:constrainttabviews:AimingConstraintAngularToleranceLabel'))
        ANGTOLERANCETOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:AimingConstraintAngularToleranceTooltip'))

        CONSTRAINTWEIGHTLABEL = string(message('robotics:ikdesigner:constrainttabviews:AimingConstraintConstraintWeightLabel'))
        CONSTRAINTWEIGHTTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:AimingConstraintConstraintWeightTooltip'))
    end

    methods
        function obj = AimingConstraintTab(toolstrip)
        %AimingConstraintTab Constructor

            tabTag = robotics.ikdesigner.internal.constants.Toolstrip.AimingConstraintTabTag;
            obj@robotics.ikdesigner.internal.toolstrip.ConstraintTab(toolstrip, tabTag)

        end

        function setup(obj, robot, rigidBodyKeyMap)
        %setup Set up the tab with new session data

        % Initialize the body name dropdowns from the robot
            obj.initializeBodyNameDropdown(obj.EndEffectorBodyDropdown, robot, rigidBodyKeyMap, rigidBodyKeyMap.Count);
            obj.initializeBodyNameDropdown(obj.ReferenceBodyDropdown, robot, rigidBodyKeyMap, 1);

        end

        function show(obj)
        %show Extension to the parent method
        %   When the constraint tab is displayed, the visual aids in
        %   the scene canvas must be made apparent. This method extends
        %   the default version in the superclass to add these
        %   elements.

            show@robotics.ikdesigner.internal.toolstrip.ConstraintTab(obj)
            obj.ConstraintName.Enabled = true;

            updateType = robotics.ikdesigner.internal.toolstrip.ConstraintUpdate.NewOverlay;
            obj.updateConstraintVisual(updateType);
        end

        function hide(obj)
        %hide Extension to the parent method
        %   When the constraint tab is hidden, the visual aids in the
        %   scene canvas must be remove from view. This method extends
        %   the default version in the superclass to delete these
        %   elements.

            hide@robotics.ikdesigner.internal.toolstrip.Tab(obj)

            updateType = robotics.ikdesigner.internal.toolstrip.ConstraintUpdate.DeleteOverlay;
            obj.updateConstraintVisual(updateType);
        end

        function resetToDefaultState(obj)
        %resetToDefaultState Reset the user-facing UI components to their default values

            obj.ApplyButton.Enabled = true;
            obj.TargetPoint = obj.DEFAULTTARGETPOINT;
            obj.EndEffectorBodyDropdown.Value = obj.EndEffectorBodyDropdown.Items{end, 1};
            obj.ReferenceBodyDropdown.Value = obj.EndEffectorBodyDropdown.Items{1, 1};
            obj.AngularTolerance = obj.DEFAULTANGULARTOLERANCE;
            obj.ConstraintWeight = obj.DEFAULTCONSTRAINTWEIGHT;
        end

        function tgtPt = get.TargetPoint(obj)
        %get.TargetPoint Return a numerical target point value from the values of the three component edit fields

            tgtPt = [str2num(obj.XField.Value) str2num(obj.YField.Value) str2num(obj.ZField.Value)];

        end

        function set.TargetPoint(obj, tgtPt)
        %set.TargetPoint Update the values of the three component edit fields from a single numerical vector

            obj.XField.Value = num2str(tgtPt(1));
            obj.YField.Value = num2str(tgtPt(2));
            obj.ZField.Value = num2str(tgtPt(3));

        end

        function tol = get.AngularTolerance(obj)
        %get.AngularTolerance Get a numerical value from the edit field string

            degTol = str2num(obj.AngularToleranceField.Value);
            tol = deg2rad(degTol);

        end

        function set.AngularTolerance(obj, tol)
        %set.AngularTolerance Translate a scalar number to the edit field string

            degTol = rad2deg(tol);
            obj.AngularToleranceField.Value = num2str(degTol);

        end

        function wt = get.ConstraintWeight(obj)
        %get.ConstraintWeight Get a numerical value from the edit field string

            wt = str2num(obj.ConstraintWeightField.Value);

        end

        function set.ConstraintWeight(obj, wt)
        %set.AngularTolerance Translate a scalar number to the edit field string

            obj.ConstraintWeightField.Value = num2str(wt(1));

        end

        function populateTabFieldsFromDataStruct(obj, eventDataStruct)
        %populateTabFieldsFromDataStruct Populate the tab with values from a struct
        %   When constraint data is passed between the view and model,
        %   the constraint properties are stored in a structure that is
        %   unique to that constraint type. This method populates the
        %   user-facing edit fields with values from that structure.
        %   This is needed e.g. to populate a tab for edit.

            obj.TargetPoint = eventDataStruct.TargetPoint;
            obj.EndEffectorBodyDropdown.Value = eventDataStruct.EEBodyKey;
            obj.ReferenceBodyDropdown.Value = eventDataStruct.RefBodyKey;
            obj.AngularTolerance = eventDataStruct.AngularTolerance;
            obj.ConstraintWeight = eventDataStruct.Weights;

            % Once the data is populated, the apply button should be
            % disabled.
            obj.ApplyButton.Enabled = false;

        end

        function constraintDataEvent = populateConstraintDataEvent(obj)
        %populateConstraintDataEvent Populate the tab ui components with values for a new constraint
        %   When new or existing constraint is edited, the data must be
        %   stored in the event before being sent to the model. This
        %   method details how this data is stored for the aiming
        %   constraint.

            constraintKey = obj.ActiveConstraintKey;
            constraintName = string(obj.ConstraintName.Value);
            constraintType = robotics.ikdesigner.internal.toolstrip.ConstraintType.Aiming;
            constraintData = obj.populateDataStruct();
            constraintDataEvent = robotics.ikdesigner.internal.event.SolverConstraintEventData(constraintKey, constraintName, constraintType, constraintData);
        end
    end

    methods (Access = protected)
        function addSettingsSection(obj)
        %addSettingsSection Add the settings section to the tab

            obj.createAimingConstraintSettingsSection();
            obj.TabHandle.add(obj.SettingsSection);
        end

        function updateConstraintVisual(obj, updateType)
        %updateConstraintVisual Update the visuals associated with the constraint

            updateConstraintVisual@robotics.ikdesigner.internal.toolstrip.ConstraintTab(obj, updateType);

            constraintVisualsEvent = obj.populateConstraintUpdateEvent(updateType);
            obj.sendNotification("ConstraintVisualsChange", constraintVisualsEvent);
        end
    end

    methods (Access = private)
        function isValid = validateAndSetTargetField(obj, src, evt, fieldProp, fieldName)
            %validateAndSetTargetField Validation for X, Y, and Z Target fields

            isValid = obj.validateAndSetNumericInput(src, evt, fieldProp, fieldName, {'nonempty', 'real', 'nonnan', 'finite', 'scalar'});
            if isValid
                obj.updateConstraintVisual(robotics.ikdesigner.internal.toolstrip.ConstraintUpdate.DataUpdate);
            end

        end

        function constraintVisualsEvent = populateConstraintUpdateEvent(obj, updateType)
        %populateConstraintUpdateEvent Populate the event used to update the constraint overlay

            constraintVisualsEvent = robotics.ikdesigner.internal.event.AimingConstraintDisplayEvent(updateType);

            % Update the visual indicators
            constraintVisualsEvent.TargetPoint = obj.TargetPoint;
            constraintVisualsEvent.EEBodyKey = obj.EndEffectorBodyDropdown.Value;
            constraintVisualsEvent.RefBodyKey = obj.ReferenceBodyDropdown.Value;

        end

        function dataStruct = populateDataStruct(obj)
        %populateDataStruct Populate the Data struct in the SolverConstraintEvent
        %   When constraint data is passed between the view and model,
        %   it is stored in an event that takes a standard form across
        %   all constraint types, except for the Data field, which
        %   contains a structure that has contents particular to that
        %   constraint (constraint type is indicated by an enum). This
        %   method details how that struct is populated for the aiming
        %   constraint.

            dataStruct = struct();

            % Update the visual indicators
            dataStruct.TargetPoint = obj.TargetPoint;
            dataStruct.AngularTolerance = obj.AngularTolerance;
            dataStruct.Weights = obj.ConstraintWeight;
            dataStruct.EEBodyKey = obj.EndEffectorBodyDropdown.Value;
            dataStruct.RefBodyKey = obj.ReferenceBodyDropdown.Value;
        end

        function createAimingConstraintSettingsSection(obj)
        %createAimingConstraintSettingsSection Create settings section for the aiming constraint tab

            obj.SettingsSection = matlab.ui.internal.toolstrip.Section(obj.SETTINGSTITLE);
            obj.SettingsSection.Tag = "SettingsSection";

            [obj.EndEffectorBodyDropdown, obj.ReferenceBodyDropdown] = obj.addBodyDropdowns();
            obj.addTargetPointFields();
            obj.addAngularToleranceAndWeight();
        end

        function addTargetPointFields(obj)
        %addTargetPointFields Add fields that let user define a target point

        % The second column contains the target point data. Again, the
        % labels are in col2a, and the field in column 2b
            colA = obj.SettingsSection.addColumn();
            colB = obj.SettingsSection.addColumn('Width', 50);

            label = matlab.ui.internal.toolstrip.Label(obj.XTARGETLABEL);
            label.Tag = "XLabel";
            label.Description = obj.XTARGETTOOLTIP;
            colA.add(label);

            obj.XField = matlab.ui.internal.toolstrip.EditField('0');
            obj.XField.Tag = "XTargetPos";
            obj.XField.Description = obj.XTARGETTOOLTIP;
            obj.XField.ValueChangedFcn = @(src,evt)obj.validateAndSetTargetField(src,evt,obj.XField,obj.XTARGETLABEL);
            colB.add(obj.XField);

            label = matlab.ui.internal.toolstrip.Label(obj.YTARGETLABEL);
            label.Tag = "YLabel";
            label.Description = obj.YTARGETTOOLTIP;
            colA.add(label);

            obj.YField = matlab.ui.internal.toolstrip.EditField('0');
            obj.YField.Tag = "YTargetPos";
            obj.YField.Description = obj.YTARGETTOOLTIP;
            obj.YField.ValueChangedFcn = @(src,evt)obj.validateAndSetTargetField(src,evt,obj.YField,obj.YTARGETLABEL);
            colB.add(obj.YField);

            label = matlab.ui.internal.toolstrip.Label(obj.ZTARGETLABEL);
            label.Tag = "ZLabel";
            label.Description = obj.ZTARGETTOOLTIP;
            colA.add(label);

            obj.ZField = matlab.ui.internal.toolstrip.EditField('0');
            obj.ZField.Tag = "ZTargetPos";
            obj.ZField.Description = obj.ZTARGETTOOLTIP;
            obj.ZField.ValueChangedFcn = @(src,evt)obj.validateAndSetTargetField(src,evt,obj.ZField,obj.ZTARGETLABEL);
            colB.add(obj.ZField);
        end

        function addAngularToleranceAndWeight(obj)
        %addAngularToleranceAndWeight Add fields that let user set the tolerance and weight

        % The last column has the tolerances
            colA = obj.SettingsSection.addColumn();
            colB = obj.SettingsSection.addColumn('Width', 50);

            label = matlab.ui.internal.toolstrip.Label(obj.ANGTOLERANCELABEL);
            label.Tag = "AngularToleranceLabel";
            label.Description = obj.ANGTOLERANCETOOLTIP;
            colA.add(label);

            obj.AngularToleranceField = matlab.ui.internal.toolstrip.EditField('0');
            obj.AngularToleranceField.Tag = "AngularTolerance";
            obj.AngularToleranceField.Description = obj.ANGTOLERANCETOOLTIP;
            obj.AngularToleranceField.ValueChangedFcn = @(src,evt)obj.validateAndSetToleranceFields(src, evt, obj.AngularToleranceField, obj.ANGTOLERANCELABEL);
            colB.add(obj.AngularToleranceField);

            label = matlab.ui.internal.toolstrip.Label(obj.CONSTRAINTWEIGHTLABEL);
            label.Tag = "ConstraintWeightLabel";
            label.Description = obj.CONSTRAINTWEIGHTTOOLTIP;
            colA.add(label);

            obj.ConstraintWeightField = matlab.ui.internal.toolstrip.EditField('1');
            obj.ConstraintWeightField.Tag = "ConstraintWeight";
            obj.ConstraintWeightField.Description = obj.CONSTRAINTWEIGHTTOOLTIP;
            obj.ConstraintWeightField.ValueChangedFcn = @(src,evt)obj.validateAndSetWeightFields(src, evt, obj.ConstraintWeightField, obj.CONSTRAINTWEIGHTLABEL);
            colB.add(obj.ConstraintWeightField);

        end
    end

end

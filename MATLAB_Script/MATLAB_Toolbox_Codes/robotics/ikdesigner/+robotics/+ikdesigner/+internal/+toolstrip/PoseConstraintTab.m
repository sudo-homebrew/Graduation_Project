classdef PoseConstraintTab < robotics.ikdesigner.internal.toolstrip.ConstraintTab
%This class is for internal use only. It may be removed in the future.

%PoseConstraintTab Tab view that for the pose constraint in the inverse kinematics designer toolstrip

%   Copyright 2021-2022 The MathWorks, Inc.

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

        %EulerXField Field for entering the target orientation Euler X Data
        EulerXField

        %EulerYField Field for entering the target orientation Euler Y Data
        EulerYField

        %EulerZField Field for entering the target orientation Euler Z Data
        EulerZField

        %PositionToleranceField Field for entering the position tolerance
        PositionToleranceField

        %PositionWeightField Field for entering the position weight
        PositionWeightField

        %OrientationToleranceField Field for entering the orientation tolerance
        OrientationToleranceField

        %OrientationWeightField Field for entering the orientation weight
        OrientationWeightField

    end

    properties (Dependent)
        %TargetTransform Numerical target transform matrix
        TargetTransform

        %OrientationTolerance Numerical orientation tolerance value
        OrientationTolerance

        %PositionTolerance Numerical position tolerance value
        PositionTolerance

        %Weights Numerical position and orientation weight vector
        Weights

        %IsMarkerPoseView Flag to indicate that this view is displaying the marker pose
        IsMarkerPoseView
    end

    properties (Constant)
        %DefaultTargetPoint Default target point values
        DefaultTargetTransform = eye(4)

        %DefaultOrientationTolerance Default orientation tolerance values
        DefaultOrientationTolerance = pi/180

        %DefaultPositionTolerance Default position tolerance values
        DefaultPositionTolerance = 0.01

        %DefaultPositionWeight Default position weight values
        DefaultWeights = [1 1]
    end

    properties (Constant)
        SETTINGSTITLE = string(message('robotics:ikdesigner:constrainttabviews:PoseConstraintSettingsTitle'))

        POSXLABEL = string(message('robotics:ikdesigner:constrainttabviews:PoseConstraintXPosLabel'))
        POSXTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:PoseConstraintXPosTooltip'))

        POSYLABEL = string(message('robotics:ikdesigner:constrainttabviews:PoseConstraintYPosLabel'))
        POSYTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:PoseConstraintYPosTooltip'))

        POSZLABEL = string(message('robotics:ikdesigner:constrainttabviews:PoseConstraintZPosLabel'))
        POSZTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:PoseConstraintZPosTooltip'))

        ROTXLABEL = string(message('robotics:ikdesigner:constrainttabviews:PoseConstraintXRotLabel'))
        ROTXTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:PoseConstraintXRotTooltip'))

        ROTYLABEL = string(message('robotics:ikdesigner:constrainttabviews:PoseConstraintYRotLabel'))
        ROTYTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:PoseConstraintYRotTooltip'))

        ROTZLABEL = string(message('robotics:ikdesigner:constrainttabviews:PoseConstraintZRotLabel'))
        ROTZTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:PoseConstraintZRotTooltip'))

        POSTOLLABEL = string(message('robotics:ikdesigner:constrainttabviews:PoseConstraintPosToleranceLabel'))
        POSTOLTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:PoseConstraintPosToleranceTooltip'))

        POSWEIGHTLABEL = string(message('robotics:ikdesigner:constrainttabviews:PoseConstraintPosWeightLabel'))
        POSWEIGHTTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:PoseConstraintPosWeightTooltip'))

        ORITOLLABEL = string(message('robotics:ikdesigner:constrainttabviews:PoseConstraintOriToleranceLabel'))
        ORITOLTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:PoseConstraintOriToleranceTooltip'))

        ORIWEIGHTLABEL = string(message('robotics:ikdesigner:constrainttabviews:PoseConstraintOriWeightLabel'))
        ORIWEIGHTTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:PoseConstraintOriWeightTooltip'))
    end

    methods
        function obj = PoseConstraintTab(toolstrip)
        %PoseConstraintTab Constructor

            tabTag = robotics.ikdesigner.internal.constants.Toolstrip.PoseTargetConstraintTabTag;
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

            % The marker pose view doesn't use an overlay because the
            % marker pose target is used instead, but for all other
            % cases, an overlay is in place
            if obj.IsMarkerPoseView
                obj.applyMarkerPoseView(true);
                obj.sendNotification("RequestDisableMarker", []);
            else
                obj.applyMarkerPoseView(false);
            end

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

            if obj.IsMarkerPoseView
                obj.sendNotification("RequestEnableMarker", []);
            end

            updateType = robotics.ikdesigner.internal.toolstrip.ConstraintUpdate.DeleteOverlay;
            obj.updateConstraintVisual(updateType);
        end

        function resetToDefaultState(obj)
        %resetToDefaultState Reset the user-facing UI components to their default values

            obj.ApplyButton.Enabled = true;
            obj.TargetTransform = obj.DefaultTargetTransform;
            obj.EndEffectorBodyDropdown.Value = obj.EndEffectorBodyDropdown.Items{end, 1};
            obj.ReferenceBodyDropdown.Value = obj.EndEffectorBodyDropdown.Items{1, 1};
            obj.OrientationTolerance = obj.DefaultOrientationTolerance;
            obj.PositionTolerance = obj.DefaultPositionTolerance;
            obj.Weights = obj.DefaultWeights;
        end

        function tform = get.TargetTransform(obj)
        %get.TargetTransform Return a numerical target transform matrix from the six component edit fields

            tform = obj.retrieveTransformFromTab();
        end

        function set.TargetTransform(obj, tform)
        %set.TargetTransform Update the values of the six component edit fields from a single numerical matrix

            obj.displayTransformInTab(tform);
        end

        function tol = get.OrientationTolerance(obj)
        %get.OrientationTolerance Get a numerical value from the edit field string
        %   The user-facing value is in degrees, while the internal value
        %   is in radians.

            tol = deg2rad(str2num(obj.OrientationToleranceField.Value)); %#ok<ST2NM> 

        end

        function set.OrientationTolerance(obj, tol)
        %set.OrientationTolerance Translate a scalar number to the edit field string
        %   The user-facing value is in degrees, while the internal value
        %   is in radians.

            obj.OrientationToleranceField.Value = num2str(rad2deg(tol(1)));

        end

        function tol = get.PositionTolerance(obj)
        %get.PositionTolerance Get a numerical value from the edit field string

            tol = str2num(obj.PositionToleranceField.Value); %#ok<ST2NM> 

        end

        function set.PositionTolerance(obj, tol)
        %set.PositionTolerance Translate a scalar number to the edit field string

            obj.PositionToleranceField.Value = num2str(tol(1));

        end

        function wt = get.Weights(obj)
        %get.Weights Get a numerical vector of weights from the two edit field strings

            wt = ones(1,2);
            wt(1) = str2num(obj.OrientationWeightField.Value); %#ok<ST2NM> 
            wt(2) = str2num(obj.PositionWeightField.Value); %#ok<ST2NM> 

        end

        function set.Weights(obj, wt)
        %set.Weights Translate a two-element vector to the two edit field strings

            obj.OrientationWeightField.Value = num2str(wt(1));
            obj.PositionWeightField.Value = num2str(wt(2));

        end

        function isEnabled = get.IsMarkerPoseView(obj)

            isEnabled = (obj.ActiveConstraintKey == robotics.ikdesigner.internal.constants.Data.MARKERPOSETARGETKEY);

        end

        function populateTabFieldsFromDataStruct(obj, eventDataStruct)
        %populateTabFieldsFromDataStruct Populate the tab with values from a struct
        %   When constraint data is passed between the view and model,
        %   the constraint properties are stored in a structure that is
        %   unique to that constraint type. This method populates the
        %   user-facing edit fields with values from that structure.
        %   This is needed e.g. to populate a tab for edit.

            obj.TargetTransform = eventDataStruct.TargetTransform;
            obj.EndEffectorBodyDropdown.Value = eventDataStruct.EEBodyKey;
            obj.ReferenceBodyDropdown.Value = eventDataStruct.RefBodyKey;
            obj.PositionTolerance = eventDataStruct.PositionTolerance;
            obj.OrientationTolerance = eventDataStruct.OrientationTolerance;
            obj.Weights = eventDataStruct.Weights;

            % Once the data is populated, the apply button should be
            % disabled.
            obj.ApplyButton.Enabled = false;

        end

        function constraintDataEvent = populateConstraintDataEvent(obj)
        %populateConstraintDataEvent Populate the tab ui components with values for a new constraint
        %   When new or existing constraint is edited, the data must be
        %   stored in the event before being sent to the model. This
        %   method details how this data is stored for the pose
        %   constraint.

            constraintKey = obj.ActiveConstraintKey;
            constraintName = string(obj.ConstraintName.Value);
            constraintType = robotics.ikdesigner.internal.toolstrip.ConstraintType.Pose;
            constraintData = obj.populateDataStruct();
            constraintDataEvent = robotics.ikdesigner.internal.event.SolverConstraintEventData(constraintKey, constraintName, constraintType, constraintData);
        end
    end

    methods (Access = protected)
        function addSettingsSection(obj)
        %addSettingsSection Add the settings section to the tab

            obj.createPoseConstraintSettingsSection();
            obj.TabHandle.add(obj.SettingsSection);
        end

        function notifyConstraintApplied(obj)

            if obj.IsMarkerPoseView
                % Update the marker pose prior to updating the constraint.
                % This is needed because the solver uses the marker pose as
                % the canonical input for the marker pose constraint
                poseEvent = robotics.ikdesigner.internal.event.PoseEventData(obj.TargetTransform);
                obj.sendNotification("RequestUpdateMarkerPose", poseEvent);
            end
            notifyConstraintApplied@robotics.ikdesigner.internal.toolstrip.ConstraintTab(obj);

        end
    end

    methods (Access = protected)

        function updateConstraintVisual(obj, updateType)
        %updateConstraintVisual Update the visuals associated with the constraint

            updateConstraintVisual@robotics.ikdesigner.internal.toolstrip.ConstraintTab(obj, updateType);

            constraintVisualsEvent = obj.populateConstraintUpdateEvent(updateType);
            obj.sendNotification("ConstraintVisualsChange", constraintVisualsEvent);
        end
    end

    methods (Access = private)
        function isValid = validateAndSetTransformFields(obj, src, evt, fieldProp, fieldName)
            %validateAndSetTransformFields Validation for pose transform fields

            isValid = obj.validateAndSetNumericInput(src, evt, fieldProp, fieldName, {'nonempty', 'real', 'nonnan', 'finite', 'scalar'});
            if isValid
                obj.updateConstraintVisual(robotics.ikdesigner.internal.toolstrip.ConstraintUpdate.DataUpdate);
            end

        end

        function constraintVisualsEvent = populateConstraintUpdateEvent(obj, updateType)
        %populateConstraintUpdateEvent Populate the event used to update the constraint overlay

            constraintVisualsEvent = robotics.ikdesigner.internal.event.PoseConstraintDisplayEvent(updateType);

            % Update the visual indicators
            constraintVisualsEvent.TargetTransform = obj.TargetTransform;
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
        %   method details how that struct is populated for the pose
        %   constraint.

            dataStruct = struct();

            % Update the visual indicators
            dataStruct.TargetTransform = obj.TargetTransform;
            dataStruct.OrientationTolerance = obj.OrientationTolerance;
            dataStruct.PositionTolerance = obj.PositionTolerance;
            dataStruct.Weights = obj.Weights;
            dataStruct.EEBodyKey = obj.EndEffectorBodyDropdown.Value;
            dataStruct.RefBodyKey = obj.ReferenceBodyDropdown.Value;
        end

        function applyMarkerPoseView(obj, isEnabled)

            if isEnabled
                obj.ReferenceBodyDropdown.Enabled = false;
                obj.ConstraintGallery.Enabled = false;
                obj.ConstraintName.Enabled = false;
            else
                obj.ReferenceBodyDropdown.Enabled = true;
                obj.ConstraintGallery.Enabled = true;
                obj.ConstraintName.Enabled = true;
            end

        end

        function createPoseConstraintSettingsSection(obj)
        %createPoseConstraintSettingsSection Create settings section for the pose constraint tab

            obj.SettingsSection = matlab.ui.internal.toolstrip.Section(obj.SETTINGSTITLE);
            obj.SettingsSection.Tag = "SettingsSection";

            [obj.EndEffectorBodyDropdown, obj.ReferenceBodyDropdown] = obj.addBodyDropdowns();
            obj.addTargetPositionFields();
            obj.addTargetOrientationFields();
            obj.addPositionToleranceAndWeight();
            obj.addOrientationToleranceAndWeight();
        end

        function addTargetPositionFields(obj)
        %addTargetPositionFields Add fields that let user set the translation component of the target pose

        % The second column contains the target position data. Again,
        % the labels are in col2a, and the field in column 2b
            colA = obj.SettingsSection.addColumn();
            colB = obj.SettingsSection.addColumn('Width', 75);


            label = matlab.ui.internal.toolstrip.Label(obj.POSXLABEL);
            label.Tag = "XLabel";
            label.Description = obj.POSXTOOLTIP;
            colA.add(label);

            obj.XField = matlab.ui.internal.toolstrip.EditField('0');
            obj.XField.Tag = "PosX";
            obj.XField.Description = obj.POSXTOOLTIP;
            obj.XField.ValueChangedFcn = @(src,evt)obj.validateAndSetTransformFields(src,evt,obj.XField,obj.POSXLABEL);
            colB.add(obj.XField);

            label = matlab.ui.internal.toolstrip.Label(obj.POSYLABEL);
            label.Tag = "YLabel";
            label.Description = obj.POSYTOOLTIP;
            colA.add(label);

            obj.YField = matlab.ui.internal.toolstrip.EditField('0');
            obj.YField.Tag = "PosY";
            obj.YField.Description = obj.POSYTOOLTIP;
            obj.YField.ValueChangedFcn = @(src,evt)obj.validateAndSetTransformFields(src,evt,obj.YField,obj.POSYLABEL);
            colB.add(obj.YField);

            label = matlab.ui.internal.toolstrip.Label(obj.POSZLABEL);
            label.Tag = "ZLabel";
            label.Description = obj.POSZTOOLTIP;
            colA.add(label);

            obj.ZField = matlab.ui.internal.toolstrip.EditField('0');
            obj.ZField.Tag = "PosZ";
            obj.ZField.Description = obj.POSZTOOLTIP;
            obj.ZField.ValueChangedFcn = @(src,evt)obj.validateAndSetTransformFields(src,evt,obj.ZField,obj.POSZLABEL);
            colB.add(obj.ZField);
        end

        function addTargetOrientationFields(obj)
        %addTargetOrientationFields Add fields that let user set the orientation component of the target pose

        % The second column contains the target position data. Again,
        % the labels are in col2a, and the field in column 2b
            colA = obj.SettingsSection.addColumn();
            colB = obj.SettingsSection.addColumn('Width', 50);


            label = matlab.ui.internal.toolstrip.Label(obj.ROTXLABEL);
            label.Tag = "EulerXLabel";
            label.Description = obj.ROTXTOOLTIP;
            colA.add(label);

            obj.EulerXField = matlab.ui.internal.toolstrip.EditField('0');
            obj.EulerXField.Tag = "EulerX";
            obj.EulerXField.Description = label.Description;
            obj.EulerXField.ValueChangedFcn = @(src,evt)obj.validateAndSetTransformFields(src,evt,obj.EulerXField,obj.ROTXLABEL);
            colB.add(obj.EulerXField);

            label = matlab.ui.internal.toolstrip.Label(obj.ROTYLABEL);
            label.Tag = "EulerYLabel";
            label.Description = obj.ROTYTOOLTIP;
            colA.add(label);

            obj.EulerYField = matlab.ui.internal.toolstrip.EditField('0');
            obj.EulerYField.Tag = "EulerY";
            obj.EulerYField.Description = label.Description;
            obj.EulerYField.ValueChangedFcn = @(src,evt)obj.validateAndSetTransformFields(src,evt,obj.EulerYField,obj.ROTYLABEL);
            colB.add(obj.EulerYField);

            label = matlab.ui.internal.toolstrip.Label(obj.ROTZLABEL);
            label.Tag = "EulerZLabel";
            label.Description = obj.ROTZTOOLTIP;
            colA.add(label);

            obj.EulerZField = matlab.ui.internal.toolstrip.EditField('0');
            obj.EulerZField.Tag = "EulerZ";
            obj.EulerZField.Description = label.Description;
            obj.EulerZField.ValueChangedFcn = @(src,evt)obj.validateAndSetTransformFields(src,evt,obj.EulerZField,obj.ROTZLABEL);
            colB.add(obj.EulerZField);
        end

        function addPositionToleranceAndWeight(obj)
        %addPositionToleranceAndWeight Add fields that let user set the tolerance and weight on the position component of the target pose constraint

        % The last column has the tolerances
            colA = obj.SettingsSection.addColumn();
            colB = obj.SettingsSection.addColumn('Width', 50);

            label = matlab.ui.internal.toolstrip.Label(obj.POSTOLLABEL);
            label.Tag = "PositionToleranceLabel";
            label.Description = obj.POSTOLTOOLTIP;
            colA.add(label);

            obj.PositionToleranceField = matlab.ui.internal.toolstrip.EditField('0');
            obj.PositionToleranceField.Tag = "PositionTolerance";
            obj.PositionToleranceField.Description = label.Description;
            obj.PositionToleranceField.ValueChangedFcn = @(src,evt)obj.validateAndSetToleranceFields(src,evt,obj.PositionToleranceField,obj.POSTOLLABEL);
            colB.add(obj.PositionToleranceField);

            label = matlab.ui.internal.toolstrip.Label(obj.POSWEIGHTLABEL);
            label.Tag = "PositionWeightLabel";
            label.Description = obj.POSWEIGHTTOOLTIP;
            colA.add(label);

            obj.PositionWeightField = matlab.ui.internal.toolstrip.EditField('1');
            obj.PositionWeightField.Tag = "PositionWeight";
            obj.PositionWeightField.Description = label.Description;
            obj.PositionWeightField.ValueChangedFcn = @(src,evt)obj.validateAndSetWeightFields(src,evt,obj.PositionWeightField,obj.POSWEIGHTLABEL);
            colB.add(obj.PositionWeightField);
        end

        function addOrientationToleranceAndWeight(obj)
        %addOrientationToleranceAndWeight Add fields that let user set the tolerance and weight on the orientation component of the target pose constraint

        % The last column has the tolerances
            colA = obj.SettingsSection.addColumn();
            colB = obj.SettingsSection.addColumn('Width', 50);

            label = matlab.ui.internal.toolstrip.Label(obj.ORITOLLABEL);
            label.Tag = "OrientationToleranceLabel";
            label.Description = obj.ORITOLTOOLTIP;
            colA.add(label);

            obj.OrientationToleranceField = matlab.ui.internal.toolstrip.EditField('0');
            obj.OrientationToleranceField.Tag = "OrientationTolerance";
            obj.OrientationToleranceField.Description = label.Description;
            obj.OrientationToleranceField.ValueChangedFcn = @(src,evt)obj.validateAndSetToleranceFields(src,evt,obj.OrientationToleranceField,obj.ORITOLLABEL);
            colB.add(obj.OrientationToleranceField);

            label = matlab.ui.internal.toolstrip.Label(obj.ORIWEIGHTLABEL);
            label.Tag = "OrientationWeightLabel";
            label.Description = obj.ORIWEIGHTTOOLTIP;
            colA.add(label);

            obj.OrientationWeightField = matlab.ui.internal.toolstrip.EditField('1');
            obj.OrientationWeightField.Tag = "OrientationWeight";
            obj.OrientationWeightField.Description = label.Description;
            obj.OrientationWeightField.ValueChangedFcn = @(src,evt)obj.validateAndSetWeightFields(src,evt,obj.OrientationWeightField,obj.ORIWEIGHTLABEL);
            colB.add(obj.OrientationWeightField);
        end
    end

end

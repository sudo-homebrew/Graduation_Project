classdef CartesianBoundsConstraintTab < robotics.ikdesigner.internal.toolstrip.ConstraintTab
%This class is for internal use only. It may be removed in the future.

%CartesianBoundsConstraintTab Tab view that for the cartesian bounds constraint in the inverse kinematics designer toolstrip

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

        %EulerXField Field for entering the target orientation Euler X Data
        EulerXField

        %EulerYField Field for entering the target orientation Euler Y Data
        EulerYField

        %EulerZField Field for entering the target orientation Euler Z Data
        EulerZField

        %XMinField Field for entering the minimum X value
        XMinField

        %YMinField Field for entering the minimum Y value
        YMinField

        %ZMinField Field for entering the minimum Z value
        ZMinField

        %XMaxField Field for entering the maximum X value
        XMaxField

        %YMaxField Field for entering the maximum Y value
        YMaxField

        %ZMaxField Field for entering the maximum Z value
        ZMaxField

        %XWeight Field for entering the X weight
        XWeightField

        %YWeightField Field for entering the Y weight
        YWeightField

        %ZWeightField Field for entering the Z weight
        ZWeightField

    end

    properties (Dependent)
        %TargetTransform Numerical target transform matrix
        TargetTransform

        %Bounds Numerical bounds matrix value
        Bounds

        %ConstraintWeights Numerical XYZ weight value
        ConstraintWeights
    end

    properties (Constant)
        %DEFAULTTARGETTRANSFORM Default target transform matrix
        DEFAULTTARGETTRANSFORM = eye(4)

        %DEFAULTBOUNDS Default values for the cartesian bounds matrix
        DEFAULTBOUNDS = repmat([-0.5 0.5], 3, 1)

        %DEFAULTCONSTRAINTWEIGHTS Default constraint weight values
        DEFAULTCONSTRAINTWEIGHTS = [1 1 1]
    end

    properties (Constant)
        SETTINGSTITLE = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintSettingsTitle'))

        POSXLABEL = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintXPosLabel'))
        POSXTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintXPosTooltip'))

        POSYLABEL = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintYPosLabel'))
        POSYTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintYPosTooltip'))

        POSZLABEL = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintZPosLabel'))
        POSZTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintZPosTooltip'))

        ROTXLABEL = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintXRotLabel'))
        ROTXTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintXRotTooltip'))

        ROTYLABEL = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintYRotLabel'))
        ROTYTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintYRotTooltip'))

        ROTZLABEL = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintZRotLabel'))
        ROTZTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintZRotTooltip'))

        XMINLABEL = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintXMinLabel'))
        XMINTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintXMinTooltip'))

        YMINLABEL = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintYMinLabel'))
        YMINTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintYMinTooltip'))

        ZMINLABEL = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintZMinLabel'))
        ZMINTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintZMinTooltip'))

        XMAXLABEL = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintXMaxLabel'))
        XMAXTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintXMaxTooltip'))

        YMAXLABEL = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintYMaxLabel'))
        YMAXTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintYMaxTooltip'))

        ZMAXLABEL = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintZMaxLabel'))
        ZMAXTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintZMaxTooltip'))

        XWEIGHTLABEL = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintXWeightLabel'))
        XWEIGHTTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintXWeightTooltip'))

        YWEIGHTLABEL = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintYWeightLabel'))
        YWEIGHTTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintYWeightTooltip'))

        ZWEIGHTLABEL = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintZWeightLabel'))
        ZWEIGHTTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:CartesianConstraintZWeightTooltip'))
    end

    methods
        function obj = CartesianBoundsConstraintTab(toolstrip)
        %CartesianBoundsConstraintTab Constructor

            tabTag = robotics.ikdesigner.internal.constants.Toolstrip.CartesianBoundsConstraintTabTag;
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
            obj.TargetTransform = obj.DEFAULTTARGETTRANSFORM;
            obj.EndEffectorBodyDropdown.Value = obj.EndEffectorBodyDropdown.Items{end, 1};
            obj.ReferenceBodyDropdown.Value = obj.EndEffectorBodyDropdown.Items{1, 1};
            obj.Bounds = obj.DEFAULTBOUNDS;
            obj.ConstraintWeights = obj.DEFAULTCONSTRAINTWEIGHTS;
        end

        function tform = get.TargetTransform(obj)
        %get.TargetTransform Return a numerical target transform matrix from the six component edit fields

            tform = obj.retrieveTransformFromTab();
        end

        function set.TargetTransform(obj, tform)
        %set.TargetTransform Update the values of the six component edit fields from a single numerical matrix

            obj.displayTransformInTab(tform);
        end

        function boundsMatrix = get.Bounds(obj)
        %get.Bounds Return a numerical target bounds matrix from the six component edit fields

            lowerBound = [str2num(obj.XMinField.Value) str2num(obj.YMinField.Value) str2num(obj.ZMinField.Value)];
            upperBound = [str2num(obj.XMaxField.Value) str2num(obj.YMaxField.Value) str2num(obj.ZMaxField.Value)];
            boundsMatrix = [lowerBound(:) upperBound(:)];
        end

        function set.Bounds(obj, boundsMat)
        %set.Bounds Update the values of the six component edit fields from a single numerical matrix

            obj.XMinField.Value = num2str(boundsMat(1,1));
            obj.YMinField.Value = num2str(boundsMat(2,1));
            obj.ZMinField.Value = num2str(boundsMat(3,1));
            obj.XMaxField.Value = num2str(boundsMat(1,2));
            obj.YMaxField.Value = num2str(boundsMat(2,2));
            obj.ZMaxField.Value = num2str(boundsMat(3,2));
        end

        function wt = get.ConstraintWeights(obj)
        %get.ConstraintWeights Get a numerical vector from the edit fields

            wt = [str2num(obj.XWeightField.Value) str2num(obj.YWeightField.Value) str2num(obj.ZWeightField.Value)];

        end

        function set.ConstraintWeights(obj, wt)
        %set.ConstraintWeights Translate a vector to the edit field strings

            obj.XWeightField.Value = num2str(wt(1));
            obj.YWeightField.Value = num2str(wt(2));
            obj.ZWeightField.Value = num2str(wt(3));

        end

        function populateTabFieldsFromDataStruct(obj, eventDataStruct)
        %populateTabFieldsFromDataStruct Populate the tab with values from a struct
        %   When constraint data is passed between the view and model,
        %   the constraint properties are stored in a structure that is
        %   unique to that constraint type. This method populates the
        %   user-facing edit fields with values from that structure.
        %   This is needed e.g. to populate a tab for edit.

            obj.EndEffectorBodyDropdown.Value = eventDataStruct.EEBodyKey;
            obj.ReferenceBodyDropdown.Value = eventDataStruct.RefBodyKey;
            obj.TargetTransform = eventDataStruct.TargetTransform;
            obj.Bounds = eventDataStruct.Bounds;
            obj.ConstraintWeights = eventDataStruct.Weights;

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
            constraintType = robotics.ikdesigner.internal.toolstrip.ConstraintType.Cartesian;
            constraintData = obj.populateDataStruct();
            constraintDataEvent = robotics.ikdesigner.internal.event.SolverConstraintEventData(constraintKey, constraintName, constraintType, constraintData);
        end
    end

    methods (Access = protected)
        function addSettingsSection(obj)
        %addSettingsSection Add the settings section to the tab

            obj.SettingsSection = matlab.ui.internal.toolstrip.Section(obj.SETTINGSTITLE);
            obj.SettingsSection.Tag = "SettingsSection";

            [obj.EndEffectorBodyDropdown, obj.ReferenceBodyDropdown] = obj.addBodyDropdowns();
            obj.addTargetPositionFields();
            obj.addTargetOrientationFields();
            obj.addLowerBoundFields();
            obj.addUpperBoundFields();
            obj.addWeightFields();

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

        function isValid = validateAndSetTransformFields(obj, src, evt, fieldProp, fieldName)
            %validateAndSetTransformFields Validation for pose transform fields

            isValid = obj.validateAndSetNumericInput(src, evt, fieldProp, fieldName, {'nonempty',  'nonnan', 'finite', 'scalar', 'real'});
            if isValid
                obj.updateConstraintVisual(robotics.ikdesigner.internal.toolstrip.ConstraintUpdate.DataUpdate);
            end

        end

        function isValid = validateAndSetMinBoundField(obj, src, evt, minField, minFieldLabel, maxField)
            %validateAndSetMinField Validation for lower bounds
            %   In addition to general validation, the lower bound must
            %   always be less than or equal to the corresponding upper
            %   bound.

            maxBoundValue = str2num(maxField.Value);
            isValid = obj.validateAndSetNumericInput(src, evt, minField, minFieldLabel, {'nonempty', 'real',  'nonnan', 'finite', 'scalar', '<', maxBoundValue});
            if isValid
                obj.updateConstraintVisual(robotics.ikdesigner.internal.toolstrip.ConstraintUpdate.DataUpdate);
            end

        end

        function isValid = validateAndSetMaxBoundField(obj, src, evt, maxField, maxFieldLabel, minField)
            %validateAndSetMinField Validation for lower bounds
            %   In addition to general validation, the upper bound must
            %   always be greater than or equal to the corresponding lower
            %   bound.

            minBoundValue = str2num(minField.Value);
            isValid = obj.validateAndSetNumericInput(src, evt, maxField, maxFieldLabel, {'nonempty', 'real',  'nonnan', 'finite', 'scalar', '>', minBoundValue});
            if isValid
                obj.updateConstraintVisual(robotics.ikdesigner.internal.toolstrip.ConstraintUpdate.DataUpdate);
            end

        end

        function constraintVisualsEvent = populateConstraintUpdateEvent(obj, updateType)
        %populateConstraintUpdateEvent Populate the event used to update the constraint overlay

            constraintVisualsEvent = robotics.ikdesigner.internal.event.CartesianConstraintDisplayEvent(updateType);

            % Update the visual indicators
            constraintVisualsEvent.TargetTransform = obj.TargetTransform;
            constraintVisualsEvent.Bounds = obj.Bounds;
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
            dataStruct.TargetTransform = obj.TargetTransform;
            dataStruct.Bounds = obj.Bounds;
            dataStruct.Weights = obj.ConstraintWeights;
            dataStruct.EEBodyKey = obj.EndEffectorBodyDropdown.Value;
            dataStruct.RefBodyKey = obj.ReferenceBodyDropdown.Value;
        end

        function addTargetPositionFields(obj)
        %addTargetPositionFields Add fields that let user set the position component of the target transform

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
        %addTargetOrientationFields Add fields that let user set the orientation component of the target transform

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

        function addLowerBoundFields(obj)
        %addLowerBoundFields Add fields the let the user set the lower bound on the constraint bounds

        % The second column contains the target position data. Again,
        % the labels are in col2a, and the field in column 2b
            colA = obj.SettingsSection.addColumn();
            colB = obj.SettingsSection.addColumn('Width', 75);


            label = matlab.ui.internal.toolstrip.Label(obj.XMINLABEL);
            label.Tag = "XMinLabel";
            label.Description = obj.XMINTOOLTIP;
            colA.add(label);

            obj.XMinField = matlab.ui.internal.toolstrip.EditField('0');
            obj.XMinField.Tag = "XMin";
            obj.XMinField.Description = label.Description;
            obj.XMinField.ValueChangedFcn = @(src,evt)obj.validateAndSetMinBoundField(src,evt,obj.XMinField,obj.XMINLABEL,obj.XMaxField);
            colB.add(obj.XMinField);

            label = matlab.ui.internal.toolstrip.Label(obj.YMINLABEL);
            label.Tag = "YMinLabel";
            label.Description = obj.YMINTOOLTIP;
            colA.add(label);

            obj.YMinField = matlab.ui.internal.toolstrip.EditField('0');
            obj.YMinField.Tag = "YMin";
            obj.YMinField.Description = label.Description;
            obj.YMinField.ValueChangedFcn = @(src,evt)obj.validateAndSetMinBoundField(src,evt,obj.YMinField,obj.YMINLABEL,obj.YMaxField);
            colB.add(obj.YMinField);

            label = matlab.ui.internal.toolstrip.Label(obj.ZMINLABEL);
            label.Tag = "ZMinLabel";
            label.Description = obj.ZMINTOOLTIP;
            colA.add(label);

            obj.ZMinField = matlab.ui.internal.toolstrip.EditField('0');
            obj.ZMinField.Tag = "ZMin";
            obj.ZMinField.Description = label.Description;
            obj.ZMinField.ValueChangedFcn = @(src,evt)obj.validateAndSetMinBoundField(src,evt,obj.ZMinField,obj.ZMINLABEL,obj.ZMaxField);
            colB.add(obj.ZMinField);
        end

        function addUpperBoundFields(obj)
        %addUpperBoundFields Add fields the let the user set the upper bound on the constraint bounds

        % The second column contains the target position data. Again,
        % the labels are in col2a, and the field in column 2b
            colA = obj.SettingsSection.addColumn();
            colB = obj.SettingsSection.addColumn('Width', 75);


            label = matlab.ui.internal.toolstrip.Label(obj.XMAXLABEL);
            label.Tag = "XMaxLabel";
            label.Description = obj.XMAXTOOLTIP;
            colA.add(label);

            obj.XMaxField = matlab.ui.internal.toolstrip.EditField('0');
            obj.XMaxField.Tag = "XMax";
            obj.XMaxField.Description = label.Description;
            obj.XMaxField.ValueChangedFcn = @(src,evt)obj.validateAndSetMaxBoundField(src,evt,obj.XMaxField,obj.XMAXLABEL,obj.XMinField);
            colB.add(obj.XMaxField);

            label = matlab.ui.internal.toolstrip.Label(obj.YMAXLABEL);
            label.Tag = "YMaxLabel";
            label.Description = obj.YMAXTOOLTIP;
            colA.add(label);

            obj.YMaxField = matlab.ui.internal.toolstrip.EditField('0');
            obj.YMaxField.Tag = "YMax";
            obj.YMaxField.Description = label.Description;
            obj.YMaxField.ValueChangedFcn = @(src,evt)obj.validateAndSetMaxBoundField(src,evt,obj.YMaxField,obj.YMAXLABEL,obj.YMinField);
            colB.add(obj.YMaxField);

            label = matlab.ui.internal.toolstrip.Label(obj.ZMAXLABEL);
            label.Tag = "ZMaxLabel";
            label.Description = obj.ZMAXTOOLTIP;
            colA.add(label);

            obj.ZMaxField = matlab.ui.internal.toolstrip.EditField('0');
            obj.ZMaxField.Tag = "ZMax";
            obj.ZMaxField.Description = label.Description;
            obj.ZMaxField.ValueChangedFcn = @(src,evt)obj.validateAndSetMaxBoundField(src,evt,obj.ZMaxField,obj.ZMAXLABEL,obj.ZMinField);
            colB.add(obj.ZMaxField);
        end

        function addWeightFields(obj)
        %addWeightFields Add fields that let the user set the weights on the constraint bounds

        % The last column has the tolerances
            colA = obj.SettingsSection.addColumn();
            colB = obj.SettingsSection.addColumn('Width', 50);

            label = matlab.ui.internal.toolstrip.Label(obj.XWEIGHTLABEL);
            label.Tag = "XWeightLabel";
            label.Description = obj.XWEIGHTTOOLTIP;
            colA.add(label);

            obj.XWeightField = matlab.ui.internal.toolstrip.EditField('0');
            obj.XWeightField.Tag = "XWeight";
            obj.XWeightField.Description = label.Description;
            obj.XWeightField.ValueChangedFcn = @(src,evt)obj.validateAndSetWeightFields(src, evt, obj.XWeightField, obj.XWEIGHTLABEL);
            colB.add(obj.XWeightField);

            label = matlab.ui.internal.toolstrip.Label(obj.YWEIGHTLABEL);
            label.Tag = "YWeightLabel";
            label.Description = obj.YWEIGHTTOOLTIP;
            colA.add(label);

            obj.YWeightField = matlab.ui.internal.toolstrip.EditField('1');
            obj.YWeightField.Tag = "YWeight";
            obj.YWeightField.Description = label.Description;
            obj.YWeightField.ValueChangedFcn = @(src,evt)obj.validateAndSetWeightFields(src, evt, obj.YWeightField, obj.YWEIGHTLABEL);
            colB.add(obj.YWeightField);

            label = matlab.ui.internal.toolstrip.Label(obj.ZWEIGHTLABEL);
            label.Tag = "ZWeightLabel";
            label.Description = obj.ZWEIGHTTOOLTIP;
            colA.add(label);

            obj.ZWeightField = matlab.ui.internal.toolstrip.EditField('1');
            obj.ZWeightField.Tag = "ZWeight";
            obj.ZWeightField.Description = label.Description;
            obj.ZWeightField.ValueChangedFcn = @(src,evt)obj.validateAndSetWeightFields(src, evt, obj.ZWeightField, obj.ZWEIGHTLABEL);
            colB.add(obj.ZWeightField);
        end
    end

end

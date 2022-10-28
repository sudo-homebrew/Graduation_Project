classdef JointBoundsConstraintTab < robotics.ikdesigner.internal.toolstrip.ConstraintTab
%This class is for internal use only. It may be removed in the future.

%JointBoundsConstraintTab Tab view that for the joint bounds constraint in the inverse kinematics designer toolstrip

%   Copyright 2021-2022 The MathWorks, Inc.

    properties (SetAccess = {?robotics.ikdesigner.internal.view.View, ?matlab.unittest.TestCase})

        %ConfigurationView Object used to convert between internal and user-facing joint configuration representation
        ConfigurationView

        %MinJtValuesField Field for entering the vector of minimum joint values
        MinJtValuesField

        %MaxJtValuesField Field for entering the vector of maximum joint values
        MaxJtValuesField

        %WeightsField Field for entering the vector of weights
        WeightsField
    end

    properties (Dependent)
        %Bounds
        Bounds

        %Weights
        Weights

        %NumDoF Number of degrees of freedom
        NumDoF
    end

    properties (Constant)
        SETTINGSSECTIONTITLE = string(message('robotics:ikdesigner:constrainttabviews:JointBoundsConstraintSettingsTitle'))

        MINJTBOUNDSLABEL = string(message('robotics:ikdesigner:constrainttabviews:JointBoundsConstraintLowerBoundLabel'))
        MINTJTBOUNDSTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:JointBoundsConstraintLowerBoundTooltip'))

        MAXJTBOUNDSLABEL = string(message('robotics:ikdesigner:constrainttabviews:JointBoundsConstraintUpperBoundLabel'))
        MAXJTBOUNDSTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:JointBoundsConstraintUpperBoundTooltip'))

        JTBOUNDSWEIGHTSLABEL = string(message('robotics:ikdesigner:constrainttabviews:JointBoundsConstraintWeightsLabel'))
        JTBOUNDSWEIGHTSTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:JointBoundsConstraintWeightsTooltip'))

        INCREASINGJOINTBOUNDSERRSTRING = string(message('robotics:ikdesigner:constrainttabviews:IncreasingJointBoundsError'))
    end

    methods
        function obj = JointBoundsConstraintTab(toolstrip)
        %JointBoundsConstraintTab Constructor

            tabTag = robotics.ikdesigner.internal.constants.Toolstrip.JointBoundsConstraintTabTag;
            obj@robotics.ikdesigner.internal.toolstrip.ConstraintTab(toolstrip, tabTag)

        end

        function setup(obj, robot)
        %setup Set up the tab with new session data

            % Initialize default parameters based on the robot dimension
            jointConfigAngularIdx = robotics.manip.internal.RigidBodyTreeUtils.identifyRevoluteJoint(robot);
            obj.ConfigurationView = robotics.ikdesigner.internal.view.ConfigurationView(jointConfigAngularIdx);
            obj.assignDefaultParameterValues;

        end

        function show(obj)
        %show Extension to the parent method

            show@robotics.ikdesigner.internal.toolstrip.ConstraintTab(obj)
            obj.ConstraintName.Enabled = true;
        end

        function resetToDefaultState(obj)
        %resetToDefaultState Reset the user-facing UI components to their default values

            obj.ApplyButton.Enabled = true;
            obj.assignDefaultParameterValues;
        end

        function numDoF = get.NumDoF(obj)
        %get.NumDoF

            numDoF = numel(obj.ConfigurationView.IsAngularValue);

        end

        function jtBounds = get.Bounds(obj)
        %get.MinJointValues

            lowerJtBounds = obj.ConfigurationView.getInternalConfig(obj.MinJtValuesField.Value);
            upperJtBounds = obj.ConfigurationView.getInternalConfig(obj.MaxJtValuesField.Value);
            jtBounds = [lowerJtBounds(:) upperJtBounds(:)];

        end

        function set.Bounds(obj, jtBounds)
        %set.MinJointValues

            obj.MinJtValuesField.Value = obj.ConfigurationView.getUserFacingConfigString(jtBounds(:,1));
            obj.MaxJtValuesField.Value = obj.ConfigurationView.getUserFacingConfigString(jtBounds(:,2));

        end

        function weights = get.Weights(obj)
        %get.Weights

            weights = str2num(obj.WeightsField.Value); %#ok<ST2NM> 

        end

        function set.Weights(obj, weights)
        %set.Weights

            obj.WeightsField.Value = "[" + num2str(weights) + "]";

        end

        function populateTabFieldsFromDataStruct(obj, eventDataStruct)
        %populateTabFieldsFromDataStruct Populate the tab with values from a struct
        %   When constraint data is passed between the view and model,
        %   the constraint properties are stored in a structure that is
        %   unique to that constraint type. This method populates the
        %   user-facing edit fields with values from that structure.
        %   This is needed e.g. to populate a tab for edit.

            obj.Bounds = eventDataStruct.Bounds;
            obj.Weights = eventDataStruct.Weights;

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
            constraintType = robotics.ikdesigner.internal.toolstrip.ConstraintType.JointBounds;
            constraintData = obj.populateDataStruct();
            constraintDataEvent = robotics.ikdesigner.internal.event.SolverConstraintEventData(constraintKey, constraintName, constraintType, constraintData);
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
            dataStruct.Bounds = obj.Bounds;
            dataStruct.Weights = obj.Weights;
        end
    end

    methods (Access = protected)
        function addSettingsSection(obj)
        %addSettingsSection Add settings section to the tab

            obj.createJointBoundsConstraintSettings();
            obj.TabHandle.add(obj.SettingsSection);

        end
    end

    methods (Access = private)

        function isValid = validateAndSetMaxJointBounds(obj, src, evt)
            %validateAndSetMaxJointBounds Validation for upper joint limits

            isValid = obj.validateAndSetNumericInput(src, evt, obj.MaxJtValuesField, obj.MAXJTBOUNDSLABEL, {'nonempty',  'nonnan', 'finite', 'vector', 'ncols', obj.NumDoF});
            if isValid
                areValuesValid = obj.validateBounds();
                if areValuesValid
                    obj.updateConstraintVisual(robotics.ikdesigner.internal.toolstrip.ConstraintUpdate.DataUpdate);
                else
                    obj.MaxJtValuesField.Value = evt.EventData.OldValue;
                end
            end
        end

        function isValid = validateAndSetMinJointBounds(obj, src, evt)
            %validateAndSetMaxJointBounds Validation for lower joint limits

            isValid = obj.validateAndSetNumericInput(src, evt, obj.MinJtValuesField, obj.MINJTBOUNDSLABEL, {'nonempty',  'nonnan', 'finite', 'vector', 'ncols', obj.NumDoF});
            if isValid
                areValuesValid = obj.validateBounds();
                if areValuesValid
                    obj.updateConstraintVisual(robotics.ikdesigner.internal.toolstrip.ConstraintUpdate.DataUpdate);
                else
                    obj.MinJtValuesField.Value = evt.EventData.OldValue;
                end
            end

        end

        function isValid = validateAndSetJointBoundWeights(obj, src, evt)
            %validateAndSetMaxJointBounds Validation for weights on joint limits

            isValid = obj.validateAndSetNumericInput(src, evt, obj.WeightsField, obj.JTBOUNDSWEIGHTSLABEL, {'nonempty',  'nonnan', 'finite', 'nonnegative', 'vector', 'ncols', obj.NumDoF});
            if isValid
                obj.enforceConstraintEditedActions();
            end
        end

        function isValid = validateBounds(obj)
            %validateBounds Ensure bounds are valid for constraint
            
            try
                validateattributes(obj.Bounds', {'numeric'}, {'nondecreasing'});
                isValid = true;
            catch
                isValid = false;
                obj.uialert(obj.INCREASINGJOINTBOUNDSERRSTRING)
            end
        end

        function createJointBoundsConstraintSettings(obj)
        %createJointBoundsConstraintSettings Create the settings for joint bounds constraint

            obj.SettingsSection = matlab.ui.internal.toolstrip.Section(obj.SETTINGSSECTIONTITLE);
            obj.SettingsSection.Tag = "SettingsSection";

            obj.addJointBoundFields();

        end

        function assignDefaultParameterValues(obj)
        %assignDefaultParameterValues Assign default values to the settings

            obj.Bounds = [-pi*ones(obj.NumDoF,1) pi*ones(obj.NumDoF,1)];
            obj.Weights = ones(1,obj.NumDoF);

        end

        function addJointBoundFields(obj)
        %addJointBoundFields Add fields that let users define joint bounds and associated weights

        % Create two columns, with the first used for labels and the
        % second for edit fields

            colA = obj.SettingsSection.addColumn();
            colB = obj.SettingsSection.addColumn('Width', 200);

            label = matlab.ui.internal.toolstrip.Label(obj.MAXJTBOUNDSLABEL);
            label.Tag = "MaxJtBoundLabel";
            label.Description = obj.MAXJTBOUNDSTOOLTIP;
            colA.add(label);

            obj.MaxJtValuesField = matlab.ui.internal.toolstrip.EditField('0');
            obj.MaxJtValuesField.Tag = "MaxJtBound";
            obj.MaxJtValuesField.Description = obj.MAXJTBOUNDSTOOLTIP;
            obj.MaxJtValuesField.ValueChangedFcn = @(src,evt)obj.validateAndSetMaxJointBounds(src,evt);
            colB.add(obj.MaxJtValuesField);

            label = matlab.ui.internal.toolstrip.Label(obj.MINJTBOUNDSLABEL);
            label.Tag = "MinJtBoundLabel";
            label.Description = obj.MINJTBOUNDSLABEL;
            colA.add(label);

            obj.MinJtValuesField = matlab.ui.internal.toolstrip.EditField('0');
            obj.MinJtValuesField.Tag = "MinJtBound";
            obj.MinJtValuesField.Description = obj.MINJTBOUNDSLABEL;
            obj.MinJtValuesField.ValueChangedFcn = @(src,evt)obj.validateAndSetMinJointBounds(src,evt);
            colB.add(obj.MinJtValuesField);

            label = matlab.ui.internal.toolstrip.Label(obj.JTBOUNDSWEIGHTSLABEL);
            label.Tag = "WeightsLabel";
            label.Description = obj.JTBOUNDSWEIGHTSTOOLTIP;
            colA.add(label);

            obj.WeightsField = matlab.ui.internal.toolstrip.EditField('0');
            obj.WeightsField.Tag = "Weights";
            obj.WeightsField.Description = obj.JTBOUNDSWEIGHTSTOOLTIP;
            obj.WeightsField.ValueChangedFcn = @(src,evt)obj.validateAndSetJointBoundWeights(src,evt);
            colB.add(obj.WeightsField);
        end
    end

end

classdef ConstraintTab < robotics.ikdesigner.internal.toolstrip.Tab
%This class is for internal use only. It may be removed in the future.

%ConstraintTab Tab view that for the generic constraint tab in the inverse kinematics designer toolstrip
%   This class creates a default constraint object, i.e. one with
%   standard views and no settings, since settings are specific to
%   particular constraints. This class is also a superclass from which
%   the specific settable constraints are derived.

%   Copyright 2021-2022 The MathWorks, Inc.

    properties (SetAccess = {?robotics.ikdesigner.internal.view.View, ?matlab.unittest.TestCase})

        %ConstraintSection Constraint selection section
        ConstraintSection

        %ConstraintName
        ConstraintName

        %ConstraintGallery
        ConstraintGallery

        %AimingConstraintButton Button in the constraint gallery
        AimingConstraintButton

        %JointBoundsConstraintButton Button in the constraint gallery
        JointBoundsConstraintButton

        %PoseConstraintButton Button in the constraint gallery
        PoseConstraintButton

        %CartesianConstraintButton Button in the constraint gallery
        CartesianConstraintButton

        %CloseSection Close section
        CloseSection

        %ApplyButton
        ApplyButton

        %CloseButton
        CloseButton
    end

    properties (Constant)
        AIMINGICON = fullfile(robotics.ikdesigner.internal.constants.Data.CUSTOMICONSDIRECTORY, 'aiming.png')
        CARTESIANICON = fullfile(robotics.ikdesigner.internal.constants.Data.CUSTOMICONSDIRECTORY, 'cartesian.png')
        JOINTBOUNDSICON = fullfile(robotics.ikdesigner.internal.constants.Data.CUSTOMICONSDIRECTORY, 'jointBounds.png')
        POSEICON = fullfile(robotics.ikdesigner.internal.constants.Data.CUSTOMICONSDIRECTORY, 'pose.png')

        TABTITLE = string(message('robotics:ikdesigner:constrainttabviews:TabTitle'))
    end

    properties (Constant)

        CONSTRAINTSECTIONLABEL = string(message('robotics:ikdesigner:constrainttabviews:ConstraintSectionTitle'))

        CLOSESECTIONLABEL = string(message('robotics:ikdesigner:constrainttabviews:CloseSectionTitle'))

        BUILTINCONSTRAINTSLABEL = string(message('robotics:ikdesigner:constrainttabviews:GalleryBuiltInConstraintsLabel'))

        REFBODYLABEL = string(message('robotics:ikdesigner:constrainttabviews:RefBodyLabel'))
        REFBODYTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:RefBodyTooltip'))

        EEBODYLABEL = string(message('robotics:ikdesigner:constrainttabviews:EEBodyLabel'))
        EEBODYTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:EEBodyTooltip'))

        NAMELABEL = string(message('robotics:ikdesigner:constrainttabviews:ConstraintNameLabel'))
        NAMETOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:ConstraintNameTooltip'))

        APPLYBUTTONLABEL = string(message('robotics:ikdesigner:constrainttabviews:ApplyButtonLabel'))
        APPLYBUTTONTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:ApplyButtonTooltip'))

        CLOSEBUTTONLABEL = string(message('robotics:ikdesigner:constrainttabviews:CloseButtonLabel'))
        CLOSEBUTTONTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:CloseButtonTooltip'))

        POSEBUTTONLABEL = string(message('robotics:ikdesigner:constrainttabviews:PoseGalleryButtonLabel'))
        POSEBUTTONTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:PoseGalleryButtonTooltip'))

        CARTESIANBUTTONLABEL = string(message('robotics:ikdesigner:constrainttabviews:CartesianGalleryButtonLabel'))
        CARTESIANBUTTONTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:CartesianGalleryButtonTooltip'))

        AIMINGBUTTONLABEL = string(message('robotics:ikdesigner:constrainttabviews:AimingGalleryButtonLabel'))
        AIMINGBUTTONTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:AimingGalleryButtonTooltip'))

        JOINTBOUNDSBUTTONLABEL = string(message('robotics:ikdesigner:constrainttabviews:JointBoundsGalleryButtonLabel'))
        JOINTBOUNDSBUTTONTOOLTIP = string(message('robotics:ikdesigner:constrainttabviews:JointBoundsGalleryButtonTooltip'))
    end

    properties (Access = protected)

        %SettingsSection Constraint settings section
        SettingsSection

    end

    properties (Dependent)
        % Properties that are shared across constraint tabs

        %ActiveConstraintName Name of the active constraint
        ActiveConstraintName

        %ActiveConstraintKey Key associated with the active constraint
        ActiveConstraintKey

    end

    methods (Access = protected)
        function addSettingsSection(~)
        %addSettingsSection Add setting section
        %   This is a no-op method in the default case, since the
        %   default constraint tab has no displayed settings.

        end
    end

    methods
        function obj = ConstraintTab(toolstrip, tabTag)
        %ConstraintTab Constructor

        % Construct the tab and associate it with the toolstrip
            tabTitle = robotics.ikdesigner.internal.toolstrip.ConstraintTab.TABTITLE;
            obj@robotics.ikdesigner.internal.toolstrip.Tab(toolstrip, tabTitle, tabTag);

            % Add the sections
            obj.addConstraintSection();
            obj.addSettingsSection();
            obj.addCloseSection();

            % Editing the constraint name should enforce constraint edit rules
            obj.ConstraintName.ValueChangedFcn = @(src,evt)obj.enforceConstraintEditedActions();

            % Move tab to initial state when app is loaded
            obj.disableAll();
        end

        function initialize(obj)

        % Enable all except the solver button
            obj.enableAll;
            obj.ApplyButton.Enabled = false;

        end

        function show(obj)
        %show Extension to the parent method
        %   When the constraint tab is displayed, the visual aids in
        %   the scene canvas must be made apparent. This method extends
        %   the default version in the superclass to add these
        %   elements.

            obj.ConstraintName.Value = obj.ActiveConstraintName;
            obj.ConstraintName.Enabled = false;

            show@robotics.ikdesigner.internal.toolstrip.Tab(obj)
        end

        function assignSpecificConstraintTab(obj, constraintTabTypeTag)
        %assignSpecificConstraintTab Populate constraint settings for the aiming constraint

            tabsToHide = {obj.TabHandle.Tag};
            tabsToShow = {constraintTabTypeTag};
            tabToSelect = constraintTabTypeTag;
            isVisibleTabAConstraint = true;
            obj.updateToolstripTabDisplay(tabsToHide, tabsToShow, tabToSelect, isVisibleTabAConstraint);
        end

        function key = get.ActiveConstraintKey(obj)
        %get.ActiveConstraintKey Return the stored active constraint key

        %TODO: (g2570347) Encapsulate constraint tab content stored in toolstrip view
            key = obj.Parent.ActiveConstraintKey;
        end

        function set.ActiveConstraintKey(obj, key)
        %set.ActiveConstraintKey Set the stored active constraint key

        %TODO: (g2570347) Encapsulate constraint tab content stored in toolstrip view
            obj.Parent.ActiveConstraintKey = key;
        end

        function name = get.ActiveConstraintName(obj)
        %get.ActiveConstraintName Return the stored active constraint name

        %TODO: (g2570347) Encapsulate constraint tab content stored in toolstrip view
            name = obj.Parent.ActiveConstraintName;
        end

        function set.ActiveConstraintName(obj, name)
        %set.ActiveConstraintName Set the stored active constraint name

        %TODO: (g2570347) Encapsulate constraint tab content stored in toolstrip view
            obj.Parent.ActiveConstraintName = name;
        end
    end

    methods (Access = protected)
        function notifyConstraintApplied(obj)
        %notifyConstraintApplied Notify the controller that a constraint has been applied

            constraintDataEvent = obj.populateConstraintDataEvent;
            obj.sendNotification("ConstraintEdited", constraintDataEvent);

            % When the constraint is applied, the apply button should be
            % disabled until further changes are incurred
            obj.ApplyButton.Enabled = false;
        end

        function updateConstraintVisual(obj, updateType)
        %updateConstraintVisual Update the constraint tab visuals

        % When the update type is the result of the constraint being
        % edited, update the tab display to reflect this change.
            if updateType == robotics.ikdesigner.internal.toolstrip.ConstraintUpdate.DataUpdate ...
                    || updateType == robotics.ikdesigner.internal.toolstrip.ConstraintUpdate.RefBodySelection ...
                    || updateType == robotics.ikdesigner.internal.toolstrip.ConstraintUpdate.EEBodySelection
                obj.enforceConstraintEditedActions();
            end
        end

        function enforceConstraintEditedActions(obj)
        %enforceConstraintEditedActions Update the toolstrip once the constraint has been edited

            obj.ApplyButton.Enabled = true;
        end

        function isValid = validateAndSetToleranceFields(obj, src, evt, fieldProp, fieldName)
            %validateAndSetToleranceFields Validate inputs for constraint tolerances

            toleranceInputAttributes = {'nonempty',  'nonnan', 'finite', 'scalar', 'nonnegative'};
            isValid = obj.validateAndSetNumericInput(src, evt, fieldProp, fieldName, toleranceInputAttributes);
            if isValid
                obj.enforceConstraintEditedActions();
            end

        end

        function isValid = validateAndSetWeightFields(obj, src, evt, fieldProp, fieldName)
            %validateAndSetToleranceFields Validate inputs for constraint weights

            weightInputAttributes = {'nonempty',  'nonnan', 'finite', 'scalar', 'nonnegative'};
            isValid = obj.validateAndSetNumericInput(src, evt, fieldProp, fieldName, weightInputAttributes);
            if isValid
                obj.enforceConstraintEditedActions();
            end

        end

        function [eeDropdown, refBodyDropdown] = addBodyDropdowns(obj)
        %addBodyDropdowns Add drop-downs to the constraint tab
        %   This method adds reference and end effector body name
        %   dropdowns to the constraint. Since not all tabs have these
        %   fields, this method simply returns the dropdowns, which are
        %   then assigned in the tabs that have them as properties.

        % The first column contains the body information. The labels go
        % in column 1a, and the associated fields in 1b.
            colA = obj.SettingsSection.addColumn();
            colB = obj.SettingsSection.addColumn('Width', 100);

            % Add the end effector label
            label = matlab.ui.internal.toolstrip.Label(obj.EEBODYLABEL);
            label.Tag = "EELabel";
            label.Description = obj.EEBODYTOOLTIP;
            colA.add(label);

            % The end effector dropdown is initially unpopulated but will
            % be populated with marker body names during setup
            eeDropdown = matlab.ui.internal.toolstrip.DropDown({' '});
            eeDropdown.Tag = "EndEffectorDropdown";
            eeDropdown.Description = obj.EEBODYTOOLTIP;
            eeDropdown.Value = ' ';
            eeDropdown.ValueChangedFcn = @(evt,src)obj.updateConstraintVisual(robotics.ikdesigner.internal.toolstrip.ConstraintUpdate.EEBodySelection);
            colB.add(eeDropdown);

            % Add the reference body label
            label = matlab.ui.internal.toolstrip.Label(obj.REFBODYLABEL);
            label.Tag = "RefBodyLabel";
            label.Description = obj.REFBODYTOOLTIP;
            colA.add(label);

            % The reference body dropdown is initially unpopulated but will
            % be populated with marker body names during setup
            refBodyDropdown = matlab.ui.internal.toolstrip.DropDown({' '});
            refBodyDropdown.Tag = "ReferenceBodyDropdown";
            refBodyDropdown.Description = obj.REFBODYTOOLTIP;
            refBodyDropdown.Value = ' ';
            refBodyDropdown.ValueChangedFcn = @(evt,src)obj.updateConstraintVisual(robotics.ikdesigner.internal.toolstrip.ConstraintUpdate.RefBodySelection);
            colB.add(refBodyDropdown);
        end

        function displayTransformInTab(obj, tform)
            %displayTransformInTab Update the values of the six component edit fields from a single numerical matrix
            %   This is a shared method used by multiple derived constraint
            %   tab views. This necessitates that they have the expected
            %   property names.

            obj.XField.Value = num2str(tform(1,4));
            obj.YField.Value = num2str(tform(2,4));
            obj.ZField.Value = num2str(tform(3,4));

            eulVector = rad2deg(tform2eul(tform, robotics.ikdesigner.internal.constants.Data.EULERCONVENTIONORDER));
            obj.EulerXField.Value = num2str(eulVector(1));
            obj.EulerYField.Value = num2str(eulVector(2));
            obj.EulerZField.Value = num2str(eulVector(3));
        end

        function tform = retrieveTransformFromTab(obj)
        %retrieveTransformFromTab Return a numerical target transform matrix from the six component edit fields
            %   This is a shared method used by multiple derived constraint
            %   tab views. This necessitates that they have the expected
            %   property names. Use str2num because it is compatible with
            %   more common inputs and ensures ease of validation.

            eulerVector = [str2num(obj.EulerXField.Value) str2num(obj.EulerYField.Value) str2num(obj.EulerZField.Value)]; %#ok<ST2NM> 
            eulerVectorRad = deg2rad(eulerVector);
            posVector = [str2num(obj.XField.Value) str2num(obj.YField.Value) str2num(obj.ZField.Value)]; %#ok<ST2NM> 
            tform = [eul2rotm(eulerVectorRad, robotics.ikdesigner.internal.constants.Data.EULERCONVENTIONORDER) posVector(:); 0 0 0 1];
        end
    end

    methods (Access = private)
        function addConstraintSection(obj)
        %addConstraintSection Add the constraint section to the tab

            obj.ConstraintSection = matlab.ui.internal.toolstrip.Section(obj.CONSTRAINTSECTIONLABEL);
            obj.ConstraintSection.Tag = "ConstraintSection";

            col1 = obj.ConstraintSection.addColumn();
            label = matlab.ui.internal.toolstrip.Label(obj.NAMELABEL);
            label.Tag = "ConstraintNameLabel";
            label.Description = obj.NAMETOOLTIP;
            col1.add(label);

            col2 = obj.ConstraintSection.addColumn();
            obj.ConstraintName = matlab.ui.internal.toolstrip.EditField("Constraint1");
            obj.ConstraintName.Tag = "ConstraintName";
            obj.ConstraintName.Description = obj.NAMETOOLTIP;
            obj.ConstraintName.Enabled = false;
            col2.add(obj.ConstraintName);

            col3 = obj.ConstraintSection.addColumn();
            obj.createConstraintGallery();
            col3.add(obj.ConstraintGallery);

            obj.TabHandle.add(obj.ConstraintSection);
        end

        function addCloseSection(obj)
        %addCloseSection Add the Close section to the tab

            obj.CloseSection = matlab.ui.internal.toolstrip.Section(obj.CLOSESECTIONLABEL);
            obj.CloseSection.Tag = "CloseSection";
            obj.TabHandle.add(obj.CloseSection);

            % Add the Apply button, which applies the constraint and runs
            % the solver
            obj.ApplyButton = matlab.ui.internal.toolstrip.Button(obj.APPLYBUTTONLABEL, matlab.ui.internal.toolstrip.Icon.CONFIRM_24);
            obj.ApplyButton.Tag = "ApplyConstraintButton";
            obj.ApplyButton.Description = obj.APPLYBUTTONTOOLTIP;
            obj.ApplyButton.ButtonPushedFcn = @(src,evt)obj.notifyConstraintApplied();
            col = obj.CloseSection.addColumn();
            col.add(obj.ApplyButton);

            % Add the Close button, which returns to the initial modal view
            obj.CloseButton = matlab.ui.internal.toolstrip.Button(obj.CLOSEBUTTONLABEL, matlab.ui.internal.toolstrip.Icon.CLOSE_24);
            obj.CloseButton.Tag = "CloseConstraintButton";
            obj.CloseButton.Description = obj.CLOSEBUTTONTOOLTIP;
            obj.CloseButton.ButtonPushedFcn = @(src,evt)obj.updateToolstripMode(robotics.ikdesigner.internal.toolstrip.ModeEnum.DefaultMode);
            col = obj.CloseSection.addColumn();
            col.add(obj.CloseButton);
        end

        function createConstraintGallery(obj)
        %createConstraintGallery Create the gallery of constraints

            galleryPopup = matlab.ui.internal.toolstrip.GalleryPopup('GalleryItemTextLineCount', 2);
            galleryCategory = matlab.ui.internal.toolstrip.GalleryCategory(obj.BUILTINCONSTRAINTSLABEL);

            % Add the joint bounds constraint button to the gallery
            obj.PoseConstraintButton = matlab.ui.internal.toolstrip.GalleryItem(...
                obj.POSEBUTTONLABEL, ...
                obj.POSEICON);
            obj.PoseConstraintButton.Tag = "PoseConstraintButton";
            obj.PoseConstraintButton.Description = obj.POSEBUTTONTOOLTIP;
            obj.PoseConstraintButton.ItemPushedFcn = @(~,~)assignSpecificConstraintTab(obj, robotics.ikdesigner.internal.constants.Toolstrip.PoseTargetConstraintTabTag);

            % Add the joint bounds constraint button to the gallery
            obj.CartesianConstraintButton = matlab.ui.internal.toolstrip.GalleryItem(...
                obj.CARTESIANBUTTONLABEL, ...
                obj.CARTESIANICON);
            obj.CartesianConstraintButton.Tag = "CartesianConstraintButton";
            obj.CartesianConstraintButton.Description = obj.CARTESIANBUTTONTOOLTIP;
            obj.CartesianConstraintButton.ItemPushedFcn = @(~,~)assignSpecificConstraintTab(obj, robotics.ikdesigner.internal.constants.Toolstrip.CartesianBoundsConstraintTabTag);

            % Add the aiming constraint button to the gallery
            obj.AimingConstraintButton = matlab.ui.internal.toolstrip.GalleryItem(...
                obj.AIMINGBUTTONLABEL, ...
                obj.AIMINGICON);
            obj.AimingConstraintButton.Tag = "AimingConstraintButton";
            obj.AimingConstraintButton.Description = obj.AIMINGBUTTONTOOLTIP;
            obj.AimingConstraintButton.ItemPushedFcn = @(~,~)assignSpecificConstraintTab(obj, robotics.ikdesigner.internal.constants.Toolstrip.AimingConstraintTabTag);

            % Add the joint bounds constraint button to the gallery
            obj.JointBoundsConstraintButton = matlab.ui.internal.toolstrip.GalleryItem(...
                obj.JOINTBOUNDSBUTTONLABEL, ...
                obj.JOINTBOUNDSICON);
            obj.JointBoundsConstraintButton.Tag = "JointBoundsConstraintButton";
            obj.JointBoundsConstraintButton.Description = obj.JOINTBOUNDSBUTTONTOOLTIP;
            obj.JointBoundsConstraintButton.ItemPushedFcn = @(~,~)assignSpecificConstraintTab(obj, robotics.ikdesigner.internal.constants.Toolstrip.JointBoundsConstraintTabTag);

            galleryCategory.add(obj.PoseConstraintButton);
            galleryCategory.add(obj.CartesianConstraintButton);
            galleryCategory.add(obj.AimingConstraintButton);
            galleryCategory.add(obj.JointBoundsConstraintButton);
            galleryPopup.add(galleryCategory);

            obj.ConstraintGallery = matlab.ui.internal.toolstrip.Gallery(galleryPopup, 'MaxColumnCount', 3);
        end
    end

end

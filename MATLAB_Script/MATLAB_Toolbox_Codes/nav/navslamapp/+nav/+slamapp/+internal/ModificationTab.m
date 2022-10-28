classdef ModificationTab < handle & ...
        nav.slamapp.internal.mixin.ShowHideHelper & ...
        robotics.appscore.internal.mixin.MsgCatalogHelper
    %This class is for internal use only. It may be removed in the future.

    %MODIFICATIONTAB Tab for modifying the loop closure and incremental
    %   matches

    % Copyright 2018 The MathWorks, Inc.
    properties
        LinearButton

        AngularButton

        IgnoreButton

        ResetButton

        XSpinner

        YSpinner

        ThetaSpinner

        AcceptButton

        CancelButton
    end


    events
    ModificationTab_Linear

    ModificationTab_Angular

    ModificationTab_Ignore

    ModificationTab_Reset
end

methods
    function obj = ModificationTab(tabGroup)
    %MODIFICATIONTAB Constructor
        import matlab.ui.internal.toolstrip.*

        obj.MsgIDPrefix = 'nav:navslamapp:modificationtab';
        tabName = upper(obj.retrieveMsg('ModificationTabName'));
        tag = 'ModificationTab';

        obj.TabGroup = tabGroup;
        obj.Tab = Tab(tabName);
        obj.TabGroup.add(obj.Tab);
        obj.Tab.Tag = tag;

        obj.addIgnoreSection();
        obj.addModificationSection();
        obj.addRelativePoseSection();
        obj.addResetSection();
        obj.addCloseSection();
    end

    function addIgnoreSection(obj)
    %addIgnoreSection
        import matlab.ui.internal.toolstrip.*

        section = obj.Tab.addSection(upper(obj.retrieveMsg('IgnoreSectionName')));

        igGroup = ButtonGroup;

        column = section.addColumn();
        button = ToggleButton(obj.retrieveMsg('IgnoreButtonName'), Icon.DELETE_24, igGroup);
        button.Tag = 'Ignore';
        button.Description = obj.retrieveMsg('IgnoreButtonDescription');
        button.ValueChangedFcn =  @(src, evt) obj.ignoreButtonCallback(src, evt);
        obj.IgnoreButton = button;
        column.add(button);

    end

    function addModificationSection(obj)
    %addModificationSection
        import matlab.ui.internal.toolstrip.*
        import nav.slamapp.internal.CustomizedIcon

        section = obj.Tab.addSection(upper(obj.retrieveMsg('ModificationSectionName')));

        modGroup = ButtonGroup;

        column = section.addColumn();
        button = ToggleButton(obj.retrieveMsg('LinearButtonName'), CustomizedIcon.LINEAR_24, modGroup);
        button.Tag = 'Linear';
        button.Description = obj.retrieveMsg('LinearButtonDescription');
        button.ValueChangedFcn = @(src, evt) obj.linearButtonCallback(src, evt);
        obj.LinearButton = button;
        column.add(button);

        column = section.addColumn();
        button = ToggleButton(obj.retrieveMsg('AngularButtonName'), CustomizedIcon.ANGULAR_24, modGroup);
        button.Tag = 'Angular';
        button.Description = obj.retrieveMsg('AngularButtonDescription');
        button.ValueChangedFcn = @(src, evt) obj.angularButtonCallback(src, evt);
        obj.AngularButton = button;
        column.add(button);


    end

    function addRelativePoseSection(obj)
    %addRelativePoseSection
        import matlab.ui.internal.toolstrip.*

        section = obj.Tab.addSection(upper(obj.retrieveMsg('RelativePoseSectionName')));

        % labels
        column = section.addColumn();
        labelX = Label(obj.retrieveMsg('XLabelName'));
        labelY = Label(obj.retrieveMsg('YLabelName'));
        labelTheta = Label(obj.retrieveMsg('ThetaLabelName'));

        column.add(labelX);
        column.add(labelY);
        column.add(labelTheta);

        column = section.addColumn('width', 90);

        obj.XSpinner = Spinner([-100, 100], 0);
        obj.XSpinner.Tag = 'XSpinner';
        obj.XSpinner.NumberFormat = 'double';
        obj.XSpinner.StepSize = 0.01;
        obj.XSpinner.DecimalFormat = '4f';
        obj.XSpinner.Description = obj.retrieveMsg('XSpinnerDescription');

        obj.YSpinner = Spinner([-100, 100], 0);
        obj.YSpinner.Tag = 'YSpinner';
        obj.YSpinner.NumberFormat = 'double';
        obj.YSpinner.StepSize = 0.01;
        obj.YSpinner.DecimalFormat = '4f';
        obj.YSpinner.Description = obj.retrieveMsg('YSpinnerDescription');

        obj.ThetaSpinner = Spinner([-pi pi], 0);
        obj.ThetaSpinner.Tag = 'ThetaSpinner';
        obj.ThetaSpinner.NumberFormat = 'double';
        obj.ThetaSpinner.StepSize = 0.01;
        obj.ThetaSpinner.DecimalFormat = '4f';
        obj.ThetaSpinner.Description = obj.retrieveMsg('ThetaSpinnerDescription');

        column.add(obj.XSpinner);
        column.add(obj.YSpinner);
        column.add(obj.ThetaSpinner);
    end

    function addResetSection(obj)
    %addResetSection
        import matlab.ui.internal.toolstrip.*
        import nav.slamapp.internal.CustomizedIcon

        section = obj.Tab.addSection(upper(obj.retrieveMsg('ResetSectionName')));


        column = section.addColumn();
        button = Button(obj.retrieveMsg('ResetButtonName'), Icon.RESTORE_24);
        button.Tag = 'Reset';
        button.Description = obj.retrieveMsg('ResetButtonDescription');
        button.ButtonPushedFcn = @(src, evt) obj.resetButtonCallback;
        obj.ResetButton = button;
        column.add(button);


    end

    function addCloseSection(obj)
    %addCloseSection
        import matlab.ui.internal.toolstrip.*

        section = obj.Tab.addSection(upper(obj.retrieveMsg('CloseSectionName')));
        section.CollapsePriority = 10;
        %
        column = section.addColumn();
        button = Button(obj.retrieveMsg('AcceptButtonName'), Icon.CONFIRM_24); % accept and close
        button.Tag = 'Accept';
        button.Description = obj.retrieveMsg('AcceptButtonDescription');
        column.add(button);
        obj.AcceptButton = button;

        column = section.addColumn();
        button = Button(obj.retrieveMsg('CancelButtonName'), Icon.CLOSE_24); % cancel actions and close
        button.Tag = 'Cancel';
        button.Description = obj.retrieveMsg('CancelButtonDescription');
        column.add(button);
        obj.CancelButton = button;
    end
end

methods
    function updateLinearAngularToggleButtons(obj, v)
    %updateLinearAngularToggleButtons
        switch(v)
          case 0
            obj.LinearButton.Value = true;
          case 1
            obj.AngularButton.Value = true;
          otherwise
        end
    end

    function linearButtonCallback(obj, ~, evt)
    %linearButtonCallback
        if evt.EventData.NewValue == 1
            obj.notify('ModificationTab_Linear');
        end
    end

    function angularButtonCallback(obj, ~, evt)
    %angularButtonCallback
        if evt.EventData.NewValue == 1
            obj.notify('ModificationTab_Angular');
        end
    end

    function ignoreButtonCallback(obj, ~, evt)
    %ignoreButtonCallback
        if evt.EventData.NewValue == 1
            obj.notify('ModificationTab_Ignore');
        end
    end

    function resetButtonCallback(obj)
    %resetButtonCallback

        obj.notify('ModificationTab_Reset');

    end

    function updateToolstrip(obj, isIncremental, isIgnored)
    %updateToolstrip
        if isIncremental
            obj.LinearButton.Enabled = true;
            obj.AngularButton.Enabled = true;
            obj.XSpinner.Enabled = true;
            obj.YSpinner.Enabled = true;
            obj.ThetaSpinner.Enabled = true;
            obj.IgnoreButton.Enabled = false;
            obj.IgnoreButton.Value = false;

        elseif ~isIncremental && isIgnored
            obj.LinearButton.Enabled = false;
            obj.AngularButton.Enabled = false;
            obj.XSpinner.Enabled = false;
            obj.YSpinner.Enabled = false;
            obj.ThetaSpinner.Enabled = false;
            obj.IgnoreButton.Enabled = true;
            obj.IgnoreButton.Value = true;

        else
            obj.LinearButton.Enabled = true;
            obj.AngularButton.Enabled = true;
            obj.XSpinner.Enabled = true;
            obj.YSpinner.Enabled = true;
            obj.ThetaSpinner.Enabled = true;
            obj.IgnoreButton.Enabled = true;
            obj.IgnoreButton.Value = false;
        end
    end

    function title = getTitle(obj, num1, num2)
    %getTitle
        if nargin > 2
            title = upper(obj.retrieveMsg('LoopClosureTabNamePrefix', num1, num2));
        else
            title = upper(obj.retrieveMsg('IncrementalTabNamePrefix', num1-1, num1));
        end
    end
end

end

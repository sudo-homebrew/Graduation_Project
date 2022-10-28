classdef WorkspaceImporterTab < handle & ...
        nav.slamapp.internal.mixin.ShowHideHelper & ...
        robotics.appscore.internal.mixin.MsgCatalogHelper
    %This class is for internal use only. It may be removed in the future.

    %WORKSPACEIMPORTERTAB Modal tab for importing sensor data from
    %   workspace variables

    % Copyright 2020 The MathWorks, Inc.


    properties % toolstrip widgets

        RefreshWSButton
        
        SelectScansDropDown

        SelectPosesDropDown

        PercentageDropDown

        ApplyButton

        CloseButton
    end

    events % events emitted by WorkspaceImporterTab

        WorkspaceImporterTab_RefreshWorkspaceVariables_Request
        
        WorkspaceImporterTab_ScansSelectionUpdated
        
        WorkspaceImporterTab_PosesSelectionUpdated
        
        WorkspaceImporterTab_Apply
        
        WorkspaceImporterTab_Close

    end

    methods
        function obj = WorkspaceImporterTab(tabGroup)
        %BAGIMPORTERTAB Constructor

            import matlab.ui.internal.toolstrip.*

            obj.MsgIDPrefix = 'nav:navslamapp:wsimportertab';
            tabName = upper(obj.retrieveMsg('WorkspaceImporterTabName'));
            tag = 'WorkspaceImporterTab';

            obj.TabGroup = tabGroup;
            obj.Tab = Tab(tabName);
            obj.TabGroup.add(obj.Tab);
            obj.Tab.Tag = tag;

            % add sections
            obj.addWSRefreshButtonSection();
            obj.addSelectScansSection();
            obj.addSelectPosesSection();
            obj.addPercentageSection();
            obj.addCloseSection();
        end

        function addWSRefreshButtonSection(obj)
        %addWSRefreshButtonSection Add "refresh workspace" section
            import matlab.ui.internal.toolstrip.*
            
            section = obj.Tab.addSection(upper(obj.retrieveMsg('RefreshWSSectionName')));
            column = section.addColumn();
            
            button = Button(obj.retrieveMsg('RefreshWSButtonName'), Icon.REFRESH_24);
            button.Description = obj.retrieveMsg('RefreshWSButtonDescription');
            button.Tag = 'RefreshWS';
            
            button.ButtonPushedFcn = @(src, evt) obj.refreshWSCallback(src, evt);
            column.add(button);
            obj.RefreshWSButton = button;
        end
        
        function addSelectScansSection(obj)
        %addSelectScansSection Add "select scans" section
            import matlab.ui.internal.toolstrip.*
            section = obj.Tab.addSection(upper(obj.retrieveMsg('ScansSectionName')));

            column = section.addColumn('width', 200);

            % labels
            scansLabel = Label(obj.retrieveMsg('ScansLabelName'));
            column.add(scansLabel);

            % dropdown
            selectScansDropDown = DropDown({''});
            selectScansDropDown.Tag = 'SelectScansDropDown';
            selectScansDropDown.Description = obj.retrieveMsg('SelectScansDropDownDescription');
            selectScansDropDown.ValueChangedFcn = @(src, evt) obj.selectScansDropDownCallback(src, evt);
            obj.SelectScansDropDown = selectScansDropDown;
            

            column.add(selectScansDropDown);

        end
        
        function addSelectPosesSection(obj)
            %addSelectPosesSection Add "select poses" section
            
            import matlab.ui.internal.toolstrip.*
            section = obj.Tab.addSection(upper(obj.retrieveMsg('PosesSectionName')));
            column = section.addColumn('width', 200);

            % labels
            PosesLabel = Label(obj.retrieveMsg('PosesLabelName'));
            column.add(PosesLabel);

            % dropdown
            selectPosesDropDown = DropDown({''});
            selectPosesDropDown.Tag = 'SelectPosesDropDown';
            selectPosesDropDown.Description = obj.retrieveMsg('SelectPosesDropDownDescription');
            selectPosesDropDown.ValueChangedFcn = @(src, evt) obj.selectPosesDropDownCallback(src, evt);
            obj.SelectPosesDropDown = selectPosesDropDown;

            column.add(selectPosesDropDown);           
        end

        function addPercentageSection(obj)
        %addPercentageSection Add "percentage" section
            import matlab.ui.internal.toolstrip.*

            section = obj.Tab.addSection(upper(obj.retrieveMsg('DataDownsamplingSectionName')));

            % labels and spinners for percentage selection
            column = section.addColumn();
            label1 = Label(obj.retrieveMsg('KeepLabelName1'));
            dropDownPercentage = DropDown({''});
            
            dropDownPercentage.Tag = 'PercentageDropDown';
            dropDownPercentage.Description = obj.retrieveMsg('KeepDropDownDescription');
            obj.PercentageDropDown = dropDownPercentage;

            column.add(label1);
            column.add(dropDownPercentage);

        end

        function addCloseSection(obj)
        %addCloseSection
            import matlab.ui.internal.toolstrip.*

            section = obj.Tab.addSection(upper(obj.retrieveMsg('CloseSectionName')));
            section.CollapsePriority = 10;
            %
            column = section.addColumn();
            button = Button(obj.retrieveMsg('ApplyButtonName'), Icon.CONFIRM_24); % accept and close
            button.Tag = 'WSImporterApply';
            button.Description = obj.retrieveMsg('ApplyButtonDescription');
            button.ButtonPushedFcn = @(src, evt) obj.applyCallback(src, evt);
            column.add(button);
            obj.ApplyButton = button;

            column = section.addColumn();
            button = Button(obj.retrieveMsg('CloseButtonName'), Icon.CLOSE_24); % cancel actions and close
            button.Tag = 'WSImporterClose';
            button.Description = obj.retrieveMsg('CloseButtonDescription');
            button.ButtonPushedFcn = @(src, evt) obj.closeCallback(src, evt);
            column.add(button);
            obj.CloseButton = button;
        end


        %% callbacks
        function refreshWSCallback(obj, ~, ~)
        %refreshWSCallback
            obj.notify('WorkspaceImporterTab_RefreshWorkspaceVariables_Request');
        end

        function selectScansDropDownCallback(obj, ~, evt)
        %selectScansDropDownCallback

            varName = evt.EventData.NewValue;

            import robotics.appscore.internal.eventdata.*
            obj.notify('WorkspaceImporterTab_ScansSelectionUpdated', VectorEventData(varName)); %char vector
        end

        function selectPosesDropDownCallback(obj, ~, evt)
        %selectPosesDropDownCallback

            varName = evt.EventData.NewValue;

            import robotics.appscore.internal.eventdata.*
            obj.notify('WorkspaceImporterTab_PosesSelectionUpdated', VectorEventData(varName)); %char vector
        end


        function applyCallback(obj, ~, ~)
        %applyCallback
            obj.notify('WorkspaceImporterTab_Apply');
        end

        function closeCallback(obj, ~, ~)
        %closeCallback
            obj.notify('WorkspaceImporterTab_Close');
        end


       %% tab view updaters
        function updateDropDowns(obj, dropDownStatusInfo)
        %updateDropDowns
            obj.SelectScansDropDown.replaceAllItems(dropDownStatusInfo.AvailableScansVars);
            obj.SelectScansDropDown.SelectedIndex = dropDownStatusInfo.SelectedScansVarIndex;

            obj.SelectPosesDropDown.replaceAllItems(dropDownStatusInfo.AvailablePosesVars);
            obj.SelectPosesDropDown.SelectedIndex = dropDownStatusInfo.SelectedPosesVarIndex;

            obj.PercentageDropDown.replaceAllItems(dropDownStatusInfo.AvailablePercentagesToKeep);
            obj.PercentageDropDown.SelectedIndex = dropDownStatusInfo.SelectedPercentageToKeepIndex;
        end

    end
end

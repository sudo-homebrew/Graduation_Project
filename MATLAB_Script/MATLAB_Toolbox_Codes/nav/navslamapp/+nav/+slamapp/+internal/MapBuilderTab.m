classdef MapBuilderTab < handle & ...
        nav.slamapp.internal.mixin.ShowHideHelper & ...
        robotics.appscore.internal.mixin.MsgCatalogHelper
    %This class is for internal use only. It may be removed in the future.

    %MAPBUILDERTAB Main tab for the SLAM app

    % Copyright 2018-2022 The MathWorks, Inc.

    properties % widgets on MapBuilderTab
        OpenSessionButton

        SaveSessionSplitButton

        SaveSessionListItem

        SaveSessionAsListItem

        LoadFromBagSplitButton

        LoadFromBagListItem

        LoadFromWSListItem

        SettingsButton

        InspectDataButton

        OccMapPreviewButton

        RunButton

        SyncSplitButton

        ModifyIncrementalButton

        ModifyLoopClosureButton

        DefaultLayoutSplitButton

        DefaultLayoutListItem

        DefaultLayout1ListItem

        ExportSplitButton
    end

    events % events emitted by MapBuilderTab
    MapBuilderTab_BagSelected
    
    MapBuilderTab_RequestWSImporterTab

    MapBuilderTab_RequestOpenSession

    MapBuilderTab_RequestSaveSession

    MapBuilderTab_RequestSaveSessionAs

    MapBuilderTab_ResumeDefaultLayout

    MapBuilderTab_ResumeDefaultLayout_1

    MapBuilderTab_RequestSettingsDialog

    MapBuilderTab_RunButtonClicked

    MapBuilderTab_BringUpModificationTab_Incremental

    MapBuilderTab_BringUpModificationTab_LoopClosure

    MapBuilderTab_RefreshToolstrip_MappingPaused

    MapBuilderTab_RefreshToolstrip_Mapped

    MapBuilderTab_InitiateSyncFast

    MapBuilderTab_InitiateSync

    MapBuilderTab_ExportOccMapToWS

    MapBuilderTab_ExportOccMapToFile
end

methods
    function obj = MapBuilderTab(tabGroup)
    %MAPBUILDERTAB Constructor

    %createMapBuilderTab
        import matlab.ui.internal.toolstrip.*

        obj.MsgIDPrefix = 'nav:navslamapp:mapbuildertab';
        tabName = upper(obj.retrieveMsg('MapBuilderTabName'));
        tag = 'MapBuilderTab';

        obj.TabGroup = tabGroup;
        obj.Tab = Tab(tabName);
        obj.TabGroup.add(obj.Tab);
        obj.Tab.Tag = tag;


        obj.addFileSection();
        obj.addConfigurationSection();
        obj.addBuildMapSection();
        obj.addPreviewSection();
        obj.addManualSection();
        obj.addLayoutSection();
        obj.addExportSection();

    end

    function section = addFileSection(obj)
    %addFileSection
        import matlab.ui.internal.toolstrip.*
        import nav.slamapp.internal.CustomizedIcon

        section = obj.Tab.addSection(upper(obj.retrieveMsg('FileSectionName')));

        column = section.addColumn();
        button = Button(obj.retrieveMsg('OpenSessionButtonName'), Icon.OPEN_24);
        button.Tag = 'OpenSession';
        button.Description = obj.retrieveMsg('OpenSessionButtonDescription');
        button.ButtonPushedFcn = @(varargin)obj.openSessionCallback;
        obj.OpenSessionButton = button;
        column.add(button);

        column = section.addColumn();
        button = SplitButton(obj.retrieveMsg('SaveSessionButtonName'), Icon.SAVE_24);
        button.Tag = 'SaveSession';
        button.Description = obj.retrieveMsg('SaveSessionButtonDescription');
        button.ButtonPushedFcn = @(varargin)obj.saveSessionCallback;
        obj.SaveSessionSplitButton = button;

        popup = PopupList();
        button.Popup = popup;
        popup.Tag = 'SaveSessionButtonPopUp';

        item = ListItem(obj.retrieveMsg('SaveSessionListItemName'), Icon.SAVE_16);
        item.Tag = 'Save';
        item.ShowDescription = false;
        item.ItemPushedFcn = @(varargin)obj.saveSessionCallback;
        popup.add(item);
        obj.SaveSessionListItem = item;

        item = ListItem(obj.retrieveMsg('SaveSessionAsListItemName'), Icon.SAVE_AS_16);
        item.Tag = 'SaveAs';
        item.ShowDescription = false;
        item.ItemPushedFcn = @(varargin)obj.saveSessionAsCallback;
        popup.add(item);
        obj.SaveSessionAsListItem = item;

        column.add(button);

        column = section.addColumn();
        button = SplitButton(obj.retrieveMsg('LoadButtonName'), Icon.IMPORT_24);
        button.Tag = 'LoadData';
        button.Description = obj.retrieveMsg('LoadButtonDescription');
        button.ButtonPushedFcn = @(varargin)obj.loadFromWSCallback;
        obj.LoadFromBagSplitButton = button;

        popup = PopupList();
        button.Popup = popup;
        popup.Tag = 'LoadButtonPopUp';

        item = ListItem(obj.retrieveMsg('LoadFromWorkspaceListItem'), CustomizedIcon.IMPORT_FROM_WORKSPACE_16);
        item.Tag = 'LoadFromWSListItem';
        item.Description = obj.retrieveMsg('LoadFromWorkspaceListItemDescription');
        item.ItemPushedFcn = @(varargin)obj.loadFromWSCallback;
        
        popup.add(item);
        obj.LoadFromWSListItem = item;

        item = ListItem(obj.retrieveMsg('LoadFromBagListItem'), Icon.IMPORT_16);
        item.Tag = 'LoadFromBagListItem';
        item.Description = obj.retrieveMsg('LoadFromBagListItemDescription');
        item.ItemPushedFcn = @(varargin)obj.loadFromBagCallback;

        popup.add(item);
        obj.LoadFromBagListItem = item;
        if ~robotics.internal.license.isROSToolboxLicensed
            item.Enabled = false;
        end

        column.add(button);

    end


    function section = addConfigurationSection(obj)
    %addConfigurationSection
        import matlab.ui.internal.toolstrip.*

        section = obj.Tab.addSection(upper(obj.retrieveMsg('ConfigurationSectionName')));
        column = section.addColumn();
        button = Button(obj.retrieveMsg('SettingsButtonName'), Icon.SETTINGS_24);
        button.Tag = 'Settings';
        button.Description = obj.retrieveMsg('SettingsButtonDescription');
        button.ButtonPushedFcn = @(varargin) obj.settingsCallback;
        column.add(button);
        obj.SettingsButton = button;

        button = Button(obj.retrieveMsg('InspectDataButtonName'), Icon.SEARCH_16);
        button.Tag = 'InspectSensorData';
        button.Description = obj.retrieveMsg('InspectDataButtonDescription');
        %column.add(button);
        obj.InspectDataButton = button;
    end

    function addBuildMapSection(obj)
    %addBuildMapSection
        import matlab.ui.internal.toolstrip.*
        import nav.slamapp.internal.CustomizedIcon

        section = obj.Tab.addSection(upper(obj.retrieveMsg('BuildMapSectionName')));
        column = section.addColumn();
        button = Button(obj.retrieveMsg('RunButtonName'), Icon.RUN_24);
        button.Tag = 'Run';
        button.Description = obj.retrieveMsg('RunButtonDescription');

        column.add(button);
        obj.RunButton = button;

        column = section.addColumn();
        button = SplitButton(obj.retrieveMsg('SyncButtonName'), CustomizedIcon.SYNC_MAP_24);
        button.Tag = 'Sync';
        button.Description = obj.retrieveMsg('SyncButtonDescription');
        button.ButtonPushedFcn = @(varargin) obj.syncCallback;
        column.add(button);
        obj.SyncSplitButton = button;

        popup = PopupList();
        button.Popup = popup;
        popup.Tag = 'SyncButtonPopUp';

        item = ListItem(obj.retrieveMsg('SyncListItemName'), CustomizedIcon.SYNC_MAP_24);
        item.Tag = 'SyncRegular';
        item.Description = obj.retrieveMsg('SyncListItemDescription');
        addlistener(item, 'ItemPushed', @(varargin)obj.syncCallback);
        popup.add(item);

        item = ListItem(obj.retrieveMsg('SyncFastListItemName'), CustomizedIcon.SYNC_MAP_FAST_24);
        item.Tag = 'SyncFast';
        item.Description = obj.retrieveMsg('SyncFastListItemDescription');
        addlistener(item, 'ItemPushed', @(varargin)obj.syncFastCallback);
        popup.add(item);

    end

    function addPreviewSection(obj)
    %addPreviewSection
        import matlab.ui.internal.toolstrip.*
        import nav.slamapp.internal.CustomizedIcon
        %section = obj.Tab.addSection(upper(obj.retrieveMsg('PreviewSectionName')));
        %column = section.addColumn();

        button = Button(obj.retrieveMsg('OccMapButtonName'), CustomizedIcon.OCC_MAP_24);
        button.Tag = 'OccMap';
        button.Description = obj.retrieveMsg('OccMapButtonDescription');
        button.ButtonPushedFcn = @(varargin) obj.occMapCallback;
        %column.add(button);
        obj.OccMapPreviewButton = button;
    end

    function addManualSection(obj)
    %addManualSection
        import matlab.ui.internal.toolstrip.*
        import nav.slamapp.internal.CustomizedIcon

        section = obj.Tab.addSection(upper(obj.retrieveMsg('ManualSectionName')));

        column = section.addColumn();

        button = Button(obj.retrieveMsg('ModifyIncrementalButtonName'), CustomizedIcon.MODIFY_INCREMENTAL_24);
        button.Tag = 'ModifyIncremental';
        button.Description = obj.retrieveMsg('ModifyIncrementalButtonDescription');
        addlistener(button, 'ButtonPushed', @(varargin) obj.modifyIncrementalCallback);
        obj.ModifyIncrementalButton = button;
        column.add(button);


        column = section.addColumn();

        button = Button(obj.retrieveMsg('ModifyLoopClosureButtonName'), CustomizedIcon.MODIFY_LOOPCLOSURE_24);
        button.Tag = 'ModifyLoopClosure';
        button.Description = obj.retrieveMsg('ModifyLoopClosureButtonDescription');
        addlistener(button, 'ButtonPushed', @(varargin) obj.modifyLoopClosureCallback);
        obj.ModifyLoopClosureButton = button;
        column.add(button);

    end

    function addLayoutSection(obj)
    %addLayoutSection
        import matlab.ui.internal.toolstrip.*
        import nav.slamapp.internal.CustomizedIcon


        section = obj.Tab.addSection(upper(obj.retrieveMsg('LayoutSectionName')));
        column = section.addColumn();

        button = Button(obj.retrieveMsg('DefaultLayoutButtonName'), CustomizedIcon.LAYOUT_THREE_24);
        button.Tag = 'DefaultLayout';
        button.Enabled = true;
        button.Description = obj.retrieveMsg('DefaultLayoutButtonDescription');
        button.ButtonPushedFcn = @(src, evt) obj.defaultLayoutCallback(src, evt);
        column.add(button);
        obj.DefaultLayoutSplitButton = button;


    end

    function addExportSection(obj)
    %addExportSection
        import matlab.ui.internal.toolstrip.*
        import nav.slamapp.internal.CustomizedIcon

        section = obj.Tab.addSection(upper(obj.retrieveMsg('ExportSectionName')));
        section.CollapsePriority = 10;
        column = section.addColumn();

        button = SplitButton(obj.retrieveMsg('ExportButtonName'), Icon.CONFIRM_24);
        button.Tag = 'Export';
        button.Description = obj.retrieveMsg('ExportButtonDescription');
        addlistener(button, 'ButtonPushed', @(src, evt) obj.exportMapToWSCallback());
        obj.ExportSplitButton = button;

        popup = PopupList();
        button.Popup = popup;
        popup.Tag = 'ExportButtonPopup';

        item = ListItem(obj.retrieveMsg('ExportOccGridToWS'), Icon.CONFIRM_16);
        item.Tag = 'ExportMapToWS';
        item.ShowDescription = false;
        addlistener(item, 'ItemPushed', @(src, evt) obj.exportMapToWSCallback);
        popup.add(item);

        item = ListItem(obj.retrieveMsg('ExportOccGridToFile'), CustomizedIcon.GENERATE_MATLAB_SCRIPT_16);
        item.Tag = 'ExportMapToFile';
        item.ShowDescription = false;
        addlistener(item, 'ItemPushed', @(src, evt) obj.exportMapToFileCallback);
        popup.add(item);


        column.add(button);

    end



%% callbacks
function loadFromBagCallback(obj)
%loadFromBagCallback
    import nav.slamapp.internal.eventdata.*

    bagName = selectFile(obj, true, '*.bag', 'BagFileExtensionHint', 'SelectBagFileDialogTitle');

    if ~isempty(bagName)
        obj.notify('MapBuilderTab_BagSelected', FileSelectedEventData(bagName));
    end
end

function loadFromWSCallback(obj)
%loadFromWSCallback
    import nav.slamapp.internal.eventdata.*

    obj.notify('MapBuilderTab_RequestWSImporterTab');
    
end

function openSessionCallback(obj)
%openSessionCallback
    obj.notify('MapBuilderTab_RequestOpenSession');
end

function saveSessionCallback(obj)
%saveSessionCallback
    obj.notify('MapBuilderTab_RequestSaveSession');
end

function saveSessionAsCallback(obj)
%saveSessionAsCallback
    obj.notify('MapBuilderTab_RequestSaveSessionAs');
end


function settingsCallback(obj)
%settingsCallback
    obj.notify('MapBuilderTab_RequestSettingsDialog');
end

function syncCallback(obj)
%syncCallback
    obj.notify('MapBuilderTab_InitiateSync');
end

function syncFastCallback(obj)
%syncFastCallback
    obj.notify('MapBuilderTab_InitiateSyncFast');
end

function modifyIncrementalCallback(obj)
%modifyIncrementalCallback
    obj.notify('MapBuilderTab_BringUpModificationTab_Incremental');
end



function modifyLoopClosureCallback(obj)
%modifyLoopClosureCallback
    obj.notify('MapBuilderTab_BringUpModificationTab_LoopClosure');

end


function defaultLayoutCallback(obj, src, evt) %#ok<INUSD>
%defaultLayoutCallback
    obj.notify('MapBuilderTab_ResumeDefaultLayout');
end

function exportMapToWSCallback(obj)
%exportMapToWSCallback
    obj.notify('MapBuilderTab_ExportOccMapToWS');
end

function exportMapToFileCallback(obj)
%exportMapToFileCallback
    obj.notify('MapBuilderTab_ExportOccMapToFile');
end

%% helpers
function setRunButtonIconAndText(obj, v)
%setRunButtonIconAndText
    import matlab.ui.internal.toolstrip.*
    if v
        obj.RunButton.Icon = Icon.PAUSE_MATLAB_24;
        obj.RunButton.Text = obj.retrieveMsg('PauseButtonName');
        obj.RunButton.Description = obj.retrieveMsg('PauseButtonDescription');
    else
        obj.RunButton.Icon = Icon.RUN_24;
        obj.RunButton.Text = obj.retrieveMsg('RunButtonName');
        obj.RunButton.Description = obj.retrieveMsg('RunButtonDescription');
    end
end

function bagName = selectFile(obj, isGet, fileType, hintId, fileDialogTitleId)
%selectFile
    persistent cachedPath;
    if isempty(cachedPath)
        cachedPath = ''; % If the specified path does not exist, uigetfile opens dialog box in current folder
    end

    filterSpec = cell(2);
    filterSpec{1,1} = fileType;
    filterSpec{1,2} = obj.retrieveMsg(hintId);
    filterSpec{2,1} = '*.*';
    filterSpec{2,2} = obj.retrieveMsg('AllFilesExtensionHint');
    dialogTitle = obj.retrieveMsg(fileDialogTitleId);

    if isGet
        [fileName, pathName, filterIndex] = uigetfile(filterSpec, dialogTitle, cachedPath);
    else
        try
            [fileName, pathName, filterIndex] = uiputfile(filterSpec, dialogTitle, cachedPath);
        catch
            % reset invalid cache path due to previous invalid selection
            cachedPath = '';
            [fileName, pathName, filterIndex] = uiputfile(filterSpec, dialogTitle, cachedPath);
        end
    end

    % If user successfully chooses a file, cache the path
    userCanceled = (filterIndex == 0);
    if ~userCanceled
        bagName = fullfile(pathName, fileName);

        % Do not cache the path if we restore the app to default state upon
        % deserialize failure
        if ~contains(pathName, fullfile(matlabroot,'test', 'toolbox', 'nav', 'navalgs', 'testdata', 'slamapp'))
            cachedPath = pathName;
        end
    else
        bagName = '';
    end
end


function setWidgetAvailability(obj, maskIn)
%setWidgetAvailability
    n = 12;
    if isscalar(maskIn)
        mask = logical(robotics.core.internal.de2bi(maskIn, n));
    else
        mask = logical(maskIn);
    end

    % 12 widgets can be enabled/disabled
    obj.OpenSessionButton.Enabled =         mask(1);
    obj.SaveSessionSplitButton.Enabled =    mask(2);
    obj.LoadFromBagSplitButton.Enabled =    mask(3);
    obj.SettingsButton.Enabled =            mask(4);
    obj.InspectDataButton.Enabled =         mask(5);
    obj.RunButton.Enabled =                 mask(6);
    obj.SyncSplitButton.Enabled =           mask(7);
    obj.OccMapPreviewButton.Enabled =       mask(8);
    obj.ModifyIncrementalButton.Enabled =   mask(9);
    obj.ModifyLoopClosureButton.Enabled =   mask(10);
    obj.DefaultLayoutSplitButton.Enabled =  mask(11);
    obj.ExportSplitButton.Enabled =         mask(12);
end
    end
end

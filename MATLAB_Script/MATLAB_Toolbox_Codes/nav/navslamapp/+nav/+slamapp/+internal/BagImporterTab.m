classdef BagImporterTab < handle & ...
        nav.slamapp.internal.mixin.ShowHideHelper & ...
        robotics.appscore.internal.mixin.MsgCatalogHelper
    %This class is for internal use only. It may be removed in the future.

    %BAGIMPORTERTAB Modal tab for importing sensor data from rosbag

    % Copyright 2018-2021 The MathWorks, Inc.


    properties % toolstrip widgets

        SliderStartTime

        SliderEndTime

        EditStartTime

        EditEndTime

        ScanTopicDropDown

        OdomTopicDropDown

        FromFrameDropDown

        ToFrameDropDown

        PercentageDropDown

        ApplyButton

        CloseButton
    end

    events % events emitted by BagImporterTab

    BagImporterTab_TimeValueUpdated

    BagImporterTab_OdomOptionSelectionUpdated

    BagImporterTab_Close

    BagImporterTab_Apply
end

properties % callback fires during initial rendering, which is a swing behavior. The property below is to prevent misfire
    SliderEndTimeFirstTimeFire = false

end

methods
    function obj = BagImporterTab(tabGroup)
    %BAGIMPORTERTAB Constructor

        import matlab.ui.internal.toolstrip.*

        obj.MsgIDPrefix = 'nav:navslamapp:bagimportertab';
        tabName = upper(obj.retrieveMsg('BagImporterTabName'));
        tag = 'BagImporterTab';

        obj.TabGroup = tabGroup;
        obj.Tab = Tab(tabName);
        obj.TabGroup.add(obj.Tab);
        obj.Tab.Tag = tag;

        obj.addLidarTopicSection();
        obj.addOdomTopicSection();
        obj.addTfSection;
        obj.addDataSelectionSection();
        obj.addCloseSection();
    end

    function addLidarTopicSection(obj)
    %addLidarTopicSection
        import matlab.ui.internal.toolstrip.*
        section = obj.Tab.addSection(upper(obj.retrieveMsg('LidarTopicSectionName')));

        column = section.addColumn('width', 100);

        % labels
        labelScanTopic = Label(obj.retrieveMsg('ScanTopicLabelName'));
        column.add(labelScanTopic);

        % dropdowns
        scanTopicDropDown = DropDown({''});
        scanTopicDropDown.Tag = 'ScanTopicDropDown';
        scanTopicDropDown.Description = obj.retrieveMsg('ScanTopicDropDownDescription');
        obj.ScanTopicDropDown = scanTopicDropDown;

        column.add(scanTopicDropDown);

    end

    function addOdomTopicSection(obj)
    %addOdomTopicSection
        import matlab.ui.internal.toolstrip.*
        section = obj.Tab.addSection(upper(obj.retrieveMsg('OdomTopicSectionName')));

        column = section.addColumn('width', 100);

        % labels
        labelOdomTopic = Label(obj.retrieveMsg('OdomTopicLabelName'));
        column.add(labelOdomTopic);

        % dropdowns
        odomTopicDropDown = DropDown({''});
        odomTopicDropDown.Tag = 'OdomTopicDropDown';
        odomTopicDropDown.Description = obj.retrieveMsg('OdomTopicDropDownDescription');
        obj.OdomTopicDropDown = odomTopicDropDown;
        obj.OdomTopicDropDown.ValueChangedFcn = @(src, evt) obj.odomTopicDropDownCallback(src, evt);

        column.add(odomTopicDropDown);
    end

    function addTfSection(obj)
    %addTfSection
        import matlab.ui.internal.toolstrip.*

        % tf section
        section = obj.Tab.addSection(upper(obj.retrieveMsg('TfSectionName')));

        % labels
        column = section.addColumn();
        labelFromFrame = Label(obj.retrieveMsg('FromFrameLabelName'));
        labelToFrame = Label(obj.retrieveMsg('ToFrameLabelName'));

        column.add(labelFromFrame);
        column.add(labelToFrame);

        % dropdowns
        column = section.addColumn('width', 80);


        fromFrameDropDown = DropDown({''});
        fromFrameDropDown.Tag = 'FromFrameDropDown';
        fromFrameDropDown.Description = obj.retrieveMsg('FromFrameDropDownDescription');
        obj.FromFrameDropDown = fromFrameDropDown;

        toFrameDropDown = DropDown({''});
        toFrameDropDown.Tag = 'ToFrameDropDown';
        toFrameDropDown.Description = obj.retrieveMsg('ToFrameDropDownDescription');
        obj.ToFrameDropDown = toFrameDropDown;

        column.add(fromFrameDropDown);
        column.add(toFrameDropDown);

    end

    function addDataSelectionSection(obj)
    %addDataSelectionSection time range selection
        import matlab.ui.internal.toolstrip.*

        section = obj.Tab.addSection(upper(obj.retrieveMsg('DataSelectionSectionName')));

        % labels
        column = section.addColumn();
        labelStart = Label( obj.retrieveMsg('StartLabelName'));
        labelEnd = Label( obj.retrieveMsg('EndLabelName') );

        column.add(labelStart);
        column.add(labelEnd);

        % sliders
        column = section.addColumn('width',250);
        obj.SliderStartTime = Slider([0 100], 15);
        obj.SliderStartTime.Tag = 'SliderStartTime';
        column.add(obj.SliderStartTime);

        obj.SliderEndTime = Slider([0 100], 80);
        obj.SliderEndTime.Tag = 'SliderEndTime';
        column.add(obj.SliderEndTime);

        % editboxes
        obj.EditStartTime = EditField(sprintf('%d', obj.SliderStartTime.Value));
        obj.EditStartTime.Tag = 'EditFieldStartTime';
        obj.EditStartTime.Description = obj.retrieveMsg('EditFieldStartTimeDescription');

        obj.EditEndTime = EditField(sprintf('%d', obj.SliderEndTime.Value));
        obj.EditEndTime.Tag = 'EditFieldEndTime';
        obj.EditEndTime.Description = obj.retrieveMsg('EditFieldEndTimeDescription');

        column = section.addColumn('width', 60);
        column.add(obj.EditStartTime);
        column.add(obj.EditEndTime)

        % empty column
        section.addColumn('width', 20);

        % labels and spinners for percentage selection
        column = section.addColumn();
        label1 = Label(obj.retrieveMsg('KeepLabelName1'));
        label2 = Label(obj.retrieveMsg('KeepLabelName2'));
        dropDownPercentage = DropDown({''});
        obj.PercentageDropDown = dropDownPercentage;
        obj.PercentageDropDown.Tag = 'PercentageDropDown';
        obj.PercentageDropDown.Description = obj.retrieveMsg('KeepDropDownDescription');

        column.add(label1);
        column.add(label2);
        column.add(dropDownPercentage);


        % hook-up local callbacks
        obj.SliderStartTime.ValueChangedFcn = @(src, evt) obj.sliderStartTimeCallback(src, evt); % listen to "ValueChanging" event would be better, but callback is not fast enough
        obj.SliderEndTime.ValueChangedFcn = @(src, evt) obj.sliderEndTimeCallback(src, evt);

        obj.EditStartTime.ValueChangedFcn = @(src, evt) obj.editStartTimeCallback(src, evt);
        obj.EditEndTime.ValueChangedFcn = @(src, evt) obj.editEndTimeCallback(src, evt);

    end

    function addCloseSection(obj)
    %addCloseSection
        import matlab.ui.internal.toolstrip.*

        section = obj.Tab.addSection(upper(obj.retrieveMsg('CloseSectionName')));
        section.CollapsePriority = 10;
        %
        column = section.addColumn();
        button = Button(obj.retrieveMsg('ApplyButtonName'), Icon.CONFIRM_24); % accept and close
        button.Tag = 'Apply';
        button.Description = obj.retrieveMsg('ApplyButtonDescription');
        button.ButtonPushedFcn = @(src, evt) obj.applyCallback(src, evt);
        column.add(button);
        obj.ApplyButton = button;

        column = section.addColumn();
        button = Button(obj.retrieveMsg('CloseButtonName'), Icon.CLOSE_24); % cancel actions and close
        button.Tag = 'Close';
        button.Description = obj.retrieveMsg('CloseButtonDescription');
        button.ButtonPushedFcn = @(src, evt) obj.closeCallback(src, evt);
        column.add(button);
        obj.CloseButton = button;
    end


%% callbacks

function sliderStartTimeCallback(obj, src, ~)
%sliderStartTimeCallback

    import robotics.appscore.internal.eventdata.*
    obj.notify('BagImporterTab_TimeValueUpdated', VectorEventData([0 src.Value]));
end

function sliderEndTimeCallback(obj, src, ~)
%sliderEndTimeCallback

    import robotics.appscore.internal.eventdata.*
    if obj.SliderEndTimeFirstTimeFire
        obj.SliderEndTimeFirstTimeFire = false;
    else
        obj.notify('BagImporterTab_TimeValueUpdated', VectorEventData([1 src.Value]));
    end
end

function editStartTimeCallback(obj, ~, evt)
%editStartTimeCallback
    val = str2double(evt.EventData.NewValue);

    import robotics.appscore.internal.eventdata.*
    obj.notify('BagImporterTab_TimeValueUpdated', VectorEventData([0 val]));
end

function editEndTimeCallback(obj, ~, evt)
%editEndTimeCallback
    val = str2double(evt.EventData.NewValue);

    import robotics.appscore.internal.eventdata.*
    obj.notify('BagImporterTab_TimeValueUpdated', VectorEventData([1 val]));
end

function odomTopicDropDownCallback(obj, ~, evt)
%odomTopicDropDownCallback

    v = 1;
    if ~strcmp(evt.EventData.NewValue, 'N/A')
        v = 2;
    end
    import robotics.appscore.internal.eventdata.*
    obj.notify('BagImporterTab_OdomOptionSelectionUpdated', VectorEventData(v));
end

function applyCallback(obj, ~, ~)
%applyCallback
    obj.notify('BagImporterTab_Apply');
end

function closeCallback(obj, ~, ~)
%closeCallback
    obj.notify('BagImporterTab_Close');
end



%% refresh
function updateTimeSliderAndEditDisplay(obj, startTime, endTime, selectedStartTime, selectedEndTime)
%updateTimeSliderAndEditDisplay Refresh the time range sliders
%   and edits

    obj.SliderStartTime.Limits = [startTime, endTime];
    obj.SliderEndTime.Limits = [startTime, endTime];
    obj.SliderStartTime.Value = selectedStartTime;
    obj.SliderEndTime.Value = selectedEndTime;
    obj.EditStartTime.Value = sprintf('%6.2f', selectedStartTime);
    obj.EditEndTime.Value = sprintf('%6.2f', selectedEndTime);
    pause(0.1);
end


function updateTabView(obj, bagInfo, tabStatusInfo)
%updateTabView
    obj.ScanTopicDropDown.replaceAllItems(bagInfo.AvailableScanTopics);
    obj.ScanTopicDropDown.SelectedIndex = tabStatusInfo.SelectedScanTopicIndex;


    obj.OdomTopicDropDown.replaceAllItems(bagInfo.AvailableOdomOptions);
    obj.OdomTopicDropDown.SelectedIndex = tabStatusInfo.SelectedOdomOptionIndex;

    if isempty(bagInfo.AvailableTFFrames)
        list = {''};
    else
        list = bagInfo.AvailableTFFrames;
    end
    obj.FromFrameDropDown.replaceAllItems(list);
    obj.FromFrameDropDown.SelectedIndex = tabStatusInfo.SelectedTargetFrameIndex;
    obj.ToFrameDropDown.replaceAllItems(list);
    obj.ToFrameDropDown.SelectedIndex = tabStatusInfo.SelectedSourceFrameIndex;

    obj.PercentageDropDown.replaceAllItems(tabStatusInfo.AvailablePercentagesToKeep);
    obj.PercentageDropDown.SelectedIndex = tabStatusInfo.SelectedPercentageToKeepIndex;

    obj.enableDisableTf(tabStatusInfo.SelectedOdomOptionIndex);
end


function enableDisableTf(obj, selectedOdomTopicIndex)
%enableDisableTf
    if selectedOdomTopicIndex > 1
        obj.FromFrameDropDown.Enabled = true;
        obj.ToFrameDropDown.Enabled = true;
    else
        obj.FromFrameDropDown.Enabled = false;
        obj.ToFrameDropDown.Enabled = false;
    end
end

    end
end
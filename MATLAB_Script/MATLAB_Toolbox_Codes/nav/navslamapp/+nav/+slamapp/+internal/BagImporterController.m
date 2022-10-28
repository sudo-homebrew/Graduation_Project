classdef BagImporterController < handle
%This class is for internal use only. It may be removed in the future.

%BagImporterController This class encapsulates the Controller portion
%   of the Model/View/Controller design pattern used in the SLAM app for
%   importing data from a rosbag

% Copyright 2018-2021 The MathWorks, Inc.

    properties
        BagImporterTab
        AppViewOrganizer

        BagImporterModel
        MapBuilderModel
        ScanFigureSliderModel
        StateMachineModel
    end

    methods
        function obj = BagImporterController(bagImporterModel, mapBuilderModel, scanFigSliderModel, stateMachineModel, bagImporterTab, appViewOrg )
        %BagImporterController Constructor

            obj.BagImporterTab = bagImporterTab;
            obj.AppViewOrganizer = appViewOrg;
            obj.BagImporterModel = bagImporterModel;
            obj.MapBuilderModel = mapBuilderModel;
            obj.ScanFigureSliderModel = scanFigSliderModel;
            obj.StateMachineModel = stateMachineModel;

            obj.BagImporterModel.ScanFcnIndex = 1;

            obj.addViewListeners();
            obj.addModelListeners();

        end

    end


    % listen to View events
    methods
        function addViewListeners(obj)
        %addViewListeners

        % bag importer listeners

            addlistener(obj.BagImporterTab, 'BagImporterTab_TimeValueUpdated', @(source, event) obj.refreshTimeSliderAndEdit(event.Vector));
            addlistener(obj.BagImporterTab, 'BagImporterTab_OdomOptionSelectionUpdated', @(source, event) obj.updateOdomOptionSelection(event.Vector) );

            addlistener(obj.BagImporterTab.ScanTopicDropDown, 'ValueChanged', @(source, event) obj.setBagImporterModelTentativeProps('ScanTopicIndex', 'AvailableScanTopics', event.EventData.NewValue));
            addlistener(obj.BagImporterTab.FromFrameDropDown, 'ValueChanged', @(source, event) obj.setBagImporterModelTentativeProps('TargetFrameIndex', 'AvailableTFFrames', event.EventData.NewValue));
            addlistener(obj.BagImporterTab.ToFrameDropDown, 'ValueChanged', @(source, event) obj.setBagImporterModelTentativeProps('SourceFrameIndex', 'AvailableTFFrames', event.EventData.NewValue));
            addlistener(obj.BagImporterTab.PercentageDropDown, 'ValueChanged', @(source, event) obj.setBagImporterModelTentativeProps('PercentageToKeepIndex', 'AvailablePercentagesToKeep', event.EventData.NewValue));

            addlistener(obj.BagImporterTab, 'BagImporterTab_Apply', @(src, evt) obj.applyBagImportSettingsCallback() );
            addlistener(obj.BagImporterTab, 'BagImporterTab_Close', @(src, evt) obj.closeBagImporterTabCallback() );
        end
    end


    % listen to Model events
    methods
        function addModelListeners(obj)
        %addModelListeners

            addlistener(obj.ScanFigureSliderModel, 'SliderModel_CurrentValueUpdated', @(source, event) obj.drawCurrentScan(event.Vector));

            addlistener(obj.BagImporterModel, 'Tentative', 'PostSet', @(source, event) obj.toggleBannerVisibility);
            addlistener(obj.BagImporterModel, 'TabRefreshTrigger', 'PostSet', @(source, event) obj.refreshBagImporterTab);
            addlistener(obj.BagImporterModel, 'InfoStrings', 'PostSet', @(source, event) obj.refreshInfoFigure(event.AffectedObject.InfoStrings));

            addlistener(obj.BagImporterModel, 'BagImporterModel_RequestUpdateBagImporterTab', @(source, event) obj.refreshBagImporterTab() );
        end
    end


    methods

        function refreshBagImporterTab(obj)
        %refreshBagImporterTab
            import nav.slamapp.internal.eventdata.*

            bagInfo = struct();
            bagInfo.AvailableScanTopics = obj.BagImporterModel.AvailableScanTopics;
            bagInfo.NumScanMessages = obj.BagImporterModel.NumScanMessages;
            bagInfo.AvailableOdomTopics = obj.BagImporterModel.AvailableOdomTopics;
            bagInfo.AvailableOdomOptions = obj.BagImporterModel.AvailableOdomOptions;
            bagInfo.NumOdomMessages = obj.BagImporterModel.NumOdomMessages;
            bagInfo.ScanStartTime = obj.BagImporterModel.ScanStartTime;
            bagInfo.ScanEndTime = obj.BagImporterModel.ScanEndTime;
            bagInfo.AvailableTFFrames = obj.BagImporterModel.AvailableTFFrames;

            tabStatusInfo = struct();
            tabStatusInfo.AvailablePercentagesToKeep = obj.BagImporterModel.AvailablePercentagesToKeep;

            tabStatusInfo.SelectedScanTopicIndex = obj.BagImporterModel.Tentative.ScanTopicIndex;
            tabStatusInfo.SelectedOdomOptionIndex = obj.BagImporterModel.Tentative.OdomOptionIndex;
            tabStatusInfo.SelectedTargetFrameIndex = obj.BagImporterModel.Tentative.TargetFrameIndex;
            tabStatusInfo.SelectedSourceFrameIndex = obj.BagImporterModel.Tentative.SourceFrameIndex;

            tabStatusInfo.SelectedPercentageToKeepIndex = obj.BagImporterModel.Tentative.PercentageToKeepIndex;

            obj.BagImporterTab.updateTabView(bagInfo, tabStatusInfo);
            obj.ScanFigureSliderModel.resetMinMaxValue(1, bagInfo.NumScanMessages);

            obj.BagImporterTab.updateTimeSliderAndEditDisplay(obj.BagImporterModel.ScanStartTime, ...
                                                              obj.BagImporterModel.ScanEndTime, ...
                                                              obj.BagImporterModel.Tentative.ScanStartTime, ...
                                                              obj.BagImporterModel.Tentative.ScanEndTime);

        end

        function updateOdomOptionSelection(obj, odomOptionIndex)
        %updateOdomTopicSelection
            obj.AppViewOrganizer.freeze();
            obj.BagImporterModel.Tentative.OdomOptionIndex = odomOptionIndex;
            obj.BagImporterTab.enableDisableTf(odomOptionIndex);

            if odomOptionIndex > 1
                rootFrameIndex = estimateTfRootFrame(obj.BagImporterModel, odomOptionIndex);
                [lidarFrameInTFFrames, lidarFrameIndex] = ismember(obj.BagImporterModel.LidarFrame, obj.BagImporterModel.AvailableTFFrames);
                if ~lidarFrameInTFFrames % if frame specified in lidar message does not present in TF
                    lidarFrameIndex = -1;
                end
            else
                rootFrameIndex = -1;
                lidarFrameIndex = -1;
            end
            obj.BagImporterModel.Tentative.TargetFrameIndex = lidarFrameIndex;
            obj.BagImporterModel.Tentative.SourceFrameIndex = rootFrameIndex;
            obj.refreshBagImporterTab;
            obj.AppViewOrganizer.thaw();
        end



        function refreshTimeSliderAndEdit(obj, vec)
        %refreshTimeSliderAndEdit
            type = vec(1);
            value = vec(2);
            obj.BagImporterModel.syncTimeSelection(type, value);

            obj.BagImporterTab.updateTimeSliderAndEditDisplay(obj.BagImporterModel.ScanStartTime, ...
                                                              obj.BagImporterModel.ScanEndTime, ...
                                                              obj.BagImporterModel.Tentative.ScanStartTime, ...
                                                              obj.BagImporterModel.Tentative.ScanEndTime);
        end


        function setBagImporterModelTentativeProps(obj, propName, propAvailableChoices, value)
        %setBagImporterModelTentativeProps
            choices = obj.BagImporterModel.(propAvailableChoices);
            [~, loc] = ismember(value, choices);
            obj.BagImporterModel.Tentative.(propName) = loc;

        end

        function toggleBannerVisibility(obj)
        %toggleBannerVisibility
            if ~isequal(obj.BagImporterModel.Tentative, obj.BagImporterModel.Applied)
                obj.AppViewOrganizer.ScanFigDoc.showBanner(true);
                obj.AppViewOrganizer.OdomFigDoc.showBanner(true);
                obj.BagImporterTab.ApplyButton.Enabled = true;
            else
                obj.AppViewOrganizer.ScanFigDoc.showBanner(false);
                obj.AppViewOrganizer.OdomFigDoc.showBanner(false);
                obj.BagImporterTab.ApplyButton.Enabled = false;
            end
        end

        function drawCurrentScan(obj, v)
        %drawCurrentScan
            import nav.slamapp.internal.eventdata.*
            import nav.slamapp.internal.*
            if obj.StateMachineModel.State == States.LoadingBag
                [scan, scanTime] = obj.BagImporterModel.ScanFcn(v);
                if ~isempty(scan)
                    obj.AppViewOrganizer.ScanFigDoc.drawCurrentRawScan(scan, obj.BagImporterModel.LidarRange(2), scanTime);

                    if ~isempty(obj.BagImporterModel.Ts)
                        T = obj.BagImporterModel.Ts{v};
                        obj.AppViewOrganizer.OdomFigDoc.drawSensor(T);
                    end
                end
            end
        end


        function applyBagImportSettingsCallback(obj)
        %applyBagImportSettingsCallback
            obj.AppViewOrganizer.freeze();
            obj.applyBagImportSettings();
            obj.AppViewOrganizer.thaw();
        end

        function applyBagImportSettings(obj)
        %applyBagImportSettings
            [isSuccess, errTag] = obj.BagImporterModel.extractData;
            if ~isSuccess
                % In window error dialog will always pop-up at the center
                % of app window
                dlgs = nav.slamapp.internal.Dialogs(obj.AppViewOrganizer.ToolGroup);
                dlgs.errorMsgDialog(errTag);
            end

            obj.BagImporterModel.notify('BagImporterModel_RequestUpdateBagImporterTab');

            obj.BagImporterModel.ScanFcnIndex = 2;

            if ~isempty(obj.BagImporterModel.Ts)
                obj.AppViewOrganizer.OdomFigDoc.drawOdom(obj.BagImporterModel.Ts);
            else
                obj.AppViewOrganizer.OdomFigDoc.clearOdom();
            end
            obj.refreshBagImporterTab;
        end

        function closeBagImporterTabCallback(obj)
        %closeBagImporterTabCallback
            % In window question dialog will always pop-up at the center
            % of app window
            dlgs = nav.slamapp.internal.Dialogs(obj.AppViewOrganizer.ToolGroup);

            if ~isequal(obj.BagImporterModel.Tentative, obj.BagImporterModel.Applied)
                % pending toolstrip setting changes

                [selection, possibleSelections] = dlgs.questionDialog('ImportIncomplete');
                switch selection
                  case possibleSelections{1} % apply changes and proceed
                    obj.AppViewOrganizer.freeze();
                    obj.applyBagImportSettings();
                    obj.MapBuilderModel.loadProcessedData(obj.BagImporterModel.Scans, ...
                                                          obj.BagImporterModel.PosesXY);
                    obj.StateMachineModel.toSensorDataReady();
                    obj.AppViewOrganizer.thaw();

                    obj.MapBuilderModel.IsAppDirty = true;
                end
            else
                % all toolstrip settings have been applied
                if isempty(obj.BagImporterModel.Scans) % if user hasn't changed any default settings on the toolstrip
                    obj.AppViewOrganizer.freeze();
                    applyBagImportSettings(obj);
                    pause(0.5);
                    obj.AppViewOrganizer.thaw();
                end
                obj.StateMachineModel.toSensorDataReady();
                obj.MapBuilderModel.loadProcessedData(obj.BagImporterModel.Scans, ...
                                                      obj.BagImporterModel.PosesXY);
                obj.MapBuilderModel.IsAppDirty = true;

            end


        end

        function refreshInfoFigure(obj, infoStrings)
        %refreshInfoFigure
            obj.AppViewOrganizer.InfoFigDoc.refreshTextFields(infoStrings);
        end
    end
end
classdef MapBuilderController < handle & ...
        nav.algs.internal.InternalAccess
    %This class is for internal use only. It may be removed in the future.

    %MapBuilderController This class encapsulates the Controller portion
    %   of the Model/View/Controller design pattern used in the SLAM app for
    %   the primary workflow (building a map)

    % Copyright 2018-2021 The MathWorks, Inc.

    properties
        AppViewOrganizer
        MapBuilderTab
        SettingsDialog

        BagImporterModel
        WSImporterModel
        StateMachineModel
        MapBuilderModel
        SettingsModel
        MapFigureSliderModel
        ScanFigureSliderModel
        ModificationModel

        Count
    end

    properties
        SessionFileName
    end



    methods
        function obj = MapBuilderController(bagImporterModel, wsImporterModel, stateMachineModel, mapBuilderModel, settingsModel, mapFigSliderModel, scanFigSliderModel, modificationModel, mapBuilderTab, appViewOrg, settingsDialog )
        %MapBuilderController Constructor

            obj.MapBuilderTab = mapBuilderTab;
            obj.AppViewOrganizer = appViewOrg;
            obj.BagImporterModel = bagImporterModel;
            obj.WSImporterModel = wsImporterModel;
            obj.SettingsModel = settingsModel;
            obj.StateMachineModel = stateMachineModel;
            obj.MapBuilderModel = mapBuilderModel;
            obj.MapFigureSliderModel = mapFigSliderModel;
            obj.ScanFigureSliderModel = scanFigSliderModel;
            obj.ModificationModel = modificationModel;
            obj.SettingsDialog = settingsDialog;

            obj.addViewListeners();
            obj.addModelListeners();

            obj.Count = 0;
        end

    end


    % listen to View events
    methods
        function addViewListeners(obj)
        %addViewListeners

            addlistener(obj.MapBuilderTab, 'MapBuilderTab_BagSelected', @(source, event) obj.investigateBag(event.FileName) );
            addlistener(obj.MapBuilderTab, 'MapBuilderTab_RequestWSImporterTab', @(source, event) obj.switchToWSImporterTab );
            addlistener(obj.MapBuilderTab, 'MapBuilderTab_ResumeDefaultLayout', @(source, event) obj.AppViewOrganizer.showDefaultMapBuilderFigureDocuments(0));
            addlistener(obj.MapBuilderTab, 'MapBuilderTab_ResumeDefaultLayout_1', @(source, event) obj.AppViewOrganizer.showDefaultMapBuilderFigureDocuments(1));
            obj.MapBuilderTab.RunButton.ButtonPushedFcn = @(source, event)  obj.triageRunPause(); % must use the toolstrip button's OEM ButtonPushedFcn callback

            addlistener(obj.MapBuilderTab, 'MapBuilderTab_BringUpModificationTab_Incremental', @(source, event) obj.startModifyingIncremental );
            addlistener(obj.MapBuilderTab, 'MapBuilderTab_BringUpModificationTab_LoopClosure', @(source, event) obj.startModifyingLoopClosure );

            obj.AppViewOrganizer.LoopClosureFigDoc.PrevLoopClosureStepper.ButtonPushedFcn = @(source, event) obj.prevLCStepperCallback();
            obj.AppViewOrganizer.LoopClosureFigDoc.NextLoopClosureStepper.ButtonPushedFcn = @(source, event) obj.nextLCStepperCallback();

            addlistener(obj.MapBuilderTab, 'MapBuilderTab_RefreshToolstrip_MappingPaused', @(source, event) obj.refreshToolstripAndFigureDocsForMappingPauseState);
            addlistener(obj.MapBuilderTab, 'MapBuilderTab_RefreshToolstrip_Mapped', @(source, event) obj.refreshToolstripAndFigureDocsForMappedState);
            addlistener(obj.MapBuilderTab, 'MapBuilderTab_InitiateSyncFast', @(source, event) obj.executeSyncFast);
            addlistener(obj.MapBuilderTab, 'MapBuilderTab_InitiateSync', @(source, event) obj.executeSync);

            addlistener(obj.MapBuilderTab, 'MapBuilderTab_ExportOccMapToWS', @(source, event) obj.bringUpExportMapToWSDialog());
            addlistener(obj.MapBuilderTab, 'MapBuilderTab_ExportOccMapToFile', @(source, event) obj.bringUpExportMapToFileDialog());

            addlistener(obj.MapBuilderTab, 'MapBuilderTab_RequestOpenSession', @(source, event) obj.handleOpenSessionRequest());
            addlistener(obj.MapBuilderTab, 'MapBuilderTab_RequestSaveSession', @(source, event) obj.handleSaveSessionRequest());
            addlistener(obj.MapBuilderTab, 'MapBuilderTab_RequestSaveSessionAs', @(source, event) obj.handleSaveSessionAsRequest());

            addlistener(obj.SettingsDialog, 'SettingsDialog_ApplyCurrentSettings', @(src, evt) obj.applyCurrentSettings());

            addlistener(obj.AppViewOrganizer.LoopClosureFigDoc, 'FigureDocument_KeyboardRequest', @(src, evt) obj.handleKeyPressCombo(evt.Vector));
            addlistener(obj.AppViewOrganizer.IncrementalFigDoc, 'FigureDocument_KeyboardRequest', @(src, evt) obj.handleKeyPressCombo(evt.Vector));
            addlistener(obj.AppViewOrganizer.MapFigDoc, 'FigureDocument_KeyboardRequest', @(src, evt) obj.handleKeyPressCombo(evt.Vector));
        end
    end


    % listen to Model events
    methods
        function addModelListeners(obj)
        %addModelListeners
            addlistener(obj.MapFigureSliderModel, 'SliderModel_CurrentValueUpdated', @(source, event) obj.updateFigDocs(event.Vector));
            addlistener(obj.MapBuilderModel, 'MapBuilderModel_RequestRefreshBadge', @(source, event) obj.updateFigDocs(event.Vector));

            addlistener(obj.MapBuilderModel, 'IsPoseGraphDirty', 'PostSet', @(source, event) obj.enableDisableSyncButton(event.AffectedObject.IsPoseGraphDirty));
            addlistener(obj.ModificationModel, 'ModificationModel_RequestRefreshMapSlider', @(source, event) obj.updateScanIdToStartSync(event.Vector));

            addlistener(obj.MapBuilderModel, 'IsAppDirty', 'PostSet', @(source, event) obj.updateAppTitle(event.AffectedObject.IsAppDirty));
        end
    end


    methods

        function investigateBag(obj, bagName)
        %investigateBag
            import nav.slamapp.internal.eventdata.*
            import nav.slamapp.internal.*
            obj.AppViewOrganizer.freeze();
            
            % bring app to front after bag selection
            obj.AppViewOrganizer.ToolGroup.bringToFront;
            dlgs = nav.slamapp.internal.Dialogs(obj.AppViewOrganizer.ToolGroup);
            if obj.StateMachineModel.State ~= States.Init
                hasThirdChoice = true;
                [selection, possibleSelections] = dlgs.questionDialog('SaveSession', obj.AppViewOrganizer.MapBuilderTab.retrieveMsg('NewImportHint'), hasThirdChoice);
                
                switch(selection)
                  case possibleSelections{1}
                    isSuccess = obj.saveSessionWithErrorHandling;
                    if ~isSuccess
                        return;
                    end
                    obj.BagImporterModel.clean;
                    obj.WSImporterModel.clean;
                    obj.MapBuilderModel.clean;
                    obj.StateMachineModel.PrevState = obj.StateMachineModel.State;
                    obj.StateMachineModel.State = States.Init;
                  case possibleSelections{2}
                    obj.BagImporterModel.clean;
                    obj.WSImporterModel.clean;
                    obj.MapBuilderModel.clean;
                    obj.StateMachineModel.PrevState = obj.StateMachineModel.State;
                    obj.StateMachineModel.State = States.Init;
                  otherwise
                    obj.AppViewOrganizer.thaw();
                    return;
                end
            end

            [isSuccess, errTag, errID] = obj.extractBagInfo(bagName);

            if isSuccess
                obj.StateMachineModel.toLoadingBag();
                pause(0.1);
                obj.BagImporterModel.notify('BagImporterModel_RequestUpdateBagImporterTab');
                
                obj.AppViewOrganizer.OdomFigDoc.clearOdom();
            else
                dlgs.errorMsgDialog(errTag, errID);
                obj.AppViewOrganizer.thaw();
            end

        end
        
        function switchToWSImporterTab(obj)
            %switchToWSImporterTab
            
            import nav.slamapp.internal.eventdata.*
            import nav.slamapp.internal.*
            obj.AppViewOrganizer.freeze();
            
            dlgs = nav.slamapp.internal.Dialogs(obj.AppViewOrganizer.ToolGroup);
            if obj.StateMachineModel.State ~= States.Init
                hasThirdChoice = true;
                % ask the user if they want to save the current work before starting to import new data
                [selection, possibleSelections] = dlgs.questionDialog('SaveSession', obj.AppViewOrganizer.MapBuilderTab.retrieveMsg('NewImportHint'), hasThirdChoice);

                switch(selection)
                  case possibleSelections{1}
                    isSuccess = obj.saveSessionWithErrorHandling;
                    if ~isSuccess
                        return;
                    end
                    obj.BagImporterModel.clean;
                    obj.WSImporterModel.clean;
                    obj.WSImporterModel.resetUserConfig();
                    obj.MapBuilderModel.clean;
                    obj.StateMachineModel.PrevState = obj.StateMachineModel.State;
                    obj.StateMachineModel.State = States.Init;
                  case possibleSelections{2}
                    obj.BagImporterModel.clean;
                    obj.WSImporterModel.clean;
                    obj.WSImporterModel.resetUserConfig();
                    obj.MapBuilderModel.clean;
                    obj.StateMachineModel.PrevState = obj.StateMachineModel.State;
                    obj.StateMachineModel.State = States.Init;
                  otherwise
                    obj.AppViewOrganizer.thaw();
                    return;
                end

            end

            obj.WSImporterModel.updateAvailableScansAndPosesVariables();
            obj.ScanFigureSliderModel.resetMinMaxValue(1, 1);
            obj.WSImporterModel.notify('WorkspaceImporterModel_RequestUpdateWSImporterTab');
           
            obj.AppViewOrganizer.OdomFigDoc.clearOdom();
            obj.AppViewOrganizer.ScanFigDoc.clearScan();
            
            obj.StateMachineModel.toLoadingFromWorkspace();
            obj.AppViewOrganizer.thaw();
        end
            
        function [isSuccess, errTag, errID] = extractBagInfo(obj, bagName)
        %extractBagInfo
            [isSuccess, errTag, errID, fileSize, startTime, endTime, topics] = obj.BagImporterModel.checkBag(bagName);
            [~, name, ext] = fileparts(bagName);

            if fileSize == -1
                fileSizeStr = obj.AppViewOrganizer.InfoFigDoc.retrieveMsg('FileSizeUnknown');
            else
                fileSizeStr = sprintf('%.2f  MB', fileSize);
            end

            if isSuccess
                bagNameString = [name, ext, ' (', fileSizeStr,')'];
                startTimeString = char(startTime);
                endTimeString = char(endTime);
                obj.BagImporterModel.InfoStrings = [bagNameString; startTimeString; endTimeString; topics];
            end
        end

        function triageRunPause(obj)
        %triageRunPause
            obj.MapBuilderModel.IsAppDirty = true;
            if obj.Count <= 1
                obj.Count = obj.Count + 1;
                obj.MapBuilderModel.toggleRunPause();

                obj.MapBuilderTab.setRunButtonIconAndText(obj.MapBuilderModel.IsRunning);
                if obj.MapBuilderModel.IsRunning
                    obj.AppViewOrganizer.freezeWorkingArea();
                else
                    obj.AppViewOrganizer.thawWorkingArea();
                end
                
                drawnow;

                if obj.MapBuilderModel.IsRunning
                    obj.continueMapBuilding();
                    obj.Count = obj.Count - 1;
                else
                    obj.pauseMapBuilding();
                    obj.Count = obj.Count - 1;
                end

            end

        end


        function initMapBuilding(obj)
        %initMapBuilding

            obj.MapBuilderModel.initialize( obj.SettingsModel.Applied);
            obj.AppViewOrganizer.IncrementalFigDoc.setXYLimits( obj.SettingsModel.Applied.LidarRange);
            obj.AppViewOrganizer.LoopClosureFigDoc.setXYLimits( obj.SettingsModel.Applied.LidarRange);
        end

        function continueMapBuilding(obj)
        %continueMapBuilding

            if strcmp(obj.StateMachineModel.State, 'SensorDataReady')
                obj.initMapBuilding();
                obj.MapFigureSliderModel.resetMinMaxValue(1, obj.MapBuilderModel.NumScans);
                obj.AppViewOrganizer.MapFigDoc.preloadScans(obj.MapBuilderModel.Scans);
                obj.AppViewOrganizer.MapFigDoc.show;
                obj.AppViewOrganizer.IncrementalFigDoc.show();
                obj.AppViewOrganizer.LoopClosureFigDoc.show();
            end

            obj.StateMachineModel.toMapping();

            while (obj.MapBuilderModel.FrontScanIdx <= obj.MapBuilderModel.NumScans) && obj.MapBuilderModel.IsRunning

                obj.MapFigureSliderModel.updateMaxAllowedValue(obj.MapBuilderModel.FrontScanIdx);
                obj.MapFigureSliderModel.updateCurrentValue(obj.MapBuilderModel.FrontScanIdx);
                obj.MapFigureSliderModel.updateSyncStartValue(obj.MapBuilderModel.FrontScanIdx);

                [scanIDs, poses, lcEdges] = obj.MapBuilderModel.step();

                obj.AppViewOrganizer.MapFigDoc.showCurrentMap(scanIDs, poses, lcEdges);

                obj.updateIncrementalPlot(obj.MapBuilderModel.FrontScanIdx - 1);
                obj.updateLoopClosurePlot(obj.MapBuilderModel.FrontScanIdx - 1);
                drawnow;

            end

            if obj.MapBuilderModel.FrontScanIdx > obj.MapBuilderModel.NumScans
                obj.MapBuilderModel.IsRunning = 0;
                obj.StateMachineModel.toMapped();
                % un freeze the working area that was frozen while building
                % the map
                obj.AppViewOrganizer.thawWorkingArea();
            end

        end


        function pauseMapBuilding(obj)
        %pauseMapBuilding
            obj.MapBuilderModel.IsRunning = 0;
            obj.StateMachineModel.toMappingPaused;

        end

        function updateIncrementalPlot(obj, v)
        %updateIncrementalPlot
            [clearInc, refScan, currScan, relPose, scanIdPair] = obj.MapBuilderModel.retrieveIncrementalScanPairAndConstraint(v);
            if clearInc
                obj.AppViewOrganizer.IncrementalFigDoc.clearScanPair();
            else
                obj.AppViewOrganizer.IncrementalFigDoc.drawScanPair(refScan, currScan, relPose, scanIdPair);
            end

            if clearInc || obj.MapBuilderModel.IsRunning
                obj.MapBuilderTab.ModifyIncrementalButton.Enabled = false;
            else
                obj.MapBuilderTab.ModifyIncrementalButton.Enabled = true;
            end

            if ~isempty(scanIdPair) && ~isempty(obj.MapBuilderModel.ModifiedEdges) && ismember(scanIdPair, obj.MapBuilderModel.ModifiedEdges, 'row')
                obj.AppViewOrganizer.IncrementalFigDoc.BadgeModify.Visible = 'on';
            else
                obj.AppViewOrganizer.IncrementalFigDoc.BadgeModify.Visible = 'off';
            end

        end

        function updateLoopClosurePlot(obj, userFacingScanId, varargin)
        %updateLoopClosurePlot

            [noLoopClosure, userFacingScanIdPairs] = obj.MapBuilderModel.getAvailableLoopClosureEdgesAssociatedWith(userFacingScanId);


            % update plots
            scanIdPair = [];
            if noLoopClosure
                obj.AppViewOrganizer.LoopClosureFigDoc.clearScanPair();
                obj.MapBuilderModel.LCScanIdPairSelected = [];
            else
                userFacingScanIdPairsSorted = sortrows(userFacingScanIdPairs, 1); % sort according to the first row
                if nargin == 2
                    scanIdPair = userFacingScanIdPairsSorted(1,:); % always display the first loop closure (if not specified and there are multiple) during map building
                else
                    scanIdPair = [varargin{1}, userFacingScanId];
                end
                obj.MapBuilderModel.LCScanIdPairSelected = scanIdPair;
                [refScan, currScan, relPose] = obj.MapBuilderModel.getLoopClosureScanPairAndRelPose(scanIdPair);

                obj.AppViewOrganizer.LoopClosureFigDoc.drawScanPair(refScan, currScan, relPose, scanIdPair);
            end

            % update toolstrip button
            if noLoopClosure || obj.MapBuilderModel.IsRunning
                obj.MapBuilderTab.ModifyLoopClosureButton.Enabled = false;
            else
                obj.MapBuilderTab.ModifyLoopClosureButton.Enabled = true;
            end

            % update badges
            if (~isempty(scanIdPair) && ~isempty(obj.MapBuilderModel.ModifiedEdges) && ismember(scanIdPair, obj.MapBuilderModel.ModifiedEdges, 'row')) || ...
                    (~isempty(scanIdPair) && ~isempty(obj.MapBuilderModel.IgnoredEdges) && ismember(scanIdPair, obj.MapBuilderModel.IgnoredEdges, 'row'))
                obj.AppViewOrganizer.LoopClosureFigDoc.BadgeModify.Visible = 'on';

            else
                obj.AppViewOrganizer.LoopClosureFigDoc.BadgeModify.Visible = 'off';
            end

            % adjust dimming
            if (~isempty(scanIdPair) && ~isempty(obj.MapBuilderModel.IgnoredEdges) && ismember(scanIdPair, obj.MapBuilderModel.IgnoredEdges, 'row'))
                obj.AppViewOrganizer.LoopClosureFigDoc.dim();
            else
                obj.AppViewOrganizer.LoopClosureFigDoc.lightUp();
            end

        end

        function updateFigDocs(obj, v)
        %updateFigDocs
            if ~obj.MapBuilderModel.IsRunning
                if v > numel(obj.MapBuilderModel.History)
                    return;
                end
                s = obj.MapBuilderModel.History{v};
                obj.AppViewOrganizer.MapFigDoc.showCurrentMap(s.ScanIDs, s.Poses, s.LcEdges);
                obj.updateIncrementalPlot(v);
                if isempty(obj.MapBuilderModel.ScanIdPairToShow)
                    obj.updateLoopClosurePlot(v);
                else
                    obj.updateLoopClosurePlot(v, obj.MapBuilderModel.ScanIdPairToShow(1) );
                    obj.MapBuilderModel.ScanIdPairToShow = []; % clear
                end
            end
        end

        function startModifyingIncremental(obj)
        %startModifyingIncremental
            obj.AppViewOrganizer.IncrementalFigDoc.allModesOff;

            userFacingScanId = obj.MapFigureSliderModel.CurrentValue;

            if userFacingScanId == 1 || ~obj.MapBuilderModel.History{userFacingScanId}.IsScanAccepted

                return;
            end
            v = obj.MapBuilderModel.InboundMap(userFacingScanId);

            edgeId = obj.MapBuilderModel.LidarSLAMProcessor.PoseGraph.findEdgeID([v-1, v]);

            relPose = obj.MapBuilderModel.LidarSLAMProcessor.PoseGraph.edgeConstraints(edgeId);
            xyLimit = 2*obj.MapBuilderModel.LidarSLAMProcessor.MaxLidarRange;

            userFacingScanIdPair = obj.MapBuilderModel.OutboundMap(v-1: v);
            isIncremental = true;
            isIgnored = false;
            obj.ModificationModel.prepareModification(isIncremental, isIgnored, relPose, xyLimit, userFacingScanIdPair);

            obj.ModificationModel.TabTitleName = obj.AppViewOrganizer.ModificationTab.getTitle(userFacingScanId);

            if strcmp(obj.AppViewOrganizer.IncrementalFigDoc.BadgeModify.Visible, 'off')
                obj.AppViewOrganizer.ModificationTab.ResetButton.Enabled = false;
            else
                obj.AppViewOrganizer.ModificationTab.ResetButton.Enabled = true;
            end
            obj.StateMachineModel.toModifyingIncremental;
        end

        function startModifyingLoopClosure(obj)
        %startModifyingLoopClosure

            obj.AppViewOrganizer.LoopClosureFigDoc.allModesOff;

            userFacingScanId = obj.MapFigureSliderModel.CurrentValue;
            if userFacingScanId == 1 || ~obj.MapBuilderModel.History{userFacingScanId}.IsScanAccepted

                return;
            end

            scanIdPair = obj.MapBuilderModel.LCScanIdPairSelected;

            lcEdge = obj.MapBuilderModel.InboundMap(scanIdPair);

            edgeId = obj.MapBuilderModel.LidarSLAMProcessor.PoseGraph.findEdgeID(lcEdge);
            relPose = obj.MapBuilderModel.LidarSLAMProcessor.PoseGraph.edgeConstraints(edgeId);
            xyLimit = 2*obj.MapBuilderModel.LidarSLAMProcessor.MaxLidarRange;

            % is the current loop closure ignored?
            if ~isempty(scanIdPair) && ~isempty(obj.MapBuilderModel.IgnoredEdges) && ismember(scanIdPair, obj.MapBuilderModel.IgnoredEdges, 'row')
                isIgnored = true;
            else
                isIgnored = false;
            end

            isIncremental = false;
            obj.ModificationModel.prepareModification(isIncremental, isIgnored, relPose, xyLimit, scanIdPair);

            obj.ModificationModel.TabTitleName = obj.AppViewOrganizer.ModificationTab.getTitle(scanIdPair(1), scanIdPair(2));

            obj.AppViewOrganizer.LoopClosureFigDoc.PrevLoopClosureStepper.Visible = 'off';
            obj.AppViewOrganizer.LoopClosureFigDoc.NextLoopClosureStepper.Visible = 'off';

            if strcmp(obj.AppViewOrganizer.LoopClosureFigDoc.BadgeModify.Visible, 'off')
                obj.AppViewOrganizer.ModificationTab.ResetButton.Enabled = false;
            else
                obj.AppViewOrganizer.ModificationTab.ResetButton.Enabled = true;
            end

            obj.StateMachineModel.toModifyingLoopClosure;
        end


        function enableDisableSyncButtonMappingPaused(obj, isDirty)
        %enableDisableSyncButtonMappingPaused
            import matlab.ui.internal.toolstrip.*

            % continue_matlab_24 not usable in app container g2409144
            obj.AppViewOrganizer.MapBuilderTab.RunButton.Icon = Icon.CONTINUE_24;
            obj.AppViewOrganizer.MapBuilderTab.RunButton.Text = obj.MapBuilderTab.retrieveMsg('ContinueButtonName');
            obj.AppViewOrganizer.MapBuilderTab.RunButton.Description = obj.MapBuilderTab.retrieveMsg('ContinueButtonDescription');

            if isDirty
                obj.AppViewOrganizer.MapBuilderTab.SyncSplitButton.Enabled = true;
                obj.AppViewOrganizer.MapBuilderTab.RunButton.Enabled = false;
                obj.AppViewOrganizer.MapFigDoc.Banner.Visible = 'on';
            else
                obj.AppViewOrganizer.MapBuilderTab.SyncSplitButton.Enabled = false;
                obj.AppViewOrganizer.MapBuilderTab.RunButton.Enabled = true;
                obj.AppViewOrganizer.MapFigDoc.Banner.Visible = 'off';
            end
        end

        function enableDisableSyncButtonMapped(obj, isDirty)
        %enableDisableSyncButtonMapped
            import matlab.ui.internal.toolstrip.*

            obj.AppViewOrganizer.MapBuilderTab.RunButton.Icon = Icon.RUN_24;
            obj.AppViewOrganizer.MapBuilderTab.RunButton.Text = obj.MapBuilderTab.retrieveMsg('RunButtonName');
            obj.AppViewOrganizer.MapBuilderTab.RunButton.Description = obj.MapBuilderTab.retrieveMsg('RunButtonDescription');

            if isDirty
                obj.AppViewOrganizer.MapBuilderTab.SyncSplitButton.Enabled = true;
                obj.AppViewOrganizer.MapBuilderTab.RunButton.Enabled = false;
                obj.AppViewOrganizer.MapFigDoc.Banner.Visible = 'on';
            else
                obj.AppViewOrganizer.MapBuilderTab.SyncSplitButton.Enabled = false;
                obj.AppViewOrganizer.MapBuilderTab.RunButton.Enabled = false;
                obj.AppViewOrganizer.MapFigDoc.Banner.Visible = 'off';
            end
        end

        function enableDisableSyncButton(obj, isDirty)
        %enableDisableSyncButton
            obj.AppViewOrganizer.MapFigDoc.InfoText.Text = obj.AppViewOrganizer.MapFigDoc.retrieveMsg('ReminderToSyncMap');
            if obj.StateMachineModel.State == nav.slamapp.internal.States.MappingPaused
                enableDisableSyncButtonMappingPaused(obj, isDirty);
            else
                enableDisableSyncButtonMapped(obj, isDirty);
            end

        end


        function refreshToolstripAndFigureDocsForMappingPauseState(obj)
        %refreshToolstripAndFigureDocsForMappingPauseState
            import matlab.ui.internal.toolstrip.*
            % continue_matlab_24 not usable in app container g2409144
            obj.MapBuilderTab.RunButton.Icon = Icon.CONTINUE_24;
            obj.MapBuilderTab.RunButton.Text = obj.MapBuilderTab.retrieveMsg('ContinueButtonName');
            obj.MapBuilderTab.RunButton.Description = obj.MapBuilderTab.retrieveMsg('ContinueButtonDescription');

            obj.MapBuilderTab.setWidgetAvailability(3907); % this is [1 1 1 1 0 1 0 0 0 0 1 1] with left msb
            obj.enableDisableSyncButtonMappingPaused(obj.MapBuilderModel.IsPoseGraphDirty);
            obj.updateFigDocs(obj.MapFigureSliderModel.CurrentValue);


        end

        function refreshToolstripAndFigureDocsForMappedState(obj)
        %refreshToolstripAndFigureDocsForMappedState

            obj.MapBuilderTab.setWidgetAvailability(3843); % this is [1 1 1 1 0 0 0 0 0 0 1 1] with left msb
            obj.enableDisableSyncButtonMapped(obj.MapBuilderModel.IsPoseGraphDirty);
            obj.updateFigDocs(obj.MapFigureSliderModel.CurrentValue);

        end

        function updateScanIdToStartSync(obj, scanIdPair)
        %updateScanIdToSync
            if scanIdPair(2) < obj.MapFigureSliderModel.SyncStartValue
                obj.MapFigureSliderModel.updateSyncStartValue(scanIdPair(2));
            end

        end

        function executeSyncFast(obj)
        %executeSyncFast
            obj.AppViewOrganizer.freeze();
            obj.MapBuilderModel.IsAppDirty = true;

            obj.MapBuilderModel.reoptimizePoseGraph(obj.MapFigureSliderModel.SyncStartValue);
            obj.MapFigureSliderModel.updateSyncStartValue(obj.MapBuilderModel.FrontScanIdx - 1);
            obj.MapFigureSliderModel.updateCurrentValue(obj.MapBuilderModel.FrontScanIdx - 1);
            obj.AppViewOrganizer.thaw();
        end

        function executeSync(obj)
        %executeSync
            obj.AppViewOrganizer.freeze();
            obj.MapBuilderModel.IsAppDirty = true;

            % inform syncing in progress
            obj.AppViewOrganizer.MapFigDoc.InfoText.Text = obj.AppViewOrganizer.MapFigDoc.retrieveMsg('ReminderSyncInProgress');

            % reset FrontScanIdx
            if obj.MapFigureSliderModel.SyncStartValue == 1
                obj.MapFigureSliderModel.SyncStartValue = 2;
            end
            syncStartVal = obj.MapFigureSliderModel.SyncStartValue;
            obj.MapBuilderModel.FrontScanIdx = syncStartVal;
            obj.MapBuilderModel.initSync(syncStartVal);


            lslam = obj.MapBuilderModel.LidarSLAMProcessor;
            lslam.UseCertifiedData = true;

            while (obj.MapBuilderModel.FrontScanIdx <= obj.MapFigureSliderModel.MaxAllowedValue)

                obj.MapBuilderModel.findCertifiedEdgesIfAvailable(obj.MapBuilderModel.FrontScanIdx);

                [scanIDs, poses, lcEdges] = obj.MapBuilderModel.step(); % it will auto-increment FrontScanIdx

                obj.MapFigureSliderModel.updateCurrentValue(obj.MapBuilderModel.FrontScanIdx-1);
                obj.MapFigureSliderModel.updateSyncStartValue(obj.MapBuilderModel.FrontScanIdx-1);


                obj.AppViewOrganizer.MapFigDoc.showCurrentMap(scanIDs, poses, lcEdges);

                obj.updateIncrementalPlot(obj.MapBuilderModel.FrontScanIdx - 1);
                obj.updateLoopClosurePlot(obj.MapBuilderModel.FrontScanIdx - 1);
                drawnow;

            end

            lslam.UseCertifiedData = false;

            % disable the sync button
            obj.MapBuilderTab.SyncSplitButton.Enabled = false;

            % hide the banner
            obj.AppViewOrganizer.MapFigDoc.showBanner(0);

            % if in MappingPaused mode, turn the continue button back on
            if obj.StateMachineModel.State == nav.slamapp.internal.States.MappingPaused
                obj.MapBuilderTab.RunButton.Enabled = true;
            end

            obj.MapBuilderModel.IsPoseGraphDirty = false;

            obj.AppViewOrganizer.thaw();
        end

        function bringUpExportMapToWSDialog(obj)
        %bringUpExportMapToWSDialog
            exportDlg = nav.slamapp.internal.ExportDialog(obj.AppViewOrganizer.ToolGroup);
            exportDlg.prepareExportMapDialog(obj.MapFigureSliderModel.CurrentValue, obj.MapBuilderModel.FrontScanIdx-1);
            uiwait(exportDlg.Dialog);
            obj.AppViewOrganizer.freeze();
            if ~isempty(exportDlg.MapVarName)
                if exportDlg.NeedLatestMap
                    map = obj.MapBuilderModel.getOccupancyGridMap();
                else
                    s = obj.MapBuilderModel.History{obj.MapFigureSliderModel.CurrentValue};
                    map = buildMap(obj.MapBuilderModel.Scans(s.ScanIDs), s.Poses, ...
                                   obj.MapBuilderModel.LidarSLAMProcessor.MapResolution, ...
                                   obj.MapBuilderModel.LidarSLAMProcessor.MaxLidarRange);
                end
                assignin('base', exportDlg.MapVarName, map);
            end

            obj.AppViewOrganizer.thaw();
        end

        function bringUpExportMapToFileDialog(obj)
        %bringUpExportMapToFileDialog
            [fileName, path] = uiputfile('*.mat', obj.MapBuilderTab.retrieveMsg('SaveToFileDialogTitle'));
            if ~isempty(fileName) && ~isequal(fileName,0) % fileName is 0 when user presses Cancel
                myOccMap = obj.MapBuilderModel.getOccupancyGridMap();
                save(fullfile(path, fileName), 'myOccMap');
            end
        end

        function updateNextScanPairToShow(obj, stepToPrevLC)
        %updateNextScanPairToShow Determine which loop closure scan
        %   pair to show next when LC stepper is clicked

        % stepToPrevLC:
        % 1 -- step to previous loop closure scan pair
        % 0 -- step to next loop closure scan pair

            currScanId = obj.MapFigureSliderModel.CurrentValue;
            noLC = obj.MapBuilderModel.getAvailableLoopClosureEdgesAssociatedWith(currScanId);

            lcEdgeIds = obj.MapBuilderModel.LidarSLAMProcessor.PoseGraph.LoopClosureEdgeIDs;
            lcEdges = obj.MapBuilderModel.LidarSLAMProcessor.PoseGraph.edges(lcEdgeIds);

            allScanIdPairsSorted = sortrows([obj.MapBuilderModel.OutboundMap(lcEdges(:,1))', ...
                                obj.MapBuilderModel.OutboundMap(lcEdges(:,2))'], [2, 1]); % first sort according to 2nd column, then according to 1st column

            obj.MapBuilderModel.ScanIdPairToShow = [];
            if noLC
                if stepToPrevLC
                    locs = find(allScanIdPairsSorted(:,2) < currScanId);
                    if ~isempty(locs)
                        obj.MapBuilderModel.ScanIdPairToShow = allScanIdPairsSorted(locs(end), :);
                    end
                else
                    locs = find(allScanIdPairsSorted(:,2) > currScanId);
                    if ~isempty(locs)
                        obj.MapBuilderModel.ScanIdPairToShow = allScanIdPairsSorted(locs(1), :);
                    end
                end

            else
                currScanIdPair = obj.MapBuilderModel.LCScanIdPairSelected;
                [~, loc] = ismember(currScanIdPair, allScanIdPairsSorted, 'row');
                if stepToPrevLC && loc > 1
                    obj.MapBuilderModel.ScanIdPairToShow = allScanIdPairsSorted(loc-1, :);
                end
                if ~stepToPrevLC && loc < size(allScanIdPairsSorted,1)
                    obj.MapBuilderModel.ScanIdPairToShow = allScanIdPairsSorted(loc+1, :);
                end
            end

        end

        function prevLCStepperCallback(obj)
        %prevLCStepperCallback

            obj.updateNextScanPairToShow(1);

            if ~isempty(obj.MapBuilderModel.ScanIdPairToShow)
                obj.MapFigureSliderModel.updateCurrentValue(obj.MapBuilderModel.ScanIdPairToShow(2)); % ScanIdPairToShow is used internally
            end
        end

        function nextLCStepperCallback(obj)
        %nextLCStepperCallback
            obj.updateNextScanPairToShow(0);

            if ~isempty(obj.MapBuilderModel.ScanIdPairToShow)
                obj.MapFigureSliderModel.updateCurrentValue(obj.MapBuilderModel.ScanIdPairToShow(2)); % ScanIdPairToShow is used internally
            end
        end


        function [appData1, appData2, appData3] = serialize(obj, filename)
        %serialize Save current app model properties to a mat file

            % appData1 includes all the regular (non-set-observable, non-transient, non-observable and public) model properties
            % appData2 includes set-observable model properties
            % appData3 includes session

            % major app models
            [appData1.MapBuilderModelData,   appData2.MapBuilderModelData]   = obj.MapBuilderModel.saveProperties;
            [appData1.BagImporterModelData,  appData2.BagImporterModelData]  = obj.BagImporterModel.saveProperties;
            [appData1.WSImporterModelData,   appData2.WSImporterModelData]   = obj.WSImporterModel.saveProperties;
            [appData1.StateMachineModelData, appData2.StateMachineModelData] = obj.StateMachineModel.saveProperties;
            [appData1.ModificationModelData, appData2.ModificationModelData] = obj.ModificationModel.saveProperties;
            appData1.SettingsModelData = obj.SettingsModel.saveProperties;

            % slider models (all observable)
            appData2.MapFigureSliderModelData = obj.MapFigureSliderModel.report;
            appData2.ScanFigureSliderModelData = obj.ScanFigureSliderModel.report;

            % additional information
            appData3.SessionTime = char(datetime('now'));
            appData3.MLVersion = version;
            try
                appData3.ComputerName = char(matlab.net.internal.InetAddress.getLocalHost.getHostName);
                appData3.User = char(matlab.net.internal.InetAddress.getLocalHost.getHostName);
            catch
                appData3.ComputerName = [];
                appData3.User = [];
            end

            % some figure axes visibility
            appData3.MapFigAxVisible = obj.AppViewOrganizer.MapFigDoc.Axes.Visible;
            appData3.LcFigAxVisible = obj.AppViewOrganizer.LoopClosureFigDoc.Axes.Visible;
            appData3.IncFigAxVisible = obj.AppViewOrganizer.IncrementalFigDoc.Axes.Visible;

            appData3.MapFigBannerColor = obj.AppViewOrganizer.MapFigDoc.InfoText.BackgroundColor;

            slamAppSession.AppData1 = appData1;
            slamAppSession.AppData2 = appData2;
            slamAppSession.AppData3 = appData3;

            save(filename, 'slamAppSession');
        end

        function deserialize(obj, sessionFile)
        %deserialize Restore app state from a previously saved session file (.mat)

            data = load(sessionFile, 'slamAppSession');

            sessionData = data.slamAppSession;
            appData1 = sessionData.AppData1;
            appData2 = sessionData.AppData2;
            appData3 = sessionData.AppData3;

            obj.AppViewOrganizer.freeze();
            obj.appViewReset(appData3);

            obj.SettingsModel.loadProperties(appData1.SettingsModelData);
            obj.BagImporterModel.loadProperties(appData1.BagImporterModelData, appData2.BagImporterModelData);
            
            if isfield(appData1, 'WSImporterModelData') % WSImporterModel is introduced in 20b
                obj.WSImporterModel.loadProperties(appData1.WSImporterModelData, appData2.WSImporterModelData);
            else
                defaultWSImporterModel = nav.slamapp.internal.WorkspaceImporterModel();
                [a1, a2] = defaultWSImporterModel.saveProperties;
                obj.WSImporterModel.loadProperties(a1, a2);
            end
            obj.MapBuilderModel.loadProperties(appData1.MapBuilderModelData, appData2.MapBuilderModelData);

            % manually cache all scan graphic objects in map figure
            obj.AppViewOrganizer.MapFigDoc.preloadScans(obj.MapBuilderModel.Scans);

            obj.ModificationModel.loadProperties(appData1.ModificationModelData, appData2.ModificationModelData);
            obj.StateMachineModel.loadProperties(appData1.StateMachineModelData, appData2.StateMachineModelData);

            if ~isempty(obj.MapBuilderModel.Scans) % don't update map figure slider if the data is not ready
                obj.MapFigureSliderModel.reinstate(appData2.MapFigureSliderModelData);
            end

            if ~isempty(obj.BagImporterModel.ScanBagSelection) % don't update scan figure slider if user starts app directly from workspace data
                obj.ScanFigureSliderModel.reinstate(appData2.ScanFigureSliderModelData);
            end

            % unsubscribe figure callbacks if the app is currently NOT
            % under modification tab
            if ~(obj.StateMachineModel.State == nav.slamapp.internal.States.ModifyingIncremental || ...
                 obj.StateMachineModel.State == nav.slamapp.internal.States.ModifyingLoopClosure)
                obj.AppViewOrganizer.LoopClosureFigDoc.Figure.WindowButtonDownFcn = [];
                obj.AppViewOrganizer.IncrementalFigDoc.Figure.WindowButtonDownFcn = [];
            end

            obj.appViewFinalTweak(appData3);

            obj.AppViewOrganizer.thaw();
        end

        function appViewReset(obj, appData3)
        %appViewReset Reset the app view organizer state

            obj.AppViewOrganizer.wipeMainWorkingArea;
            obj.AppViewOrganizer.IncrementalFigDoc.setXYLimits( obj.SettingsModel.Applied.LidarRange);
            obj.AppViewOrganizer.LoopClosureFigDoc.setXYLimits( obj.SettingsModel.Applied.LidarRange);

            obj.AppViewOrganizer.MapFigDoc.show(appData3.MapFigAxVisible);
            obj.AppViewOrganizer.IncrementalFigDoc.show(appData3.LcFigAxVisible);
            obj.AppViewOrganizer.LoopClosureFigDoc.show(appData3.IncFigAxVisible);
        end

        function appViewFinalTweak(obj, appData3)
        %appViewFinalTweak Additional app view adjustments after all
        %   model data is set

        % special treatment for odom plot
            if ~isempty(obj.BagImporterModel.Ts)
                obj.AppViewOrganizer.OdomFigDoc.drawOdom(obj.BagImporterModel.Ts);
            else
                obj.AppViewOrganizer.OdomFigDoc.clearOdom();
            end

            % banner and info text colors
            obj.AppViewOrganizer.MapFigDoc.InfoText.BackgroundColor = appData3.MapFigBannerColor;
            obj.AppViewOrganizer.MapFigDoc.Banner.BackgroundColor = appData3.MapFigBannerColor;

            % if in modification tab, enable restore auto button if the
            % current scan pair has been previously modified
            if ~ismember(obj.ModificationModel.UserFacingScanIdPair, obj.MapBuilderModel.ModifiedEdges)
                obj.AppViewOrganizer.ModificationTab.ResetButton.Enabled = false;
            end

            % if there are already loop closures, turn on the loop closure
            % navigation buttons
            if ~isempty(obj.MapBuilderModel.LidarSLAMProcessor) && obj.MapBuilderModel.LidarSLAMProcessor.PoseGraph.NumLoopClosureEdges
                obj.AppViewOrganizer.LoopClosureFigDoc.NextLoopClosureStepper.Visible = 'on';
                obj.AppViewOrganizer.LoopClosureFigDoc.PrevLoopClosureStepper.Visible = 'on';
            end

            % if the loop closure is
            % 1) tentatively ignored under modification tab but NOT yet
            %    confirmed OR
            % 2) confirmed ignored under map building tab
            % dim the scan pairs
            import nav.slamapp.internal.States
            obj.AppViewOrganizer.LoopClosureFigDoc.lightUp;
            if (~isempty(obj.ModificationModel.Tentative) && logical(obj.ModificationModel.Tentative.IsIgnored && obj.StateMachineModel.State == States.ModifyingLoopClosure) ...
                || (~isempty(obj.ModificationModel.IsIgnored) && logical(obj.ModificationModel.IsIgnored) && obj.StateMachineModel.State ~= States.ModifyingLoopClosure))
                obj.AppViewOrganizer.LoopClosureFigDoc.dim;
            end
        end

        function handleOpenSessionRequest(obj)
        %handleOpenSessionRequest Callback for "Open Session" button click

            dlgs = nav.slamapp.internal.Dialogs(obj.AppViewOrganizer.ToolGroup);
            hasThirdChoice = true;
            if obj.MapBuilderModel.IsAppDirty
                [selection, possibleSelections] = dlgs.questionDialog('SaveSession', ' ', hasThirdChoice);
                switch selection
                  case possibleSelections{1} % request to save current session

                    isSuccess = obj.saveSessionWithErrorHandling;
                    if ~isSuccess
                        return;
                    end
                    % now open the second file selector asking which
                    % session file to open
                    sessionFileToOpen = selectFile(obj.MapBuilderTab, true, '*.mat', 'SessionFileExtensionHint', 'SelectSessionFileToOpenDialogTitle');
                    % bring app to the front after selecting the session file
                    obj.AppViewOrganizer.ToolGroup.bringToFront;
                    obj.openSessionWithErrorHandling(sessionFileToOpen);

                  case possibleSelections{2} % request to NOT save current session
                    sessionFileToOpen = selectFile(obj.MapBuilderTab, true, '*.mat', 'SessionFileExtensionHint', 'SelectSessionFileToOpenDialogTitle');
                    % bring app to the front after selecting the session file
                    obj.AppViewOrganizer.ToolGroup.bringToFront;
                    obj.openSessionWithErrorHandling(sessionFileToOpen);
                    obj.MapBuilderModel.IsAppDirty = false;
                end
            else
                sessionFileToOpen = selectFile(obj.MapBuilderTab, true, '*.mat', 'SessionFileExtensionHint', 'SelectSessionFileToOpenDialogTitle');
                % bring app to the front after selecting the session file
                obj.AppViewOrganizer.ToolGroup.bringToFront;
                obj.openSessionWithErrorHandling(sessionFileToOpen);

            end
        end

        function handleSaveSessionRequest(obj)
        %handleSaveSessionRequest Callback for "Save Session" button
        %   click and Save Session list item click
            obj.saveSessionWithErrorHandling;
        end

        function handleSaveSessionAsRequest(obj)
        %handleSaveSessionAsRequest Callback for "Save Session As" list
        %   item click
            sessionFileToSave = selectFile(obj.MapBuilderTab, false, '*.mat', 'SessionFileExtensionHint', 'SelectSessionFileToSaveDialogTitle');
            if ~isempty(sessionFileToSave)
                obj.saveSessionAsWithErrorHandling(sessionFileToSave);
            end
        end



        function openSessionWithErrorHandling(obj, fileName)
        %openSessionWithErrorHandling
            dlgs = nav.slamapp.internal.Dialogs(obj.AppViewOrganizer.ToolGroup);
            try
                obj.AppViewOrganizer.freeze();
                data = load(fileName); %precheck session file to prevent nonexistent field warning
                if ~isfield(data, 'slamAppSession')
                    error(obj.MapBuilderTab.retrieveMsg('InvalidSessionFile', fileName));
                end
                obj.deserialize(fileName);
                obj.SessionFileName = fileName;
                % the SessionFileName must be set prior to calling 'setAppTitle'
                obj.setAppTitle();
                obj.AppViewOrganizer.thaw();
            catch ME
                if strcmp(ME.identifier, 'MATLAB:load:emptyFileName')
                    obj.AppViewOrganizer.thaw();
                else
                    dlgs.errorMsgDialog('DeserializationFailed', [], ME.message);
                    defaultFile = fullfile(matlabroot, 'toolbox','nav', 'navslamapp','data','SLAMAppDefaultSession.mat');
                    obj.deserialize(defaultFile);
                    obj.SessionFileName = '';
                    % the SessionFileName must be set prior to calling 'setAppTitle'
                    obj.setAppTitle();
                    obj.AppViewOrganizer.thaw();
                end
            end
        end

        function isSuccess = saveSessionAsWithErrorHandling(obj, fileName)
        %saveSessionAsWithErrorHandling
            dlgs = nav.slamapp.internal.Dialogs(obj.AppViewOrganizer.ToolGroup);
            try
                obj.serialize(fileName);
                obj.SessionFileName = fileName;
                obj.setAppTitle();
                isSuccess = true;
                obj.MapBuilderModel.IsAppDirty = false;
            catch ME
                dlgs.errorMsgDialog('SerializationFailed', [], ME.message);
                isSuccess = false;
            end

        end

        function isSuccess = saveSessionWithErrorHandling(obj)
        %saveSessionWithErrorHandling
            if isempty(obj.SessionFileName)
                sessionFileToSave = selectFile(obj.MapBuilderTab, false, '*.mat', 'SessionFileExtensionHint', 'SelectSessionFileToSaveDialogTitle');
            else
                sessionFileToSave = obj.SessionFileName;
            end

            isSuccess = false;
            if ~isempty(sessionFileToSave)
                isSuccess = obj.saveSessionAsWithErrorHandling(sessionFileToSave);
            end
        end

        function setAppTitle(obj)
        %setAppTitle
            % obj.SessionFileName must be set prior to calling 'setAppTitle'
            if (isempty(obj.SessionFileName))
                obj.AppViewOrganizer.ToolGroup.Title = obj.AppViewOrganizer.retrieveMsg('AppTitle');
            else
                obj.AppViewOrganizer.ToolGroup.Title = [obj.AppViewOrganizer.retrieveMsg('AppTitle') ' - ' obj.SessionFileName];
            end
        end

        function applyCurrentSettings(obj)
        %applyCurrentSettings
            import nav.slamapp.internal.*
            if obj.SettingsModel.settingsAreChanged && (obj.StateMachineModel.State ~= States.SensorDataReady)
                obj.AppViewOrganizer.ToolGroup.Busy = true;
                dlgs = nav.slamapp.internal.Dialogs(obj.SettingsDialog.Dialog);
                hasThirdChoice = true;
                [selection, possibleSelections] = dlgs.questionDialog('SaveSession', obj.MapBuilderTab.retrieveMsg('NewSettingsHint'), hasThirdChoice);

                switch(selection)
                  case possibleSelections{1}
                    isSuccess = obj.saveSessionWithErrorHandling;
                    if ~isSuccess
                        return;
                    end
                    obj.MapBuilderModel.cleanPartial;
                    obj.StateMachineModel.PrevState = obj.StateMachineModel.State;
                    obj.StateMachineModel.State = States.SensorDataReady;
                  case possibleSelections{2}
                    obj.MapBuilderModel.cleanPartial;
                    obj.StateMachineModel.PrevState = obj.StateMachineModel.State;
                    obj.StateMachineModel.State = States.SensorDataReady;
                    otherwise
                      obj.AppViewOrganizer.ToolGroup.Busy = false;
                    return;
                end

            end
            if obj.SettingsModel.settingsAreChanged
                obj.MapBuilderModel.IsAppDirty = true;
            end
            obj.SettingsModel.writeToApplied();
            obj.AppViewOrganizer.ToolGroup.Busy = false;
        end

        function handleKeyPressCombo(obj, keyComboId)
        %handleKeyPressCombo

            if obj.MapBuilderModel.IsRunning
                return
            end

            import nav.slamapp.internal.*
            if obj.StateMachineModel.State ~= States.SensorDataReady && ...
                    obj.StateMachineModel.State ~= States.Mapping && ...
                    obj.StateMachineModel.State ~= States.MappingPaused && ...
                    obj.StateMachineModel.State ~= States.Mapped && ...
                    obj.StateMachineModel.State ~= States.Init

                return
            end

            switch(keyComboId)
              case 1
                obj.handleSaveSessionRequest();
              case 0
                obj.handleOpenSessionRequest();
            end
        end

        function updateAppTitle(obj, isDirty)
        %updateAppTitle

            title = obj.AppViewOrganizer.ToolGroup.Title;
            cleanTitle = strrep(title, '*', '');
            if isDirty
                obj.AppViewOrganizer.ToolGroup.Title = cleanTitle+"*";
            else
                obj.AppViewOrganizer.ToolGroup.Title = cleanTitle;
            end
        end
    end
end

classdef WorkspaceImporterController < handle
%This class is for internal use only. It may be removed in the future.

%WorkspaceImporterController This class encapsulates the Controller portion
%   of the Model/View/Controller design pattern used in the SLAM app for
%   importing data from MATLAB workspace

% Copyright 2020-2021 The MathWorks, Inc.

    properties
        WSImporterTab
        AppViewOrganizer

        WSImporterModel
        MapBuilderModel
        ScanFigureSliderModel
        StateMachineModel
    end

    methods
        function obj = WorkspaceImporterController(wsImporterModel, mapBuilderModel, scanFigSliderModel, stateMachineModel, wsImporterTab, appViewOrg )
        %WorkspaceImporterController Constructor

            obj.WSImporterTab = wsImporterTab;
            obj.AppViewOrganizer = appViewOrg;
            obj.WSImporterModel = wsImporterModel;
            obj.MapBuilderModel = mapBuilderModel;
            obj.ScanFigureSliderModel = scanFigSliderModel;
            obj.StateMachineModel = stateMachineModel;

            obj.addViewListeners();
            obj.addModelListeners();

        end

    end


    % listen to View events
    methods
        function addViewListeners(obj)
        %addViewListeners

            addlistener(obj.WSImporterTab.SelectScansDropDown, 'ValueChanged', ...
                        @(source, event) obj.setWSImporterModelTentativeProps('SelectedScansVarIndex', 'AvailableScansVars', event.EventData.NewValue));
            addlistener(obj.WSImporterTab.SelectPosesDropDown, 'ValueChanged', ...
                        @(source, event) obj.setWSImporterModelTentativeProps('SelectedPosesVarIndex', 'AvailablePosesVars', event.EventData.NewValue));
            addlistener(obj.WSImporterTab.PercentageDropDown, 'ValueChanged', ...
                        @(source, event) obj.setWSImporterModelTentativeProps('SelectedPercentageToKeepIndex', 'AvailablePercentagesToKeep', event.EventData.NewValue));
            addlistener(obj.WSImporterTab.RefreshWSButton, 'ButtonPushed', @(source, event) obj.refreshWorkspaceVariables());
                    
            addlistener(obj.WSImporterTab, 'WorkspaceImporterTab_Apply', @(src, evt) obj.applyWSImporterSettingsCallback() );
            addlistener(obj.WSImporterTab, 'WorkspaceImporterTab_Close', @(src, evt) obj.closeWSImporterTabCallback() );
        end
    end


    % listen to Model events
    methods
        function addModelListeners(obj)
        %addModelListeners
            addlistener(obj.ScanFigureSliderModel, 'SliderModel_CurrentValueUpdated', @(source, event) obj.drawCurrentScan(event.Vector));

            addlistener(obj.WSImporterModel, 'Tentative', 'PostSet', @(source, event) obj.toggleBannerVisibility);
            addlistener(obj.WSImporterModel, 'InfoStrings', 'PostSet', @(source, event) obj.refreshInfoFigure(event.AffectedObject.InfoStrings));
            addlistener(obj.WSImporterModel, 'TabRefreshTrigger', 'PostSet', @(source, event) obj.refreshWSImporterTab);
            
            addlistener(obj.WSImporterModel, 'WorkspaceImporterModel_RequestUpdateWSImporterTab', @(source, event) obj.refreshWSImporterTab() );
        end
    end


    methods

        function refreshWSImporterTab(obj)
        %refreshWSImporterTab
            import nav.slamapp.internal.eventdata.*

            dropDownStatusInfo = struct();

            dropDownStatusInfo.AvailablePercentagesToKeep = obj.WSImporterModel.AvailablePercentagesToKeep;
            dropDownStatusInfo.AvailableScansVars = obj.WSImporterModel.AvailableScansVars;
            dropDownStatusInfo.AvailablePosesVars = obj.WSImporterModel.AvailablePosesVars;
            
            dropDownStatusInfo.SelectedScansVarIndex = obj.WSImporterModel.SelectedScansVarIndex;
            dropDownStatusInfo.SelectedPosesVarIndex = obj.WSImporterModel.SelectedPosesVarIndex;
            dropDownStatusInfo.SelectedPercentageToKeepIndex = obj.WSImporterModel.SelectedPercentageToKeepIndex;

            obj.WSImporterTab.updateDropDowns(dropDownStatusInfo);

        end
        
        function refreshWorkspaceVariables(obj)
            %refreshWorkspaceVariables
            
            obj.WSImporterModel.updateAvailableScansAndPosesVariables();
            
            % if the currently selected scans and poses variables are no
            % longer present in workspace, select empty for user
            [isAvailable, loc] = ismember(obj.WSImporterModel.Tentative.SelectedScansVar, obj.WSImporterModel.AvailableScansVars);
            if isAvailable
                obj.WSImporterModel.SelectedScansVarIndex = loc;
                
            else % select empty
                obj.WSImporterModel.SelectedScansVarIndex = 1;
                obj.WSImporterModel.Tentative.SelectedScansVar = obj.WSImporterModel.AvailableScansVars{1};
            end

            [isAvailable, loc] = ismember(obj.WSImporterModel.Tentative.SelectedPosesVar, obj.WSImporterModel.AvailablePosesVars);
            if isAvailable
                obj.WSImporterModel.SelectedPosesVarIndex = loc;
            else % select empty
                obj.WSImporterModel.SelectedPosesVarIndex = 1;
                obj.WSImporterModel.Tentative.SelectedPosesVar = obj.WSImporterModel.AvailablePosesVars{1};
            end            
            refreshWSImporterTab(obj);
        end

        function setWSImporterModelTentativeProps(obj, propName, propAvailableChoices, value)
        %setWSImporterModelTentativeProps
            choices = obj.WSImporterModel.(propAvailableChoices);
            [~, loc] = ismember(value, choices);
            obj.WSImporterModel.(propName) = loc;
            propName2 = strrep(propName, 'Index', '');
            obj.WSImporterModel.Tentative.(propName2) = value;
        end
        
        function toggleBannerVisibility(obj)
        %toggleBannerVisibility
            if ~isequal(obj.WSImporterModel.Tentative, obj.WSImporterModel.Applied)
                obj.AppViewOrganizer.ScanFigDoc.showBanner(true);
                obj.AppViewOrganizer.OdomFigDoc.showBanner(true);
                obj.WSImporterTab.ApplyButton.Enabled = true;
            else
                obj.AppViewOrganizer.ScanFigDoc.showBanner(false);
                obj.AppViewOrganizer.OdomFigDoc.showBanner(false);
                obj.WSImporterTab.ApplyButton.Enabled = false;
            end
        end
        
        function drawCurrentScan(obj, v)
        %drawCurrentScan
            import nav.slamapp.internal.eventdata.*
            import nav.slamapp.internal.*

            if obj.StateMachineModel.State == States.LoadingFromWS
                if ~isempty(obj.WSImporterModel.Scans)
                    scan = obj.WSImporterModel.Scans{v};
                    if ~isempty(scan)
                        obj.AppViewOrganizer.ScanFigDoc.drawCurrentRawScan(scan, obj.WSImporterModel.LidarRange(2));

                        if ~isempty(obj.WSImporterModel.Poses)
                            pose = obj.WSImporterModel.Poses{v};
                            T = robotics.core.internal.SEHelpers.poseToTformSE3( [pose(1), pose(2), 0, eul2quat([pose(3), 0, 0])] );
                            obj.AppViewOrganizer.OdomFigDoc.drawSensor(T);
                        end
                    end
                end
            end

        end

        function applyWSImporterSettingsCallback(obj)
        %applyWSImporterSettingsCallback
            obj.AppViewOrganizer.freeze();
            obj.applyWSImporterSettings();
            obj.AppViewOrganizer.thaw();
        end

        function isSuccess = applyWSImporterSettings(obj)
        %applyWSImporterSettings
            [isSuccess, errTag, errArgs] = obj.WSImporterModel.extractData();
            if ~isSuccess
                dlgs = nav.slamapp.internal.Dialogs(obj.AppViewOrganizer.ToolGroup);
                dlgs.errorMsgDialog(errTag, [], [], errArgs{:});
                return
            end
            obj.ScanFigureSliderModel.resetMinMaxValue(1, numel(obj.WSImporterModel.Scans));
            
            if ~isempty(obj.WSImporterModel.Poses)
                obj.AppViewOrganizer.OdomFigDoc.drawOdom(obj.WSImporterModel.Poses);
            else
                obj.AppViewOrganizer.OdomFigDoc.clearOdom();
            end
            
            toggleBannerVisibility(obj);
        end

        function closeWSImporterTabCallback(obj)
        %closeWSImporterTabCallback
            if ~isequal(obj.WSImporterModel.Tentative, obj.WSImporterModel.Applied)
                % pending toolstrip setting changes
                dlgs = nav.slamapp.internal.Dialogs(obj.AppViewOrganizer.ToolGroup);
                [selection, possibleSelections] = dlgs.questionDialog('ImportIncomplete', [], true);
                switch selection
                  case possibleSelections{1} % apply changes and proceed
                    obj.AppViewOrganizer.freeze();
                    isSuccess = obj.applyWSImporterSettings();
                    if isSuccess
                        obj.MapBuilderModel.loadProcessedData(obj.WSImporterModel.Scans, ...
                                                          obj.WSImporterModel.Poses);
                        obj.StateMachineModel.toSensorDataReady();
                        obj.MapBuilderModel.IsAppDirty = true;
                    end
                    obj.AppViewOrganizer.thaw();
                  
                  case possibleSelections{2} % abort import
                    obj.AppViewOrganizer.freeze();
                    obj.StateMachineModel.backToInit();
                    pause(0.5);
                    obj.AppViewOrganizer.thaw();
    
                end
            elseif isempty(obj.WSImporterModel.Scans)
                % if nothing has been changed in workspace importer tab and
                % the user wants to close it
                obj.AppViewOrganizer.freeze();
                obj.StateMachineModel.backToInit();
                pause(0.5);
                obj.AppViewOrganizer.thaw();
            else
                % all toolstrip settings have been applied and are valid
                obj.StateMachineModel.toSensorDataReady();
                obj.MapBuilderModel.loadProcessedData(obj.WSImporterModel.Scans, ...
                                                      obj.WSImporterModel.Poses);
                obj.MapBuilderModel.IsAppDirty = true;
            end
        end

        function refreshInfoFigure(obj, infoStrings)
        %refreshInfoFigure
            obj.AppViewOrganizer.InfoWSVarsFigDoc.refreshTextFields(infoStrings);
        end
    end
end

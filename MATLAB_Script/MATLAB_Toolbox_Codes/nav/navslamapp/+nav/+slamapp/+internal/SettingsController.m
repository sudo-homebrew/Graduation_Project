classdef SettingsController < handle
%This class is for internal use only. It may be removed in the future.

%SETTINGSCONTROLLER Controller for SLAM settings dialog

% Copyright 2018-2020 The MathWorks, Inc.

    properties
        SettingsDialog
        MapBuilderTab

        SettingsModel
        BagImporterModel
        WSImporterModel
        StateMachineModel
        MapBuilderModel
    end


    methods
        function obj = SettingsController(settingsModel, bagImporterModel, wsImporterModel, stateMachineModel, mapBuilderModel, settingsDialog, mapBuilderTab)
        %SETTINGSCONTROLLER Constructor

            obj.SettingsDialog = settingsDialog;
            obj.MapBuilderTab = mapBuilderTab;
            obj.SettingsModel = settingsModel;
            obj.BagImporterModel = bagImporterModel;
            obj.WSImporterModel = wsImporterModel;
            obj.StateMachineModel = stateMachineModel;
            obj.MapBuilderModel = mapBuilderModel;

            obj.addViewListeners();
            obj.addModelListeners();

        end

    end


    % listen to View events
    methods
        function addViewListeners(obj)
        %addViewListeners

            addlistener(obj.SettingsDialog, 'SettingsDialog_RequestCurrentSettings', @(src, evt) obj.populateSettings());
            addlistener(obj.MapBuilderTab, 'MapBuilderTab_RequestSettingsDialog', @(src, evt) obj.SettingsDialog.prepareDialog());
            addlistener(obj.SettingsDialog, 'SettingsDialog_SendUserUpdate', @(src, evt) obj.processUserUpdate(evt));

        end

        function addModelListeners(obj)
        %addModelListeners
            addlistener(obj.BagImporterModel, 'BagImporterModel_MaxLidarRangeEstimateAvailable', @(src, evt) obj.presetLidarRange(evt.Vector));
            addlistener(obj.WSImporterModel, 'WorkspaceImporterModel_MaxLidarRangeEstimateAvailable', @(src, evt) obj.presetLidarRange(evt.Vector));
        end
    end


    methods
        function populateSettings(obj)
        %populateSettings

            obj.SettingsModel.readFromApplied;
            dataStruct = obj.SettingsModel.extract;
            obj.SettingsDialog.refreshDialog(dataStruct);
            obj.SettingsDialog.show();
        end

        function processUserUpdate(obj, evt)
        %processUserUpdate

            if ~strcmp(evt.Tag, 'LoopClosureAutoRollback')
                if ~isempty(evt.NumericStr)
                    obj.SettingsModel.(evt.Tag) = evt.NumericStr;
                end
            else
                obj.SettingsModel.(evt.Tag) = evt.Value;
            end
            dataStruct = obj.SettingsModel.extract;
            obj.SettingsDialog.refreshDialog(dataStruct);
        end

        function presetLidarRange(obj, val)
        %presetLidarRange
            obj.SettingsModel.LidarRange = val;
            maxR = val(2);
            obj.SettingsModel.MapResolution = 1/( (maxR*2)/((2^5)*10) );
            obj.SettingsModel.writeToApplied;
        end


    end
end

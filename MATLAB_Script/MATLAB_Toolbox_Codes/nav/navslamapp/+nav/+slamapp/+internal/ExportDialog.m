classdef ExportDialog < handle & ...
        robotics.appscore.internal.mixin.MsgCatalogHelper
    %This class is for internal use only. It may be removed in the future.

    %EXPORTDIALOG Dialog for exporting from the app

    % Copyright 2018-2021 The MathWorks, Inc.

    properties
        %DialogSize
        DialogSize

        %Dialog
        Dialog

        %MapVarNameEdit
        MapVarNameEdit

        %AllScansRadioButton
        AllScansRadioButton
        
        %ContainerWindow
        ContainerWindow
    end

    properties (Transient)
        %MapVarName
        MapVarName

        %NeedLatestMap
        NeedLatestMap
    end

    methods
        function obj = ExportDialog(containerWindow)
        %EXPORTDIALOG Constructor

            obj.MsgIDPrefix = 'nav:navslamapp:exportdialog';
            obj.DialogSize = [450 250];
            obj.ContainerWindow = containerWindow;
        end

        function prepareExportMapDialog(obj, currScanId, latestScanId)
        %prepareExportMapDialog
            % export dialog pop-up is a modal uifigure window. Due to issue
            % in modal figure behavior, app needs to be put to inaccessible
            % state when the dialog is open.
            obj.ContainerWindow.Busy = true;
            dlg = robotics.appscore.internal.createModelDialog(obj.retrieveMsg('ExportMapDialogName'));
            dlg.Position(3:4) = obj.DialogSize;

            obj.Dialog = dlg;

            btngroup = uibuttongroup('Parent', dlg, 'BorderType', 'none', 'Position',[20 20 310 250]);

            obj.AllScansRadioButton = uiradiobutton(btngroup, 'Text', obj.retrieveMsg('AllScansOption', 1, latestScanId),...
                                                'Position',[25 140 300 25], 'HandleVisibility','off');

            uiradiobutton(btngroup, 'Text', obj.retrieveMsg('UpToCurrentScanOption', 1, currScanId),...
                      'Position',[25 110 300 25], 'HandleVisibility','off');

            uilabel(dlg,  ...
                      'Text', obj.retrieveMsg('LabelExportMap'), ...
                      'HorizontalAlignment', 'left', ...
                      'Position', [20, 180, 200, 25], 'WordWrap', 'on');

            obj.MapVarNameEdit = uieditfield(dlg, ...
                                           'Value', obj.retrieveMsg('DefaultMapVariableName'), ...
                                           'HorizontalAlignment', 'left', ...
                                           'Position', [250, 185, 150, 25]);

            uibutton(dlg, 'push', ....
                      'Text', obj.retrieveMsg('ExportButtonName'), ...
                      'ButtonPushedFcn', @(src, evt) obj.exportMapButtonCallback(src, evt), ...
                      'Position', [260, 12, 50, 25]);


            uibutton(dlg, 'push', ....
                      'Text', obj.retrieveMsg('CancelButtonName'), ...
                      'ButtonPushedFcn', @(src, evt) obj.cancelButtonCallback(src, evt), ...
                      'Position', [320, 12, 50, 25]);

        end

        function exportMapButtonCallback(obj, src, evt) %#ok<INUSD>
        %exportMapButtonCallback
            mapVarName = obj.MapVarNameEdit.Value;
            % attach the error dialogs to opened export pop-up if any
            dlgs = nav.slamapp.internal.Dialogs(obj.Dialog);
            if ~isvarname(mapVarName)
                errorMsgDialog(dlgs, 'InvalidVariableName');
                return
            end

            if  evalin('base', ['exist(''', mapVarName, ''', ''var'')']) % evaluate in base workspace
                errorMsgDialog(dlgs, 'ExistingVariableName');
                return
            end

            obj.MapVarName = mapVarName;
            obj.NeedLatestMap = logical(obj.AllScansRadioButton.Value);
            % container is made accessible before closing the pop-up
            obj.ContainerWindow.Busy = false;
            close(obj.Dialog);
        end

        function cancelButtonCallback(obj, src, evt) %#ok<INUSD>
        %cancelButtonCallback
            % container is made accessible before closing the pop-up
            obj.ContainerWindow.Busy = false;
            close(obj.Dialog);
        end

    end
end
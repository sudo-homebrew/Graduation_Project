classdef SettingsDialog< handle & ...
        robotics.appscore.internal.mixin.MsgCatalogHelper
    %This class is for internal use only. It may be removed in the future.

    %SETTINGSDIALOG Dialog for setting SLAM problem parameters

    % Copyright 2018-2021 The MathWorks, Inc.

    properties
        %DialogSize Size of the dialog ([wid len] in pixels)
        DialogSize

        %Name Dialog name (title)
        Name

        %BackgroundColor
        BackgroundColor

        %Dialog
        Dialog
        
        %ContainerWindow App Window
        ContainerWindow
    end

    events
    SettingsDialog_RequestCurrentSettings

    SettingsDialog_SendUserUpdate

    SettingsDialog_ApplyCurrentSettings
end

methods
    function obj = SettingsDialog(containerWindow)
    %SETTINGSDIALOG Constructor
        obj.MsgIDPrefix = 'nav:navslamapp:settingsdialog';
        obj.DialogSize = [560 380];
        obj.BackgroundColor = [1 1 1]; %white
        obj.ContainerWindow = containerWindow;

    end

    function prepareDialog(obj)
    %prepare Dynamically create a dialog and hide it for later
        
        % settings dialog pop-up is a modal uifigure window. Due to issue
        % in modal figure behavior, app needs to be put to inaccessible
        % state when the dialog is open.
        obj.ContainerWindow.Busy = true;
        dlg = robotics.appscore.internal.createModelDialog( obj.retrieveMsg('DialogName'));
        dlg.CloseRequestFcn = @obj.closeCallBack;
        dlg.Position(3:4) = obj.DialogSize;

        obj.Dialog = dlg;

        obj.addAcceptAndCancelButtons(obj.Dialog);

        tabGroup = uitabgroup(dlg, 'Units', 'pixels', 'Position',[1 50 obj.DialogSize(1) obj.DialogSize(2) - 50]);
        tabLS = uitab(tabGroup, 'Units', 'pixels', 'Title', obj.retrieveMsg('TabNameLS'));
        tabNLP = uitab(tabGroup, 'Units', 'pixels', 'Title', obj.retrieveMsg('TabNameNLP'));

        obj.addLabel(tabLS, obj.retrieveMsg('LabelMapResolution'), obj.retrieveMsg('LabelMapResolutionTooltip'), 1, 1);
        obj.addLabel(tabLS, obj.retrieveMsg('LabelLidarRange'), obj.retrieveMsg('LabelLidarRangeTooltip'), 2, 1);
        obj.addLabel(tabLS, obj.retrieveMsg('LabelLoopClosureThreshold'),  obj.retrieveMsg('LabelLoopClosureThresholdTooltip'), 3, 1);
        obj.addLabel(tabLS, obj.retrieveMsg('LabelLoopClosureSearchRadius'),  obj.retrieveMsg('LabelLoopClosureSearchRadiusTooltip'), 4, 1);
        obj.addLabel(tabLS, obj.retrieveMsg('LabelLoopClosureMaxAttempts'),  obj.retrieveMsg('LabelLoopClosureMaxAttemptsTooltip'), 5, 1);
        obj.addLabel(tabLS, obj.retrieveMsg('LabelLoopClosureAutoRollback'), obj.retrieveMsg('LabelLoopClosureAutoRollbackTooltip'), 6, 1);
        obj.addLabel(tabLS, obj.retrieveMsg('LabelOptimizationInterval'), obj.retrieveMsg('LabelOptimizationIntervalTooltip'), 7, 1);
        obj.addLabel(tabLS, obj.retrieveMsg('LabelMovementThreshold'), obj.retrieveMsg('LabelMovementThresholdTooltip'), 8, 1);

        obj.addLabel(tabNLP, obj.retrieveMsg('LabelMaxIterations'), obj.retrieveMsg('LabelMaxIterationsTooltip'), 1, 1);
        obj.addLabel(tabNLP, obj.retrieveMsg('LabelMaxTime'), obj.retrieveMsg('LabelMaxTimeTooltip'), 2, 1);
        obj.addLabel(tabNLP, obj.retrieveMsg('LabelGradientTolerance'), obj.retrieveMsg('LabelGradientToleranceTooltip'), 3, 1);
        obj.addLabel(tabNLP, obj.retrieveMsg('LabelFunctionTolerance'), obj.retrieveMsg('LabelFunctionToleranceTooltip'), 4, 1);
        obj.addLabel(tabNLP, obj.retrieveMsg('LabelStepTolerance'), obj.retrieveMsg('LabelStepToleranceTooltip'), 5, 1);
        obj.addLabel(tabNLP, obj.retrieveMsg('LabelFirstNodePose'), obj.retrieveMsg('LabelFirstNodePoseTooltip'), 6, 1);


        obj.addEdit(tabLS, 'MapResolution', 1, 2);
        obj.addEdit(tabLS, 'LidarRange', 2, 2);
        obj.addEdit(tabLS, 'LoopClosureThreshold', 3, 2);
        obj.addEdit(tabLS, 'LoopClosureSearchRadius', 4, 2);
        obj.addEdit(tabLS, 'LoopClosureMaxAttempts', 5, 2);
        obj.addEditToggleButton(tabLS, 'LoopClosureAutoRollback', 6, 2);
        obj.addEdit(tabLS, 'OptimizationInterval', 7, 2);
        obj.addEdit(tabLS, 'MovementThreshold', 8, 2);

        obj.addEdit(tabNLP, 'MaxIterations', 1, 2);
        obj.addEdit(tabNLP, 'MaxTime', 2, 2);
        obj.addEdit(tabNLP, 'GradientTolerance', 3, 2);
        obj.addEdit(tabNLP, 'FunctionTolerance', 4, 2);
        obj.addEdit(tabNLP, 'StepTolerance', 5, 2);
        obj.addEdit(tabNLP, 'FirstNodePose', 6, 2);

        obj.notify('SettingsDialog_RequestCurrentSettings');

    end

    function show(obj)
    %show
        if ~isempty(obj.Dialog) && isvalid(obj.Dialog)
            obj.Dialog.Visible = 'on';
        end
    end

    function addAcceptAndCancelButtons(obj, parent)
    %addAcceptAndCancelButtons
        uibutton(parent, 'push', ....
                'Text', obj.retrieveMsg('AcceptButtonName'), ...
                'ButtonPushedFcn', @(src, evt) obj.acceptButtonCallback(src, evt), ...
                'Position', [350, 12, 50, 25]);

        uibutton(parent, 'push', ....
            'Text', obj.retrieveMsg('CancelButtonName'), ...
            'ButtonPushedFcn', @(src, evt) obj.cancelButtonCallback(src, evt), ...
            'Position', [410, 12, 50, 25]);
    end

    function label = addLabel(obj, parent, str, tooltipStr, row, col)
    %addLabel
        label = uilabel(parent, 'Text', str,...
                'FontSize', 9, 'HorizontalAlignment', 'left',...
                'WordWrap', 'on', 'Tooltip',tooltipStr);
        label.Tooltip =  tooltipStr;
        placeWidgetInGrid(obj, parent, label, row, col);
    end

    function edit = addEdit(obj, parent, tag, row, col)
    %addEdit
        edit = uieditfield(parent, ...
                'Tag', tag, ...
                'FontSize', 9, 'ValueChangedFcn', @(src, evt) obj.userInputCallback(src.Tag, src.Value, 0) );
        placeWidgetInGrid(obj, parent, edit, row, col);
    end
    
    function edit = addEditToggleButton(obj, parent, tag, row, col)
            %addEdit
            edit = uibutton(parent, ...
                'Tag', tag, ...
                'FontSize', 9, 'ButtonPushedFcn', @(src, evt) obj.userInputCallback(src.Tag, src.Text, 0) );
            placeWidgetInGrid(obj, parent, edit, row, col);

        end

    function userInputCallback(obj, sourceTag, str, val)
    %userInputCallback

        numericStr = str2num(str); %#ok<ST2NM>

        if ~isempty(numericStr) && ~all(isfinite(numericStr))
            numericStr = [];
        end
        
        if strcmp(sourceTag,'LoopClosureAutoRollback')
            val = strcmp(str,'off');
        end

        import nav.slamapp.internal.eventdata.SettingsUpdateEventData
        obj.notify('SettingsDialog_SendUserUpdate', SettingsUpdateEventData(sourceTag, numericStr, val));

    end

    function acceptButtonCallback(obj, src, evt) %#ok<INUSD>
    %acceptButtonCallback
        obj.notify('SettingsDialog_ApplyCurrentSettings');
        close(obj.Dialog);
    end

    function cancelButtonCallback(obj, src, evt) %#ok<INUSD>
    %cancelButtonCallback
        close(obj.Dialog);
    end
    
    function closeCallBack(obj, src, evt) %#ok<INUSD>
    %closeCallBack
        obj.ContainerWindow.Busy = false;
        delete(obj.Dialog);
    end

    function refreshDialog(obj, dataStruct)
    %refreshDialog
        fieldNames = fieldnames(dataStruct);

        if ~isempty(obj.Dialog) && isvalid(obj.Dialog)
            for i = 1:length(fieldNames)
                fn = fieldNames{i};
                widget = findobj(obj.Dialog, 'Tag', fn);
                packet = dataStruct.(fn);
                if isa(widget,'matlab.ui.control.Button')
                    if packet{2}
                        widget.Text = 'on';
                    else
                        widget.Text = 'off';
                    end
                else
                    widget.Value = packet{1};
                end
            end
        end

    end

end


methods (Access = protected)
    function placeWidgetInGrid(~, parent, widget, rowNum, colNum)
    %placeWidgetInGrid
        xSpace = 30;
        ySpace = 10;

        if isa(widget, 'matlab.ui.control.TextArea')
            yOffset = 80;
            dw = 0;
        else
            yOffset = 75;
            dw = 80;
        end
        xOffset = 20;

        width = 250;
        height = 25;

        pos = zeros(1,4);
        pos(1) = xOffset + (colNum - 1)*(width + xSpace);

        pos(2) = parent.Position(4) - yOffset - (rowNum - 1)*(height + ySpace);
        pos(3) = width - dw;
        pos(4) = height;

        widget.Position = pos;
    end

end
end

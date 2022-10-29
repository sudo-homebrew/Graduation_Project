classdef Dialogs < handle & ...
        robotics.appscore.internal.mixin.MsgCatalogHelper
    %This class is for internal use only. It may be removed in the future.

    %DIALOGS Dialogs used in SLAM app

    % Copyright 2018-2021 The MathWorks, Inc.

    properties
        %ContainerWindow Parent app or figure window for in window 
        %error/ warning/ confirm dialogs
        ContainerWindow
    end
    
    methods

        function obj = Dialogs(containerWindow)
            %Dialogs base class to create in window error, warning and
            %   question dialogs. In window dialogs will always appead at
            %   the center of it's parent container window
            obj.MsgIDPrefix = 'nav:navslamapp:dialogs';
            obj.ContainerWindow = containerWindow;
        end

        function [selection, possibleSelections] = questionDialog(obj, tag, titleHint, hasThirdChoice)
        %questionDialog Create a question dialog
        %   titleHint shows up in the dialog title (after the App name)
        %   to give user some context information
            pf = obj.MsgIDPrefix;
            question = strcat(pf, ':', tag, 'ConfirmationQuestion');
            title = strcat(pf, ':', tag, 'DialogTitle');
            yes = strcat(pf, ':', tag, 'Yes');
            no = strcat(pf, ':', tag, 'No');
            hint = '';

            possibleSelections = {obj.getMsg(yes), obj.getMsg(no)};
            if nargin > 2
                hint = titleHint;
            end
            if nargin>3 && hasThirdChoice
                cancel = strcat(pf, ':', tag, 'Cancel');
                possibleSelections = [possibleSelections, obj.getMsg(cancel)];
            end
            
            selection = obj.openPopup(@questionDialog);
            
            function selection = questionDialog()
                % uiconfirm utility is used to create a question dialog pop-up
                % asking user to choose weather the action user is about to
                % perform is ok, not ok or cancel. uiconfirm cannot exist
                % without a parent container window.
                if isempty(hint)
                    selection = uiconfirm(obj.ContainerWindow,obj.getMsg(question), ...
                        obj.getMsg(title), ...
                        'Options', possibleSelections, ...
                        'DefaultOption', possibleSelections{end});  % last one is default selection
                else
                    selection = uiconfirm(obj.ContainerWindow,obj.getMsg(question), ...
                        obj.getMsg(title,hint), ...
                        'Options', possibleSelections, ...
                        'DefaultOption', possibleSelections{end});  % last one is default selection
                end
            end
        end

        function errorMsgDialog(obj, tag, errID, errMsg, varargin)
        %errorMsgDialog
            pf = obj.MsgIDPrefix;
            title = obj.getMsg(strcat(pf, ':', tag, 'ErrorDialogTitle'));
            if nargin > 3 && ~isempty(errMsg)
                body = errMsg;
            elseif nargin > 2 && ~isempty(errID)
                body = obj.getMsg(errID);
            else
                body = obj.getMsg(strcat(pf, ':', tag, 'ErrorDialogBody'), varargin{:});
            end
            
            errorPopupFcn = @()uiconfirm(obj.ContainerWindow, body, title,'Icon','error','Options',{'OK'});
            obj.openPopup(errorPopupFcn);
        end

        function warningMsgDialog(obj, tag, warningID)
        %warningMsgDialog
            pf = obj.MsgIDPrefix;
            title = obj.getMsg(strcat(pf, ':', tag, 'WarningDialogTitle'));
            if nargin > 2 && ~isempty(warningID)
                body = obj.getMsg(warningID);
            else
                body = obj.getMsg(strcat(pf, ':', tag, 'WarningDialogBody'));
            end
            
            warningPopupFcn = @()uiconfirm(obj.ContainerWindow, body, title,'Icon','warning','Options',{'OK'});
            obj.openPopup(warningPopupFcn);
        end
        
        function selection = openPopup(obj, popupFcn)
            %openPopup open uiconfirm dialog running popupFcn function
            %   handle and makes sure pop-up is accessible. Instead of
            %   calling pop-up function directly user needs to call this
            %   method to ensure that the generated pop-up is always
            %   accessible. This method is now called by errorMsgDialog and
            %   warningMsgDialog.
            
            if isa(obj.ContainerWindow,'matlab.ui.container.internal.AppContainer')
                % The uiconfirm dilogs are always attached to a parent
                % figure or app container. If it's attached to app window
                % then we need to make sure that it's in accessible state
                % before we open a dialog.
                appWindowState = obj.ContainerWindow.Busy;
                obj.ContainerWindow.Busy = false;
            end
            
            % run the blocking dialog creation function
            selection = popupFcn();
            
            if isa(obj.ContainerWindow,'matlab.ui.container.internal.AppContainer')
                % put the parent container if it is app window to same
                % state as it was before the pop-up after user selection
                obj.ContainerWindow.Busy = appWindowState;
            end
        end
    end


end

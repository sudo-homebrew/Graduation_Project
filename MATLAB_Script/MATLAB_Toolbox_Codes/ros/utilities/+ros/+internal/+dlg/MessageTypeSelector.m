classdef (Abstract) MessageTypeSelector < handle
%This class is for internal use only. It may be removed in the future.

%  MessageTypeSelector opens a DDG dialog that lets the user select from a
%  list of ROS message types (shared between ROS 1 and ROS 2). Once the
%  user accepts the changes (or cancels the dialog), a callback is invoked
%  with the closure action and selected message type.
%
%  Sample use:
%   selector = ros.slros.internal.dlg.MessageTypeSelector;
%   % The first argument is the message type to select by default
%   selector.openDialog('std_msgs/int32', @(isAccepted,msgType) disp(msgType));

%   Copyright 2014-2019 The MathWorks, Inc.

    properties(SetAccess=protected)
        Title = message('ros:slros:msgselector:DialogTitle').getString;
        ROSMessage = ''
        MessageList = {}
        CloseFcnHandle = function_handle.empty
    end


    methods (Access=protected, Abstract)
        % Sub-classes must implement following methods
        setMessageList(obj);
    end
    
    methods
        function obj = MessageTypeSelector()
        end

        function set.MessageList(obj, val)
            validateattributes(val, {'cell'},{'nonempty'},'setMessageList','MessageList',2);
            obj.MessageList = val;
        end


        function dlg = openDialog(obj, initialMsgSelection, closeFcnHandle)
        % closeFcnHandle: handle to function that takes two arguments
        %   closeFcn(isAcceptedSelection, rosTopic)
        %      isAcceptedSelection: true of user clicked on 'ok', false
        %        if user clicked on 'cancel' or closed window
        %      rosTopic: last selected ROS topic (string)

            assert(ischar(initialMsgSelection) || isempty(initialMsgSelection));
            validateattributes(closeFcnHandle, {'function_handle'}, {'scalar'});
            obj.CloseFcnHandle = closeFcnHandle;
            setMessageList(obj);
            dlg = DAStudio.Dialog(obj);
            if isempty(initialMsgSelection)
                return;
            end

            % Find initial message selection, if any
            index = find(strcmpi(initialMsgSelection, obj.MessageList));
            if ~isempty(index)
                dlg.setWidgetValue('rosmsglist', index-1); % zero-based
                obj.ROSMessage = obj.MessageList{index};
            else
                warning(message('ros:slros:msgselector:MessageTypeNotFound', ...
                                initialMsgSelection));
            end
        end
    end


    methods(Hidden)
        % Called when user selects an item from the list
        function dlgCallback(obj, dlg, tag, value) %#ok<INUSL>
            obj.ROSMessage = obj.MessageList{value+1}; % value is zero-based
            dlg.refresh;
        end


        function dlgClose(obj, closeaction)
        % closeaction is 'ok' if user clicked OK
        %                'cancel' if user clicked cancel or closed window
            if ~isempty(obj.CloseFcnHandle)
                isAcceptedSelection = strcmpi(closeaction, 'ok');
                try
                    feval(obj.CloseFcnHandle, isAcceptedSelection, obj.ROSMessage);
                catch
                    % Absorb all errors. If they are propagated back to
                    % DDG, this causes MATLAB to crash, (Can't convert to
                    % warnings are not as they are not displayed either).
                end
            end
        end


        function dlgstruct = getDialogSchema(obj)
            msglist.Name    = '';
            msglist.Type    = 'listbox';
            msglist.Entries = obj.MessageList;
            msglist.Tag     = 'rosmsglist';
            msglist.MultiSelect = false;
            msglist.ObjectMethod = 'dlgCallback'; % call method on UDD source object
            msglist.MethodArgs = {'%dialog', '%tag', '%value'}; % object handle is implicit first arg
            msglist.ArgDataTypes = {'handle', 'string', 'mxArray'}; % 'handle' is type of %dialog
            msglist.Value = 0;
            msglist.NameLocation = 2; % top left

            % Main dialog
            dlgstruct.DialogTitle = obj.Title;
            dlgstruct.HelpMethod = 'ros.slros.internal.helpview';
            dlgstruct.HelpArgs =  {'rosMsgSelectDlg'};  % doc topic id
            dlgstruct.CloseMethod = 'dlgClose';
            dlgstruct.CloseMethodArgs = {'%closeaction'};
            dlgstruct.CloseMethodArgsDT = {'string'};

            % Make this dialog modal wrt to other DDG dialogs
            % (doesn't block MATLAB command line)
            dlgstruct.Sticky = true;

            % Buttons to show on dialog (these are options to pass to DDG,
            % not the final strings, so there is no need to use message
            % catalog)
            dlgstruct.StandaloneButtonSet =  ...
                {'Ok', 'Cancel', 'Help'}; % also available: 'Revert', 'Apply'

            dlgstruct.Items = {msglist};
        end
    end
end

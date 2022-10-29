classdef (Abstract) TopicSelector < handle
%This class is for internal use only. It may be removed in the future.

%  TopicSelector (abstract) opens a DDG dialog that lets the user select
%  from a list of available ROS topics (shared between ROS 1 and ROS 2).
%  Once the user accepts the changes (or cancels the dialog), a callback is
%  invoked with the closure action and selected topic type.


%   Copyright 2014-2019 The MathWorks, Inc.

    properties(SetAccess=protected)
        ROSTopic = ''
        ROSMessageType = ''
        TopicList = {}      % list of available topics
        TopicMsgTypes = {}  % message type corresponding to each topic
        CloseFcnHandle = function_handle.empty        
    end

    methods (Access=protected, Abstract)
        % Sub-classes must implement following methods
        setTopicListAndTypes(obj);
    end
    
    methods
        function dlg = openDialog(obj, closeFcnHandle)
        % closeFcnHandle: handle to function that takes two arguments
        %   closeFcn(isAcceptedSelection, rosTopic, rosMsgType)
        %      isAcceptedSelection: true of user clicked on 'ok', false
        %        if user clicked on 'cancel' or closed window
        %      rosTopic: last selected ROS topic (string)
        %      rosMsgType: message type of last selected ROS topic (string)

            validateattributes(closeFcnHandle, {'function_handle'}, {'scalar'});
            obj.CloseFcnHandle = closeFcnHandle;
            setTopicListAndTypes(obj);
            dlg = DAStudio.Dialog(obj);
            dlg.setWidgetValue('rostopiclist', 0); % select first item in the list
            obj.ROSTopic = obj.TopicList{1};
            obj.ROSMessageType = obj.TopicMsgTypes{1};
        end
    end


    methods(Hidden)
        % Called when user selects an item from the list
        function dlgCallback(obj, dlg, tag, value) %#ok<INUSL>
            obj.ROSTopic = obj.TopicList{value+1}; % value is zero-based
            obj.ROSMessageType = obj.TopicMsgTypes{value+1};
            dlg.refresh;
        end

        function dlgClose(obj, closeaction)
        % closeaction is 'ok' if user clicked OK
        %                'cancel' if user clicked cancel or closed window
            if ~isempty(obj.CloseFcnHandle)
                isAcceptedSelection = strcmpi(closeaction, 'ok');
                try
                    feval(obj.CloseFcnHandle, isAcceptedSelection, obj.ROSTopic, obj.ROSMessageType);
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
            msglist.Entries = obj.TopicList;
            msglist.Tag     = 'rostopiclist';
            msglist.MultiSelect = false;
            msglist.ObjectMethod = 'dlgCallback'; % call method on UDD source object
            msglist.MethodArgs = {'%dialog', '%tag', '%value'}; % object handle is implicit first arg
            msglist.ArgDataTypes = {'handle', 'string', 'mxArray'}; % 'handle' is type of %dialog
            msglist.Value = 0;
            msglist.NameLocation = 2; % top left

            % Main dialog
            dlgstruct.DialogTitle = message('ros:slros:topicselector:DialogTitle').getString;
            dlgstruct.HelpMethod = 'ros.slros.internal.helpview';
            dlgstruct.HelpArgs =  {'rosTopicSelectDlg'};  % doc topic id
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

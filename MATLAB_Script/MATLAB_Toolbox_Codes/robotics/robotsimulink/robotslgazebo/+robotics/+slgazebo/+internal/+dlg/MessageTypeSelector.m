classdef MessageTypeSelector < handle
%This class is for internal use only. It may be removed in the future.

%  MessageTypeSelector opens a DDG dialog that lets the user select
%  from a list of Gazebo message types. Once the user accepts the changes
%  (or cancels the dialog), a callback is invoked with the closure
%  action and selected message type.
%
%  Sample use:
%   selector = robotics.slgazebo.internal.dlg.MessageTypeSelector;
%   % The first argument is the message type to select by default
%   selector.openDialog('std_msgs/int32', @(isAccepted,msgType) disp(msgType));

%   Copyright 2019-2021 The MathWorks, Inc.

    properties(SetAccess=private)
        GazeboMessage = ''
        MessageGetter
        MessageList = {}
        CloseFcnHandle = function_handle.empty
        MessageGetterType = ''
        DefaultCustomMessage = 'gazebo_msgs/TestPose';
    end


    methods
        function obj = MessageTypeSelector(messgeGetterType,messgeGetter)
            obj.MessageGetterType = messgeGetterType;
            if nargin == 2
                obj.MessageGetter = messgeGetter;
            else
                if(strcmp(messgeGetterType,'builtin'))
                    % get builtin message list
                    obj.MessageGetter = robotics.slgazebo.internal.util.GetMessageList;
                else
                    % get custom message list
                    obj.MessageGetter = robotics.slgazebo.internal.util.GetCustomMessageList;
                end
            end
        end

        function dlg = openDialog(obj, initialMsgSelection, closeFcnHandle)
        % closeFcnHandle: handle to function that takes two arguments
        %   closeFcn(isAcceptedSelection, gazeboTopic)
        %      isAcceptedSelection: true of user clicked on 'ok', false
        %        if user clicked on 'cancel' or closed window
        %      gazeboTopic: last selected gazebo topic (string)

        %get the message list.
            obj.MessageList = obj.MessageGetter.MsgList;

            assert(ischar(initialMsgSelection) || isempty(initialMsgSelection));
            validateattributes(closeFcnHandle, {'function_handle'}, {'scalar'});
            obj.CloseFcnHandle = closeFcnHandle;

            assert(numel(obj.MessageList) > 0, 'Expected non-empty message list');

            dlg = DAStudio.Dialog(obj);
            if isempty(initialMsgSelection)
                return;
            end

            % Find initial message selection, if any
            index = find(strcmpi(initialMsgSelection, obj.MessageList));
            if ~isempty(index)
                dlg.setWidgetValue('gazebomsglist', index-1); % zero-based
                obj.GazeboMessage = obj.MessageList{index};
            else
                if(contains(obj.MessageGetterType,'custom'))
                    if(strcmp(initialMsgSelection,obj.DefaultCustomMessage))
                        % Not exposing default message
                        % The message list do not contain default message
                        % So, need hard coding
                        % !!!!! TO DO : Remove this once Gazebo msg are shipped
                        dlg.setWidgetValue('gazebomsglist', 0);
                        obj.GazeboMessage = obj.MessageList{1};
                    end
                else
                    warning(message('robotics:robotslgazebo:messagetypeselector:MessageTypeNotFound', ...
                                    initialMsgSelection));
                end
            end
        end
    end


    methods(Hidden)
        % Called when user selects an item from the list
        function dlgCallback(obj, dlg, tag, value) %#ok<INUSL>
            obj.GazeboMessage = obj.MessageList{value+1}; % value is zero-based
            dlg.refresh;
        end


        function dlgClose(obj, closeaction)
        % closeaction is 'ok' if user clicked OK
        %                'cancel' if user clicked cancel or closed window
            if ~isempty(obj.CloseFcnHandle)
                isAcceptedSelection = strcmpi(closeaction, 'ok');
                try
                    feval(obj.CloseFcnHandle, isAcceptedSelection, obj.GazeboMessage);
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
            msglist.Tag     = 'gazebomsglist';
            msglist.MultiSelect = false;
            msglist.ObjectMethod = 'dlgCallback'; % call method on UDD source object
            msglist.MethodArgs = {'%dialog', '%tag', '%value'}; % object handle is implicit first arg
            msglist.ArgDataTypes = {'handle', 'string', 'mxArray'}; % 'handle' is type of %dialog
            msglist.Value = 0;
            msglist.NameLocation = 2; % top left

            % Main dialog
            dlgstruct.DialogTitle = message('robotics:robotslgazebo:messagetypeselector:DialogTitle').getString;
            dlgstruct.HelpMethod = 'helpview';

            % adds 'HelpArgs' based on block type. 'builtin' is for read
            % block while 'customSubscribe' and 'customPublish' is for
            % subscribe and publish block. This is needed for 'help-button'
            % in subroutine like 'TopicSelect' and 'MessageTypeSelect'
            switch obj.MessageGetterType
              case 'builtin'
                dlgstruct.HelpArgs =  {fullfile(docroot, 'robotics', 'helptargets.map'), 'rstGazeboRead'};
              case 'customSubscribe'
                dlgstruct.HelpArgs =  {fullfile(docroot, 'robotics', 'helptargets.map'), 'rstGazeboSubscribe'};
              case 'customPublish'
                dlgstruct.HelpArgs =  {fullfile(docroot, 'robotics', 'helptargets.map'), 'rstGazeboPublish'};
              case 'applyCommand'
                dlgstruct.HelpArgs =  {fullfile(docroot, 'robotics', 'helptargets.map'), 'rstGazeboApplyCommand'};
              case 'blankMessage'
                dlgstruct.HelpArgs =  {fullfile(docroot, 'robotics', 'helptargets.map'), 'rstGazeboBlankMessage'};
            end

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

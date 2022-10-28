classdef LogfileTopicSelector < handle
    %This class is for internal use only. It may be removed in the future.
    %
    %  Sample use:
    %   selector = ros.slros.internal.dlg.LogfileTopicSelector(bagSelection,topic);
    %   selector.openDialog(@(isAccepted,topicName) disp(topicName))

    %   Copyright 2017-2021 The MathWorks, Inc.

    properties (SetAccess = protected)
        %DataObject Object containing or allowing for access to all
        %   logfile data
        DataObject = ros.BagSelection.empty;

        %TopicIdx Index indicating which topic in table is selected
        %   Zero-based for DDG dialog tables
        TopicIdx double = 0

        %TopicList Contents to be displayed in the list of topics
        TopicList = cell2table(cell(0,12), ...
            'VariableNames',{'Topic', 'Offset', 'Duration', 'NumMessages', ...
            'DetailedOffset','DetailedDuration','StartTime', 'EndTime', 'MessageType','Index','MinimumGap', 'MedianGap'});

        %DetailsTxt Contents to be displayed for the details of a selected
        %   topic
        DetailsTxt = ''

        %CloseFcnHandle Handle to function to be called when the dialog is
        %   closed
        CloseFcnHandle = function_handle.empty
    end

    methods

        %% Constructor and dialog setup
        function obj = LogfileTopicSelector(dataobj, topic)
            %LogfileTopicSelector Construct a dialog for selecting a logfile topic
            %   DATAOBJ - object containing or allowing access to logfile data
            %   TOPIC - character vector, currently selected topic, if any

            % Save data for use
            obj.DataObject = dataobj;

            % Determine currently-selected topic
            if nargin > 1 && ~isempty(topic) && ...
                    ~isempty(obj.DataObject.AvailableTopics.Row)
                idx = find(strcmp(topic,obj.DataObject.AvailableTopics.Row), 1);
                if ~isempty(idx)
                    obj.TopicIdx = idx-1;
                end
            end
        end

        function dlg = openDialog(obj, closeFcnHandle)
            %openDialog Create, lay out, and make visible the dialog window
            %   closeFcnHandle - Handle to function to be executed when the
            %   dialog is closed. This function must take three arguments.
            %   For example, a valid function definition may look like:
            %       closeFcn(isAcceptedSelection, topic, msgType)
            %           isAcceptedSelection - True of user clicked on 'ok',
            %           false if user clicked on 'cancel' or closed window
            %           topic - Last selected topic (character vector)
            %           msgType - Message type of last selected topic (character vector)

            % Handle input
            validateattributes(closeFcnHandle, {'function_handle'}, {'scalar'});
            obj.CloseFcnHandle = closeFcnHandle;

            % Prepare data to display in widgets
            updateTopicList(obj);
            if size(obj.DataObject.AvailableTopics, 1) ~= 0
                updateDetailsTxt(obj, obj.DataObject.AvailableTopics.Row{obj.TopicIdx+1});
            end

            % Open dialog
            dlg = DAStudio.Dialog(obj);
        end
    end

    methods(Hidden)

        %% Control callbacks
        function listClickCallback(obj, dlg, row)
            %listClickCallback Called when the user newly selects a topic
            obj.TopicIdx = row; % value is zero-based
            updateDetailsTxt(obj,obj.DataObject.AvailableTopics.Row{obj.TopicIdx+1});
            dlg.refresh;
        end

        function [shouldClose, errorMsg] = dlgApplyClose(obj)
            %dlgApplyClose Callback for when the 'OK' button is pressed, but before the dialog closes

            if ~isempty(obj.CloseFcnHandle)
                try
                    shouldClose = true;
                    errorMsg = '';
                    if size(obj.DataObject.AvailableTopics, 1) == 0
                        return;
                    end
                    topic = obj.DataObject.AvailableTopics.Row{obj.TopicIdx+1};

                    feval(obj.CloseFcnHandle, topic);
                catch ME
                    % No errors should occur, but need this to avoid
                    % crashing Simulink by passing on uncaught exceptions
                    shouldClose = false;
                    errorMsg = ME.message;
                end
            end
        end

        %% Refresh data for individual controls
        function updateTopicList(obj)
            %updateTopicList Determine the values for the list to topics to select from

            % No data in logfile
            if isempty(obj.DataObject)
                obj.TopicList = cell2table(cell(0,12), ...
                    'VariableNames',{'Topic', 'Offset', 'Duration', 'NumMessages', ...
                    'DetailedOffset','DetailedDuration','StartTime', 'EndTime', 'MessageType','Index','MinimumGap', 'MedianGap'});
                return
            end

            % Preallocate
            nTopics = size(obj.DataObject.AvailableTopics, 1);
            isRos2Bag = isa(obj.DataObject,'ros2bag');
            defaultStr = getString(message('ros:slros:readlog:NotApplicableShorthand'));

            % Calculate all the fields that are required to be displayed at once for each topic and
            % store them in Obj.TopicList
            for k = 1:nTopics
                topic = char(obj.DataObject.AvailableTopics.Row{k});

                % Extract all the rows from obj.DataObject.MessageList belonging to a particular topic
                topicIndices = obj.DataObject.MessageList.Topic == topic;
                topicTable = obj.DataObject.MessageList(topicIndices,:);

                % Add msgType
                msgType = char(obj.DataObject.AvailableTopics.MessageType(k));

                % Verify if the topicTable is already sorted by Time
                % If not sort it by Time column.
                % obj.DataObject.MessageList is already sorted by Time column. Just incase
                % if its not sorted , it will be sorted by below code.
                if ~issortedrows(topicTable,'Time')
                    topicTable = sortrows(topicTable,{'Time'});
                end
                sizeOfTopicTable = size(topicTable,1);
                startTime = '0';
                endTime = '0';
                msgIndex = 0;
                offset = '0';
                duration = '0';
                detailedOffset = '0';
                detailedDuration = '0';
                numMessages = '0';
                minGapTxt = defaultStr;     % Default values if messages all have same timestamps
                medGapTxt = defaultStr;


                if sizeOfTopicTable > 0
                    startTime = topicTable{1,1};
                    endTime = topicTable{sizeOfTopicTable,1};
                    offset = num2str(startTime-obj.DataObject.StartTime, '%.2f');
                    duration = num2str(endTime-startTime, '%.2f');

                    % This detailedOffset and detailedDuration are displayed in topic details pane
                    % when the user selects a particular topic
                    detailedOffset = num2str(startTime-obj.DataObject.StartTime, '%0.9f');
                    detailedDuration = num2str(endTime-startTime,'%0.9f');
                    numMessages = num2str(obj.DataObject.AvailableTopics.NumMessages(k),'%d');
                    endTime = num2str(endTime,'%0.9f');
                    startTime = num2str(startTime,'%0.9f');

                    % message index is used to read a sample message.
                    msgIndex = find(topicIndices,1);

                    % Add minGap and median
                    minGap = ros.slros.internal.block.ReadDataBlockMask.getMinNonzeroMsgGapMessageList(topicTable,isRos2Bag);

                    if ~isempty(minGap)
                        minGapTxt = num2str(minGap, '%.4f');
                        medGapTxt = minGapTxt;  % Default value if many messages with no time gap
                        medGap = double(median(diff(topicTable.Time)));
                        if medGap ~= 0
                            medGapTxt = num2str(medGap, '%.4f');
                        end
                    end
                end

                topicRow = {topic,offset,duration,numMessages,detailedOffset,detailedDuration,startTime,endTime,msgType,msgIndex,minGapTxt,medGapTxt};
                T = cell2table(topicRow, ...
                    'VariableNames',{'Topic', 'Offset', 'Duration', 'NumMessages', ...
                    'DetailedOffset','DetailedDuration','StartTime', 'EndTime', 'MessageType','Index','MinimumGap', 'MedianGap'});
                obj.TopicList = [obj.TopicList;T];
            end
        end

        function updateDetailsTxt(obj, topic)
            %updateDetailsTxt Determine the value for the text to go in the topic details pane

            % Prepare data for examination
            topicIdx = find(strcmp(obj.TopicList{:,1}, topic), 1);
            topicRow = obj.TopicList(topicIdx,:);

            % Create empty cell array
            sampleMsg={};
            if topicRow.Index > 0
                if isa(obj.DataObject,'ros2bag')
                    sampleMsg = readMessages(obj.DataObject, topicRow.Index);
                else
                    sampleMsg = readMessages(obj.DataObject, topicRow.Index, 'DataFormat', 'struct');
                end
            end

            lineFormattedStr = '<b>%s</b> %s<br />';
            obj.DetailsTxt = '';
            defaultStr = getString(message('ros:slros:readlog:NotApplicableShorthand'));

            % Message type
            msgType = char(topicRow.MessageType);
            obj.DetailsTxt = [obj.DetailsTxt ...
                sprintf(lineFormattedStr, ...
                getString(message('ros:slros:readlog:MessageTypePrompt')), ...
                msgType)];

            % Number of messages
            obj.DetailsTxt = [obj.DetailsTxt ...
                sprintf(lineFormattedStr, ...
                getString(message('ros:slros:readlog:NumMessagesPrompt')), ...
                string(topicRow.NumMessages))];

            % Start time, end time, start offset, duration
            obj.DetailsTxt = [obj.DetailsTxt ...
                sprintf(lineFormattedStr, ...
                getString(message('ros:slros:readlog:StartTimePrompt')), ...
                string(topicRow.StartTime))];
            obj.DetailsTxt = [obj.DetailsTxt ...
                sprintf(lineFormattedStr, ...
                getString(message('ros:slros:readlog:EndTimePrompt')), ...
                string(topicRow.EndTime))];
            obj.DetailsTxt = [obj.DetailsTxt ...
                sprintf(lineFormattedStr, ...
                getString(message('ros:slros:readlog:StartOffsetPrompt')), ...
                string(topicRow.DetailedOffset))];
            obj.DetailsTxt = [obj.DetailsTxt ...
                sprintf(lineFormattedStr, ...
                getString(message('ros:slros:readlog:DurationPrompt')), ...
                string(topicRow.DetailedDuration))];

            % Minimum and median gaps between messages
            obj.DetailsTxt = [obj.DetailsTxt ...
                sprintf(lineFormattedStr, ...
                getString(message('ros:slros:readlog:MinMsgGapPrompt')), ...
                string(topicRow.MinimumGap))];
            obj.DetailsTxt = [obj.DetailsTxt ...
                sprintf(lineFormattedStr, ...
                getString(message('ros:slros:readlog:MedianMsgGapPrompt')), ...
                string(topicRow.MedianGap))];

            % Source frame
            frameTxt = defaultStr;      % Default value if no frame listed
            if ~isempty(sampleMsg) && isfield(sampleMsg{1}, 'Header') && ...
                    isfield(sampleMsg{1}.Header, 'FrameId') && ...
                    ischar(sampleMsg{1}.Header.FrameId) && ...
                    ~isempty(sampleMsg{1}.Header.FrameId)
                frameTxt = sampleMsg{1}.Header.FrameId;
            end
            obj.DetailsTxt = [obj.DetailsTxt ...
                sprintf(lineFormattedStr, ...
                getString(message('ros:slros:readlog:SourceFramePrompt')), ...
                frameTxt)];
        end

        %% Dialog layout
        function dlgstruct = getDialogSchema(obj)
            %getDialogSchema Determine the dialog window layout

            msglist.Type = 'table';
            msglist.Grid = false;
            msglist.Tag = 'msglist';
            msglist.ColHeader = ...
                {getString(message('ros:slros:readlog:SourceTitle')), ...
                getString(message('ros:slros:readlog:StartOffsetTitle')), ...
                getString(message('ros:slros:readlog:DurationTitle')), ...
                getString(message('ros:slros:readlog:NumMessagesTitle'))};
            msglist.HeaderVisibility = [0 1];   % View column headers, not row numbers
            msglist.Data = table2cell(obj.TopicList(:,(1:4)));
            msglist.Size = size(msglist.Data);
            msglist.ColumnCharacterWidth = [16 10 10 10];
            msglist.ColumnStretchable = [0 1 1 1];
            msglist.SelectedRow = obj.TopicIdx;
            msglist.CurrentItemChangedCallback = ...
                @(hDlg, row, ~) listClickCallback(obj, hDlg, row);

            paramPrompt.Type = 'text';
            paramPrompt.Name = ...
                getString(message('ros:slros:readlog:DetailsTitle'));

            paramList.Name = 'Parameters for source';
            paramList.NameLocation = 2; % top left
            paramList.FontFamily = 'Consolas';
            paramList.Type = 'textbrowser';
            paramList.Text = obj.DetailsTxt;
            paramList.MinimumSize = [400 100];

            groupBox.Type = 'group';
            groupBox.Items = {msglist,paramPrompt,paramList};
            groupBox.RowSpan = [1 1];
            groupBox.ColSpan = [1 1];
            groupBox.LayoutGrid = [1 1];

            % Main dialog
            dlgstruct.DialogTitle = ...
                getString(message('ros:slros:readlog:SelectSourceTitle'));
            dlgstruct.HelpMethod = 'ros.slros.internal.helpview';
            dlgstruct.HelpArgs =  {'rosLogfileTopicSelectorDlg'};       % doc topic id
            dlgstruct.PreApplyMethod = 'dlgApplyClose';

            % Make this dialog modal wrt to other DDG dialogs
            % (doesn't block MATLAB command line)
            dlgstruct.Sticky = true;

            % Buttons to show on dialog (these are options to pass to DDG,
            % not the final strings, so there is no need to use message
            % catalog)
            dlgstruct.StandaloneButtonSet =  ...
                {'Ok', 'Cancel', 'Help'}; % also available: 'Revert', 'Apply'

            dlgstruct.Items = {groupBox};
        end
    end

    methods (Access = ?matlab.unittest.TestCase)

        function setCloseFcnHandle(obj, closeFcnHandle)
            %setCloseFcnHandle Set the handle to call when closing the dialog
            %   Used only for testing purposes
            obj.CloseFcnHandle = closeFcnHandle;
        end

    end
end

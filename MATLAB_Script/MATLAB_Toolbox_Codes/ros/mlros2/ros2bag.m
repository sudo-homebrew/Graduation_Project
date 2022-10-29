classdef ros2bag
%ROS2BAG Open and parse ros2bag log file and get content information
%   BAGOBJ = ROS2BAG("FOLDERPATH") creates an ros2bag object BAGOBJ,
%   that contains all the messages from the ROS 2 bag log file
%   located at path FOLDERPATH.
%
%   ROS 2 bag files are used for storing message data. Their primary
%   use is in the logging of messages transmitted over a ROS 2 network.
%   The resulting bag file can be used for offline analysis, visualization,
%   and storage. MATLAB provides functionality for reading existing
%   bag files.
%
%
%   ros2bag properties:
%      FilePath         - (Read-Only) Absolute path to rosbag file
%      StartTime        - (Read-Only) Timestamp of first message
%      EndTime          - (Read-Only) Timestamp of last message
%      NumMessages      - (Read-Only) Number of messages
%      AvailableTopics  - (Read-Only) Table of topics
%      MessageList      - (Read-Only) The list of messages
%
%   ros2bag methods:
%      readMessages     - Deserialize and return message data
%
%
%   Example:
%      % Open a rosbag and retrieve information about its contents
%      folderPath = "path/to/bagfolder";
%
%      % Create a ros2bag object
%      bag = ros2bag(folderPath);
%
%      % Retrieve the messages in the ros2bag object as a cell array
%      msgs = readMessages(bag)
%
%      % Select a subset of the messages by time and topic
%      bagSelection = select(bag,"Time",...
%          [bag.StartTime bag.StartTime + 1],"Topic","/odom")
%
%      % Retrieve the messages in the selection as cell array
%      msgsFiltered = readMessages(bagSelection)

%  Copyright 2020-2021 The MathWorks, Inc.

    properties (SetAccess = private)
        %FilePath - Absolute path to ros2bag file
        FilePath = ''

        %StartTime - Timestamp of first message
        %   The time is expressed in seconds.
        StartTime = 0

        %EndTime - Timestamp of last message
        %   The time is expressed in seconds.
        EndTime = 0

        %AvailableTopics - Table of topics
        AvailableTopics = table.empty(0,3)

        %MessageList - The list of messages
        MessageList = table.empty(0,3)
    end

    properties (Dependent)
        %NumMessages - Number of messages
        NumMessages
    end

    properties (Access = private)
        %TopicTypeMap - Mapping from topic name to message type of topic
        TopicTypeMap

        %TopicDefinitionMap - Mapping from topic name to message definition
        TopicDefinitionMap

        %IsFilter - Whether filter is applied
        IsFilter = false;

        %FilterMessagesByTopicsCell - Filter bag messages based on topic
        FilterMessagesByTopicsCell = {}

        %FilterMessagesByTypesCell - Filter bag messages based on message
        %type
        FilterMessagesByTypesCell = {}

        %FilterMessageStartTime - Start time for messages filtered based
        %on timestamp
        FilterMessageStartTime = {}

        %FilterMessageStartTime - End time for messages filtered based
        %on timestamp
        FilterMessageEndTime = {}
    end

    properties (Transient, Access = private)
        %AvailableTopicsCell - Cell array of topic names
        %   This is pre-cached for fast access.
        AvailableTopicsCell

        %Bag - MCOS C++ object for reading from ros2bag
        Bag
    end
    
    methods (Static)
        function obj = loadobj(s)
            %loadobj Custom behavior on bag file load
            % This custom method ensures that the MCOS C++ object is always
            % initialized as part of the ros2bag construction. This
            % also allows the backwards-compatible loading of pre-18a MAT
            % files.%setupPaths for rosbag
            [bagFolderPath,~] = fileparts(s.FilePath);
            obj = ros2bag(bagFolderPath);
        end
    end

    methods
        function obj = ros2bag(uriPath)

        % Validate inputs
            uriPath = convertStringsToChars(uriPath);
            if isfile(uriPath)
                uriPath = ros.internal.Parsing.validateFilePath(uriPath);
            else
                uriPath = ros.internal.Parsing.validateFolderPath(uriPath);
            end
            [pathEnv, amentPrefixEnv, cleanPath, cleanAmentPath] = ros2bag.setupEnv(); %#ok<ASGLU>

            try
                obj.Bag = rosbag2.bag2.internal.Ros2bagWrapper(uriPath, "");
            catch ex
                if strcmp(ex.identifier, 'ros:mlros2:bag:FileReadingError')...
                        || strcmp(ex.identifier, 'ros:mlros2:bag:InvalidYAMLFile')...
                        || strcmp(ex.identifier, 'ros:mlros2:bag:YAMLFileNotFound')
                    error(ex.identifier, ex.message)
                else
                    rethrow(ex)
                end
            end
            % Initialize the maps
            obj.TopicTypeMap = containers.Map('KeyType', 'char', 'ValueType', 'char');
            obj.TopicDefinitionMap = containers.Map('KeyType', 'char', 'ValueType', 'char');

            bag = obj.Bag;

            obj.FilePath = bag.FilePath;
            topics = bag.topics('.*');
            topicTypes = bag.topicType(topics);

            % reset the environment variable
            setenv('AMENT_PREFIX_PATH', amentPrefixEnv);
            % Populate TopicTypeMap and TopicDefinitionMap
            for i = 1:numel(topics)
                obj.TopicTypeMap(topics{i}) = topicTypes{i};

                % Get message definition and MATLAB-compatible definition
                rosMsgDef = ros2("msg","show",topicTypes{i});
                rosMsgDefLines = string(rosMsgDef).splitlines;
                obj.TopicDefinitionMap(topics{i}) = ros.msg.internal.formatMessageDefinition(...
                    char(strjoin(rosMsgDefLines(2:end), newline)), {}, {});
            end

            % Get list of all the messages in the bag, along with relevant
            % meta-data
            msgListInfo = bag.getMessageList;
            msgTime = msgListInfo{1};
            msgTopic = categorical(msgListInfo{2}, 1:numel(msgListInfo{3}), msgListInfo{3});
            msgType = categorical(msgListInfo{2}, 1:numel(msgListInfo{4}), msgListInfo{4});

            % Populate MessageList
            % Add the message indices to the table. Note that all the
            % table entries are already sorted by message time.
            obj.MessageList = table(msgTime, msgTopic, ...
                                    msgType, 'VariableNames', ...
                                    {'Time', 'Topic', 'MessageType'});

            % Assign object properties that are derived from the full
            % message list
            if obj.NumMessages == 0
                % No point of further processing
                return;
            end

            obj.StartTime = obj.MessageList(1,1).Time;
            obj.EndTime = obj.MessageList(end,1).Time;

            % Recover the topics contained within the message list
            topics = categories(obj.MessageList.Topic);

            types = cellfun(@obj.TopicTypeMap, topics, 'UniformOutput', false);
            defs = cellfun(@obj.TopicDefinitionMap, topics, 'UniformOutput', false);

            % Build table of all topics contained in selection and sort the
            % table rows by alphabetical topic name
            numMessagePerTopic = histcounts(obj.MessageList.Topic);
            obj.AvailableTopics = table( numMessagePerTopic', ...
                                         categorical(types), defs, 'RowNames', topics, 'VariableNames', ...
                                         {'NumMessages', 'MessageType', 'MessageDefinition'});
            obj.AvailableTopics = sortrows(obj.AvailableTopics, 'RowNames');
            obj.AvailableTopicsCell = obj.AvailableTopics.Row;
        end

        function msgs = readMessages(obj, rows)
        %readMessages Read messages from ros2bag
        %   MSGS = readMessages(BAG) returns data from all of the
        %   messages in the ros2bag object, ROS2BAG. The messages are returned
        %   as a cell array of structs
        %
        %   MSGS = readMessages(BAG, ROWS) returns data from messages
        %   in the rows specified by the ROWS argument. All elements of
        %   ROWS must be within the range [1 BAG.NumMessages].
        %
        %   Example:
        %         % create ros2bag and read messages from ROS 2
        %         % bag log file present in user-specified bag folder.
        %         folderPath = "ros2_sensor_data/rosbag2_2020_09_02-21_19_47"
        %         bag = ros2bag(folderPath);
        %
        %         % Return all sensor_msgs/Image messages as a cell array of structs
        %         imageStructs = readMessages(bag);
        %         imageStructs{1}
        %
        %         % Return only the first 10 messages
        %         firstImageStructs = readMessages(bag, 1:10);
        %

            if nargin < 2
                readAll = true;
                rows = 1:height(obj.MessageList);
            else
                validateattributes(rows, {'numeric'}, {'vector', 'integer'}, 'readMessages', 'rows')
                readAll = false;
            end

            % Return right away if there is nothing to read
            if isempty(rows)
                msgs = {};
                return;
            end

            if (min(rows) < 1) || (max(rows) > height(obj.MessageList))
                error(message('ros:mlros2:bag:MsgIndicesInvalid', ...
                              num2str(1), height(obj.MessageList)));
            end

            [pathEnv, amentPrefixEnv, cleanPath, cleanAmentPath] = obj.setupEnv(); %#ok<ASGLU>

            obj.Bag.resetView(obj.FilterMessagesByTopicsCell, ...
                              obj.FilterMessagesByTypesCell, ...
                              obj.FilterMessageStartTime, ...
                              obj.FilterMessageEndTime, ...
                              obj.IsFilter, ...
                              obj.NumMessages);

            if readAll
                % Retrieve all messages if user did not specify rows
                msgs = obj.Bag.readAll();
            else
                % Retrieve subset of messages that user specified
                msgs = obj.Bag.readSubset(rows);
            end
        end

        function bagSelect = select(obj, varargin)
        %SELECT Select a subset of messages based on given criteria
        %   BAGSEL = SELECT(OBJ) returns an object, BAGSEL, that
        %   contains all of the messages in the ros2bag object,
        %   OBJ.
        %
        %   BAGSEL = SELECT(___,Name,Value) provides additional options
        %   specified by one or more Name,Value pair arguments. You can
        %   specify several name-value pair arguments in any order as
        %   Name1,Value1,...,NameN,ValueN:
        %
        %      "Time"     -   Specifies the start and end times of the
        %                     selection as an N-by-2 matrix. Each row
        %                     corresponds to one time interval that
        %                     should be included in the message
        %                     selection.
        %      "Topic"    -   Specify the topic name as string that should be
        %                     included in the selection. Multiple topic
        %                     names can be specified with a cell array
        %                     of strings.
        %      "MessageType" - Specify the message type as string that should be
        %                     included in the selection. Multiple
        %                     message types can be specified with a
        %                     cell array of strings.
        %
        %   Use SELECT to filter the messages available in the returned ros2bag object.
        %   You can use name-value pairs to specify a subset of the
        %   ros2bag messages.
        %
        %   This function does not change the contents of the original
        %   ros2bag object. It returns a new object that contains
        %   the specified message selection.
        %
        %
        %   Example:
        %      % Select messages in the first second of the ros2bag
        %      bagMsgs2 = SELECT(bagMsgs,"Time",[bagMsgs.StartTime,...
        %         bagMsgs.StartTime + 1])
        %
        %      % Select one topic
        %      bagMsgs2 = SELECT(bagMsgs,"Topic","/odom")
        %
        %      % Select multiple topics
        %      bagMsgs2 = SELECT(bagMsgs,"Topic",{"/odom", "/scan"})
        %
        %      % Select by multiple time intervals and message type
        %      bagMsgs2 = SELECT(bagMsgs,"Time",[0 1; 5 7],...
        %         "MessageType","std_msgs/String")

        % If no selection occurs, return the selection as-is
            if nargin == 1
                bagSelect = obj;
                return;
            end

            % Parse the inputs to the function
            defaults.Time = [];
            defaults.Topic = cell.empty;
            defaults.MessageType = cell.empty;

            select = obj.getSelectArguments(defaults, varargin{:});

            % Combine indexing vectors depending on conditions set in the
            % function inputs
            indexOp = logical(ones(height(obj.MessageList),1)); %#ok<LOGL>
            unionOp = ~indexOp;

            % Filter given time interval
            if ~isempty(select.Time)
                timeOp = unionOp;
                for i = 1:size(select.Time,1)
                    % The interval union is considered for selection
                    timeOp = timeOp | (obj.MessageList.Time >= select.Time(i,1) ...
                                       & obj.MessageList.Time <= select.Time(i,2));
                end
                indexOp = indexOp & timeOp;
            end

            % Filter by topic name(s)
            if ~isempty(select.Topic)
                topicOp = unionOp;
                for i = 1:numel(select.Topic)
                    % The topic name union is considered for selection
                    topicOp = topicOp | (obj.MessageList.Topic == select.Topic{i});
                end
                indexOp = indexOp & topicOp;
            end

            % Filter by message type(s)
            if ~isempty(select.MessageType)
                typeOp = unionOp;
                for i = 1:numel(select.MessageType)
                    % The messageType union is considered for selection
                    typeOp = typeOp | (obj.MessageList.MessageType == select.MessageType{i});
                end
                indexOp = indexOp & typeOp;
            end

            % create a new ros2bag object based on the filtering criteria
            % applied on original ros2bag object.
            bagSelect = obj;
            bagSelect.StartTime = 0;
            bagSelect.EndTime = 0;
            bagSelect.AvailableTopics = table.empty(0,3);
            bagSelect.MessageList = obj.MessageList(indexOp,:);
            bagSelect.IsFilter = true;

            if bagSelect.NumMessages == 0
                % No point of further processing
                return;
            end

            bagSelect.StartTime = bagSelect.MessageList(1,1).Time;
            bagSelect.EndTime = bagSelect.MessageList(end,1).Time;

            % Recover the topics contained within the message list
            topics = cellstr(unique(bagSelect.MessageList.Topic));
            types = cellfun(@bagSelect.TopicTypeMap, topics, 'UniformOutput', false);
            defs = cellfun(@bagSelect.TopicDefinitionMap, topics, 'UniformOutput', false);

            % Build table of all topics contained in selection and sort the
            % table rows by alphabetical topic name
            numMessagePerTopic = histcounts(bagSelect.MessageList.Topic);
            numMessagePerTopic = numMessagePerTopic(numMessagePerTopic~=0);
            bagSelect.AvailableTopics = table( numMessagePerTopic', ...
                                               categorical(types), defs, 'RowNames', topics, 'VariableNames', ...
                                               {'NumMessages', 'MessageType', 'MessageDefinition'});
            bagSelect.AvailableTopics = sortrows(bagSelect.AvailableTopics, 'RowNames');
            bagSelect.AvailableTopicsCell = bagSelect.AvailableTopics.Row;
            bagSelect.FilterMessagesByTopicsCell = select.Topic;
            bagSelect.FilterMessagesByTypesCell = select.MessageType;

            if ~isempty(select.Time)
                bagSelect.FilterMessageStartTime = cellstr(string(select.Time(:,1)'));
                bagSelect.FilterMessageEndTime = cellstr(string(select.Time(:,2)'));
            end
        end
    end

    methods
        function NumMessages = get.NumMessages(obj)
        % get the number of messages
            NumMessages = height(obj.MessageList);
        end
    end

    methods (Static)
        function [pathEnv, amentPrefixEnv, cleanPath, cleanAmentPath] = setupEnv()
            mlRoot = matlabroot;
            archKeys = {'win64', 'glnxa64', 'maci64'};
            arch = computer('arch');
            envPathMap = ...
                containers.Map(archKeys, ...
                               {'PATH', ...             % win64
                                'LD_LIBRARY_PATH', ...  % glnxa64
                                'DYLD_LIBRARY_PATH'});  % maci64
            pathEnv = getenv(envPathMap(arch));
            amentPrefixEnv = ros.ros2.internal.getAmentPrefixPath;

            % start directory suggestion
            startPathBase = fullfile(mlRoot, 'sys', 'ros2', ...
                                     arch, 'ros2');
            startPathMap = ...
                containers.Map(archKeys, ...
                               {'bin', ...    % win64
                                'lib', ...    % glnxa64
                                'lib'});      % maci64
            yamlPath = fullfile(startPathBase, 'opt', 'yaml_cpp_vendor', startPathMap(arch));
            curlPath = fullfile(startPathBase, 'opt', 'libcurl_vendor', startPathMap(arch));
            bagLibsPath = fullfile(startPathBase, startPathMap(arch));

            %SetupPaths for ros2bag
            customMsgRegistry = ros.internal.CustomMessageRegistry.getInstance('ros2');
            customMsgDirList = getBinDirList(customMsgRegistry);
            msgList = getMessageList(customMsgRegistry);
            msgInfoList = cellfun(@(msg) getMessageInfo(customMsgRegistry, msg), msgList);
            installDirList = arrayfun(@(msgInfo) msgInfo.installDir, ...
                                      msgInfoList, 'UniformOutput', false);

            setenv(envPathMap(arch), ...
                   strjoin([bagLibsPath,customMsgDirList,...
                            yamlPath, curlPath, pathEnv], pathsep));
            setenv('AMENT_PREFIX_PATH', strjoin([unique(installDirList), amentPrefixEnv], pathsep));
            cleanPath = onCleanup(...
                @() setenv(envPathMap(arch), pathEnv));
            cleanAmentPath = onCleanup(...
                @() setenv('AMENT_PREFIX_PATH', amentPrefixEnv));
        end
    end

    methods (Static, Access = private)
        function select = getSelectArguments(defaults, varargin)
        %getSelectArguments Parse arguments of the select function

            parser = inputParser;

            % Convert all strings to character vectors or cell arrays of
            % character vectors.
            [varargin{:}] = convertStringsToChars(varargin{:});

            % Specify valid Name-Value pairs
            addParameter(parser, 'Time', defaults.Time, @(x) validateattributes(x, ...
                                                              {'uint64'}, {'nonempty', 'ncols', 2}));
            addParameter(parser, 'Topic', defaults.Topic, @(x) validateattributes(x, ...
                                                              {'char', 'cell', 'string'}, {'nonempty', 'vector'}));
            addParameter(parser, 'MessageType', defaults.MessageType, @(x) validateattributes(x, ...
                                                              {'char', 'cell', 'string'}, {'nonempty', 'vector'}));

            % Parse the input and assign outputs
            parse(parser, varargin{:});

            select.Time = parser.Results.Time;
            select.Topic = parser.Results.Topic;
            select.MessageType = parser.Results.MessageType;

            % If single topic input, convert to cell
            % Also verifies that all cell elements are strings
            try
                select.Topic = cellstr(select.Topic);
            catch
                error(message('ros:mlros2:bag:CellStringInvalid', 'Topic'));
            end

            % Add leading slashes to the topic names (if they don't exist)
            select.Topic = cellfun(@(x) ros.ros2.internal.addLeadingSlash(x), ...
                                   select.Topic, 'UniformOutput', false);

            % If single message type input, convert to cell
            % Also verifies that all cell elements are strings
            try
                select.MessageType = cellstr(select.MessageType);
            catch
                error(message('ros:mlros2:bag:CellStringInvalid', 'MessageType'));
            end
        end
    end
end

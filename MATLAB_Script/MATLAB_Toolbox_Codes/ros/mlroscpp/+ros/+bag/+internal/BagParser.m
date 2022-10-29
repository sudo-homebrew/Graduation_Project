classdef BagParser < handle
%This class is for internal use only. It may be removed in the future.

%BagParser Class for parsing rosbag log files
%   This class takes an existing rosbag file and parses it, in the
%   process extracting a list of all messages contained within the bag.
%   This parsing is lightweight, as no message data is read into
%   memory.
%
%   This class only supports the parsing of rosbags in the 2.0 format
%   (see http://wiki.ros.org/Bags/Format/2.0). Only uncompressed
%   rosbags are supported.
%
%   PARSER = ros.bag.internal.BagParser('FILEPATH') will parse
%   the rosbag file located at FILEPATH and return a parser object
%   PARSER. FILEPATH can be either a relative or an absolute path to
%   the rosbag. The result of the parsing will be accessible through
%   public properties of the PARSER object.
%
%   BagParser properties:
%      FilePath     - (Read-Only) Absolute path to rosbag file
%      MessageList  - (Read-Only) The list of messages within the rosbag
%      TopicTypeMap - (Read-Only) Mapping from topic name to message type of topic
%      TopicDefinitionMap - (Read-Only) Mapping from topic name to message definition
%
%
%   Example:
%      % Parse a rosbag at a specific location
%      filePath = 'path/to/logfile.bag';
%      parser = ros.bag.internal.BagParser(filePath);
%
%   See also ros.Bag.

%   Copyright 2014-2021 The MathWorks, Inc.

    properties(Access = private)
        %NumMessages - The number of messages contained in the bag
        NumMessages = 0;

        % These are the temporary columns of the table

        %MsgTime - Temporary vector of message timestamps
        MsgTime = []

        %MsgTopic - Temporary cell array of message topics
        MsgTopic = cell.empty

        %MsgType - Temporary cell array of message types
        MsgType = cell.empty

        %MsgFilePos - Temporary vector of file offsets for messages
        MsgFilePos = []
    end

    properties(SetAccess = private)
        %FilePath - Absolute path to rosbag file
        FilePath = ''

        %MessageList - The list of messages within the rosbag
        MessageList = table.empty(0,4)

        %TopicTypeMap - Mapping from topic name to message type of topic
        TopicTypeMap

        %TopicDefinitionMap - Mapping from topic name to message definition
        TopicDefinitionMap

        %Bag Handle to the MCOS C++ bag object
        Bag
    end

    methods
        function obj = BagParser(filePath)
        %BagParser Constructor for the ros.bag.internal.BagParser class
        %   Create a new parser object for a given rosbag file.
        %   Please see the class documentation (help ros.bag.internal.BagParser)
        %   for more details.

        % Validate inputs
            obj.FilePath = ros.internal.Parsing.validateFilePath(filePath);

            %SetupPaths for rosbag
            cleanPath = ros.internal.setupRosEnv(); %#ok<NASGU> 

            obj.Bag = roscpp.bag.internal.RosbagWrapper(obj.FilePath);

            % Initialize maps
            obj.TopicTypeMap = containers.Map('KeyType', 'char', 'ValueType', 'char');
            obj.TopicDefinitionMap = containers.Map('KeyType', 'char', 'ValueType', 'char');

            % Parse the actual bag file
            obj.parseBag;
        end
    end

    methods(Access = private)
        function parseBag(obj)
        %parseBag Parse the bag file and extract all pertinent information

            bag = obj.Bag;
            obj.NumMessages = bag.NumMessagesInBag;

            topics = bag.topics('.*');
            topicTypes = bag.topicType(topics);

            % Populate TopicTypeMap and TopicDefinitionMap
            for i = 1:length(topics)
                obj.TopicTypeMap(topics{i}) = topicTypes{i};

                % Get message definition and MATLAB-compatible definition
                rosMsgDef = bag.definition(topicTypes{i}, false);
                rosMsgDefLines = string(rosMsgDef).splitlines;
                obj.TopicDefinitionMap(topics{i}) = ros.msg.internal.formatMessageDefinition(...
                    char(strjoin(rosMsgDefLines(2:end), newline)), {}, {});
            end

            % Populate MessageList
            obj.MsgTime = zeros(obj.NumMessages,1);
            obj.MsgTopic = cell(obj.NumMessages,1);
            obj.MsgType = cell(obj.NumMessages,1);
            obj.MsgFilePos = uint64(zeros(obj.NumMessages,1));

            % Make sure that view encompasses complete bag
            bag.resetView(topics, 0.0, intmax);

            % Get list of all the messages in the bag, along with relevant
            % meta-data
            msgListInfo = bag.getMessageList;
            [msgTime, msgTopicIdx, msgFilePos, topicStrings, typeStrings] = msgListInfo{:};

            obj.MsgTime = msgTime;
            obj.MsgFilePos = msgFilePos;

            obj.MsgTopic = categorical(msgTopicIdx, 1:length(topicStrings), topicStrings);
            obj.MsgType = categorical(msgTopicIdx, 1:length(typeStrings), typeStrings);

            % Add the message indices to the table. Note that all the
            % table entries are already sorted by message time.
            obj.MessageList = table(obj.MsgTime, obj.MsgTopic, ...
                                    obj.MsgType, obj.MsgFilePos, 'VariableNames', ...
                                    {'Time', 'Topic', 'MessageType', 'FileOffset'});
        end
    end
end

classdef BagSelection
%BagSelection This class represents a view of messages within a rosbag
%   The BagSelection object is an index of the messages in a rosbag.
%   You can use it to select messages based on specific criteria,
%   extract message data from a rosbag, or create a timeseries of the
%   message properties.
%
%   To create an initial BagSelection object, use the ros.Bag.parse
%   method and a rosbag file.
%
%   The BagSelection object properties provide information about
%   the messages in the rosbag, such as the topic names and timestamps.
%
%   The BagSelection's select method creates a subset of the index
%   in the original object, and returns it as a new ros.BagSelection
%   object. The selection method runs quickly, with low memory overhead,
%   because it operates on the index, not on the messages and data in
%   the rosbag.
%
%   BAGSEL = ros.BagSelection('FILEPATH', MESSAGELIST, TOPICTYPEMAP,
%   TOPICDEFINITIONMAP) creates a rosbag selection representing the N
%   messages listed in MESSAGELIST (Nx4 table). The rosbag file is located at
%   FILEPATH. TOPICTYPEMAP represents a map from topic name to message
%   type, while TOPICDEFINITIONMAP stores a map from topic name to message
%   definition. The call returns a new selection object BAGSEL.
%
%
%   BagSelection properties:
%      FilePath         - (Read-Only) Absolute path to rosbag file
%      StartTime        - (Read-Only) Timestamp of first message in this selection
%      EndTime          - (Read-Only) Timestamp of last message in this selection
%      NumMessages      - (Read-Only) Number of messages in this selection
%      AvailableTopics  - (Read-Only) Table of topics in this selection
%      AvailableFrames  - (Read-only) List of all available coordinate frames
%      MessageList      - (Read-Only) The list of messages in this selection
%
%   BagSelection methods:
%      readMessages     - Deserialize and return message data
%      select           - Select a subset of messages based on given criteria
%      timeseries       - Return a timeseries object for message properties
%      getTransform     - Return transformation between two coordinate frames
%      canTransform     - Verify if transformation is available
%
%
%   Example:
%      % Open a rosbag and retrieve information about its contents
%      filePath = 'path/to/logfile.bag';
%
%      % The parsing returns a selection of all messages
%      bagMsgs = ros.Bag.parse(filePath)
%
%      % Select a subset of the messages by time and topic
%      bagMsgs2 = select(bagMsgs, 'Time', ...
%          [bagMsgs.StartTime bagMsgs.StartTime + 1], 'Topic', '/odom')
%
%      % Retrieve the messages in the selection as cell array
%      msgs = readMessages(bagMsgs2)
%
%      % Return message properties as time series
%      ts = timeseries(bagMsgs, 'Pose.Pose.Position.X', ...
%          'Twist.Twist.Angular.Y')
%
%   See also ROSBAG, ros.Bag.

%   Copyright 2014-2021 The MathWorks, Inc.

    properties (SetAccess = private)
        %FilePath - Absolute path to rosbag file
        FilePath = ''

        %StartTime - Timestamp of first message in this selection
        %   The time is expressed in seconds.
        StartTime = 0

        %EndTime - Timestamp of last message in this selection
        %   The time is expressed in seconds.
        EndTime = 0

        %NumMessages - Number of messages in this selection
        NumMessages = 0

        %AvailableTopics - Table of topics in this selection
        AvailableTopics = table.empty(0,3)

        %AvailableFrames - List of all available coordinate frames
        AvailableFrames = {}

        %MessageList - The list of messages in this selection
        MessageList = table.empty(0,4)
    end

    properties (Access = protected)
        %TopicTypeMap - Mapping from topic name to message type of topic
        TopicTypeMap

        %TopicDefinitionMap - Mapping from topic name to message definition
        TopicDefinitionMap

        %messageTypeMap - Mapping from MessageType to object of that type
        messageTypeMap
    end

    properties (Transient, Access = private)
        %AvailableTopicsCell - Cell array of topic names
        %   This is pre-cached for fast access.
        AvailableTopicsCell

        %BagTF - MCOS C++ object for accessing TF information in rosbag
        BagTF
    end
    
    properties (Transient, Access = protected)
        %Bag - MCOS C++ object for reading from rosbag
        Bag
    end
    
    properties (Access = ?matlab.unittest.TestCase)
        %Parser - Helper object for parsing tasks
        Parser
    end

    methods (Access = {?ros.Bag, ?rosbagreader})
        function obj = BagSelection(filePath, bagImpl, messageList, topicTypeMap, ...
                                    topicDefinitionMap)
            %BagSelection Constructor for BagSelection class
            %   Create a new selection object based on an existing message list.
            %   Please see the class documentation (help ros.BagSelection)
            %   for more details.
            %   This constructor cannot be called directly by the user, but
            %   has to be invoked via the ros.Bag interface for initiating
            %   the rosbag parsing.

            % Validate inputs
            validateattributes(topicTypeMap, {'containers.Map'}, {}, ...
                               'BagSelection', 'topicTypeMap');
            validateattributes(topicDefinitionMap, {'containers.Map'}, {}, ...
                               'BagSelection', 'topicDefinitionMap');

            obj.Bag = bagImpl;

            obj.Parser = ros.internal.tf.TransformationTreeParser;

            % Initialize the maps
            obj.TopicTypeMap = containers.Map('KeyType', 'char', 'ValueType', 'char');
            obj.TopicDefinitionMap = containers.Map('KeyType', 'char', 'ValueType', 'char');
            obj.messageTypeMap = containers.Map.empty;

            % The messageList will be validated in custom setter function
            obj.MessageList = messageList;
            obj.FilePath = ros.internal.Parsing.validateFilePath(filePath);

            % Extract information from message list
            obj.TopicTypeMap = topicTypeMap;
            obj.TopicDefinitionMap = topicDefinitionMap;
            obj.MessageList = messageList;

            % Assign object properties that are derived from the full
            % message list
            obj.NumMessages = height(obj.MessageList);
            if obj.NumMessages == 0
                % No point of further processing
                return;
            end

            obj.StartTime = min(obj.MessageList.Time);
            obj.EndTime = max(obj.MessageList.Time);

            % Recover the topics contained within the message list
            topics = categories(obj.MessageList.Topic);
            types = cell(size(topics,1), 1);
            defs = cell(size(topics,1), 1);
            for i = 1:size(topics,1)
                topic = topics{i,1};
                types{i,1} = obj.TopicTypeMap(topic);
                defs{i,1} = obj.TopicDefinitionMap(topic);
            end

            % Build table of all topics contained in selection and sort the
            % table rows by alphabetical topic name
            numMessagePerTopic = histcounts(obj.MessageList.Topic);
            obj.AvailableTopics = table( numMessagePerTopic', ...
                                         categorical(types), defs, 'RowNames', topics, 'VariableNames', ...
                                         {'NumMessages', 'MessageType', 'MessageDefinition'});
            obj.AvailableTopics = sortrows(obj.AvailableTopics, 'RowNames');
            obj.AvailableTopicsCell = obj.AvailableTopics.Row;

            % Parse the transformation tree and assign available frames
            obj.BagTF = roscpp.bag.internal.BagTfWrapper(obj.FilePath);
            obj.BagTF.build(obj.StartTime, obj.EndTime, '/tf');
            obj.AvailableFrames = sort(obj.BagTF.AvailableFrames);
        end
    end

    methods
        function msgs = readMessages(obj, varargin)
        %readMessages Read messages from rosbag
        %   MSGS = readMessages(BAGSEL) returns data from all of the
        %   messages in the BagSelection object, BAGSEL. The messages will
        %   be returned in a cell array of message objects, MSGS.
        %
        %   MSGS = readMessages(BAGSEL,ROWS) returns data from messages
        %   in the rows specified by the ROWS argument. The maximum
        %   range of the ROWS argument is [1,BAGSEL.NumMessages].
        %
        %   MSGS = readMessages(___,"DataFormat",FORMAT) determines the
        %   format of the messages contained in the returned cell array,
        %   specified as either "object" or "struct".
        %   The "struct" format is typically faster than using message
        %   objects, and data may be retrieved from custom messages
        %   directly without generating them from the message
        %   definitions.
        %
        %   When working with large numbers of messages, this method
        %   consumes a lot of time and system memory. Consider using
        %   the select method to create a smaller BagSelection object
        %   before calling readMessages.
        %
        %
        %   Example:
        %
        %         % Load rosbag and select topic
        %         bag = rosbag("ros_turtlesim.bag");
        %         poseTopic = select(bag,"Topic","/turtle1/pose");
        %
        %         % Return all pose messages as a cell array of structs
        %         poseStructs = readMessages(poseTopic,"DataFormat","struct");
        %         poseStructs{1}
        %
        %         % Return only the first 10 messages
        %         firstPoseStructs = readMessages(bagMsgs, 1:10,"DataFormat","struct");
        %
        %         % Return all messages as a cell array of objects
        %         poseMsgs = readMessages(poseTopic);
        %         poseMsgs{1}
        %
        %   See also select.


            defaults = struct(...
                'rows', 1:height(obj.MessageList), ...
                'DataFormat', 'msg');

            % Parse inputs to function
            parser = inputParser;
            parser.CaseSensitive = false;

            % Specify valid Name-Value pairs
            addOptional(parser, 'rows', defaults.rows, ...
                        @(x) validateattributes(x, {'numeric'}, {'vector', 'real', 'nonnan', 'finite'}));
            % Leave 'msg' as an option for backwards-compatibility (synonym for 'object')
            addParameter(parser, 'DataFormat', defaults.DataFormat, ...
                         @(x) validateStringParameter(x, {'object', 'struct', 'msg'}, true));

            % Parse the input and assign outputs
            parse(parser, varargin{:});
            rows = double(parser.Results.rows);
            dataFormat = parser.Results.DataFormat;

            % Return right away if there is nothing to read
            if isempty(rows)
                msgs = {};
                return;
            end

            if (min(rows) < 1) || (max(rows) > height(obj.MessageList))
                error(message('ros:mlros:bag:MsgIndicesInvalid', ...
                              num2str(1), height(obj.MessageList)));
            end

            obj.Bag.resetView(obj.AvailableTopicsCell, obj.StartTime - eps(obj.StartTime), obj.EndTime + eps(obj.StartTime));

            if ismember('rows', parser.UsingDefaults)
                % Retrieve all messages if user did not specify rows
                msgStructs = obj.Bag.readAll(false);
            else
                % Retrieve subset of messages that user specified
                msgStructs = obj.Bag.readSubset(rows, false);
            end
            if strncmpi(dataFormat, 'struct', strlength(dataFormat))
                msgs = msgStructs;
            else
                msgs = obj.getClassFromStruct(msgStructs);
            end

            function validateStringParameter(value, options, partialMatching)
            % Separate function to suppress output and just validate
            % Only do case-insensitive matching
                ros.internal.Parsing.matchString(value, options, partialMatching);
            end
        end

        function bagSelect = select(obj, varargin)
        %SELECT Select a subset of messages based on given criteria
        %   BAGSEL = SELECT(OBJ) returns an object, BAGSEL, that
        %   contains all of the messages in the BagSelection object,
        %   OBJ.
        %
        %   BAGSEL = SELECT(___, Name, Value) provides additional options
        %   specified by one or more Name,Value pair arguments. Name must appear
        %   inside single quotes (''). You can specify several name-value pair
        %   arguments in any order as Name1,Value1,...,NameN,ValueN:
        %
        %      'Time'     -   Specifies the start and end times of the
        %                     selection as an N-by-2 matrix. Each row
        %                     corresponds to one time interval that
        %                     should be included in the message
        %                     selection.
        %      'Topic'    -   Specify the topic name as string that should be
        %                     included in the selection. Multiple topic
        %                     names can be specified with a cell array
        %                     of strings.
        %      'MessageType' - Specify the message type as string that should be
        %                     included in the selection. Multiple
        %                     message types can be specified with a
        %                     cell array of strings.
        %
        %   Use SELECT to get rosbag messages from a BagSelection object.
        %   You can use name-value pairs to specify a subset of the
        %   rosbag messages.
        %
        %   This function does not change the contents of the original
        %   BagSelection object. It returns a new object that contains
        %   the specified message selection.
        %
        %
        %   Example:
        %      % Select messages in the first second of the rosbag
        %      bagMsgs2 = SELECT(bagMsgs, 'Time', [bagMsgs.StartTime, ...
        %         bagMsgs.StartTime + 1])
        %
        %      % Select one topic
        %      bagMsgs2 = SELECT(bagMsgs, 'Topic', '/odom')
        %
        %      % Select multiple topics
        %      bagMsgs2 = SELECT(bagMsgs, 'Topic', {'/odom', '/scan'})
        %
        %      % Select by multiple time intervals and message type
        %      bagMsgs2 = SELECT(bagMsgs, 'Time', [0 1; 5 7], ...
        %         'MessageType', 'std_msgs/String')

        % If no selection occurs, return the selection as-is
            if nargin == 1
                bagSelect = obj;
                return;
            end
            
            indexOp = parseAndIndex(obj, varargin{:});

            % Return a new bag selection based on the filtering criteria
            bagSelect = ros.BagSelection(obj.FilePath, obj.Bag, ...
                                         obj.MessageList(indexOp,:), obj.TopicTypeMap, ...
                                         obj.TopicDefinitionMap);
        end
        
        function indexOp = parseAndIndex(obj, varargin)
                        % Parse the inputs to the function
            defaults.Time = [];
            defaults.Topic = cell.empty;
            defaults.MessageType = cell.empty;

            select = obj.getSelectArguments(defaults, varargin{:});

            % Combine indexing vectors depending on conditions set in the
            % function inputs
            indexOp = logical(ones(height(obj.MessageList),1)); %#ok<LOGL>
            unionOp = ~indexOp;

            % Filter each given time interval
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
        end
        
        function [ts,cols] = timeseries(obj, varargin)
        %TIMESERIES Returns a timeseries object for message properties
        %   [TS, COLS] = TIMESERIES(OBJ) returns a time series for all
        %   numeric and scalar message properties. The method evaluates
        %   each message in the current BagSelection object, OBJ, and
        %   returns a timeseries object, TS. The optional second return,
        %   COLS, stores property names as a cell array of strings.
        %
        %   [TS, COLS] = TIMESERIES(OBJ, 'PROP') returns a time series
        %   for a specific message property, PROP.
        %
        %   [TS, COLS] = TIMESERIES(OBJ, 'PROP1', ..., 'PROPN') returns
        %   a time series for a range of message properties, from PROP1
        %   to PROPN. Each property is a different column in the
        %   timeseries object, TS.
        %
        %   Use TIMESERIES to get the time series of one or more message
        %   properties.
        %   A time series is a data vector that represents the evolution
        %   of one or more dynamic properties. For rosbag messages,
        %   you can use a time series to observe the change of particular
        %   message data properties over time.
        %   This method only supports a BagSelection object, OBJ, that
        %   contains a single message topic (with a single message type).
        %
        %   The TIMESERIES object returned by this method is
        %   memory-efficient because it only stores particular message
        %   properties, not whole messages.
        %
        %
        %   Example:
        %      % Call with a single property
        %      ts = TIMESERIES(bagMsgs, 'Pose.Pose.Position.X')
        %
        %      % Extract timeseries with multiple properties
        %      ts = TIMESERIES(bagMsgs, 'Twist.Twist.Angular.X', ...
        %         'Twist.Twist.Angular.Y', 'Twist.Twist.Angular.Z')

            ts = timeseries.empty;

            % If message selection is empty return right away
            if isempty(obj.MessageList)
                return;
            end

            % Right now, timeseries only works for one topic. If we want to
            % do multiple topics in the future, we will have to take time
            % interpolation into account
            numTopics = length(categories(obj.MessageList.Topic));
            if numTopics ~= 1
                error(message('ros:mlros:bag:TimeSeriesSingleTopic', ...
                              num2str(numTopics)));
            end

            % Extract and validate property arguments
            messageType = char(categories(obj.MessageList.MessageType));
            props = obj.getTimeseriesArguments(messageType, varargin{:});

            % Time needs to be in sorted order
            index = sortrows(obj.MessageList);
            time = index.Time;
            numMsg = size(time,1);
            data = zeros(numMsg, numel(props));

            % Extract messages in chunks of 50 messages
            %Should be reasonably fast, while still preserving memory
            msgIdx = 1:50:numMsg;
            for i = 1:length(msgIdx)
                if i == length(msgIdx)
                    msgs = obj.readMessages(msgIdx(i):numMsg, 'DataFormat', 'struct');
                else
                    msgs = obj.readMessages(msgIdx(i):msgIdx(i+1)-1, 'DataFormat', 'struct');
                end

                % Extract properties from messages
                for j = 1:length(msgs)
                    data(msgIdx(i)+j-1,:) = obj.evaluateProperties(msgs{j}, props);
                end
            end

            % Now create timeseries object
            ts = timeseries(data, time, 'Name', [char(categories(obj.MessageList.Topic)) ' Properties']);
            ts.TimeInfo.Units = 'seconds';
            ts.DataInfo.UserData = props;
            cols = props;
        end

        function tf = getTransform(obj, targetFrame, sourceFrame, varargin)
        %getTransform Return transformation between two coordinate frames
        %   TF = getTransform(BAGSEL, 'TARGETFRAME', 'SOURCEFRAME') returns
        %   the latest known transformation between two coordinate frames in
        %   the rosbag associated with selection BAGSEL.
        %   TF represents the transformation that takes coordinates
        %   in the SOURCEFRAME into the corresponding coordinates in
        %   the TARGETFRAME. An error is displayed if no
        %   transformation between these frames exists in the bag.
        %
        %   TF = getTransform(BAGSEL, 'TARGETFRAME', 'SOURCEFRAME', SOURCETIME)
        %   returns the transformation at the given SOURCETIME. An error
        %   is displayed if the transformation at that time is not
        %   available.
        %
        %
        %   Example:
        %       % Load rosbag
        %       bag = rosbag('ros_turtlesim.bag');
        %
        %       % Get list of all available frames
        %       frames = bag.AvailableFrames
        %
        %       % Get the latest transformation between two coordinate frames
        %       tfMsg = getTransform(bag, 'world', frames{1})
        %
        %       % Get transformation at specific time
        %       tfMsgAtTime = getTransform(bag, 'world', frames{1}, rostime(bag.StartTime + 1))
        %
        %   See also canTransform.

            narginchk(3,4);

            defaults = struct(...
                'SourceTime', 0);
            [targetFrame, sourceFrame, sourceTime] = ...
                obj.Parser.parseGetTransformInput(defaults, targetFrame, sourceFrame, varargin{:});

            % Retrieve transformation from bag
            transform = obj.BagTF.lookupTransform(targetFrame, sourceFrame, sourceTime.seconds);

            % Create and return MATLAB transformation message
            tf = rosmessage('geometry_msgs/TransformStamped');
            tf.ChildFrameId = sourceFrame;
            tf.Header.FrameId = targetFrame;
            tf.Header.Stamp = sourceTime;

            % Assign translation
            trans = tf.Transform.Translation;
            trans.X = transform.Translation(1);
            trans.Y = transform.Translation(2);
            trans.Z = transform.Translation(3);

            % Assign rotation
            rot = tf.Transform.Rotation;
            rot.W = transform.Rotation(4);
            rot.X = transform.Rotation(1);
            rot.Y = transform.Rotation(2);
            rot.Z = transform.Rotation(3);

        end

        function isAvailable = canTransform(obj, targetFrame, sourceFrame, varargin)
        %canTransform Verify if transformation is available
        %   ISAVAILABLE = canTransform(BAGSEL, 'TARGETFRAME', 'SOURCEFRAME')
        %   verifies if a transformation that takes coordinates
        %   in the SOURCEFRAME into the corresponding coordinates in
        %   the TARGETFRAME is available. ISAVAILABLE is TRUE if that
        %   transformation is available and FALSE otherwise.
        %   Use getTransform to retrieve the transformation.
        %
        %   ISAVAILABLE = canTransform(BAGSEL, 'TARGETFRAME', 'SOURCEFRAME', SOURCETIME)
        %   verifies that the transformation is available for the time
        %   SOURCETIME. Use getTransform with the SOURCETIME argument to retrieve
        %   the transformation.
        %
        %
        %   Example:
        %       % Load rosbag
        %       bag = rosbag('ros_turtlesim.bag');
        %
        %       % Verify if transformation between world and turtle1 exists
        %       tf = canTransform(bag, 'world', 'turtle1')
        %
        %   See also getTransform.

            narginchk(3,4);

            defaultSourceTime = 0;
            [targetFrame, sourceFrame, sourceTime] = ...
                obj.Parser.parseCanTransformInput(defaultSourceTime, targetFrame, sourceFrame, varargin{:});

            % Check if transformation is available
            isAvailable = obj.BagTF.canTransform(targetFrame, sourceFrame, sourceTime.seconds);
        end

    end

    methods
        function obj = set.MessageList(obj, value)
        %set.MessageList Custom setter function for MessageList

        % Validate input
            validateattributes(value, {'table'}, {'ncols', 4}, ...
                               'BagSelection', 'MessageList');
            obj.MessageList = value;

            % Get rid of extraneous categories
            obj.MessageList.Topic = removecats(obj.MessageList.Topic);
            obj.MessageList.MessageType = removecats(obj.MessageList.MessageType);
        end
    end
    
    methods (Static)
        function obj = loadobj(s)
        %loadobj Custom behavior on bag file load
        %   This custom method ensures that the MCOS C++ object is always
        %   initialized as part of the BagSelection construction. This
        %   also allows the backwards-compatible loading of pre-18a MAT
        %   files.

            %Set env-variables for rosbag.
            cleanPath = ros.internal.setupRosEnv(); %#ok<NASGU> 

            bagImpl = roscpp.bag.internal.RosbagWrapper(s.FilePath);

            obj = ros.BagSelection(s.FilePath, bagImpl, s.MessageList, ...
                                   s.TopicTypeMap, s.TopicDefinitionMap);
        end
    end
    
    methods(Access = private)
        function ros1msg = getClassFromStruct(obj, msgStructs)
            ros1msg = cell(numel(msgStructs), 1);
            obj.messageTypeMap = containers.Map.empty;

            for k = 1:size(msgStructs,1)
                msgType = msgStructs{k}.MessageType;
                % to reuse the object of a particular message type
                if ~isKey(obj.messageTypeMap, msgType)
                    ros1msg{k,1} = rosmessage(msgType);
                    obj.messageTypeMap(msgType) = copy(ros1msg{k,1});
                else
                    ros1msg{k,1} = copy(obj.messageTypeMap(msgType));
                end
                % copying the content from message structure to message object
                ros1msg{k,1} = obj.copyFromStruct(ros1msg{k,1},msgStructs{k});
            end
        end
        
        function cobj = copyFromStruct(obj,cobj, strObj)
        % cobj- Object of the Message into which the structure is to be
        % copied
        % strObj- Structure representation of the Message
            if isempty(cobj)
                cobj = feval(class(cobj), strObj);
                return
            end
            for k = 1:numel(cobj.PropertyList)
                fc = cobj.PropertyList{k};       % Class fields
                                                 % Recurse on nested messages
                if isa(cobj.(fc), 'handle')
                    % Handling empty structures
                    if isempty(strObj.(fc))
                        cobj.(fc) = feval([class(cobj.(fc)) '.empty'], 0, 1);
                    elseif isscalar(strObj.(fc))
                        cobj.(fc) = obj.copyFromStruct(cobj.(fc), strObj.(fc));
                    else
                        innerStruct = strObj.(fc);
                        innerType = innerStruct(1).MessageType;
                        if ~isKey(obj.messageTypeMap, innerType)
                            obj.messageTypeMap(innerType) = rosmessage(innerType);
                        end
                        innerMsg = obj.messageTypeMap(innerType);
                        objArray = arrayfun(@(s) obj.copyFromStruct(copy(innerMsg), s), innerStruct);
                        cobj.(fc) = objArray(:);
                    end
                else
                    cobj.(fc) = strObj.(fc);
                end
            end
        end
    end

    methods (Static, Access = private)
        function select = getSelectArguments(defaults, varargin)
        %getSelectArguments Parse arguments of the select function

            parser = inputParser;
            parser.CaseSensitive = false;

            % Convert all strings to character vectors or cell arrays of
            % character vectors.
            [varargin{:}] = convertStringsToChars(varargin{:});

            % Specify valid Name-Value pairs
            addParameter(parser, 'Time', defaults.Time, @(x) validateattributes(x, ...
                                                              {'numeric'}, {'nonempty', 'ncols', 2}));
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
                error(message('ros:mlros:bag:CellStringInvalid', 'Topic'));
            end

            % Add leading slashes to the topic names (if they don't exist)
            select.Topic = cellfun(@(x) ros.internal.NetworkIntrospection.addLeadingSlash(x), ...
                                   select.Topic, 'UniformOutput', false);

            % If single message type input, convert to cell
            % Also verifies that all cell elements are strings
            try
                select.MessageType = cellstr(select.MessageType);
            catch
                error(message('ros:mlros:bag:CellStringInvalid', 'MessageType'));
            end
        end

        function props = getTimeseriesArguments(messageType, varargin)
        %getTimeseriesArguments Extract and validate property arguments
        %   These properties are given as input arguments to the
        %   timeseries function.

            numProps = numel(varargin);
            props = cell(numProps, 1);

            % Retrieve cell array of valid properties
            validProps = ros.BagSelection.getNumericScalarProperties(messageType);

            % If no properties are specified return a timeseries containing
            % all numeric, scalar properties
            if nargin == 1
                props = validProps;
                return;
            end

            for i = 1:numProps
                % All properties have to be strings
                prop = robotics.internal.validation.validateString(varargin{i}, false, 'timeseries', 'Property');

                % Validate that the given property is accessible and
                % a numeric scalar
                if ~ismember(prop, validProps)
                    error(message('ros:mlros:bag:MsgPropertyInvalid', ...
                                  prop, messageType));
                end

                props{i} = prop;
            end
        end

        function props = getNumericScalarProperties(messageType)
        %getNumericScalarProperties Get all properties for message type
        %
        %   PROPS = getNumericScalarProperties(MESSAGETYPE) retrieves a
        %   cell array of strings of valid numeric, scalar properties
        %   in a message of type MESSAGETYPE.

        % Create message of type and convert to structure
            testMsg = rosmessage(messageType);
            structMsg = testMsg.toStruct;

            % List all valid properties
            props = ros.BagSelection.listStructProperties(structMsg, '');
        end

        function props = listStructProperties(structMsg, prefix)
        %evaluateStructProperties Recursively list all numeric props
        %
        %   PROPS = listStructProperties(STRUCTMSG, PREFIX) recursively
        %   evaluates the input structure STRUCTMSG and extract all
        %   numeric, scalar properties. The property name will be
        %   prefixed by the PREFIX namespace. All valid properties are
        %   returned as cell array of strings PROPS.

            props = {};

            % Look through all structure fields
            strFields = fields(structMsg);
            for i = 1:length(strFields)
                % Extract field name and value
                fieldName = strFields{i};
                fieldValue = structMsg.(fieldName);

                % End of recursion
                % If property is numeric and a scalar, add to return list
                if isnumeric(fieldValue) && isscalar(fieldValue)
                    props{end+1,1} = [prefix fieldName]; %#ok<AGROW>
                    continue;
                end

                % Recurse into nested structures
                if isa(fieldValue, 'struct')
                    props = [props; ros.BagSelection.listStructProperties( ...
                        fieldValue, [prefix fieldName '.'])]; %#ok<AGROW>
                end
            end

        end

        function data = evaluateProperties(msg, props) %#ok<INUSL>
        %evaluateProperties Evaluate properties for a message object

            data = zeros(1, numel(props));
            for i = 1:length(props)
                propName = props{i};

                % Have to use eval here since propName potentially
                % contains multiple indirections / namespaces
                value = eval(['msg.' propName]);
                data(1,i) = value;
            end

        end
    end

end

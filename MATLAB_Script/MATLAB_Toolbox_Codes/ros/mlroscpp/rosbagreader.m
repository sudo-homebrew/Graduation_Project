classdef rosbagreader < ros.BagSelection
    %ROSBAGREADER Access rosbag log file information.
    %   BAGREADER = rosbagreader('FILEPATH') creates an indexable rosbagreader
    %   object, BAGREADER, that contains all the messages from the rosbag
    %   located at path FILEPATH. You can use it to select messages based on 
    %   specific criteria, extract message data from a rosbag, or create a 
    %   timeseries of the message properties.
    %
    %   rosbagreader properties:
    %      FilePath         - (Read-Only) Absolute path to rosbag file
    %      StartTime        - (Read-Only) Timestamp of first message in this selection
    %      EndTime          - (Read-Only) Timestamp of last message in this selection
    %      NumMessages      - (Read-Only) Number of messages in this selection
    %      AvailableTopics  - (Read-Only) Table summarizing information about all topics in this selection
    %      AvailableFrames  - (Read-only) List of all available coordinate frames  in this selection
    %      MessageList      - (Read-Only) List of messages in this selection
    %
    %   rosbagreader methods:
    %      readMessages     - Deserialize and return message data
    %      select           - Select a subset of messages based on given criteria
    %      timeseries       - Return a timeseries object for message properties
    %      getTransform     - Return transformation between two coordinate frames
    %      canTransform     - Verify if transformation is available
    %
    %   Use the select method to select a subset of messages from the bag file. 
    %   Then use the readMessages method to read all the messages in that selection.
    %
    %   The rosbagreader's select method creates a subset of the index
    %   in the original object, and returns it as a new rosbagreader
    %   object. The selection method runs quickly, with low memory overhead,
    %   because it operates on the index, not on the messages and data in
    %   the rosbag.
    %
    %   Example:
    %      % Open a rosbag and retrieve information about its contents
    %      bagMsgs = rosbagreader("ex_multiple_topics.bag")
    %
    %      % Select a subset of the messages by time and topic
    %      bagMsgs2 = select(bagMsgs,"Time", ...
    %          [bagMsgs.StartTime bagMsgs.StartTime + 1],"Topic","/odom")
    %
    %      % Retrieve the messages in the selection as cell array
    %      msgs = readMessages(bagMsgs2)
    %
    %      % Return message properties as time series
    %      ts = timeseries(bagMsgs2,"Pose.Pose.Position.X", ...
    %          "Twist.Twist.Angular.Y")
    %
    %   See also rosbagwriter, ROSBAG.

    %   Copyright 2021 The MathWorks, Inc.
    
    properties (Access = private)
        %CallerName - 
        CallerName
    end
    
    methods
        function obj = rosbagreader(filePath, varargin)
            %ROSBAGREADER Constructor for rosbagreader class
            %   Create a new rosbagreader object which is used to read the 
            %   bag file available in the path.
            %   Please see the class documentation (help rosbagreader)
            %   for more details.
            
            % Find bag file anywhere on the MATLAB path
            filePath = convertStringsToChars(filePath);
            absFilePath = robotics.internal.validation.findFilePath(filePath);
             
            if isequal(length(varargin), 4) && ...
                    isequal(varargin{4}, 'loadobj')
                %This operation is allowed only While loading object from 
                % a .mat file.
                
                narginchk(5,5);
                cleanPath = ros.internal.setupRosEnv(); %#ok<NASGU> 
                bagWrapper = roscpp.bag.internal.RosbagWrapper(absFilePath);
                msgList = varargin{1};
                topicTypeMap = varargin{2};
                topicDefinitionMap = varargin{3};
            elseif isequal(length(varargin), 5) && ...
                    isequal(varargin{5}, 'select')
                % This operation is allowed only if it is called from
                % select() API.
                
                narginchk(6,6);
                bagWrapper = varargin{1};
                msgList = varargin{2};
                topicTypeMap = varargin{3};
                topicDefinitionMap = varargin{4};
            else
                % Actual rosbagreader call (from user or from any other place). 
                narginchk(1,1);
                % Parse the given rosbag
                bagParser = ros.bag.internal.BagParser(absFilePath);
                bagWrapper = bagParser.Bag;
                msgList = bagParser.MessageList;
                topicTypeMap = bagParser.TopicTypeMap;
                topicDefinitionMap = bagParser.TopicDefinitionMap;
            end
             
             % Create a bag selection and return it to the user
             obj = obj@ros.BagSelection(absFilePath, bagWrapper, ...
                msgList, topicTypeMap, topicDefinitionMap);
        end
        
        function rosBagReader = select(obj, varargin)
        %SELECT Select a subset of messages based on given criteria
        %   BAGREADER = SELECT(OBJ) returns an object, BAGREADER, that
        %   contains all of the messages in the rosbagreader object,
        %   OBJ.
        %
        %   BAGREADER = SELECT(___,Name,Value) provides additional options
        %   specified by one or more Name,Value pair arguments as described 
        %   below. Specify the name-value pair arguments to select a subset 
        %   of messages to read from the rosbag. You can specify several 
        %   name-value pair arguments in any order as 
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
        %   This function creates a subset of the index in the original 
        %   object, and returns it as a new rosbagreader object that contains
        %   the specified message selection. The selection method runs quickly,
        %   with low memory overhead, because it operates on the index, not
        %   on the messages and data in the rosbag.
        %   
        %   Example:
        %      % Open a rosbag and retrieve information about its contents
        %      bagMsgs = rosbagreader("ex_multiple_topics.bag")
        %
        %      % Select messages in the first second of the rosbag
        %      bagMsgs2 = SELECT(bagMsgs,"Time",[bagMsgs.StartTime, ...
        %         bagMsgs.StartTime + 1])
        %
        %      % Select one topic
        %      bagMsgs2 = SELECT(bagMsgs,"Topic","/odom")
        %
        %      % Select multiple topics
        %      bagMsgs2 = SELECT(bagMsgs,"Topic",{"/odom","/scan"})
        %
        %      % Select by multiple time intervals and message type
        %      bagMsgs2 = SELECT(bagMsgs,"Time",[0 1; 5 7], ...
        %         "MessageType","std_msgs/String")

        % If no selection occurs, return the selection as-is
            if nargin == 1
                rosBagReader = obj;
                return;
            end

            indexOp = parseAndIndex(obj, varargin{:});

            % Return a new bag selection based on the filtering criteria
            rosBagReader = rosbagreader(obj.FilePath, obj.Bag, ...
                                         obj.MessageList(indexOp,:), obj.TopicTypeMap, ...
                                         obj.TopicDefinitionMap, 'select');
        end
    end
     
    methods (Static)
        function obj = loadobj(s)
        %loadobj Custom behavior on bag file load
        %   This custom method ensures that the MCOS C++ object is always
        %   initialized as part of the rosbagwriter construction.

            obj = rosbagreader(s.FilePath, s.MessageList, ...
                                   s.TopicTypeMap, s.TopicDefinitionMap, 'loadobj');
        end
    end
end


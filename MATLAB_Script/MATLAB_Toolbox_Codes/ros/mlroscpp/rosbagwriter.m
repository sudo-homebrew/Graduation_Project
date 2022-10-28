classdef rosbagwriter < ros.internal.mixin.ROSInternalAccess & ...
        robotics.core.internal.mixin.Unsaveable & handle & ...
        fusion.internal.UnitDisplayer

    %ROSBAGWRITER Write messages to rosbag log files.
    % BAGWRITER = rosbagwriter(PATH) creates a bag file in the location specified 
    % by PATH and returns its corresponding rosbagwriter object, BAGWRITER, 
    % which can be used to write records into the bag file. If you do not 
    % specify the name of the bag file in PATH, the function assigns the 
    % current timestamp as the file name. If the folders specified in PATH 
    % are not present in the directory, the function creates them and places 
    % the bag file accordingly.
    %
    % Each record consists of a topic, its corresponding timestamp, and the 
    % ROS message. Use the write method to write records into the bag file.
    %
    % BAGWRITER = rosbagwriter(___,Name,Value) provides additional options 
    % specified by one or more Name,Value pair arguments as described below.
    %
    %    "Compression"   -  Compression format for individual message chunks 
    %                       specified as 'bz2', 'lz4' or 'uncompressed'. The 
    %                       default compression format is 'uncompressed'
    %
    %    "ChunkSize"     -  Size of each message chunk, specified in bytes 
    %                       as a non-zero positive integer. The default chunk 
    %                       size is 786432.
    %
    %
    %   rosbagwriter properties:
    %      FilePath       - (Read-only) Path to the bag file
    %      StartTime      - (Read-only) Earliest timestamp of the messages 
    %                                   written into the bag file using this
    %                                   object
    %      EndTime        - (Read-only) Latest timestamp of the messages 
    %                                   written into the bag file using this
    %                                   object
    %      NumMessages    - (Read-only) Number of messages written into the bag file
    %      Compression    - (Read-Only) Type of compression format for writing
    %                                   messages into the bag file
    %      ChunkSize      - (Read-only) Chunk size in bytes for writing 
    %                                   messages into the bag file
    %      FileSize       - (Read-only) Current size of the bagfile
    %
    %   rosbagwriter methods:
    %      write - Write messages to bag file with corresponding topic names 
    %      and timestamps
    %
    %      delete - Close the opened rosbag file and remove the rosbagwriter
    %      object from memory
    %
    %   Example:
    %       %Create a rosbagwriter object and a bag file with given
    %       %name and path.
    %       bagWriter = ROSBAGWRITER("bagfiles/mybagfile.bag")
    %
    %       %Enter a folder path to create a bag file with current
    %       %timestamp as file name.
    %       bagWriter = ROSBAGWRITER("bagfiles/mybags")
    %
    %       %Create a rosbagwriter object with Name-Value pairs.
    %       bagWriter = ROSBAGWRITER("bagfiles/mybagfile.bag", ...
    %                   "Compression","lz4", ...
    %                   "ChunkSize",1500);
    %
    %       %Writing a single record in bag file:
    %       timeStamp = rostime("now");
    %       rosMessage = rosmessage("nav_msgs/Odometry");
    %       write(bagWriter,"/odom",timeStamp,rosMessage);
    %
    %       timeStamp = rostime("now","DataFormat","struct");
    %       rosMessage = rosmessage("nav_msgs/Odometry","DataFormat","struct");
    %       write(bagWriter, "/odom", timeStamp, rosMessage);
    %
    %       timeStamp = 1.6140e+09;
    %       rosMessage = rosmessage('nav_msgs/Odometry');
    %       write(bagWriter,"/odom",timeStamp,rosMessage);
    %
    %
    %       %Writing multiple records in bag file:
    %       timeStamp = rostime("now");
    %       rosMessage1 = rosmessage("nav_msgs/Odometry");
    %       rosMessage2 = rosmessage("geometry_msgs/Twist");
    %       rosMessage3 = rosmessage("sensor_msgs/Image");
    %
    %       write(bagWriter, ...
    %               ["/odom","cmd_vel","/camera/rgb/image_raw"], ...
    %               {timeStamp,timeStamp + 1,timeStamp + 2}, ...
    %               {rosMessage1,rosMessage2,rosMessage3});
    %
    %       %Writing multiple records of same topic in bag file:
    %       pointMsg1 = rosmessage("geometry_msgs/Point");
    %       pointMsg1.X = 1;
    %       pointMsg2 = rosmessage("geometry_msgs/Point");
    %       pointMsg2.X = 2;
    %       pointMsg3 = rosmessage("geometry_msgs/Point");
    %       pointMsg3.X = 3;
    %       write(bagWriter, ...
    %               "/point", ...
    %               {timeStamp,timeStamp + 1,timeStamp + 2}, ...
    %               {pointMsg1,pointMsg2,pointMsg3});
    %
    %       %Close the bagfile and remove the rosbagwriter object from memory.
    %       delete(bagWriter);
    %
    %   See also rosbagreader, rosbag.

    %   Copyright 2021 The MathWorks, Inc.

    properties (Dependent, SetAccess = private)
        %FilePath - Absolute path to rosbag file
        FilePath
        
        %StartTime - Earliest timestamp of the messages written to the bag
        %file through this rosbagwriter.
        %   The time is expressed in seconds.
        StartTime

        %EndTime - Mose recent timestamp of the messages written to the bag
        %file through this rosbagwriter.
        %   The time is expressed in seconds.
        EndTime

        %NumMessages - Number of messages written to the bag file
        NumMessages

        %Compression - Compression type to be used while writing to the bag
        %file.
        Compression

        %ChunkSize - Size of ChunkSize in bytes.
        ChunkSize

        %FileSize - Current bag file size in bytes.
        FileSize
    end
    properties (Hidden)
        ChunkSizeUnits = 'Bytes';
        FileSizeUnits = 'Bytes';
    end
    properties (Constant, Access = private)
        %CompressionValues - Possible values for Compression property
        CompressionValues = {'uncompressed', 'bz2', 'lz4'}
    end

    properties (Transient, Access = ?matlab.unittest.TestCase)
        %InternalBagWriter - MCOS C++ object for reading from rosbag
        InternalBagWriter
    end

    methods
        function obj = rosbagwriter(bagPath, varargin)
            %ROSBAGWRITER Constructor for rosbagwriter class
            %   BAGWRITER = ROSBAGWRITER(bagPath) Creates a .bag file and
            %   returns a new rosbagwriter object which is used to write the
            %   bag file.
            %
            %   If the input path is not available, it creates the path by
            %   creating the folders and sub folders.
            %
            %   If the input is a folder-path, it creates a bag file with
            %   file-name same as current timestamp. If the input is a
            %   file path, it creates the bag file with given name.
            %
            %   Please see the class documentation (help rosbagwriter)
            %   for more details.

            try
                narginchk(1, inf);

                % Convert all string arguments to characters
                [bagPath, varargin{:}] = convertStringsToChars(bagPath, varargin{:});

                % Parse the input parameters.
                paramParser = getParsers(obj);
                parse(paramParser, bagPath, varargin{:});
                
                [folderPath, fileName, ext] = fileparts(bagPath);

                if isempty(ext)
                    if ~isfolder(bagPath)
                        % Input is a folder which does not exist. So create the
                        % folder.
                        [status, msg, ~] = mkdir(bagPath);
                        if ~status
                            error(msg)
                        end
                    end
                    % Create a bag file with name as current timestamp.
                    fileName = ['bag_' datestr(now,'dd_mmm_yyyy_HH_MM_SS_FFF') '.bag'];
                    filePath = fullfile(bagPath,fileName);
                elseif isfolder(bagPath)
                    % If input is an existing folder which contains a '.', create 
                    % a bag file with name as current timestamp.
                    
                    fileName = ['bag_' datestr(now,'dd_mmm_yyyy_HH_MM_SS_FFF') '.bag'];
                    filePath = fullfile(bagPath,fileName);
                elseif strcmpi(ext,'.bag') && ~isempty(fileName)
                    % Input is a file path ending with .bag

                    if ~isfolder(folderPath)
                        % The folder path of the bag file does not exist.
                        % So create the folder first as per the input.
                        [status, errorMsg, errorMsgId] = mkdir(folderPath);
                        if ~status
                            error(errorMsgId, errorMsg);
                        end
                    end

                    % Set the file name as given by user.
                    filePath = bagPath;
                else
                    error(message('ros:mlros:bag:RosbagWriterInvalidFileError'))
                end

                %Set env-variables for rosbag.
                cleanPath = ros.internal.setupRosEnv(); %#ok<NASGU>

                %Create the cpp bag-writer object and open a bag file in
                %write mode by passing int32(1). For append mode int32(2)
                %should be passed but currently we don't support that.
                obj.InternalBagWriter = roscpp.bag.internal.RosbagWriterWrapper(filePath, int32(1));
            catch ex
                newEx = MException(message('ros:mlros:bag:RosbagWriterCreationError'));
                
                if isequal(ex.identifier,'ros:mlros:bag:BagFileOpeningError') && isfile(bagPath)
                    causedEx = MException(message('ros:mlros:bag:BagFileAlreadyOpenError',bagPath));
                    throw(newEx.addCause(causedEx));
                else
                    throw(newEx.addCause(ex));
                end
            end
            if ~isempty(paramParser.Results.ChunkSize)
                setChunkThreshold(obj.InternalBagWriter, ...
                    uint32(paramParser.Results.ChunkSize));
            end

            if ~isempty(paramParser.Results.Compression)
                compressionVal = char(paramParser.Results.Compression);
                compressionIdx = find(strncmpi(compressionVal, ...
                    obj.CompressionValues, ...
                    numel(compressionVal)), 1);
                setCompression(obj.InternalBagWriter,uint32(compressionIdx)-1);
            end

            function paramParser = getParsers(obj)
                % Set up separate parsers for parameters and other input


                paramParser = inputParser;

                addRequired(paramParser, 'bagPath', ...
                    @(x) validateattributes(x, ...
                    {'char', 'string'}, ...
                    {'scalartext', 'nonempty'}, ...
                    'rosbagwriter', ...
                    'bagPath'));


                addParameter(paramParser, 'ChunkSize', [], ...
                    @(x) validateattributes(x, ...
                    {'numeric'}, ...
                    {'scalar', 'finite', 'positive', 'integer'}, ...
                    'rosbagwriter', ...
                    'ChunkSize'));

                addParameter(paramParser, 'Compression', '', ...
                    @(x) ~isempty(validatestring(x, ...
                    obj.CompressionValues, ...
                    'rosbagwriter', ...
                    'Compression')));
            end
        end

        function delete(obj)
            %DELETE Close rosbag file and remove rosbagwriter object from memory
            %   delete(BAGWRITER) - removes the rosbagwriter object from 
            %   memory. The function closes the opened rosbag file before 
            %   deleting the object. 
            % 
            %   If multiple references to the rosbagwriter object exist in 
            %   the workspace, deleting the rosbagwriter object invalidates 
            %   the remaining reference. Use the clear command to delete the 
            %   remaining references to the object from the workspace.

            obj.InternalBagWriter = [];
        end

        function write(obj, topic, timeStamp, rosMessage)
            %WRITE Write messages to bag file with corresponding topic names and timestamps
            %  write(BAGWRITER,TOPIC,TIMESTAMP,MESSAGE) - writes a single record (a topic, 
            %  its timestamp, and the ROS message) or multiple records (a list 
            %  of topics, corresponding list of timestamps, and the list of ROS 
            %  messages) into the bag file.
            %
            %   Writing a single record in bag file:
            %   topic - String / char-vector
            %   timeStamp - rostime object / rostime struct / double
            %   rosMessage - ros-message class / ros-message struct
            %
            %    Example:
            %       timeStamp = rostime("now");
            %       rosMessage = rosmessage("nav_msgs/Odometry");
            %       write(bagWriter, "/odom", timeStamp, rosMessage);
            %
            %       timeStamp = rostime("now","DataFormat","struct");
            %       rosMessage = rosmessage("nav_msgs/Odometry","DataFormat","struct");
            %       write(bagWriter, "/odom", timeStamp, rosMessage);
            %
            %       timeStamp = 1.6140e+09;
            %       rosMessage = rosmessage("nav_msgs/Odometry");
            %       write(bagWriter,"/odom",timeStamp,rosMessage);
            %
            %
            %   Writing multiple records in bag file:
            %   topic - Array of String / Cell array of char-vector
            %   timeStamp - Cell array of rostime object / rostime struct / double
            %   rosMessage - ros-message class / ros-message struct
            %
            %    Example:
            %       write(bagWriter, ...
            %               {"/odom","cmd_vel","/camera/rgb/image_raw"}, ...
            %               {timeStamp1,timeStamp2,timeStamp3}, ...
            %               {rosMessage1,rosMessage2,rosMessage3});
            %
            %
            %   Writing multiple records of same topic in bag file:
            %   topic - String / char-vector
            %   timeStamp - Cell array of rostime object / rostime struct / double
            %   rosMessage - ros-message class / ros-message struct
            %
            %    Example:
            %       write(bagWriter, ...
            %               "/odom", ...
            %               {timeStamp1,timeStamp2,timeStamp3}, ...
            %               {rosMessage1,rosMessage2,rosMessage3});

            if ~isscalar(rosMessage)

                validateattributes(topic, {'string', 'cell', 'char'}, ...
                    {'vector', 'nonempty'}, 'write', 'topic');
                validateattributes(rosMessage, {'cell', 'struct', 'ros.Message'}, ...
                    {'vector', 'nonempty'}, 'write', 'rosMessage');
                validateattributes(timeStamp, {'cell', 'double', 'struct', 'ros.msg.Time'}, ...
                    {'vector', 'nonempty'}, 'write', 'timeStamp');

                if(ischar(topic) || isequal(length(topic) , 1))
                    if iscell(topic)
                        topic = topic{1};
                    end
                    topic = repmat({topic},1,length(rosMessage));
                elseif ~isequal(length(topic), length(rosMessage))
                    error(message('ros:mlros:bag:TopicListError'))
                end

                if ~isequal(length(timeStamp), length(rosMessage))
                    error(message('ros:mlros:bag:TimeStampListError'))
                end

                for ii = 1:length(rosMessage)
                    %Get the time stamp
                    if iscell(timeStamp)
                        ts = timeStamp{ii};
                    else
                        ts = timeStamp(ii);
                    end
                    %Get the ros message
                    if iscell(rosMessage)
                        rosMsg = rosMessage{ii};
                    else
                        rosMsg = rosMessage(ii);
                    end
                    %Send the current record to write
                    write(obj, topic{ii}, ts, rosMsg);
                end
            else
                try
                    msgType = rosMessage.MessageType;
                    if ~ifMessageTypeRegistered(obj.InternalBagWriter, msgType)
                        registerMessageType(obj, msgType);
                    end
                    if ~isa(rosMessage,'struct')
                        rosMessage = toROSStruct(rosMessage);
                    end
                    if isa(timeStamp,'double')
                        timeStamp = rostime(timeStamp);
                    end

                    write(obj.InternalBagWriter,...
                        msgType, topic, ...
                        int32(timeStamp.Sec), int32(timeStamp.Nsec), rosMessage);

                catch ex
                    validateattributes(topic, {'string','char'}, ...
                        {'scalartext'}, 'write', 'topic');
                    validateattributes(rosMessage, {'struct','ros.Message'}, ...
                        {'scalar'}, 'write', 'rosMessage');
                    validateattributes(timeStamp, {'struct','double','ros.msg.Time'}, ...
                        {'scalar'}, 'write', 'timeStamp');

                    newEx = MException(message('ros:mlros:bag:RosbagWriteError', ...
                        topic, msgType));
                    throw(newEx.addCause(ex));
                end
            end
        end

        function filePath = get.FilePath(obj)
            % Gets the path of the bag file.
            filePath = getBagFilePath(obj.InternalBagWriter);
        end
        
        function numMessages = get.NumMessages(obj)
            %Gets number-of-messages written to the bag-file.
            numMessages = getNumMessages(obj.InternalBagWriter);
        end

        function startTime = get.StartTime(obj)
            %Sets end-time of all the messages written to the bag file.
            timeStr = getStartTime(obj.InternalBagWriter);
            startTime = rostime(timeStr.Sec, timeStr.Nsec);
            startTime = startTime.seconds();
        end

        function endTime = get.EndTime(obj)
            %Gets end-time of all the messages written to the bag file.
            timeStr = getEndTime(obj.InternalBagWriter);
            endTime = rostime(timeStr.Sec, timeStr.Nsec);
            endTime = endTime.seconds();
        end

        function compression = get.Compression(obj)
            %Gets the compression type.
            compression = obj.CompressionValues{getCompression(obj.InternalBagWriter)+1};
        end

        function chunkSize = get.ChunkSize(obj)
            % Gets the ChunkSize value in bytes.
            chunkSize = getChunkThreshold(obj.InternalBagWriter);
        end

        function fileSize = get.FileSize(obj)
            % Gets the current bag-file size.
            fileSize = getSize(obj.InternalBagWriter);
        end
    end
    methods (Access = ?matlab.unittest.TestCase)
        function registerMessageType(obj, rosMessageType)
            %registerMessageType This is an internal method used to
            % register and load all dependent libraries of a message type.
            % It is called only once when ever a message type is going to be
            % written first time in a bag file.

            cleanPath = ros.internal.setupRosEnv(); %#ok<NASGU>
            messageInfo = ros.internal.ros.getMessageInfo(rosMessageType);
            [cppFactoryClass , cppElementType] = ...
                ros.internal.ros.getCPPFactoryClassAndType(rosMessageType);
            dllPaths = ros.internal.utilities.getPathOfDependentDlls(rosMessageType,'ros');
            dllPaths{end + 1} = messageInfo.path;

            registerMessageType(obj.InternalBagWriter, ...
                rosMessageType, cppFactoryClass, cppElementType, dllPaths)
        end
    end
    methods (Access = protected)
        function displayScalarObject(obj)
            displayScalarObjectWithUnits(obj);
        end
    end
end

% LocalWords:  bagfile bagfiles mybagfile bagwriter vel mmm yyyy HH FFF

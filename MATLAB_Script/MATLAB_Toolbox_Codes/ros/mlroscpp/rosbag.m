function output = rosbag(operation, filePath)
%ROSBAG Open a rosbag log file and get content information
%   BAGSELECT = ROSBAG('FILEPATH') creates an indexable ros.BagSelection
%   object, BAGSELECT, that contains all the messages from the rosbag
%   located at path FILEPATH.
%
%   BAGINFO = ROSBAG('info', 'FILEPATH') returns information about the
%   contents of the rosbag FILEPATH as a structure. The information
%   includes the number of messages, start/end time, topics, and message
%   types. Simplified form: ROSBAG info FILEPATH
%
%   A rosbag or bag is a file format for storing ROS message data.
%   Their primary use is in logging of messages within the ROS network.
%   The resulting bag can be used for off-line analysis, visualization,
%   and storage. MATLAB provides functionality for reading existing
%   rosbags.
%
%
%   Example:
%      % Get information about rosbag
%      ROSBAG info ex_multiple_topics.bag
%
%      % Open a rosbag and retrieve information about its contents
%      bagMsgs = ROSBAG('ex_multiple_topics.bag')
%
%      % Select a subset of the messages by time and topic
%      bagMsgs2 = select(bagMsgs, 'Time', ...
%          [bagMsgs.StartTime bagMsgs.StartTime + 1], 'Topic', '/odom')
%
%   See also rosbagreader, rosbagwriter.

%   Copyright 2014-2021 The MathWorks, Inc.

    narginchk(1,2)

    try
        % Parse the specified operation and additional arguments
        if nargin == 1
            % Syntax: rosbag('FILEPATH')
            filePath = operation;
            filePath = convertStringsToChars(filePath);
            bagSelect = ros.Bag.parse(filePath);
            output = bagSelect;
            return;
        else
            % Syntax: rosbag info FILEPATH
            supportedOperations = {'info'};
            validOperation = validatestring(operation, supportedOperations, 'rosbag', 'operation');

            % Find bag file anywhere on the MATLAB path
            filePath = convertStringsToChars(filePath);
            absFilePath = robotics.internal.validation.findFilePath(filePath);

            if nargout == 0
                rosbagImpl(validOperation, absFilePath);
            else
                output = rosbagImpl(validOperation, absFilePath);
            end
        end

    catch ex
        % Save stack traces and exception causes internally, but do not
        % print them to the console
        rosex = ros.internal.ROSException.fromException(ex);
        throwAsCaller(rosex);
    end

end

function output = rosbagImpl(operation, param)
%rosbagImpl Actual implementation of rosbag functionality.
%   Note that the validation of the input parameters has already been
%   performed in the calling function

    switch operation
      case 'info'
        % Load rosbag
        filePath = param;
        %setupPaths for rosbag
        cleanPath = ros.internal.setupRosEnv(); %#ok<NASGU>
        bag = roscpp.bag.internal.RosbagWrapper(filePath);

        if nargout == 1
            % If output argument specified, return
            output = bag.infoStruct;
        else
            % Otherwise, print on console
            disp(bag.info);
        end
    end
end

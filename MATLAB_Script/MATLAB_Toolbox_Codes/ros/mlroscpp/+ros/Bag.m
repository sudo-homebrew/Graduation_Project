classdef Bag
    %Bag This class represents an interface for reading a rosbag
    %   A rosbag or bag is a file format in ROS for storing ROS message data.
    %   These bags are often created by subscribing to one or more ROS
    %   topics, and storing the received message data in an efficient file
    %   structure.
    %   Their primary use is in logging of messages within the ROS network.
    %   The resulting bag can be used for off-line analysis, visualization,
    %   and storage.
    %
    %   This class only supports the reading of rosbags in the 2.0 format
    %   Only uncompressed rosbags are supported.
    %
    %   BAGSELECT = ros.Bag.parse('FILEPATH') parses the rosbag
    %   located at path FILEPATH and returns a ros.BagSelection
    %   object BAGSELECT representing an index of all the messages
    %   that are stored in the rosbag.
    %
    %
    %   Bag methods:
    %      parse - (Static) Parse the contents of a rosbag file
    %
    %
    %   Example:
    %
    %      % Open a rosbag and retrieve information about its contents
    %      filePath = 'path/to/logfile.bag';
    %      bagSel = ros.Bag.parse(filePath)
    %
    %      % Select a subset of the messages by time and topic
    %      bagSel2 = select(bagSel, select(bagSelect, 'Time', ...
    %          [bag.StartTime bag.StartTime + 1], 'Topic', '/odom')
    %
    %   See also ROSBAG.
    
    %   Copyright 2014-2020 The MathWorks, Inc.
                
    methods (Static)
        function bagSelect = parse(filePath)
            %parse Parse a rosbag and return a selection object
            %   BAGSELECT = ros.Bag.parse('FILEPATH') parses the rosbag
            %   located at path FILEPATH and returns a ros.BagSelection
            %   object BAGSELECT representing an index of all the messages
            %   that are stored in the rosbag.
            
            % Find bag file anywhere on the MATLAB path
            absFilePath = robotics.internal.validation.findFilePath(filePath);
            
            % Parse the given rosbag
            bagParser = ros.bag.internal.BagParser(absFilePath);
            
            % Create a bag selection and return it to the user
            bagSelect = ros.BagSelection(absFilePath, bagParser.Bag, ...
                bagParser.MessageList, bagParser.TopicTypeMap, ...
                bagParser.TopicDefinitionMap);
        end
    end
    
    methods (Access = private)
        function obj = Bag
            %Bag Constructor for ros.Bag class
            %   It has only private access to restrict the explicit creation
            %   of a ros.Bag object.
            %   This class should only be used through its static
            %   member functions.
        end
    end
end

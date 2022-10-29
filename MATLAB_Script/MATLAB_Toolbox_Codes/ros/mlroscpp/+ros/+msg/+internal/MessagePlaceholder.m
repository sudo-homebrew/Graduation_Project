classdef (Sealed, ConstructOnLoad, HandleCompatible) MessagePlaceholder < ...
        ros.Message
    %MessagePlaceholder Default message object.
    %   The MessagePlaceholder class defines the default ROS message object.
    %   Instances of this class are used to automatically initialize arrays
    %   of ROS messages. Replace these instances with valid ROS message
    %   objects where needed.
    %
    %   Example:
    %
    %      % Create a message array with 2 elements
    %      % The second element in the array will contain the Header ROS message
    %      m(2) = rosmessage("std_msgs/Header")
    %
    %      % The first array element will be a MessagePlaceholder object
    %      m(1)
    %
    %      % Replace the placeholder with a proper message
    %      m(1) = rosmessage("sensor_msgs/LaserScan")
    %
    %      % The recommended way for initializing an array of ROS messages
    %      % is to explicitly assign every element
    %      for i = 1:10
    %         msg(i) = rosmessage("std_msgs/Header");
    %      end
    %
    %   This class is for internal use only. It may be removed in the future.
    %
    %   See also ros.Message.

    %   Copyright 2015-2020 The MathWorks, Inc.

    properties(Constant)
        %MessageType - The ROS message type of this object
        MessageType = ''
    end

    properties(Hidden, Constant)
       %MD5Checksum - Recursive message definition checksum
        MD5Checksum = ''

        %PropertyList - List of non-constant message properties
        PropertyList = {}

        %ROSPropertyList - List of non-constant ROS message properties
        ROSPropertyList = {}

        %PropertyMessageTypes - Types of contained nested messages
        PropertyMessageTypes = {}
    end

    methods
        function obj = MessagePlaceholder
        % Construct the message placeholder with no ROS data properties

            obj = obj@ros.Message(struct);
        end
    end

    methods (Access = ?ros.Message)
        function strObj = saveobj(~)
        %saveobj Implements saving of message to MAT file
        %   Added method to ensure that showdetails works.

            strObj = struct.empty;
        end
    end
end

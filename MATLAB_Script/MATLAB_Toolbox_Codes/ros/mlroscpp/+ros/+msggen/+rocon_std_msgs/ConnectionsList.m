
classdef ConnectionsList < ros.Message
    %ConnectionsList MATLAB implementation of rocon_std_msgs/ConnectionsList
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'rocon_std_msgs/ConnectionsList' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '672d6ad69b684884f8fb6f4acedbd39f' % The MD5 Checksum of the message definition
        PropertyList = { 'Connections' } % List of non-constant message properties
        ROSPropertyList = { 'connections' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.rocon_std_msgs.Connection' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Connections
    end
    methods
        function set.Connections(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.rocon_std_msgs.Connection.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.rocon_std_msgs.Connection'};
            validateattributes(val, validClasses, validAttributes, 'ConnectionsList', 'Connections')
            obj.Connections = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.rocon_std_msgs.ConnectionsList.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.rocon_std_msgs.ConnectionsList(strObj);
        end
    end
end
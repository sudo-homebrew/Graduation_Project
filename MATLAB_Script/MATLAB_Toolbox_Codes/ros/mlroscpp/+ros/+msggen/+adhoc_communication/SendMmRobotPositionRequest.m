
classdef SendMmRobotPositionRequest < ros.Message
    %SendMmRobotPositionRequest MATLAB implementation of adhoc_communication/SendMmRobotPositionRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'adhoc_communication/SendMmRobotPositionRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '58e53492a8835f0741305155e91765cb' % The MD5 Checksum of the message definition
        PropertyList = { 'Position' 'DstRobot' 'Topic' } % List of non-constant message properties
        ROSPropertyList = { 'position' 'dst_robot' 'topic' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.adhoc_communication.MmRobotPosition' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Position
        DstRobot
        Topic
    end
    methods
        function set.Position(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.adhoc_communication.MmRobotPosition'};
            validateattributes(val, validClasses, validAttributes, 'SendMmRobotPositionRequest', 'Position')
            obj.Position = val;
        end
        function set.DstRobot(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'SendMmRobotPositionRequest', 'DstRobot');
            obj.DstRobot = char(val);
        end
        function set.Topic(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'SendMmRobotPositionRequest', 'Topic');
            obj.Topic = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.adhoc_communication.SendMmRobotPositionRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.adhoc_communication.SendMmRobotPositionRequest(strObj);
        end
    end
end

classdef SendMmControlRequest < ros.Message
    %SendMmControlRequest MATLAB implementation of adhoc_communication/SendMmControlRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'adhoc_communication/SendMmControlRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'ea7da37c058953e8933f11e2df156167' % The MD5 Checksum of the message definition
        PropertyList = { 'Msg' 'DstRobot' 'Topic' } % List of non-constant message properties
        ROSPropertyList = { 'msg' 'dst_robot' 'topic' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.adhoc_communication.MmControl' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Msg
        DstRobot
        Topic
    end
    methods
        function set.Msg(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.adhoc_communication.MmControl'};
            validateattributes(val, validClasses, validAttributes, 'SendMmControlRequest', 'Msg')
            obj.Msg = val;
        end
        function set.DstRobot(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'SendMmControlRequest', 'DstRobot');
            obj.DstRobot = char(val);
        end
        function set.Topic(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'SendMmControlRequest', 'Topic');
            obj.Topic = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.adhoc_communication.SendMmControlRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.adhoc_communication.SendMmControlRequest(strObj);
        end
    end
end
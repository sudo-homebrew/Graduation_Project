
classdef RfidTagVector < ros.Message
    %RfidTagVector MATLAB implementation of stdr_msgs/RfidTagVector
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'stdr_msgs/RfidTagVector' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'd1ccd79235f17c9d8c9665681cfa66f7' % The MD5 Checksum of the message definition
        PropertyList = { 'RfidTags' } % List of non-constant message properties
        ROSPropertyList = { 'rfid_tags' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.stdr_msgs.RfidTag' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        RfidTags
    end
    methods
        function set.RfidTags(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.stdr_msgs.RfidTag.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.stdr_msgs.RfidTag'};
            validateattributes(val, validClasses, validAttributes, 'RfidTagVector', 'RfidTags')
            obj.RfidTags = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.stdr_msgs.RfidTagVector.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.stdr_msgs.RfidTagVector(strObj);
        end
    end
end

classdef COMPortMessages < ros.Message
    %COMPortMessages MATLAB implementation of applanix_msgs/COMPortMessages
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'applanix_msgs/COMPortMessages' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '88218ad4124322a799282c496fb3a33f' % The MD5 Checksum of the message definition
        PropertyList = { 'PortNum' 'Messages' 'UpdateRate' } % List of non-constant message properties
        ROSPropertyList = { 'port_num' 'messages' 'update_rate' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
        MESSAGESNMEAGST = uint32(1)
        MESSAGESNMEAGGA = uint32(2)
        MESSAGESNMEAHDT = uint32(4)
        MESSAGESNMEAZDA = uint32(8)
        MESSAGESNMEAEVT1 = uint32(16)
        MESSAGESNMEAEVT2 = uint32(32)
        MESSAGESNMEAVTG = uint32(64)
        MESSAGESNMEAPASHR = uint32(128)
        MESSAGESNMEAGGA2 = uint32(8192)
        MESSAGESNMEAPPS = uint32(16384)
        MESSAGESNMEAGGK = uint32(32768)
        MESSAGESNMEARMC = uint32(65536)
        MESSAGESBINGIMBALLOOP = uint32(1)
        MESSAGESBINRDR1 = uint32(2)
        MESSAGESBINPAST2 = uint32(4)
        MESSAGESBINPPS = uint32(65536)
        MESSAGESBINTM1B = uint32(131072)
    end
    properties
        PortNum
        Messages
        UpdateRate
    end
    methods
        function set.PortNum(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'COMPortMessages', 'PortNum');
            obj.PortNum = uint8(val);
        end
        function set.Messages(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'COMPortMessages', 'Messages');
            obj.Messages = uint32(val);
        end
        function set.UpdateRate(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'COMPortMessages', 'UpdateRate');
            obj.UpdateRate = uint8(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.applanix_msgs.COMPortMessages.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.applanix_msgs.COMPortMessages(strObj);
        end
    end
end

classdef ESCStatus < ros.Message
    %ESCStatus MATLAB implementation of mavros_msgs/ESCStatus
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'mavros_msgs/ESCStatus' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '145d49eaf5cfecbdfd50ae4a22fe82d4' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'EscStatus' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'esc_status' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.mavros_msgs.ESCStatusItem' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        EscStatus
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'ESCStatus', 'Header')
            obj.Header = val;
        end
        function set.EscStatus(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.mavros_msgs.ESCStatusItem.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.mavros_msgs.ESCStatusItem'};
            validateattributes(val, validClasses, validAttributes, 'ESCStatus', 'EscStatus')
            obj.EscStatus = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.mavros_msgs.ESCStatus.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.mavros_msgs.ESCStatus(strObj);
        end
    end
end

classdef RMPBatteryStatus < ros.Message
    %RMPBatteryStatus MATLAB implementation of rmp_msgs/RMPBatteryStatus
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'rmp_msgs/RMPBatteryStatus' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '74c210c462f2339426b30d67c39a4c4a' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'SocItems' 'SocValues' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'soc_items' 'soc_values' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        SocItems
        SocValues
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'RMPBatteryStatus', 'Header')
            obj.Header = val;
        end
        function set.SocItems(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'cell', 'string'};
            if isempty(val)
                % Allow empty [] input
                val = cell.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'RMPBatteryStatus', 'SocItems');
            obj.SocItems = cell(val);
        end
        function set.SocValues(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = single.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'RMPBatteryStatus', 'SocValues');
            obj.SocValues = single(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.rmp_msgs.RMPBatteryStatus.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.rmp_msgs.RMPBatteryStatus(strObj);
        end
    end
end
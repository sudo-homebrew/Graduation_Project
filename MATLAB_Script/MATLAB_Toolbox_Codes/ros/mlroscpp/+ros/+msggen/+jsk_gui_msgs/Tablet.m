
classdef Tablet < ros.Message
    %Tablet MATLAB implementation of jsk_gui_msgs/Tablet
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'jsk_gui_msgs/Tablet' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '0bab196c7b214826d8c27d7bd5f924f6' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Action' 'Sensor' 'Touches' 'HardwareName' 'HardwareId' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'action' 'sensor' 'touches' 'hardware_name' 'hardware_id' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.jsk_gui_msgs.Action' ...
            'ros.msggen.jsk_gui_msgs.DeviceSensor' ...
            'ros.msggen.jsk_gui_msgs.Touch' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Action
        Sensor
        Touches
        HardwareName
        HardwareId
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'Tablet', 'Header')
            obj.Header = val;
        end
        function set.Action(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.jsk_gui_msgs.Action'};
            validateattributes(val, validClasses, validAttributes, 'Tablet', 'Action')
            obj.Action = val;
        end
        function set.Sensor(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.jsk_gui_msgs.DeviceSensor'};
            validateattributes(val, validClasses, validAttributes, 'Tablet', 'Sensor')
            obj.Sensor = val;
        end
        function set.Touches(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.jsk_gui_msgs.Touch.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.jsk_gui_msgs.Touch'};
            validateattributes(val, validClasses, validAttributes, 'Tablet', 'Touches')
            obj.Touches = val;
        end
        function set.HardwareName(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'Tablet', 'HardwareName');
            obj.HardwareName = char(val);
        end
        function set.HardwareId(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'Tablet', 'HardwareId');
            obj.HardwareId = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.jsk_gui_msgs.Tablet.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.jsk_gui_msgs.Tablet(strObj);
        end
    end
end
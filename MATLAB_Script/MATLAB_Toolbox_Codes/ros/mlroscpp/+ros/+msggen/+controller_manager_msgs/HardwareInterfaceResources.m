
classdef HardwareInterfaceResources < ros.Message
    %HardwareInterfaceResources MATLAB implementation of controller_manager_msgs/HardwareInterfaceResources
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'controller_manager_msgs/HardwareInterfaceResources' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'f25b55cbf1d1f76e82e5ec9e83f76258' % The MD5 Checksum of the message definition
        PropertyList = { 'HardwareInterface' 'Resources' } % List of non-constant message properties
        ROSPropertyList = { 'hardware_interface' 'resources' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        HardwareInterface
        Resources
    end
    methods
        function set.HardwareInterface(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'HardwareInterfaceResources', 'HardwareInterface');
            obj.HardwareInterface = char(val);
        end
        function set.Resources(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'cell', 'string'};
            if isempty(val)
                % Allow empty [] input
                val = cell.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'HardwareInterfaceResources', 'Resources');
            obj.Resources = cell(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.controller_manager_msgs.HardwareInterfaceResources.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.controller_manager_msgs.HardwareInterfaceResources(strObj);
        end
    end
end

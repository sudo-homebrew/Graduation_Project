
classdef Supply < ros.Message
    %Supply MATLAB implementation of hector_uav_msgs/Supply
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'hector_uav_msgs/Supply' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '26f5225a2b836fba706a87e45759fdfc' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Voltage' 'Current' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'voltage' 'current' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Voltage
        Current
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'Supply', 'Header')
            obj.Header = val;
        end
        function set.Voltage(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = single.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'Supply', 'Voltage');
            obj.Voltage = single(val);
        end
        function set.Current(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = single.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'Supply', 'Current');
            obj.Current = single(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.hector_uav_msgs.Supply.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.hector_uav_msgs.Supply(strObj);
        end
    end
end
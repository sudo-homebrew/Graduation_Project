
classdef Scan < ros.Message
    %Scan MATLAB implementation of wireless_msgs/Scan
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'wireless_msgs/Scan' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'd32911a086af571ae6871a223a6112f0' % The MD5 Checksum of the message definition
        PropertyList = { 'Networks' } % List of non-constant message properties
        ROSPropertyList = { 'networks' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.wireless_msgs.Network' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Networks
    end
    methods
        function set.Networks(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.wireless_msgs.Network.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.wireless_msgs.Network'};
            validateattributes(val, validClasses, validAttributes, 'Scan', 'Networks')
            obj.Networks = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.wireless_msgs.Scan.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.wireless_msgs.Scan(strObj);
        end
    end
end


classdef VelodynePacket < ros.Message
    %VelodynePacket MATLAB implementation of velodyne_msgs/VelodynePacket
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'velodyne_msgs/VelodynePacket' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'ae4f90a23256f44e82baa08dd45c3456' % The MD5 Checksum of the message definition
        PropertyList = { 'Stamp' 'Data' } % List of non-constant message properties
        ROSPropertyList = { 'stamp' 'data' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msg.Time' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Stamp
        Data
    end
    methods
        function set.Stamp(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.Time'};
            validateattributes(val, validClasses, validAttributes, 'VelodynePacket', 'Stamp')
            obj.Stamp = val;
        end
        function set.Data(obj, val)
            validClasses = {'numeric'};
            val = val(:);
            validAttributes = {'vector', 'numel', 1206};
            validateattributes(val, validClasses, validAttributes, 'VelodynePacket', 'Data');
            obj.Data = uint8(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.velodyne_msgs.VelodynePacket.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.velodyne_msgs.VelodynePacket(strObj);
        end
    end
end
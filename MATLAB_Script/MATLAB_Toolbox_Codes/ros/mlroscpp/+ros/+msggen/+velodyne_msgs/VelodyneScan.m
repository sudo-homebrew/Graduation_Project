
classdef VelodyneScan < ros.Message
    %VelodyneScan MATLAB implementation of velodyne_msgs/VelodyneScan
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'velodyne_msgs/VelodyneScan' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '50804fc9533a0e579e6322c04ae70566' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Packets' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'packets' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.velodyne_msgs.VelodynePacket' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Packets
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'VelodyneScan', 'Header')
            obj.Header = val;
        end
        function set.Packets(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.velodyne_msgs.VelodynePacket.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.velodyne_msgs.VelodynePacket'};
            validateattributes(val, validClasses, validAttributes, 'VelodyneScan', 'Packets')
            obj.Packets = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.velodyne_msgs.VelodyneScan.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.velodyne_msgs.VelodyneScan(strObj);
        end
    end
end

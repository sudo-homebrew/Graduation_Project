
classdef GetPointMapROIResponse < ros.Message
    %GetPointMapROIResponse MATLAB implementation of map_msgs/GetPointMapROIResponse
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'map_msgs/GetPointMapROIResponse' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '313769f8b0e724525c6463336cbccd63' % The MD5 Checksum of the message definition
        PropertyList = { 'SubMap' } % List of non-constant message properties
        ROSPropertyList = { 'sub_map' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msg.sensor_msgs.PointCloud2' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        SubMap
    end
    methods
        function set.SubMap(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.sensor_msgs.PointCloud2'};
            validateattributes(val, validClasses, validAttributes, 'GetPointMapROIResponse', 'SubMap')
            obj.SubMap = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.map_msgs.GetPointMapROIResponse.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.map_msgs.GetPointMapROIResponse(strObj);
        end
    end
end

classdef ObjectInformation < ros.Message
    %ObjectInformation MATLAB implementation of object_recognition_msgs/ObjectInformation
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'object_recognition_msgs/ObjectInformation' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '921ec39f51c7b927902059cf3300ecde' % The MD5 Checksum of the message definition
        PropertyList = { 'GroundTruthMesh' 'GroundTruthPointCloud' 'Name' } % List of non-constant message properties
        ROSPropertyList = { 'ground_truth_mesh' 'ground_truth_point_cloud' 'name' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.shape_msgs.Mesh' ...
            'ros.msg.sensor_msgs.PointCloud2' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        GroundTruthMesh
        GroundTruthPointCloud
        Name
    end
    methods
        function set.GroundTruthMesh(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.shape_msgs.Mesh'};
            validateattributes(val, validClasses, validAttributes, 'ObjectInformation', 'GroundTruthMesh')
            obj.GroundTruthMesh = val;
        end
        function set.GroundTruthPointCloud(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.sensor_msgs.PointCloud2'};
            validateattributes(val, validClasses, validAttributes, 'ObjectInformation', 'GroundTruthPointCloud')
            obj.GroundTruthPointCloud = val;
        end
        function set.Name(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'ObjectInformation', 'Name');
            obj.Name = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.object_recognition_msgs.ObjectInformation.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.object_recognition_msgs.ObjectInformation(strObj);
        end
    end
end
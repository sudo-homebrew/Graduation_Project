
classdef GraspableObjectList < ros.Message
    %GraspableObjectList MATLAB implementation of manipulation_msgs/GraspableObjectList
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'manipulation_msgs/GraspableObjectList' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'd67571f2982f1b7115de1e0027319109' % The MD5 Checksum of the message definition
        PropertyList = { 'GraspableObjects' 'Image' 'CameraInfo' 'Meshes' 'ReferenceToCamera' } % List of non-constant message properties
        ROSPropertyList = { 'graspable_objects' 'image' 'camera_info' 'meshes' 'reference_to_camera' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.manipulation_msgs.GraspableObject' ...
            'ros.msg.sensor_msgs.Image' ...
            'ros.msggen.sensor_msgs.CameraInfo' ...
            'ros.msggen.shape_msgs.Mesh' ...
            'ros.msggen.geometry_msgs.Pose' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        GraspableObjects
        Image
        CameraInfo
        Meshes
        ReferenceToCamera
    end
    methods
        function set.GraspableObjects(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.manipulation_msgs.GraspableObject.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.manipulation_msgs.GraspableObject'};
            validateattributes(val, validClasses, validAttributes, 'GraspableObjectList', 'GraspableObjects')
            obj.GraspableObjects = val;
        end
        function set.Image(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.sensor_msgs.Image'};
            validateattributes(val, validClasses, validAttributes, 'GraspableObjectList', 'Image')
            obj.Image = val;
        end
        function set.CameraInfo(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.sensor_msgs.CameraInfo'};
            validateattributes(val, validClasses, validAttributes, 'GraspableObjectList', 'CameraInfo')
            obj.CameraInfo = val;
        end
        function set.Meshes(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.shape_msgs.Mesh.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.shape_msgs.Mesh'};
            validateattributes(val, validClasses, validAttributes, 'GraspableObjectList', 'Meshes')
            obj.Meshes = val;
        end
        function set.ReferenceToCamera(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Pose'};
            validateattributes(val, validClasses, validAttributes, 'GraspableObjectList', 'ReferenceToCamera')
            obj.ReferenceToCamera = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.manipulation_msgs.GraspableObjectList.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.manipulation_msgs.GraspableObjectList(strObj);
        end
    end
end


classdef DatabaseModelPose < ros.Message
    %DatabaseModelPose MATLAB implementation of household_objects_database_msgs/DatabaseModelPose
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'household_objects_database_msgs/DatabaseModelPose' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '6bc39ef48ca57cc7d4341cfc13a5a110' % The MD5 Checksum of the message definition
        PropertyList = { 'Type' 'Pose' 'ModelId' 'Confidence' 'DetectorName' } % List of non-constant message properties
        ROSPropertyList = { 'type' 'pose' 'model_id' 'confidence' 'detector_name' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.object_recognition_msgs.ObjectType' ...
            'ros.msggen.geometry_msgs.PoseStamped' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Type
        Pose
        ModelId
        Confidence
        DetectorName
    end
    methods
        function set.Type(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.object_recognition_msgs.ObjectType'};
            validateattributes(val, validClasses, validAttributes, 'DatabaseModelPose', 'Type')
            obj.Type = val;
        end
        function set.Pose(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.PoseStamped'};
            validateattributes(val, validClasses, validAttributes, 'DatabaseModelPose', 'Pose')
            obj.Pose = val;
        end
        function set.ModelId(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'DatabaseModelPose', 'ModelId');
            obj.ModelId = int32(val);
        end
        function set.Confidence(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'DatabaseModelPose', 'Confidence');
            obj.Confidence = single(val);
        end
        function set.DetectorName(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'DatabaseModelPose', 'DetectorName');
            obj.DetectorName = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.household_objects_database_msgs.DatabaseModelPose.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.household_objects_database_msgs.DatabaseModelPose(strObj);
        end
    end
end

classdef VisibilityConstraint < ros.Message
    %VisibilityConstraint MATLAB implementation of moveit_msgs/VisibilityConstraint
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'moveit_msgs/VisibilityConstraint' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '62cda903bfe31ff2e5fcdc3810d577ad' % The MD5 Checksum of the message definition
        PropertyList = { 'TargetPose' 'SensorPose' 'TargetRadius' 'ConeSides' 'MaxViewAngle' 'MaxRangeAngle' 'SensorViewDirection' 'Weight' } % List of non-constant message properties
        ROSPropertyList = { 'target_pose' 'sensor_pose' 'target_radius' 'cone_sides' 'max_view_angle' 'max_range_angle' 'sensor_view_direction' 'weight' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.geometry_msgs.PoseStamped' ...
            'ros.msggen.geometry_msgs.PoseStamped' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
        SENSORZ = uint8(0)
        SENSORY = uint8(1)
        SENSORX = uint8(2)
    end
    properties
        TargetPose
        SensorPose
        TargetRadius
        ConeSides
        MaxViewAngle
        MaxRangeAngle
        SensorViewDirection
        Weight
    end
    methods
        function set.TargetPose(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.PoseStamped'};
            validateattributes(val, validClasses, validAttributes, 'VisibilityConstraint', 'TargetPose')
            obj.TargetPose = val;
        end
        function set.SensorPose(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.PoseStamped'};
            validateattributes(val, validClasses, validAttributes, 'VisibilityConstraint', 'SensorPose')
            obj.SensorPose = val;
        end
        function set.TargetRadius(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'VisibilityConstraint', 'TargetRadius');
            obj.TargetRadius = double(val);
        end
        function set.ConeSides(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'VisibilityConstraint', 'ConeSides');
            obj.ConeSides = int32(val);
        end
        function set.MaxViewAngle(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'VisibilityConstraint', 'MaxViewAngle');
            obj.MaxViewAngle = double(val);
        end
        function set.MaxRangeAngle(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'VisibilityConstraint', 'MaxRangeAngle');
            obj.MaxRangeAngle = double(val);
        end
        function set.SensorViewDirection(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'VisibilityConstraint', 'SensorViewDirection');
            obj.SensorViewDirection = uint8(val);
        end
        function set.Weight(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'VisibilityConstraint', 'Weight');
            obj.Weight = double(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.moveit_msgs.VisibilityConstraint.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.moveit_msgs.VisibilityConstraint(strObj);
        end
    end
end

classdef CartesianPoint < ros.Message
    %CartesianPoint MATLAB implementation of moveit_msgs/CartesianPoint
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'moveit_msgs/CartesianPoint' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'd3c213cdb4382c43adbff1f2dd2cf669' % The MD5 Checksum of the message definition
        PropertyList = { 'Pose' 'Velocity' 'Acceleration' } % List of non-constant message properties
        ROSPropertyList = { 'pose' 'velocity' 'acceleration' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.geometry_msgs.Pose' ...
            'ros.msggen.geometry_msgs.Twist' ...
            'ros.msggen.geometry_msgs.Accel' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Pose
        Velocity
        Acceleration
    end
    methods
        function set.Pose(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Pose'};
            validateattributes(val, validClasses, validAttributes, 'CartesianPoint', 'Pose')
            obj.Pose = val;
        end
        function set.Velocity(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Twist'};
            validateattributes(val, validClasses, validAttributes, 'CartesianPoint', 'Velocity')
            obj.Velocity = val;
        end
        function set.Acceleration(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Accel'};
            validateattributes(val, validClasses, validAttributes, 'CartesianPoint', 'Acceleration')
            obj.Acceleration = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.moveit_msgs.CartesianPoint.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.moveit_msgs.CartesianPoint(strObj);
        end
    end
end
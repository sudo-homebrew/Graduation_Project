
classdef CartesianTrajectoryPoint < ros.Message
    %CartesianTrajectoryPoint MATLAB implementation of moveit_msgs/CartesianTrajectoryPoint
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'moveit_msgs/CartesianTrajectoryPoint' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'e996ea294f646e6911b3e85e624f5acf' % The MD5 Checksum of the message definition
        PropertyList = { 'Point' 'TimeFromStart' } % List of non-constant message properties
        ROSPropertyList = { 'point' 'time_from_start' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.moveit_msgs.CartesianPoint' ...
            'ros.msg.Duration' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Point
        TimeFromStart
    end
    methods
        function set.Point(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.moveit_msgs.CartesianPoint'};
            validateattributes(val, validClasses, validAttributes, 'CartesianTrajectoryPoint', 'Point')
            obj.Point = val;
        end
        function set.TimeFromStart(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.Duration'};
            validateattributes(val, validClasses, validAttributes, 'CartesianTrajectoryPoint', 'TimeFromStart')
            obj.TimeFromStart = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.moveit_msgs.CartesianTrajectoryPoint.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.moveit_msgs.CartesianTrajectoryPoint(strObj);
        end
    end
end

classdef JointStateCalibrationPattern < ros.Message
    %JointStateCalibrationPattern MATLAB implementation of calibration_msgs/JointStateCalibrationPattern
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'calibration_msgs/JointStateCalibrationPattern' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'c80e9cf8e7942eba44a6d32e3c75bf59' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'ObjectPoints' 'JointPoints' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'object_points' 'joint_points' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.sensor_msgs.JointState' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        ObjectPoints
        JointPoints
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'JointStateCalibrationPattern', 'Header')
            obj.Header = val;
        end
        function set.ObjectPoints(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.geometry_msgs.Point.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'JointStateCalibrationPattern', 'ObjectPoints')
            obj.ObjectPoints = val;
        end
        function set.JointPoints(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.sensor_msgs.JointState.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.sensor_msgs.JointState'};
            validateattributes(val, validClasses, validAttributes, 'JointStateCalibrationPattern', 'JointPoints')
            obj.JointPoints = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.calibration_msgs.JointStateCalibrationPattern.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.calibration_msgs.JointStateCalibrationPattern(strObj);
        end
    end
end

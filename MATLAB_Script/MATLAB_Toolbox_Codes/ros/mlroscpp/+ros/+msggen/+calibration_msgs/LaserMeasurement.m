
classdef LaserMeasurement < ros.Message
    %LaserMeasurement MATLAB implementation of calibration_msgs/LaserMeasurement
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'calibration_msgs/LaserMeasurement' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '7fa7e818b1234a443aa5d8e315175d17' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'JointPoints' 'Snapshot' 'LaserImage' 'ImageFeatures' 'JointFeatures' 'LaserId' 'Verbose' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'joint_points' 'snapshot' 'laser_image' 'image_features' 'joint_features' 'laser_id' 'verbose' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.sensor_msgs.JointState' ...
            'ros.msggen.calibration_msgs.DenseLaserSnapshot' ...
            'ros.msg.sensor_msgs.Image' ...
            'ros.msggen.calibration_msgs.CalibrationPattern' ...
            'ros.msggen.calibration_msgs.JointStateCalibrationPattern' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        JointPoints
        Snapshot
        LaserImage
        ImageFeatures
        JointFeatures
        LaserId
        Verbose
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'LaserMeasurement', 'Header')
            obj.Header = val;
        end
        function set.JointPoints(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.sensor_msgs.JointState.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.sensor_msgs.JointState'};
            validateattributes(val, validClasses, validAttributes, 'LaserMeasurement', 'JointPoints')
            obj.JointPoints = val;
        end
        function set.Snapshot(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.calibration_msgs.DenseLaserSnapshot'};
            validateattributes(val, validClasses, validAttributes, 'LaserMeasurement', 'Snapshot')
            obj.Snapshot = val;
        end
        function set.LaserImage(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.sensor_msgs.Image'};
            validateattributes(val, validClasses, validAttributes, 'LaserMeasurement', 'LaserImage')
            obj.LaserImage = val;
        end
        function set.ImageFeatures(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.calibration_msgs.CalibrationPattern'};
            validateattributes(val, validClasses, validAttributes, 'LaserMeasurement', 'ImageFeatures')
            obj.ImageFeatures = val;
        end
        function set.JointFeatures(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.calibration_msgs.JointStateCalibrationPattern'};
            validateattributes(val, validClasses, validAttributes, 'LaserMeasurement', 'JointFeatures')
            obj.JointFeatures = val;
        end
        function set.LaserId(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'LaserMeasurement', 'LaserId');
            obj.LaserId = char(val);
        end
        function set.Verbose(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'LaserMeasurement', 'Verbose');
            obj.Verbose = logical(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.calibration_msgs.LaserMeasurement.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.calibration_msgs.LaserMeasurement(strObj);
        end
    end
end
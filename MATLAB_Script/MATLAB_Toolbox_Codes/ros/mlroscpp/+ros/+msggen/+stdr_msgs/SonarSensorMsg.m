
classdef SonarSensorMsg < ros.Message
    %SonarSensorMsg MATLAB implementation of stdr_msgs/SonarSensorMsg
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'stdr_msgs/SonarSensorMsg' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'e8b8bd75b2e357572ea0c9ee72ed58c1' % The MD5 Checksum of the message definition
        PropertyList = { 'Noise' 'Pose' 'MaxRange' 'MinRange' 'ConeAngle' 'Frequency' 'FrameId' } % List of non-constant message properties
        ROSPropertyList = { 'noise' 'pose' 'maxRange' 'minRange' 'coneAngle' 'frequency' 'frame_id' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.stdr_msgs.Noise' ...
            'ros.msggen.geometry_msgs.Pose2D' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Noise
        Pose
        MaxRange
        MinRange
        ConeAngle
        Frequency
        FrameId
    end
    methods
        function set.Noise(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.stdr_msgs.Noise'};
            validateattributes(val, validClasses, validAttributes, 'SonarSensorMsg', 'Noise')
            obj.Noise = val;
        end
        function set.Pose(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Pose2D'};
            validateattributes(val, validClasses, validAttributes, 'SonarSensorMsg', 'Pose')
            obj.Pose = val;
        end
        function set.MaxRange(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'SonarSensorMsg', 'MaxRange');
            obj.MaxRange = single(val);
        end
        function set.MinRange(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'SonarSensorMsg', 'MinRange');
            obj.MinRange = single(val);
        end
        function set.ConeAngle(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'SonarSensorMsg', 'ConeAngle');
            obj.ConeAngle = single(val);
        end
        function set.Frequency(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'SonarSensorMsg', 'Frequency');
            obj.Frequency = single(val);
        end
        function set.FrameId(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'SonarSensorMsg', 'FrameId');
            obj.FrameId = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.stdr_msgs.SonarSensorMsg.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.stdr_msgs.SonarSensorMsg(strObj);
        end
    end
end

classdef RawImuData < ros.Message
    %RawImuData MATLAB implementation of multisense_ros/RawImuData
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'multisense_ros/RawImuData' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'bab971dfc7138ffa0d1374504403ac83' % The MD5 Checksum of the message definition
        PropertyList = { 'TimeStamp' 'X' 'Y' 'Z' } % List of non-constant message properties
        ROSPropertyList = { 'time_stamp' 'x' 'y' 'z' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msg.Time' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        TimeStamp
        X
        Y
        Z
    end
    methods
        function set.TimeStamp(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.Time'};
            validateattributes(val, validClasses, validAttributes, 'RawImuData', 'TimeStamp')
            obj.TimeStamp = val;
        end
        function set.X(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'RawImuData', 'X');
            obj.X = single(val);
        end
        function set.Y(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'RawImuData', 'Y');
            obj.Y = single(val);
        end
        function set.Z(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'RawImuData', 'Z');
            obj.Z = single(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.multisense_ros.RawImuData.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.multisense_ros.RawImuData(strObj);
        end
    end
end
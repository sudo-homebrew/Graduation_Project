
classdef Magnetometer < ros.Message
    %Magnetometer MATLAB implementation of clearpath_base/Magnetometer
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'clearpath_base/Magnetometer' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '5defbd163657b4f6f639ba6d5676cc01' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'X' 'Y' 'Z' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'x' 'y' 'z' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        X
        Y
        Z
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'Magnetometer', 'Header')
            obj.Header = val;
        end
        function set.X(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Magnetometer', 'X');
            obj.X = double(val);
        end
        function set.Y(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Magnetometer', 'Y');
            obj.Y = double(val);
        end
        function set.Z(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Magnetometer', 'Z');
            obj.Z = double(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.clearpath_base.Magnetometer.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.clearpath_base.Magnetometer(strObj);
        end
    end
end

classdef HilSensor < ros.Message
    %HilSensor MATLAB implementation of mavros_msgs/HilSensor
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'mavros_msgs/HilSensor' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '2a892891e5c40d6dd1066bf1f394b5dc' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Acc' 'Gyro' 'Mag' 'AbsPressure' 'DiffPressure' 'PressureAlt' 'Temperature' 'FieldsUpdated' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'acc' 'gyro' 'mag' 'abs_pressure' 'diff_pressure' 'pressure_alt' 'temperature' 'fields_updated' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.geometry_msgs.Vector3' ...
            'ros.msggen.geometry_msgs.Vector3' ...
            'ros.msggen.geometry_msgs.Vector3' ...
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
        Header
        Acc
        Gyro
        Mag
        AbsPressure
        DiffPressure
        PressureAlt
        Temperature
        FieldsUpdated
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'HilSensor', 'Header')
            obj.Header = val;
        end
        function set.Acc(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Vector3'};
            validateattributes(val, validClasses, validAttributes, 'HilSensor', 'Acc')
            obj.Acc = val;
        end
        function set.Gyro(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Vector3'};
            validateattributes(val, validClasses, validAttributes, 'HilSensor', 'Gyro')
            obj.Gyro = val;
        end
        function set.Mag(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Vector3'};
            validateattributes(val, validClasses, validAttributes, 'HilSensor', 'Mag')
            obj.Mag = val;
        end
        function set.AbsPressure(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'HilSensor', 'AbsPressure');
            obj.AbsPressure = single(val);
        end
        function set.DiffPressure(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'HilSensor', 'DiffPressure');
            obj.DiffPressure = single(val);
        end
        function set.PressureAlt(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'HilSensor', 'PressureAlt');
            obj.PressureAlt = single(val);
        end
        function set.Temperature(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'HilSensor', 'Temperature');
            obj.Temperature = single(val);
        end
        function set.FieldsUpdated(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'HilSensor', 'FieldsUpdated');
            obj.FieldsUpdated = uint32(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.mavros_msgs.HilSensor.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.mavros_msgs.HilSensor(strObj);
        end
    end
end
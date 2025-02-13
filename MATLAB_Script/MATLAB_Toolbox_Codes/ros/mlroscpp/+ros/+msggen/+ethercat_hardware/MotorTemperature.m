
classdef MotorTemperature < ros.Message
    %MotorTemperature MATLAB implementation of ethercat_hardware/MotorTemperature
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'ethercat_hardware/MotorTemperature' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'd8c7239cd096d6f25b75bff6b63f2162' % The MD5 Checksum of the message definition
        PropertyList = { 'Stamp' 'WindingTemperature' 'HousingTemperature' 'AmbientTemperature' 'HeatingPower' } % List of non-constant message properties
        ROSPropertyList = { 'stamp' 'winding_temperature' 'housing_temperature' 'ambient_temperature' 'heating_power' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msg.Time' ...
            '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Stamp
        WindingTemperature
        HousingTemperature
        AmbientTemperature
        HeatingPower
    end
    methods
        function set.Stamp(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.Time'};
            validateattributes(val, validClasses, validAttributes, 'MotorTemperature', 'Stamp')
            obj.Stamp = val;
        end
        function set.WindingTemperature(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'MotorTemperature', 'WindingTemperature');
            obj.WindingTemperature = double(val);
        end
        function set.HousingTemperature(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'MotorTemperature', 'HousingTemperature');
            obj.HousingTemperature = double(val);
        end
        function set.AmbientTemperature(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'MotorTemperature', 'AmbientTemperature');
            obj.AmbientTemperature = double(val);
        end
        function set.HeatingPower(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'MotorTemperature', 'HeatingPower');
            obj.HeatingPower = double(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.ethercat_hardware.MotorTemperature.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.ethercat_hardware.MotorTemperature(strObj);
        end
    end
end


classdef ThermalSensorMeasurementMsg < ros.Message
    %ThermalSensorMeasurementMsg MATLAB implementation of stdr_msgs/ThermalSensorMeasurementMsg
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'stdr_msgs/ThermalSensorMeasurementMsg' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'e4b704fefcd6eb849f164e31d5084251' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'ThermalSourceDegrees' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'thermal_source_degrees' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        ThermalSourceDegrees
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'ThermalSensorMeasurementMsg', 'Header')
            obj.Header = val;
        end
        function set.ThermalSourceDegrees(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = single.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'ThermalSensorMeasurementMsg', 'ThermalSourceDegrees');
            obj.ThermalSourceDegrees = single(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.stdr_msgs.ThermalSensorMeasurementMsg.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.stdr_msgs.ThermalSensorMeasurementMsg(strObj);
        end
    end
end

classdef MotorTraceSample < ros.Message
    %MotorTraceSample MATLAB implementation of sr_edc_ethercat_drivers/MotorTraceSample
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'sr_edc_ethercat_drivers/MotorTraceSample' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'f5faf420d7c29e68b1c6bfdff440ffb8' % The MD5 Checksum of the message definition
        PropertyList = { 'CommandedEffort' 'SlowEffortLimit' 'QuickEffortLimit' 'MotorCurrent' 'MotorSupplyVoltage' 'HbridgeDuty' 'Temperature' 'ForceSensor1' 'ForceSensor2' 'ForceSensor3' 'MotorVelocity' 'Velocity' 'Position' } % List of non-constant message properties
        ROSPropertyList = { 'commanded_effort' 'slow_effort_limit' 'quick_effort_limit' 'motor_current' 'motor_supply_voltage' 'hbridge_duty' 'temperature' 'force_sensor_1' 'force_sensor_2' 'force_sensor_3' 'motor_velocity' 'velocity' 'position' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
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
        CommandedEffort
        SlowEffortLimit
        QuickEffortLimit
        MotorCurrent
        MotorSupplyVoltage
        HbridgeDuty
        Temperature
        ForceSensor1
        ForceSensor2
        ForceSensor3
        MotorVelocity
        Velocity
        Position
    end
    methods
        function set.CommandedEffort(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'MotorTraceSample', 'CommandedEffort');
            obj.CommandedEffort = double(val);
        end
        function set.SlowEffortLimit(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'MotorTraceSample', 'SlowEffortLimit');
            obj.SlowEffortLimit = double(val);
        end
        function set.QuickEffortLimit(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'MotorTraceSample', 'QuickEffortLimit');
            obj.QuickEffortLimit = double(val);
        end
        function set.MotorCurrent(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'MotorTraceSample', 'MotorCurrent');
            obj.MotorCurrent = double(val);
        end
        function set.MotorSupplyVoltage(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'MotorTraceSample', 'MotorSupplyVoltage');
            obj.MotorSupplyVoltage = double(val);
        end
        function set.HbridgeDuty(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'MotorTraceSample', 'HbridgeDuty');
            obj.HbridgeDuty = double(val);
        end
        function set.Temperature(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'MotorTraceSample', 'Temperature');
            obj.Temperature = double(val);
        end
        function set.ForceSensor1(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'MotorTraceSample', 'ForceSensor1');
            obj.ForceSensor1 = double(val);
        end
        function set.ForceSensor2(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'MotorTraceSample', 'ForceSensor2');
            obj.ForceSensor2 = double(val);
        end
        function set.ForceSensor3(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'MotorTraceSample', 'ForceSensor3');
            obj.ForceSensor3 = double(val);
        end
        function set.MotorVelocity(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'MotorTraceSample', 'MotorVelocity');
            obj.MotorVelocity = double(val);
        end
        function set.Velocity(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'MotorTraceSample', 'Velocity');
            obj.Velocity = double(val);
        end
        function set.Position(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'MotorTraceSample', 'Position');
            obj.Position = double(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.sr_edc_ethercat_drivers.MotorTraceSample.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.sr_edc_ethercat_drivers.MotorTraceSample(strObj);
        end
    end
end
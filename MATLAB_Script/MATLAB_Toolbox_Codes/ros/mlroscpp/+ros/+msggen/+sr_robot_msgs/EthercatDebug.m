
classdef EthercatDebug < ros.Message
    %EthercatDebug MATLAB implementation of sr_robot_msgs/EthercatDebug
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'sr_robot_msgs/EthercatDebug' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'ed9e30784a7d4571ecf4d526e7ff529f' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'MotorDataType' 'Sensors' 'WhichMotors' 'WhichMotorDataArrived' 'WhichMotorDataHadErrors' 'MotorDataPacketTorque' 'MotorDataPacketMisc' 'TactileDataType' 'TactileDataValid' 'Tactile' 'IdleTimeUs' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'motor_data_type' 'sensors' 'which_motors' 'which_motor_data_arrived' 'which_motor_data_had_errors' 'motor_data_packet_torque' 'motor_data_packet_misc' 'tactile_data_type' 'tactile_data_valid' 'tactile' 'idle_time_us' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.sr_robot_msgs.FromMotorDataType' ...
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
        Header
        MotorDataType
        Sensors
        WhichMotors
        WhichMotorDataArrived
        WhichMotorDataHadErrors
        MotorDataPacketTorque
        MotorDataPacketMisc
        TactileDataType
        TactileDataValid
        Tactile
        IdleTimeUs
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'EthercatDebug', 'Header')
            obj.Header = val;
        end
        function set.MotorDataType(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.sr_robot_msgs.FromMotorDataType'};
            validateattributes(val, validClasses, validAttributes, 'EthercatDebug', 'MotorDataType')
            obj.MotorDataType = val;
        end
        function set.Sensors(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = int16.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'EthercatDebug', 'Sensors');
            obj.Sensors = int16(val);
        end
        function set.WhichMotors(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'EthercatDebug', 'WhichMotors');
            obj.WhichMotors = int16(val);
        end
        function set.WhichMotorDataArrived(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'EthercatDebug', 'WhichMotorDataArrived');
            obj.WhichMotorDataArrived = int32(val);
        end
        function set.WhichMotorDataHadErrors(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'EthercatDebug', 'WhichMotorDataHadErrors');
            obj.WhichMotorDataHadErrors = int32(val);
        end
        function set.MotorDataPacketTorque(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = int16.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'EthercatDebug', 'MotorDataPacketTorque');
            obj.MotorDataPacketTorque = int16(val);
        end
        function set.MotorDataPacketMisc(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = int16.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'EthercatDebug', 'MotorDataPacketMisc');
            obj.MotorDataPacketMisc = int16(val);
        end
        function set.TactileDataType(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'EthercatDebug', 'TactileDataType');
            obj.TactileDataType = int32(val);
        end
        function set.TactileDataValid(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'EthercatDebug', 'TactileDataValid');
            obj.TactileDataValid = int16(val);
        end
        function set.Tactile(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = int16.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'EthercatDebug', 'Tactile');
            obj.Tactile = int16(val);
        end
        function set.IdleTimeUs(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'EthercatDebug', 'IdleTimeUs');
            obj.IdleTimeUs = int16(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.sr_robot_msgs.EthercatDebug.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.sr_robot_msgs.EthercatDebug(strObj);
        end
    end
end
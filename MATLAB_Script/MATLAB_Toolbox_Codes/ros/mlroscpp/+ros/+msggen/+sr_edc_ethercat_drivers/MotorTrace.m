
classdef MotorTrace < ros.Message
    %MotorTrace MATLAB implementation of sr_edc_ethercat_drivers/MotorTrace
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'sr_edc_ethercat_drivers/MotorTrace' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'd06300e55fe6989d2795bc4072205fe1' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'ActuatorInfo' 'Samples' 'Reason' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'actuator_info' 'samples' 'reason' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.sr_edc_ethercat_drivers.ActuatorInfo' ...
            'ros.msggen.sr_edc_ethercat_drivers.MotorTraceSample' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        ActuatorInfo
        Samples
        Reason
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'MotorTrace', 'Header')
            obj.Header = val;
        end
        function set.ActuatorInfo(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.sr_edc_ethercat_drivers.ActuatorInfo'};
            validateattributes(val, validClasses, validAttributes, 'MotorTrace', 'ActuatorInfo')
            obj.ActuatorInfo = val;
        end
        function set.Samples(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.sr_edc_ethercat_drivers.MotorTraceSample.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.sr_edc_ethercat_drivers.MotorTraceSample'};
            validateattributes(val, validClasses, validAttributes, 'MotorTrace', 'Samples')
            obj.Samples = val;
        end
        function set.Reason(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'MotorTrace', 'Reason');
            obj.Reason = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.sr_edc_ethercat_drivers.MotorTrace.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.sr_edc_ethercat_drivers.MotorTrace(strObj);
        end
    end
end

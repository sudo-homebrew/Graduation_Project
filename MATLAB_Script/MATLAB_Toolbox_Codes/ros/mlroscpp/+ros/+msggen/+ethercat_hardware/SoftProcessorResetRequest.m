
classdef SoftProcessorResetRequest < ros.Message
    %SoftProcessorResetRequest MATLAB implementation of ethercat_hardware/SoftProcessorResetRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'ethercat_hardware/SoftProcessorResetRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '777be25d71e9e85e62fa14223ffddb6b' % The MD5 Checksum of the message definition
        PropertyList = { 'ActuatorName' 'ProcessorName' } % List of non-constant message properties
        ROSPropertyList = { 'actuator_name' 'processor_name' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        ActuatorName
        ProcessorName
    end
    methods
        function set.ActuatorName(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'SoftProcessorResetRequest', 'ActuatorName');
            obj.ActuatorName = char(val);
        end
        function set.ProcessorName(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'SoftProcessorResetRequest', 'ProcessorName');
            obj.ProcessorName = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.ethercat_hardware.SoftProcessorResetRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.ethercat_hardware.SoftProcessorResetRequest(strObj);
        end
    end
end

classdef AidingSensorIntegrationControl < ros.Message
    %AidingSensorIntegrationControl MATLAB implementation of applanix_msgs/AidingSensorIntegrationControl
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'applanix_msgs/AidingSensorIntegrationControl' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'aa768b4fd931539b13e67337f33cf90c' % The MD5 Checksum of the message definition
        PropertyList = { 'Transaction' 'Override' } % List of non-constant message properties
        ROSPropertyList = { 'transaction' 'override' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
        OVERRIDEFORCEPRIMARYGNSSVALID = uint32(1)
        OVERRIDEFORCEPRIMARYGNSSINVALID = uint32(2)
        OVERRIDEFORCEAUXGNSSVALID = uint32(4)
        OVERRIDEFORCEAUXGNSSINVALID = uint32(8)
        OVERRIDEDISABLEGAMSHEADINGAIDING = uint32(16)
        OVERRIDEFORCEDMIVALID = uint32(32)
        OVERRIDEFORCEDMIINVALID = uint32(64)
        OVERRIDEDISABLEYZVELOCITYAIDING = uint32(128)
    end
    properties
        Transaction
        Override
    end
    methods
        function set.Transaction(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'AidingSensorIntegrationControl', 'Transaction');
            obj.Transaction = uint16(val);
        end
        function set.Override(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'AidingSensorIntegrationControl', 'Override');
            obj.Override = uint32(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.applanix_msgs.AidingSensorIntegrationControl.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.applanix_msgs.AidingSensorIntegrationControl(strObj);
        end
    end
end
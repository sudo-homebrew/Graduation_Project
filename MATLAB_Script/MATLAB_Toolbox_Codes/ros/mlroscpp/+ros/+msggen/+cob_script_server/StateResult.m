
classdef StateResult < ros.Message
    %StateResult MATLAB implementation of cob_script_server/StateResult
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'cob_script_server/StateResult' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'cfe8a15d5e2e01586cd5e05f2094f7c2' % The MD5 Checksum of the message definition
        PropertyList = { 'ReturnValue' } % List of non-constant message properties
        ROSPropertyList = { 'return_value' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        ReturnValue
    end
    methods
        function set.ReturnValue(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'StateResult', 'ReturnValue');
            obj.ReturnValue = int16(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.cob_script_server.StateResult.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.cob_script_server.StateResult(strObj);
        end
    end
end

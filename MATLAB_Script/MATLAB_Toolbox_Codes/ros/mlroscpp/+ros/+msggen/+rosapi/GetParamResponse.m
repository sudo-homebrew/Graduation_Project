
classdef GetParamResponse < ros.Message
    %GetParamResponse MATLAB implementation of rosapi/GetParamResponse
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'rosapi/GetParamResponse' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '64e58419496c7248b4ef25731f88b8c3' % The MD5 Checksum of the message definition
    end
    properties (Constant)
    end
    properties
        Value
    end
    properties (Constant, Hidden)
        PropertyList = { 'Value' } % List of non-constant message properties
        ROSPropertyList = { 'value' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
			 } % Types of contained nested messages
    end
    methods
        function set.Value(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'GetParamResponse', 'Value');
            obj.Value = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.rosapi.GetParamResponse.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.rosapi.GetParamResponse;
            obj.reload(strObj);
        end
    end
end
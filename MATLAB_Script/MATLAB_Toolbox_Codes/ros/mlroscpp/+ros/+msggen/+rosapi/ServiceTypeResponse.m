
classdef ServiceTypeResponse < ros.Message
    %ServiceTypeResponse MATLAB implementation of rosapi/ServiceTypeResponse
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'rosapi/ServiceTypeResponse' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'dc67331de85cf97091b7d45e5c64ab75' % The MD5 Checksum of the message definition
        PropertyList = { 'Type' } % List of non-constant message properties
        ROSPropertyList = { 'type' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Type
    end
    methods
        function set.Type(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'ServiceTypeResponse', 'Type');
            obj.Type = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.rosapi.ServiceTypeResponse.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.rosapi.ServiceTypeResponse(strObj);
        end
    end
end
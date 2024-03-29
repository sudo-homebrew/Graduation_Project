
classdef StringStringRequest < ros.Message
    %StringStringRequest MATLAB implementation of roseus/StringStringRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'roseus/StringStringRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '994972b6e03928b2476860ce6c4c8e17' % The MD5 Checksum of the message definition
        PropertyList = { 'Str' } % List of non-constant message properties
        ROSPropertyList = { 'str' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Str
    end
    methods
        function set.Str(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'StringStringRequest', 'Str');
            obj.Str = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.roseus.StringStringRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.roseus.StringStringRequest(strObj);
        end
    end
end

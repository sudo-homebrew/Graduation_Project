
classdef ClearClassifierRequest < ros.Message
    %ClearClassifierRequest MATLAB implementation of ml_classifiers/ClearClassifierRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'ml_classifiers/ClearClassifierRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '9c4b0f781baa7fd49cc9e186f2f56898' % The MD5 Checksum of the message definition
        PropertyList = { 'Identifier' } % List of non-constant message properties
        ROSPropertyList = { 'identifier' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Identifier
    end
    methods
        function set.Identifier(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'ClearClassifierRequest', 'Identifier');
            obj.Identifier = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.ml_classifiers.ClearClassifierRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.ml_classifiers.ClearClassifierRequest(strObj);
        end
    end
end
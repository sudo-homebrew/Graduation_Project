
classdef LoadClassifierRequest < ros.Message
    %LoadClassifierRequest MATLAB implementation of ml_classifiers/LoadClassifierRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'ml_classifiers/LoadClassifierRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '7f9e97a50616de1070817fa85606e7a5' % The MD5 Checksum of the message definition
        PropertyList = { 'Identifier' 'ClassType' 'Filename' } % List of non-constant message properties
        ROSPropertyList = { 'identifier' 'class_type' 'filename' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Identifier
        ClassType
        Filename
    end
    methods
        function set.Identifier(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'LoadClassifierRequest', 'Identifier');
            obj.Identifier = char(val);
        end
        function set.ClassType(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'LoadClassifierRequest', 'ClassType');
            obj.ClassType = char(val);
        end
        function set.Filename(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'LoadClassifierRequest', 'Filename');
            obj.Filename = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.ml_classifiers.LoadClassifierRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.ml_classifiers.LoadClassifierRequest(strObj);
        end
    end
end

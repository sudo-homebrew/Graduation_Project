
classdef ClassifyDataRequest < ros.Message
    %ClassifyDataRequest MATLAB implementation of ml_classifiers/ClassifyDataRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'ml_classifiers/ClassifyDataRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '572733f6e77cd60bddc5c0b72307999c' % The MD5 Checksum of the message definition
        PropertyList = { 'Data' 'Identifier' } % List of non-constant message properties
        ROSPropertyList = { 'data' 'identifier' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.ml_classifiers.ClassDataPoint' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Data
        Identifier
    end
    methods
        function set.Data(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.ml_classifiers.ClassDataPoint.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.ml_classifiers.ClassDataPoint'};
            validateattributes(val, validClasses, validAttributes, 'ClassifyDataRequest', 'Data')
            obj.Data = val;
        end
        function set.Identifier(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'ClassifyDataRequest', 'Identifier');
            obj.Identifier = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.ml_classifiers.ClassifyDataRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.ml_classifiers.ClassifyDataRequest(strObj);
        end
    end
end
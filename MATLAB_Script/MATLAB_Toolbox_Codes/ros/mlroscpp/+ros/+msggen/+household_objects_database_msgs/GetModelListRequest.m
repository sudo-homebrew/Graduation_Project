
classdef GetModelListRequest < ros.Message
    %GetModelListRequest MATLAB implementation of household_objects_database_msgs/GetModelListRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'household_objects_database_msgs/GetModelListRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '6bdf0a866151f41b8876e73800929933' % The MD5 Checksum of the message definition
        PropertyList = { 'ModelSet' } % List of non-constant message properties
        ROSPropertyList = { 'model_set' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        ModelSet
    end
    methods
        function set.ModelSet(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'GetModelListRequest', 'ModelSet');
            obj.ModelSet = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.household_objects_database_msgs.GetModelListRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.household_objects_database_msgs.GetModelListRequest(strObj);
        end
    end
end
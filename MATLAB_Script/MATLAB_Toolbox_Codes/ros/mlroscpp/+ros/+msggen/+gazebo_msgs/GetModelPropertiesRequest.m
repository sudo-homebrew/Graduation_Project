
classdef GetModelPropertiesRequest < ros.Message
    %GetModelPropertiesRequest MATLAB implementation of gazebo_msgs/GetModelPropertiesRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'gazebo_msgs/GetModelPropertiesRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'ea31c8eab6fc401383cf528a7c0984ba' % The MD5 Checksum of the message definition
        PropertyList = { 'ModelName' } % List of non-constant message properties
        ROSPropertyList = { 'model_name' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        ModelName
    end
    methods
        function set.ModelName(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'GetModelPropertiesRequest', 'ModelName');
            obj.ModelName = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.gazebo_msgs.GetModelPropertiesRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.gazebo_msgs.GetModelPropertiesRequest(strObj);
        end
    end
end
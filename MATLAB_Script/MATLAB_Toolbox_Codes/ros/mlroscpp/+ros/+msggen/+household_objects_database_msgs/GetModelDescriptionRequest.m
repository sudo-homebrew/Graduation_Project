
classdef GetModelDescriptionRequest < ros.Message
    %GetModelDescriptionRequest MATLAB implementation of household_objects_database_msgs/GetModelDescriptionRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'household_objects_database_msgs/GetModelDescriptionRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '28cb0598daf3b969068a38cd07aaa9f6' % The MD5 Checksum of the message definition
        PropertyList = { 'ModelId' } % List of non-constant message properties
        ROSPropertyList = { 'model_id' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        ModelId
    end
    methods
        function set.ModelId(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'GetModelDescriptionRequest', 'ModelId');
            obj.ModelId = int32(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.household_objects_database_msgs.GetModelDescriptionRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.household_objects_database_msgs.GetModelDescriptionRequest(strObj);
        end
    end
end
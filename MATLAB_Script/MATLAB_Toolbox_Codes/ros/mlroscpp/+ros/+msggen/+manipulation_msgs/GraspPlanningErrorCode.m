
classdef GraspPlanningErrorCode < ros.Message
    %GraspPlanningErrorCode MATLAB implementation of manipulation_msgs/GraspPlanningErrorCode
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'manipulation_msgs/GraspPlanningErrorCode' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'd0cbf262cc3d8075a46b994eef1bdb2a' % The MD5 Checksum of the message definition
        PropertyList = { 'Value' } % List of non-constant message properties
        ROSPropertyList = { 'value' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
        SUCCESS = int32(0)
        TFERROR = int32(1)
        OTHERERROR = int32(2)
    end
    properties
        Value
    end
    methods
        function set.Value(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'GraspPlanningErrorCode', 'Value');
            obj.Value = int32(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.manipulation_msgs.GraspPlanningErrorCode.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.manipulation_msgs.GraspPlanningErrorCode(strObj);
        end
    end
end
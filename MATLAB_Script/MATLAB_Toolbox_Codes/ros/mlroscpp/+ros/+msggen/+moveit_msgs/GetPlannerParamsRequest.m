
classdef GetPlannerParamsRequest < ros.Message
    %GetPlannerParamsRequest MATLAB implementation of moveit_msgs/GetPlannerParamsRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'moveit_msgs/GetPlannerParamsRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'f5065dceae6a10319c47163ab1012104' % The MD5 Checksum of the message definition
        PropertyList = { 'PipelineId' 'PlannerConfig' 'Group' } % List of non-constant message properties
        ROSPropertyList = { 'pipeline_id' 'planner_config' 'group' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        PipelineId
        PlannerConfig
        Group
    end
    methods
        function set.PipelineId(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'GetPlannerParamsRequest', 'PipelineId');
            obj.PipelineId = char(val);
        end
        function set.PlannerConfig(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'GetPlannerParamsRequest', 'PlannerConfig');
            obj.PlannerConfig = char(val);
        end
        function set.Group(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'GetPlannerParamsRequest', 'Group');
            obj.Group = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.moveit_msgs.GetPlannerParamsRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.moveit_msgs.GetPlannerParamsRequest(strObj);
        end
    end
end

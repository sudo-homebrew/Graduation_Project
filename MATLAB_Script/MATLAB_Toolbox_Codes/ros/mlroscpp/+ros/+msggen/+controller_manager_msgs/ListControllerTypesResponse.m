
classdef ListControllerTypesResponse < ros.Message
    %ListControllerTypesResponse MATLAB implementation of controller_manager_msgs/ListControllerTypesResponse
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'controller_manager_msgs/ListControllerTypesResponse' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'c1d4cd11aefa9f97ba4aeb5b33987f4e' % The MD5 Checksum of the message definition
        PropertyList = { 'Types' 'BaseClasses' } % List of non-constant message properties
        ROSPropertyList = { 'types' 'base_classes' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Types
        BaseClasses
    end
    methods
        function set.Types(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'cell', 'string'};
            if isempty(val)
                % Allow empty [] input
                val = cell.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'ListControllerTypesResponse', 'Types');
            obj.Types = cell(val);
        end
        function set.BaseClasses(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'cell', 'string'};
            if isempty(val)
                % Allow empty [] input
                val = cell.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'ListControllerTypesResponse', 'BaseClasses');
            obj.BaseClasses = cell(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.controller_manager_msgs.ListControllerTypesResponse.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.controller_manager_msgs.ListControllerTypesResponse(strObj);
        end
    end
end
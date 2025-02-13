
classdef EndpointStates < ros.Message
    %EndpointStates MATLAB implementation of baxter_core_msgs/EndpointStates
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'baxter_core_msgs/EndpointStates' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'a0ca50a066809a5f065f39f37aa028fb' % The MD5 Checksum of the message definition
        PropertyList = { 'States' 'Names' } % List of non-constant message properties
        ROSPropertyList = { 'states' 'names' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.baxter_core_msgs.EndpointState' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        States
        Names
    end
    methods
        function set.States(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.baxter_core_msgs.EndpointState.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.baxter_core_msgs.EndpointState'};
            validateattributes(val, validClasses, validAttributes, 'EndpointStates', 'States')
            obj.States = val;
        end
        function set.Names(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'cell', 'string'};
            if isempty(val)
                % Allow empty [] input
                val = cell.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'EndpointStates', 'Names');
            obj.Names = cell(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.baxter_core_msgs.EndpointStates.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.baxter_core_msgs.EndpointStates(strObj);
        end
    end
end

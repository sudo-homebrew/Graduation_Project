
classdef ConfigGoal < ros.Message
    %ConfigGoal MATLAB implementation of joint_states_settler/ConfigGoal
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'joint_states_settler/ConfigGoal' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '863b25359077f9aa231257b3d1612b63' % The MD5 Checksum of the message definition
        PropertyList = { 'MaxStep' 'JointNames' 'Tolerances' 'CacheSize' } % List of non-constant message properties
        ROSPropertyList = { 'max_step' 'joint_names' 'tolerances' 'cache_size' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msg.Duration' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        MaxStep
        JointNames
        Tolerances
        CacheSize
    end
    methods
        function set.MaxStep(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.Duration'};
            validateattributes(val, validClasses, validAttributes, 'ConfigGoal', 'MaxStep')
            obj.MaxStep = val;
        end
        function set.JointNames(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'cell', 'string'};
            if isempty(val)
                % Allow empty [] input
                val = cell.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'ConfigGoal', 'JointNames');
            obj.JointNames = cell(val);
        end
        function set.Tolerances(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'ConfigGoal', 'Tolerances');
            obj.Tolerances = double(val);
        end
        function set.CacheSize(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ConfigGoal', 'CacheSize');
            obj.CacheSize = uint32(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.joint_states_settler.ConfigGoal.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.joint_states_settler.ConfigGoal(strObj);
        end
    end
end

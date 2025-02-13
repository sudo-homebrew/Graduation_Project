
classdef Registers < ros.Message
    %Registers MATLAB implementation of robotnik_msgs/Registers
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'robotnik_msgs/Registers' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '12d8645a7a01078095f8477105240cef' % The MD5 Checksum of the message definition
        PropertyList = { 'Registers_' } % List of non-constant message properties
        ROSPropertyList = { 'registers' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.robotnik_msgs.Register' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Registers_
    end
    methods
        function set.Registers_(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.robotnik_msgs.Register.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.robotnik_msgs.Register'};
            validateattributes(val, validClasses, validAttributes, 'Registers', 'Registers_')
            obj.Registers_ = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.robotnik_msgs.Registers.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.robotnik_msgs.Registers(strObj);
        end
    end
end

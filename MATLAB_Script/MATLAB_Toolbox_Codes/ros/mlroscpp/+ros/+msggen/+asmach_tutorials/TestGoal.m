
classdef TestGoal < ros.Message
    %TestGoal MATLAB implementation of asmach_tutorials/TestGoal
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'asmach_tutorials/TestGoal' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '96f1fc969cebfe9056357b5db1aa501e' % The MD5 Checksum of the message definition
        PropertyList = { 'Goal' } % List of non-constant message properties
        ROSPropertyList = { 'goal' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Goal
    end
    methods
        function set.Goal(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'TestGoal', 'Goal');
            obj.Goal = double(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.asmach_tutorials.TestGoal.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.asmach_tutorials.TestGoal(strObj);
        end
    end
end
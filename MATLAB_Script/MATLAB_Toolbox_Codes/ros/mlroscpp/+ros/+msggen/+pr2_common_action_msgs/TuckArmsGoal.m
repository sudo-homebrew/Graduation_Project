
classdef TuckArmsGoal < ros.Message
    %TuckArmsGoal MATLAB implementation of pr2_common_action_msgs/TuckArmsGoal
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'pr2_common_action_msgs/TuckArmsGoal' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'a07b11078a50f9881dc3004ca1174834' % The MD5 Checksum of the message definition
        PropertyList = { 'TuckLeft' 'TuckRight' } % List of non-constant message properties
        ROSPropertyList = { 'tuck_left' 'tuck_right' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        TuckLeft
        TuckRight
    end
    methods
        function set.TuckLeft(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'TuckArmsGoal', 'TuckLeft');
            obj.TuckLeft = logical(val);
        end
        function set.TuckRight(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'TuckArmsGoal', 'TuckRight');
            obj.TuckRight = logical(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.pr2_common_action_msgs.TuckArmsGoal.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.pr2_common_action_msgs.TuckArmsGoal(strObj);
        end
    end
end
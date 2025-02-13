
classdef MoveBaseActionResult < ros.Message
    %MoveBaseActionResult MATLAB implementation of move_base_msgs/MoveBaseActionResult
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'move_base_msgs/MoveBaseActionResult' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '1eb06eeff08fa7ea874431638cb52332' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Status' 'Result' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'status' 'result' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.actionlib_msgs.GoalStatus' ...
            'ros.msggen.move_base_msgs.MoveBaseResult' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Status
        Result
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'MoveBaseActionResult', 'Header')
            obj.Header = val;
        end
        function set.Status(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.actionlib_msgs.GoalStatus'};
            validateattributes(val, validClasses, validAttributes, 'MoveBaseActionResult', 'Status')
            obj.Status = val;
        end
        function set.Result(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.move_base_msgs.MoveBaseResult'};
            validateattributes(val, validClasses, validAttributes, 'MoveBaseActionResult', 'Result')
            obj.Result = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.move_base_msgs.MoveBaseActionResult.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.move_base_msgs.MoveBaseActionResult(strObj);
        end
    end
end

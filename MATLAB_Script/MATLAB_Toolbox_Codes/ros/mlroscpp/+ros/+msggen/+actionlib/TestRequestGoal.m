
classdef TestRequestGoal < ros.Message
    %TestRequestGoal MATLAB implementation of actionlib/TestRequestGoal
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'actionlib/TestRequestGoal' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'db5d00ba98302d6c6dd3737e9a03ceea' % The MD5 Checksum of the message definition
        PropertyList = { 'DelayAccept' 'DelayTerminate' 'PauseStatus' 'TerminateStatus' 'IgnoreCancel' 'ResultText' 'TheResult' 'IsSimpleClient' } % List of non-constant message properties
        ROSPropertyList = { 'delay_accept' 'delay_terminate' 'pause_status' 'terminate_status' 'ignore_cancel' 'result_text' 'the_result' 'is_simple_client' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msg.Duration' ...
            'ros.msg.Duration' ...
            'ros.msg.Duration' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
        TERMINATESUCCESS = int32(0)
        TERMINATEABORTED = int32(1)
        TERMINATEREJECTED = int32(2)
        TERMINATELOSE = int32(3)
        TERMINATEDROP = int32(4)
        TERMINATEEXCEPTION = int32(5)
    end
    properties
        DelayAccept
        DelayTerminate
        PauseStatus
        TerminateStatus
        IgnoreCancel
        ResultText
        TheResult
        IsSimpleClient
    end
    methods
        function set.DelayAccept(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.Duration'};
            validateattributes(val, validClasses, validAttributes, 'TestRequestGoal', 'DelayAccept')
            obj.DelayAccept = val;
        end
        function set.DelayTerminate(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.Duration'};
            validateattributes(val, validClasses, validAttributes, 'TestRequestGoal', 'DelayTerminate')
            obj.DelayTerminate = val;
        end
        function set.PauseStatus(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.Duration'};
            validateattributes(val, validClasses, validAttributes, 'TestRequestGoal', 'PauseStatus')
            obj.PauseStatus = val;
        end
        function set.TerminateStatus(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'TestRequestGoal', 'TerminateStatus');
            obj.TerminateStatus = int32(val);
        end
        function set.IgnoreCancel(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'TestRequestGoal', 'IgnoreCancel');
            obj.IgnoreCancel = logical(val);
        end
        function set.ResultText(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'TestRequestGoal', 'ResultText');
            obj.ResultText = char(val);
        end
        function set.TheResult(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'TestRequestGoal', 'TheResult');
            obj.TheResult = int32(val);
        end
        function set.IsSimpleClient(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'TestRequestGoal', 'IsSimpleClient');
            obj.IsSimpleClient = logical(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.actionlib.TestRequestGoal.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.actionlib.TestRequestGoal(strObj);
        end
    end
end
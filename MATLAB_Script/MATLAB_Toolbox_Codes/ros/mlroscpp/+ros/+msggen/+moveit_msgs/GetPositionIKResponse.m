
classdef GetPositionIKResponse < ros.Message
    %GetPositionIKResponse MATLAB implementation of moveit_msgs/GetPositionIKResponse
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'moveit_msgs/GetPositionIKResponse' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '3943ba9ed5631a4f63888551da37cd16' % The MD5 Checksum of the message definition
        PropertyList = { 'Solution' 'ErrorCode' } % List of non-constant message properties
        ROSPropertyList = { 'solution' 'error_code' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.moveit_msgs.RobotState' ...
            'ros.msggen.moveit_msgs.MoveItErrorCodes' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Solution
        ErrorCode
    end
    methods
        function set.Solution(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.moveit_msgs.RobotState'};
            validateattributes(val, validClasses, validAttributes, 'GetPositionIKResponse', 'Solution')
            obj.Solution = val;
        end
        function set.ErrorCode(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.moveit_msgs.MoveItErrorCodes'};
            validateattributes(val, validClasses, validAttributes, 'GetPositionIKResponse', 'ErrorCode')
            obj.ErrorCode = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.moveit_msgs.GetPositionIKResponse.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.moveit_msgs.GetPositionIKResponse(strObj);
        end
    end
end

classdef RegisterGuiResponse < ros.Message
    %RegisterGuiResponse MATLAB implementation of stdr_msgs/RegisterGuiResponse
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'stdr_msgs/RegisterGuiResponse' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '5ba49d43b5f1ad43f9cbea11348366c5' % The MD5 Checksum of the message definition
        PropertyList = { 'Robots' } % List of non-constant message properties
        ROSPropertyList = { 'robots' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.stdr_msgs.RobotIndexedMsg' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Robots
    end
    methods
        function set.Robots(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.stdr_msgs.RobotIndexedMsg.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.stdr_msgs.RobotIndexedMsg'};
            validateattributes(val, validClasses, validAttributes, 'RegisterGuiResponse', 'Robots')
            obj.Robots = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.stdr_msgs.RegisterGuiResponse.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.stdr_msgs.RegisterGuiResponse(strObj);
        end
    end
end
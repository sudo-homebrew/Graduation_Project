
classdef PtuGotoResult < ros.Message
    %PtuGotoResult MATLAB implementation of ptu_control/PtuGotoResult
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'ptu_control/PtuGotoResult' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'b869e8f6f1d03107da0fd57ef24c9c1d' % The MD5 Checksum of the message definition
        PropertyList = { 'State' } % List of non-constant message properties
        ROSPropertyList = { 'state' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.sensor_msgs.JointState' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        State
    end
    methods
        function set.State(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.sensor_msgs.JointState'};
            validateattributes(val, validClasses, validAttributes, 'PtuGotoResult', 'State')
            obj.State = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.ptu_control.PtuGotoResult.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.ptu_control.PtuGotoResult(strObj);
        end
    end
end
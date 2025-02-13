
classdef SetEncoderTurnsRequest < ros.Message
    %SetEncoderTurnsRequest MATLAB implementation of robotnik_msgs/SetEncoderTurnsRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'robotnik_msgs/SetEncoderTurnsRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '34cf70b52bbe3c3e3567eb0d481c62de' % The MD5 Checksum of the message definition
        PropertyList = { 'EncoderTurns' } % List of non-constant message properties
        ROSPropertyList = { 'encoder_turns' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.robotnik_msgs.MotorHeadingOffset' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        EncoderTurns
    end
    methods
        function set.EncoderTurns(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.robotnik_msgs.MotorHeadingOffset'};
            validateattributes(val, validClasses, validAttributes, 'SetEncoderTurnsRequest', 'EncoderTurns')
            obj.EncoderTurns = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.robotnik_msgs.SetEncoderTurnsRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.robotnik_msgs.SetEncoderTurnsRequest(strObj);
        end
    end
end

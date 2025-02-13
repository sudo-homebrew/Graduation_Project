
classdef AddSoundSourceRequest < ros.Message
    %AddSoundSourceRequest MATLAB implementation of stdr_msgs/AddSoundSourceRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'stdr_msgs/AddSoundSourceRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'c29faee1e29b2a8ababeae8f802069e9' % The MD5 Checksum of the message definition
        PropertyList = { 'NewSource' } % List of non-constant message properties
        ROSPropertyList = { 'newSource' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.stdr_msgs.SoundSource' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        NewSource
    end
    methods
        function set.NewSource(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.stdr_msgs.SoundSource'};
            validateattributes(val, validClasses, validAttributes, 'AddSoundSourceRequest', 'NewSource')
            obj.NewSource = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.stdr_msgs.AddSoundSourceRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.stdr_msgs.AddSoundSourceRequest(strObj);
        end
    end
end

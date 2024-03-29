
classdef LoadLaunchRequest < ros.Message
    %LoadLaunchRequest MATLAB implementation of fkie_multimaster_msgs/LoadLaunchRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'fkie_multimaster_msgs/LoadLaunchRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '1d00cd540af97efeb6b1589112fab63e' % The MD5 Checksum of the message definition
        PropertyList = { 'Path' } % List of non-constant message properties
        ROSPropertyList = { 'path' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Path
    end
    methods
        function set.Path(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'LoadLaunchRequest', 'Path');
            obj.Path = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.fkie_multimaster_msgs.LoadLaunchRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.fkie_multimaster_msgs.LoadLaunchRequest(strObj);
        end
    end
end

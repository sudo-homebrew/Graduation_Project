
classdef BoardConfigResponse < ros.Message
    %BoardConfigResponse MATLAB implementation of wge100_camera/BoardConfigResponse
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'wge100_camera/BoardConfigResponse' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '6cca7c80398b8b31af04b80534923f16' % The MD5 Checksum of the message definition
        PropertyList = { 'Success' } % List of non-constant message properties
        ROSPropertyList = { 'success' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Success
    end
    methods
        function set.Success(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'BoardConfigResponse', 'Success');
            obj.Success = uint8(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.wge100_camera.BoardConfigResponse.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.wge100_camera.BoardConfigResponse(strObj);
        end
    end
end
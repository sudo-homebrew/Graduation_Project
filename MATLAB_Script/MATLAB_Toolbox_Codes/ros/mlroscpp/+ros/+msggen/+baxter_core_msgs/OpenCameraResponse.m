
classdef OpenCameraResponse < ros.Message
    %OpenCameraResponse MATLAB implementation of baxter_core_msgs/OpenCameraResponse
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'baxter_core_msgs/OpenCameraResponse' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'b6e094011a4dfaee5eddf447220446cf' % The MD5 Checksum of the message definition
        PropertyList = { 'Err' } % List of non-constant message properties
        ROSPropertyList = { 'err' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Err
    end
    methods
        function set.Err(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'OpenCameraResponse', 'Err');
            obj.Err = int32(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.baxter_core_msgs.OpenCameraResponse.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.baxter_core_msgs.OpenCameraResponse(strObj);
        end
    end
end

classdef ConnectResponse < ros.Message
    %ConnectResponse MATLAB implementation of shared_serial/ConnectResponse
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'shared_serial/ConnectResponse' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '89a3ab074876917505d8ad961102ef9f' % The MD5 Checksum of the message definition
        PropertyList = { 'Socket' } % List of non-constant message properties
        ROSPropertyList = { 'socket' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Socket
    end
    methods
        function set.Socket(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ConnectResponse', 'Socket');
            obj.Socket = int32(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.shared_serial.ConnectResponse.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.shared_serial.ConnectResponse(strObj);
        end
    end
end

classdef RecvResponse < ros.Message
    %RecvResponse MATLAB implementation of shared_serial/RecvResponse
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'shared_serial/RecvResponse' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '9aa8c2746a0876552ecc2a81ad0d58a5' % The MD5 Checksum of the message definition
        PropertyList = { 'Socket' 'Data' } % List of non-constant message properties
        ROSPropertyList = { 'socket' 'data' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Socket
        Data
    end
    methods
        function set.Socket(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'RecvResponse', 'Socket');
            obj.Socket = uint32(val);
        end
        function set.Data(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = uint8.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'RecvResponse', 'Data');
            obj.Data = uint8(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.shared_serial.RecvResponse.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.shared_serial.RecvResponse(strObj);
        end
    end
end

classdef SendCommandResponse < ros.Message
    %SendCommandResponse MATLAB implementation of nav2d_navigator/SendCommandResponse
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'nav2d_navigator/SendCommandResponse' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '2cea8cb5717c628edaef5080f770244d' % The MD5 Checksum of the message definition
        PropertyList = { 'Response' } % List of non-constant message properties
        ROSPropertyList = { 'response' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Response
    end
    methods
        function set.Response(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'SendCommandResponse', 'Response');
            obj.Response = int8(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.nav2d_navigator.SendCommandResponse.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.nav2d_navigator.SendCommandResponse(strObj);
        end
    end
end

classdef RemoteGatewayInfoRequest < ros.Message
    %RemoteGatewayInfoRequest MATLAB implementation of gateway_msgs/RemoteGatewayInfoRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'gateway_msgs/RemoteGatewayInfoRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'e005eaac1f4b29980f211758e562aa6e' % The MD5 Checksum of the message definition
        PropertyList = { 'Gateways' } % List of non-constant message properties
        ROSPropertyList = { 'gateways' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Gateways
    end
    methods
        function set.Gateways(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'cell', 'string'};
            if isempty(val)
                % Allow empty [] input
                val = cell.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'RemoteGatewayInfoRequest', 'Gateways');
            obj.Gateways = cell(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.gateway_msgs.RemoteGatewayInfoRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.gateway_msgs.RemoteGatewayInfoRequest(strObj);
        end
    end
end
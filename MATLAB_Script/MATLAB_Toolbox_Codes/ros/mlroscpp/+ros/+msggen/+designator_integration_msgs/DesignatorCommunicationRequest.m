
classdef DesignatorCommunicationRequest < ros.Message
    %DesignatorCommunicationRequest MATLAB implementation of designator_integration_msgs/DesignatorCommunicationRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'designator_integration_msgs/DesignatorCommunicationRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '6949a26db11a1131db0d1dd5dc7c65c4' % The MD5 Checksum of the message definition
        PropertyList = { 'Request' } % List of non-constant message properties
        ROSPropertyList = { 'request' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.designator_integration_msgs.DesignatorRequest' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Request
    end
    methods
        function set.Request(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.designator_integration_msgs.DesignatorRequest'};
            validateattributes(val, validClasses, validAttributes, 'DesignatorCommunicationRequest', 'Request')
            obj.Request = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.designator_integration_msgs.DesignatorCommunicationRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.designator_integration_msgs.DesignatorCommunicationRequest(strObj);
        end
    end
end
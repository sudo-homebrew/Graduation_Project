
classdef head_ctrlRequest < ros.Message
    %head_ctrlRequest MATLAB implementation of rovio_shared/head_ctrlRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'rovio_shared/head_ctrlRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '522f8591c845ace5ac8c5c5852170802' % The MD5 Checksum of the message definition
        PropertyList = { 'HeadPos' } % List of non-constant message properties
        ROSPropertyList = { 'head_pos' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
        HEADUP = int8(11)
        HEADDOWN = int8(12)
        HEADMIDDLE = int8(13)
    end
    properties
        HeadPos
    end
    methods
        function set.HeadPos(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'head_ctrlRequest', 'HeadPos');
            obj.HeadPos = int8(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.rovio_shared.head_ctrlRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.rovio_shared.head_ctrlRequest(strObj);
        end
    end
end
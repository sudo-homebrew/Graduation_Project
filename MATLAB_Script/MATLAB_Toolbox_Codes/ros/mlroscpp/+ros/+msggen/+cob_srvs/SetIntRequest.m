
classdef SetIntRequest < ros.Message
    %SetIntRequest MATLAB implementation of cob_srvs/SetIntRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'cob_srvs/SetIntRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'da5909fbe378aeaf85e547e830cc1bb7' % The MD5 Checksum of the message definition
        PropertyList = { 'Data' } % List of non-constant message properties
        ROSPropertyList = { 'data' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Data
    end
    methods
        function set.Data(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'SetIntRequest', 'Data');
            obj.Data = int32(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.cob_srvs.SetIntRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.cob_srvs.SetIntRequest(strObj);
        end
    end
end
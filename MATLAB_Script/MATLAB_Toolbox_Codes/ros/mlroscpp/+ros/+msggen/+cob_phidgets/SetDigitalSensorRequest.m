
classdef SetDigitalSensorRequest < ros.Message
    %SetDigitalSensorRequest MATLAB implementation of cob_phidgets/SetDigitalSensorRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'cob_phidgets/SetDigitalSensorRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '96cc93c2442fee91eb711a986476a959' % The MD5 Checksum of the message definition
        PropertyList = { 'Uri' 'State' } % List of non-constant message properties
        ROSPropertyList = { 'uri' 'state' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Uri
        State
    end
    methods
        function set.Uri(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'SetDigitalSensorRequest', 'Uri');
            obj.Uri = char(val);
        end
        function set.State(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'SetDigitalSensorRequest', 'State');
            obj.State = int8(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.cob_phidgets.SetDigitalSensorRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.cob_phidgets.SetDigitalSensorRequest(strObj);
        end
    end
end
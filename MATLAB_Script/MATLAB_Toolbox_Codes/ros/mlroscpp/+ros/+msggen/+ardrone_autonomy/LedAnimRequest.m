
classdef LedAnimRequest < ros.Message
    %LedAnimRequest MATLAB implementation of ardrone_autonomy/LedAnimRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'ardrone_autonomy/LedAnimRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '23392fc8200b12a3585ff6a32d597821' % The MD5 Checksum of the message definition
        PropertyList = { 'Type' 'Freq' 'Duration' } % List of non-constant message properties
        ROSPropertyList = { 'type' 'freq' 'duration' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Type
        Freq
        Duration
    end
    methods
        function set.Type(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'LedAnimRequest', 'Type');
            obj.Type = uint8(val);
        end
        function set.Freq(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'LedAnimRequest', 'Freq');
            obj.Freq = single(val);
        end
        function set.Duration(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'LedAnimRequest', 'Duration');
            obj.Duration = uint8(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.ardrone_autonomy.LedAnimRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.ardrone_autonomy.LedAnimRequest(strObj);
        end
    end
end

classdef Connection < ros.Message
    %Connection MATLAB implementation of wireless_msgs/Connection
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'wireless_msgs/Connection' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '5d1c374df3f804bce13d0e5d9bb76c6d' % The MD5 Checksum of the message definition
        PropertyList = { 'Bitrate' 'Txpower' 'LinkQualityRaw' 'LinkQuality' 'SignalLevel' 'NoiseLevel' 'Essid' 'Bssid' } % List of non-constant message properties
        ROSPropertyList = { 'bitrate' 'txpower' 'link_quality_raw' 'link_quality' 'signal_level' 'noise_level' 'essid' 'bssid' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Bitrate
        Txpower
        LinkQualityRaw
        LinkQuality
        SignalLevel
        NoiseLevel
        Essid
        Bssid
    end
    methods
        function set.Bitrate(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Connection', 'Bitrate');
            obj.Bitrate = single(val);
        end
        function set.Txpower(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Connection', 'Txpower');
            obj.Txpower = int16(val);
        end
        function set.LinkQualityRaw(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'Connection', 'LinkQualityRaw');
            obj.LinkQualityRaw = char(val);
        end
        function set.LinkQuality(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Connection', 'LinkQuality');
            obj.LinkQuality = single(val);
        end
        function set.SignalLevel(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Connection', 'SignalLevel');
            obj.SignalLevel = int16(val);
        end
        function set.NoiseLevel(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Connection', 'NoiseLevel');
            obj.NoiseLevel = int16(val);
        end
        function set.Essid(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'Connection', 'Essid');
            obj.Essid = char(val);
        end
        function set.Bssid(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'Connection', 'Bssid');
            obj.Bssid = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.wireless_msgs.Connection.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.wireless_msgs.Connection(strObj);
        end
    end
end
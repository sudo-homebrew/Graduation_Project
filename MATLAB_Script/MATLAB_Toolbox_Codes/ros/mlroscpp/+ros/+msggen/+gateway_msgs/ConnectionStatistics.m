
classdef ConnectionStatistics < ros.Message
    %ConnectionStatistics MATLAB implementation of gateway_msgs/ConnectionStatistics
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'gateway_msgs/ConnectionStatistics' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '672067d5ab3e6157dcf8224022da3ee6' % The MD5 Checksum of the message definition
        PropertyList = { 'GatewayAvailable' 'TimeSinceLastSeen' 'PingLatencyMin' 'PingLatencyMax' 'PingLatencyAvg' 'PingLatencyMdev' 'NetworkInfoAvailable' 'NetworkType' 'WirelessBitrate' 'WirelessLinkQuality' 'WirelessSignalLevel' 'WirelessNoiseLevel' } % List of non-constant message properties
        ROSPropertyList = { 'gateway_available' 'time_since_last_seen' 'ping_latency_min' 'ping_latency_max' 'ping_latency_avg' 'ping_latency_mdev' 'network_info_available' 'network_type' 'wireless_bitrate' 'wireless_link_quality' 'wireless_signal_level' 'wireless_noise_level' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            '' ...
            '' ...
            '' ...
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
        WIRED = int8(1)
        WIRELESS = int8(2)
        MAXTTL = int32(86400)
    end
    properties
        GatewayAvailable
        TimeSinceLastSeen
        PingLatencyMin
        PingLatencyMax
        PingLatencyAvg
        PingLatencyMdev
        NetworkInfoAvailable
        NetworkType
        WirelessBitrate
        WirelessLinkQuality
        WirelessSignalLevel
        WirelessNoiseLevel
    end
    methods
        function set.GatewayAvailable(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ConnectionStatistics', 'GatewayAvailable');
            obj.GatewayAvailable = logical(val);
        end
        function set.TimeSinceLastSeen(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ConnectionStatistics', 'TimeSinceLastSeen');
            obj.TimeSinceLastSeen = int64(val);
        end
        function set.PingLatencyMin(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ConnectionStatistics', 'PingLatencyMin');
            obj.PingLatencyMin = single(val);
        end
        function set.PingLatencyMax(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ConnectionStatistics', 'PingLatencyMax');
            obj.PingLatencyMax = single(val);
        end
        function set.PingLatencyAvg(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ConnectionStatistics', 'PingLatencyAvg');
            obj.PingLatencyAvg = single(val);
        end
        function set.PingLatencyMdev(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ConnectionStatistics', 'PingLatencyMdev');
            obj.PingLatencyMdev = single(val);
        end
        function set.NetworkInfoAvailable(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ConnectionStatistics', 'NetworkInfoAvailable');
            obj.NetworkInfoAvailable = logical(val);
        end
        function set.NetworkType(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ConnectionStatistics', 'NetworkType');
            obj.NetworkType = int8(val);
        end
        function set.WirelessBitrate(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ConnectionStatistics', 'WirelessBitrate');
            obj.WirelessBitrate = single(val);
        end
        function set.WirelessLinkQuality(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ConnectionStatistics', 'WirelessLinkQuality');
            obj.WirelessLinkQuality = int8(val);
        end
        function set.WirelessSignalLevel(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ConnectionStatistics', 'WirelessSignalLevel');
            obj.WirelessSignalLevel = single(val);
        end
        function set.WirelessNoiseLevel(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ConnectionStatistics', 'WirelessNoiseLevel');
            obj.WirelessNoiseLevel = single(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.gateway_msgs.ConnectionStatistics.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.gateway_msgs.ConnectionStatistics(strObj);
        end
    end
end
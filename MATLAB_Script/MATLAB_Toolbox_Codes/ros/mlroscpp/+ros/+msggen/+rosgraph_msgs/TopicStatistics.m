
classdef TopicStatistics < ros.Message
    %TopicStatistics MATLAB implementation of rosgraph_msgs/TopicStatistics
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'rosgraph_msgs/TopicStatistics' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '10152ed868c5097a5e2e4a89d7daa710' % The MD5 Checksum of the message definition
        PropertyList = { 'WindowStart' 'WindowStop' 'PeriodMean' 'PeriodStddev' 'PeriodMax' 'StampAgeMean' 'StampAgeStddev' 'StampAgeMax' 'Topic' 'NodePub' 'NodeSub' 'DeliveredMsgs' 'DroppedMsgs' 'Traffic' } % List of non-constant message properties
        ROSPropertyList = { 'window_start' 'window_stop' 'period_mean' 'period_stddev' 'period_max' 'stamp_age_mean' 'stamp_age_stddev' 'stamp_age_max' 'topic' 'node_pub' 'node_sub' 'delivered_msgs' 'dropped_msgs' 'traffic' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msg.Time' ...
            'ros.msg.Time' ...
            'ros.msg.Duration' ...
            'ros.msg.Duration' ...
            'ros.msg.Duration' ...
            'ros.msg.Duration' ...
            'ros.msg.Duration' ...
            'ros.msg.Duration' ...
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
        WindowStart
        WindowStop
        PeriodMean
        PeriodStddev
        PeriodMax
        StampAgeMean
        StampAgeStddev
        StampAgeMax
        Topic
        NodePub
        NodeSub
        DeliveredMsgs
        DroppedMsgs
        Traffic
    end
    methods
        function set.WindowStart(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.Time'};
            validateattributes(val, validClasses, validAttributes, 'TopicStatistics', 'WindowStart')
            obj.WindowStart = val;
        end
        function set.WindowStop(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.Time'};
            validateattributes(val, validClasses, validAttributes, 'TopicStatistics', 'WindowStop')
            obj.WindowStop = val;
        end
        function set.PeriodMean(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.Duration'};
            validateattributes(val, validClasses, validAttributes, 'TopicStatistics', 'PeriodMean')
            obj.PeriodMean = val;
        end
        function set.PeriodStddev(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.Duration'};
            validateattributes(val, validClasses, validAttributes, 'TopicStatistics', 'PeriodStddev')
            obj.PeriodStddev = val;
        end
        function set.PeriodMax(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.Duration'};
            validateattributes(val, validClasses, validAttributes, 'TopicStatistics', 'PeriodMax')
            obj.PeriodMax = val;
        end
        function set.StampAgeMean(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.Duration'};
            validateattributes(val, validClasses, validAttributes, 'TopicStatistics', 'StampAgeMean')
            obj.StampAgeMean = val;
        end
        function set.StampAgeStddev(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.Duration'};
            validateattributes(val, validClasses, validAttributes, 'TopicStatistics', 'StampAgeStddev')
            obj.StampAgeStddev = val;
        end
        function set.StampAgeMax(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.Duration'};
            validateattributes(val, validClasses, validAttributes, 'TopicStatistics', 'StampAgeMax')
            obj.StampAgeMax = val;
        end
        function set.Topic(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'TopicStatistics', 'Topic');
            obj.Topic = char(val);
        end
        function set.NodePub(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'TopicStatistics', 'NodePub');
            obj.NodePub = char(val);
        end
        function set.NodeSub(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'TopicStatistics', 'NodeSub');
            obj.NodeSub = char(val);
        end
        function set.DeliveredMsgs(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'TopicStatistics', 'DeliveredMsgs');
            obj.DeliveredMsgs = int32(val);
        end
        function set.DroppedMsgs(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'TopicStatistics', 'DroppedMsgs');
            obj.DroppedMsgs = int32(val);
        end
        function set.Traffic(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'TopicStatistics', 'Traffic');
            obj.Traffic = int32(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.rosgraph_msgs.TopicStatistics.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.rosgraph_msgs.TopicStatistics(strObj);
        end
    end
end
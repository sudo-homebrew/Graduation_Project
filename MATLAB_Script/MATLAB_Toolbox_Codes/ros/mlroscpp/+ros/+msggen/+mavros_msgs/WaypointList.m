
classdef WaypointList < ros.Message
    %WaypointList MATLAB implementation of mavros_msgs/WaypointList
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'mavros_msgs/WaypointList' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '2cacdc0c2c212eb99fdee9f12d2e1fa4' % The MD5 Checksum of the message definition
        PropertyList = { 'Waypoints' 'CurrentSeq' } % List of non-constant message properties
        ROSPropertyList = { 'waypoints' 'current_seq' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.mavros_msgs.Waypoint' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Waypoints
        CurrentSeq
    end
    methods
        function set.Waypoints(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.mavros_msgs.Waypoint.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.mavros_msgs.Waypoint'};
            validateattributes(val, validClasses, validAttributes, 'WaypointList', 'Waypoints')
            obj.Waypoints = val;
        end
        function set.CurrentSeq(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'WaypointList', 'CurrentSeq');
            obj.CurrentSeq = uint16(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.mavros_msgs.WaypointList.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.mavros_msgs.WaypointList(strObj);
        end
    end
end
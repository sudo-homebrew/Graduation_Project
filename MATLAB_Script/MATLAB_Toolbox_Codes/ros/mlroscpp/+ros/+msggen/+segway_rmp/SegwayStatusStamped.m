
classdef SegwayStatusStamped < ros.Message
    %SegwayStatusStamped MATLAB implementation of segway_rmp/SegwayStatusStamped
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'segway_rmp/SegwayStatusStamped' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'f11247ac7dcf2a8717603cc53878eec2' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Segway' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'segway' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.segway_rmp.SegwayStatus' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Segway
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'SegwayStatusStamped', 'Header')
            obj.Header = val;
        end
        function set.Segway(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.segway_rmp.SegwayStatus'};
            validateattributes(val, validClasses, validAttributes, 'SegwayStatusStamped', 'Segway')
            obj.Segway = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.segway_rmp.SegwayStatusStamped.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.segway_rmp.SegwayStatusStamped(strObj);
        end
    end
end
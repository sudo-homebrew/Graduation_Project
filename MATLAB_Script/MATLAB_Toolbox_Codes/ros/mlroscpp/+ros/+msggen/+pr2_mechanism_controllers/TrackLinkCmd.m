
classdef TrackLinkCmd < ros.Message
    %TrackLinkCmd MATLAB implementation of pr2_mechanism_controllers/TrackLinkCmd
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'pr2_mechanism_controllers/TrackLinkCmd' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '08ccfe603e4e21c792896712c3b72de2' % The MD5 Checksum of the message definition
        PropertyList = { 'Point' 'Enable' 'LinkName' } % List of non-constant message properties
        ROSPropertyList = { 'point' 'enable' 'link_name' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.geometry_msgs.Point' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Point
        Enable
        LinkName
    end
    methods
        function set.Point(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'TrackLinkCmd', 'Point')
            obj.Point = val;
        end
        function set.Enable(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'TrackLinkCmd', 'Enable');
            obj.Enable = int8(val);
        end
        function set.LinkName(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'TrackLinkCmd', 'LinkName');
            obj.LinkName = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.pr2_mechanism_controllers.TrackLinkCmd.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.pr2_mechanism_controllers.TrackLinkCmd(strObj);
        end
    end
end

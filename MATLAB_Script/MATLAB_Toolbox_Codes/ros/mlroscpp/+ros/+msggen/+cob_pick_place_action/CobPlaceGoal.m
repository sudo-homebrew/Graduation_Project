
classdef CobPlaceGoal < ros.Message
    %CobPlaceGoal MATLAB implementation of cob_pick_place_action/CobPlaceGoal
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'cob_pick_place_action/CobPlaceGoal' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'eac848fa6d0bba67596f88cde36f673b' % The MD5 Checksum of the message definition
        PropertyList = { 'Destinations' 'ObjectClass' 'ObjectName' 'SupportSurface' } % List of non-constant message properties
        ROSPropertyList = { 'destinations' 'object_class' 'object_name' 'support_surface' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.geometry_msgs.PoseStamped' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Destinations
        ObjectClass
        ObjectName
        SupportSurface
    end
    methods
        function set.Destinations(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.geometry_msgs.PoseStamped.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.geometry_msgs.PoseStamped'};
            validateattributes(val, validClasses, validAttributes, 'CobPlaceGoal', 'Destinations')
            obj.Destinations = val;
        end
        function set.ObjectClass(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'CobPlaceGoal', 'ObjectClass');
            obj.ObjectClass = uint32(val);
        end
        function set.ObjectName(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'CobPlaceGoal', 'ObjectName');
            obj.ObjectName = char(val);
        end
        function set.SupportSurface(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'CobPlaceGoal', 'SupportSurface');
            obj.SupportSurface = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.cob_pick_place_action.CobPlaceGoal.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.cob_pick_place_action.CobPlaceGoal(strObj);
        end
    end
end
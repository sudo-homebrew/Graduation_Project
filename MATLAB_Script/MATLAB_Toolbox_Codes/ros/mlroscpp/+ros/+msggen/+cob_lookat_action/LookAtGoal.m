
classdef LookAtGoal < ros.Message
    %LookAtGoal MATLAB implementation of cob_lookat_action/LookAtGoal
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'cob_lookat_action/LookAtGoal' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '859d6d22351f34337a597d0ad30bf590' % The MD5 Checksum of the message definition
        PropertyList = { 'TargetFrame' 'PointingFrame' 'PointingAxisType' 'BaseActive' } % List of non-constant message properties
        ROSPropertyList = { 'target_frame' 'pointing_frame' 'pointing_axis_type' 'base_active' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
        XPOSITIVE = uint8(0)
        YPOSITIVE = uint8(1)
        ZPOSITIVE = uint8(2)
        XNEGATIVE = uint8(3)
        YNEGATIVE = uint8(4)
        ZNEGATIVE = uint8(5)
    end
    properties
        TargetFrame
        PointingFrame
        PointingAxisType
        BaseActive
    end
    methods
        function set.TargetFrame(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'LookAtGoal', 'TargetFrame');
            obj.TargetFrame = char(val);
        end
        function set.PointingFrame(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'LookAtGoal', 'PointingFrame');
            obj.PointingFrame = char(val);
        end
        function set.PointingAxisType(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'LookAtGoal', 'PointingAxisType');
            obj.PointingAxisType = uint8(val);
        end
        function set.BaseActive(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'LookAtGoal', 'BaseActive');
            obj.BaseActive = logical(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.cob_lookat_action.LookAtGoal.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.cob_lookat_action.LookAtGoal(strObj);
        end
    end
end
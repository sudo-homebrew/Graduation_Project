
classdef KeyValuePair < ros.Message
    %KeyValuePair MATLAB implementation of designator_integration_msgs/KeyValuePair
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'designator_integration_msgs/KeyValuePair' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'bf2b07f354b90ce1746743f304bc36a5' % The MD5 Checksum of the message definition
        PropertyList = { 'ValuePosestamped' 'ValuePose' 'ValuePoint' 'ValueWrench' 'Id' 'Parent' 'Type' 'Key' 'ValueString' 'ValueFloat' 'ValueData' 'ValueArray' } % List of non-constant message properties
        ROSPropertyList = { 'value_posestamped' 'value_pose' 'value_point' 'value_wrench' 'id' 'parent' 'type' 'key' 'value_string' 'value_float' 'value_data' 'value_array' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.geometry_msgs.PoseStamped' ...
            'ros.msggen.geometry_msgs.Pose' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Wrench' ...
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
        TYPESTRING = int32(0)
        TYPEFLOAT = int32(1)
        TYPEDATA = int32(2)
        TYPELIST = int32(3)
        TYPEPOSESTAMPED = int32(4)
        TYPEPOSE = int32(5)
        TYPEDESIGNATORACTION = int32(6)
        TYPEDESIGNATOROBJECT = int32(7)
        TYPEDESIGNATORLOCATION = int32(8)
        TYPEDESIGNATORHUMAN = int32(9)
        TYPEPOINT = int32(10)
        TYPEWRENCH = int32(11)
        TYPEMATRIX = int32(12)
        TYPEVECTOR = int32(13)
    end
    properties
        ValuePosestamped
        ValuePose
        ValuePoint
        ValueWrench
        Id
        Parent
        Type
        Key
        ValueString
        ValueFloat
        ValueData
        ValueArray
    end
    methods
        function set.ValuePosestamped(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.PoseStamped'};
            validateattributes(val, validClasses, validAttributes, 'KeyValuePair', 'ValuePosestamped')
            obj.ValuePosestamped = val;
        end
        function set.ValuePose(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Pose'};
            validateattributes(val, validClasses, validAttributes, 'KeyValuePair', 'ValuePose')
            obj.ValuePose = val;
        end
        function set.ValuePoint(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'KeyValuePair', 'ValuePoint')
            obj.ValuePoint = val;
        end
        function set.ValueWrench(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Wrench'};
            validateattributes(val, validClasses, validAttributes, 'KeyValuePair', 'ValueWrench')
            obj.ValueWrench = val;
        end
        function set.Id(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'KeyValuePair', 'Id');
            obj.Id = int32(val);
        end
        function set.Parent(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'KeyValuePair', 'Parent');
            obj.Parent = int32(val);
        end
        function set.Type(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'KeyValuePair', 'Type');
            obj.Type = int32(val);
        end
        function set.Key(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'KeyValuePair', 'Key');
            obj.Key = char(val);
        end
        function set.ValueString(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'KeyValuePair', 'ValueString');
            obj.ValueString = char(val);
        end
        function set.ValueFloat(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'KeyValuePair', 'ValueFloat');
            obj.ValueFloat = double(val);
        end
        function set.ValueData(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = uint8.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'KeyValuePair', 'ValueData');
            obj.ValueData = uint8(val);
        end
        function set.ValueArray(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'KeyValuePair', 'ValueArray');
            obj.ValueArray = double(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.designator_integration_msgs.KeyValuePair.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.designator_integration_msgs.KeyValuePair(strObj);
        end
    end
end
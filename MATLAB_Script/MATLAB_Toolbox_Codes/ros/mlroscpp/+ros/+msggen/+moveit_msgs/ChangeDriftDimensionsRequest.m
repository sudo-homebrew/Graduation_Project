
classdef ChangeDriftDimensionsRequest < ros.Message
    %ChangeDriftDimensionsRequest MATLAB implementation of moveit_msgs/ChangeDriftDimensionsRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'moveit_msgs/ChangeDriftDimensionsRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '4a5ce44f94cdee672e699df89b1ebaf1' % The MD5 Checksum of the message definition
        PropertyList = { 'TransformJogFrameToDriftFrame' 'DriftXTranslation' 'DriftYTranslation' 'DriftZTranslation' 'DriftXRotation' 'DriftYRotation' 'DriftZRotation' } % List of non-constant message properties
        ROSPropertyList = { 'transform_jog_frame_to_drift_frame' 'drift_x_translation' 'drift_y_translation' 'drift_z_translation' 'drift_x_rotation' 'drift_y_rotation' 'drift_z_rotation' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.geometry_msgs.Transform' ...
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
        TransformJogFrameToDriftFrame
        DriftXTranslation
        DriftYTranslation
        DriftZTranslation
        DriftXRotation
        DriftYRotation
        DriftZRotation
    end
    methods
        function set.TransformJogFrameToDriftFrame(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Transform'};
            validateattributes(val, validClasses, validAttributes, 'ChangeDriftDimensionsRequest', 'TransformJogFrameToDriftFrame')
            obj.TransformJogFrameToDriftFrame = val;
        end
        function set.DriftXTranslation(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ChangeDriftDimensionsRequest', 'DriftXTranslation');
            obj.DriftXTranslation = logical(val);
        end
        function set.DriftYTranslation(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ChangeDriftDimensionsRequest', 'DriftYTranslation');
            obj.DriftYTranslation = logical(val);
        end
        function set.DriftZTranslation(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ChangeDriftDimensionsRequest', 'DriftZTranslation');
            obj.DriftZTranslation = logical(val);
        end
        function set.DriftXRotation(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ChangeDriftDimensionsRequest', 'DriftXRotation');
            obj.DriftXRotation = logical(val);
        end
        function set.DriftYRotation(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ChangeDriftDimensionsRequest', 'DriftYRotation');
            obj.DriftYRotation = logical(val);
        end
        function set.DriftZRotation(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ChangeDriftDimensionsRequest', 'DriftZRotation');
            obj.DriftZRotation = logical(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.moveit_msgs.ChangeDriftDimensionsRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.moveit_msgs.ChangeDriftDimensionsRequest(strObj);
        end
    end
end
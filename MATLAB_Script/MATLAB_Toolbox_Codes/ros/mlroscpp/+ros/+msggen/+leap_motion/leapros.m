
classdef leapros < ros.Message
    %leapros MATLAB implementation of leap_motion/leapros
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'leap_motion/leapros' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'e37447f7532c765d6c587f418fd5dd03' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Direction' 'Normal' 'Palmpos' 'Ypr' 'ThumbMetacarpal' 'ThumbProximal' 'ThumbIntermediate' 'ThumbDistal' 'ThumbTip' 'IndexMetacarpal' 'IndexProximal' 'IndexIntermediate' 'IndexDistal' 'IndexTip' 'MiddleMetacarpal' 'MiddleProximal' 'MiddleIntermediate' 'MiddleDistal' 'MiddleTip' 'RingMetacarpal' 'RingProximal' 'RingIntermediate' 'RingDistal' 'RingTip' 'PinkyMetacarpal' 'PinkyProximal' 'PinkyIntermediate' 'PinkyDistal' 'PinkyTip' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'direction' 'normal' 'palmpos' 'ypr' 'thumb_metacarpal' 'thumb_proximal' 'thumb_intermediate' 'thumb_distal' 'thumb_tip' 'index_metacarpal' 'index_proximal' 'index_intermediate' 'index_distal' 'index_tip' 'middle_metacarpal' 'middle_proximal' 'middle_intermediate' 'middle_distal' 'middle_tip' 'ring_metacarpal' 'ring_proximal' 'ring_intermediate' 'ring_distal' 'ring_tip' 'pinky_metacarpal' 'pinky_proximal' 'pinky_intermediate' 'pinky_distal' 'pinky_tip' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.geometry_msgs.Vector3' ...
            'ros.msggen.geometry_msgs.Vector3' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Vector3' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Point' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Direction
        Normal
        Palmpos
        Ypr
        ThumbMetacarpal
        ThumbProximal
        ThumbIntermediate
        ThumbDistal
        ThumbTip
        IndexMetacarpal
        IndexProximal
        IndexIntermediate
        IndexDistal
        IndexTip
        MiddleMetacarpal
        MiddleProximal
        MiddleIntermediate
        MiddleDistal
        MiddleTip
        RingMetacarpal
        RingProximal
        RingIntermediate
        RingDistal
        RingTip
        PinkyMetacarpal
        PinkyProximal
        PinkyIntermediate
        PinkyDistal
        PinkyTip
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'Header')
            obj.Header = val;
        end
        function set.Direction(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Vector3'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'Direction')
            obj.Direction = val;
        end
        function set.Normal(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Vector3'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'Normal')
            obj.Normal = val;
        end
        function set.Palmpos(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'Palmpos')
            obj.Palmpos = val;
        end
        function set.Ypr(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Vector3'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'Ypr')
            obj.Ypr = val;
        end
        function set.ThumbMetacarpal(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'ThumbMetacarpal')
            obj.ThumbMetacarpal = val;
        end
        function set.ThumbProximal(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'ThumbProximal')
            obj.ThumbProximal = val;
        end
        function set.ThumbIntermediate(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'ThumbIntermediate')
            obj.ThumbIntermediate = val;
        end
        function set.ThumbDistal(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'ThumbDistal')
            obj.ThumbDistal = val;
        end
        function set.ThumbTip(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'ThumbTip')
            obj.ThumbTip = val;
        end
        function set.IndexMetacarpal(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'IndexMetacarpal')
            obj.IndexMetacarpal = val;
        end
        function set.IndexProximal(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'IndexProximal')
            obj.IndexProximal = val;
        end
        function set.IndexIntermediate(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'IndexIntermediate')
            obj.IndexIntermediate = val;
        end
        function set.IndexDistal(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'IndexDistal')
            obj.IndexDistal = val;
        end
        function set.IndexTip(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'IndexTip')
            obj.IndexTip = val;
        end
        function set.MiddleMetacarpal(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'MiddleMetacarpal')
            obj.MiddleMetacarpal = val;
        end
        function set.MiddleProximal(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'MiddleProximal')
            obj.MiddleProximal = val;
        end
        function set.MiddleIntermediate(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'MiddleIntermediate')
            obj.MiddleIntermediate = val;
        end
        function set.MiddleDistal(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'MiddleDistal')
            obj.MiddleDistal = val;
        end
        function set.MiddleTip(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'MiddleTip')
            obj.MiddleTip = val;
        end
        function set.RingMetacarpal(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'RingMetacarpal')
            obj.RingMetacarpal = val;
        end
        function set.RingProximal(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'RingProximal')
            obj.RingProximal = val;
        end
        function set.RingIntermediate(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'RingIntermediate')
            obj.RingIntermediate = val;
        end
        function set.RingDistal(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'RingDistal')
            obj.RingDistal = val;
        end
        function set.RingTip(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'RingTip')
            obj.RingTip = val;
        end
        function set.PinkyMetacarpal(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'PinkyMetacarpal')
            obj.PinkyMetacarpal = val;
        end
        function set.PinkyProximal(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'PinkyProximal')
            obj.PinkyProximal = val;
        end
        function set.PinkyIntermediate(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'PinkyIntermediate')
            obj.PinkyIntermediate = val;
        end
        function set.PinkyDistal(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'PinkyDistal')
            obj.PinkyDistal = val;
        end
        function set.PinkyTip(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'leapros', 'PinkyTip')
            obj.PinkyTip = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.leap_motion.leapros.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.leap_motion.leapros(strObj);
        end
    end
end
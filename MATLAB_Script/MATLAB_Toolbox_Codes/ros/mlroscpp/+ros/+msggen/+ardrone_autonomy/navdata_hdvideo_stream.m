
classdef navdata_hdvideo_stream < ros.Message
    %navdata_hdvideo_stream MATLAB implementation of ardrone_autonomy/navdata_hdvideo_stream
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'ardrone_autonomy/navdata_hdvideo_stream' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '1ba321578916df95f899ca2f5348f234' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'DroneTime' 'Tag' 'Size' 'HdvideoState' 'StorageFifoNbPackets' 'StorageFifoSize' 'UsbkeySize' 'UsbkeyFreespace' 'FrameNumber' 'UsbkeyRemainingTime' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'drone_time' 'tag' 'size' 'hdvideo_state' 'storage_fifo_nb_packets' 'storage_fifo_size' 'usbkey_size' 'usbkey_freespace' 'frame_number' 'usbkey_remaining_time' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
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
    end
    properties
        Header
        DroneTime
        Tag
        Size
        HdvideoState
        StorageFifoNbPackets
        StorageFifoSize
        UsbkeySize
        UsbkeyFreespace
        FrameNumber
        UsbkeyRemainingTime
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'navdata_hdvideo_stream', 'Header')
            obj.Header = val;
        end
        function set.DroneTime(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'navdata_hdvideo_stream', 'DroneTime');
            obj.DroneTime = double(val);
        end
        function set.Tag(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'navdata_hdvideo_stream', 'Tag');
            obj.Tag = uint16(val);
        end
        function set.Size(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'navdata_hdvideo_stream', 'Size');
            obj.Size = uint16(val);
        end
        function set.HdvideoState(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'navdata_hdvideo_stream', 'HdvideoState');
            obj.HdvideoState = uint32(val);
        end
        function set.StorageFifoNbPackets(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'navdata_hdvideo_stream', 'StorageFifoNbPackets');
            obj.StorageFifoNbPackets = uint32(val);
        end
        function set.StorageFifoSize(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'navdata_hdvideo_stream', 'StorageFifoSize');
            obj.StorageFifoSize = uint32(val);
        end
        function set.UsbkeySize(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'navdata_hdvideo_stream', 'UsbkeySize');
            obj.UsbkeySize = uint32(val);
        end
        function set.UsbkeyFreespace(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'navdata_hdvideo_stream', 'UsbkeyFreespace');
            obj.UsbkeyFreespace = uint32(val);
        end
        function set.FrameNumber(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'navdata_hdvideo_stream', 'FrameNumber');
            obj.FrameNumber = uint32(val);
        end
        function set.UsbkeyRemainingTime(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'navdata_hdvideo_stream', 'UsbkeyRemainingTime');
            obj.UsbkeyRemainingTime = uint32(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.ardrone_autonomy.navdata_hdvideo_stream.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.ardrone_autonomy.navdata_hdvideo_stream(strObj);
        end
    end
end

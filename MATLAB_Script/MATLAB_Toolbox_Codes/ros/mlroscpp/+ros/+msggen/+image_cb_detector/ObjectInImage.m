
classdef ObjectInImage < ros.Message
    %ObjectInImage MATLAB implementation of image_cb_detector/ObjectInImage
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'image_cb_detector/ObjectInImage' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '0996b0d8499882526b533fe6e96aa418' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'ModelPoints' 'ImagePoints' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'model_points' 'image_points' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.image_cb_detector.ImagePoint' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        ModelPoints
        ImagePoints
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'ObjectInImage', 'Header')
            obj.Header = val;
        end
        function set.ModelPoints(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.geometry_msgs.Point.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'ObjectInImage', 'ModelPoints')
            obj.ModelPoints = val;
        end
        function set.ImagePoints(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.image_cb_detector.ImagePoint.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.image_cb_detector.ImagePoint'};
            validateattributes(val, validClasses, validAttributes, 'ObjectInImage', 'ImagePoints')
            obj.ImagePoints = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.image_cb_detector.ObjectInImage.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.image_cb_detector.ObjectInImage(strObj);
        end
    end
end

classdef Plot < ros.Message
    %Plot MATLAB implementation of pr2_self_test_msgs/Plot
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'pr2_self_test_msgs/Plot' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '7b2b3d1ff7d1699544a2479e9175f3fb' % The MD5 Checksum of the message definition
        PropertyList = { 'Title' 'Image' 'ImageFormat' } % List of non-constant message properties
        ROSPropertyList = { 'title' 'image' 'image_format' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Title
        Image
        ImageFormat
    end
    methods
        function set.Title(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'Plot', 'Title');
            obj.Title = char(val);
        end
        function set.Image(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = int8.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'Plot', 'Image');
            obj.Image = int8(val);
        end
        function set.ImageFormat(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'Plot', 'ImageFormat');
            obj.ImageFormat = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.pr2_self_test_msgs.Plot.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.pr2_self_test_msgs.Plot(strObj);
        end
    end
end
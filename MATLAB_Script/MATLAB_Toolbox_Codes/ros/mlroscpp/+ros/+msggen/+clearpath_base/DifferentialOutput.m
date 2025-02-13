
classdef DifferentialOutput < ros.Message
    %DifferentialOutput MATLAB implementation of clearpath_base/DifferentialOutput
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'clearpath_base/DifferentialOutput' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '8f32685125452f5bdf68130369af5296' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Left' 'Right' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'left' 'right' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Left
        Right
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'DifferentialOutput', 'Header')
            obj.Header = val;
        end
        function set.Left(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'DifferentialOutput', 'Left');
            obj.Left = double(val);
        end
        function set.Right(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'DifferentialOutput', 'Right');
            obj.Right = double(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.clearpath_base.DifferentialOutput.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.clearpath_base.DifferentialOutput(strObj);
        end
    end
end


classdef Axis < ros.Message
    %Axis MATLAB implementation of robotnik_msgs/Axis
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'robotnik_msgs/Axis' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'e5a99fe291a5635d3f85f17b629d8088' % The MD5 Checksum of the message definition
        PropertyList = { 'Pan' 'Tilt' 'Zoom' 'Focus' 'Brightness' 'Iris' 'Autofocus' 'Autoiris' } % List of non-constant message properties
        ROSPropertyList = { 'pan' 'tilt' 'zoom' 'focus' 'brightness' 'iris' 'autofocus' 'autoiris' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
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
        Pan
        Tilt
        Zoom
        Focus
        Brightness
        Iris
        Autofocus
        Autoiris
    end
    methods
        function set.Pan(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Axis', 'Pan');
            obj.Pan = single(val);
        end
        function set.Tilt(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Axis', 'Tilt');
            obj.Tilt = single(val);
        end
        function set.Zoom(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Axis', 'Zoom');
            obj.Zoom = single(val);
        end
        function set.Focus(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Axis', 'Focus');
            obj.Focus = single(val);
        end
        function set.Brightness(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Axis', 'Brightness');
            obj.Brightness = single(val);
        end
        function set.Iris(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Axis', 'Iris');
            obj.Iris = single(val);
        end
        function set.Autofocus(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Axis', 'Autofocus');
            obj.Autofocus = logical(val);
        end
        function set.Autoiris(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Axis', 'Autoiris');
            obj.Autoiris = logical(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.robotnik_msgs.Axis.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.robotnik_msgs.Axis(strObj);
        end
    end
end

classdef MovingEdgeSite < ros.Message
    %MovingEdgeSite MATLAB implementation of visp_tracker/MovingEdgeSite
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'visp_tracker/MovingEdgeSite' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'd67448def98304944978d0ca12803af8' % The MD5 Checksum of the message definition
        PropertyList = { 'X' 'Y' 'Suppress' } % List of non-constant message properties
        ROSPropertyList = { 'x' 'y' 'suppress' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        X
        Y
        Suppress
    end
    methods
        function set.X(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'MovingEdgeSite', 'X');
            obj.X = double(val);
        end
        function set.Y(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'MovingEdgeSite', 'Y');
            obj.Y = double(val);
        end
        function set.Suppress(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'MovingEdgeSite', 'Suppress');
            obj.Suppress = int32(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.visp_tracker.MovingEdgeSite.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.visp_tracker.MovingEdgeSite(strObj);
        end
    end
end

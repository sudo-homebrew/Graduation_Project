
classdef set_odometryRequest < ros.Message
    %set_odometryRequest MATLAB implementation of robotnik_msgs/set_odometryRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'robotnik_msgs/set_odometryRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'b9cc91561ab081df3c838809b2058a1b' % The MD5 Checksum of the message definition
        PropertyList = { 'X' 'Y' 'Z' 'Orientation' } % List of non-constant message properties
        ROSPropertyList = { 'x' 'y' 'z' 'orientation' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        X
        Y
        Z
        Orientation
    end
    methods
        function set.X(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'set_odometryRequest', 'X');
            obj.X = single(val);
        end
        function set.Y(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'set_odometryRequest', 'Y');
            obj.Y = single(val);
        end
        function set.Z(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'set_odometryRequest', 'Z');
            obj.Z = single(val);
        end
        function set.Orientation(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'set_odometryRequest', 'Orientation');
            obj.Orientation = single(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.robotnik_msgs.set_odometryRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.robotnik_msgs.set_odometryRequest(strObj);
        end
    end
end

classdef SpawnRequest < ros.Message
    %SpawnRequest MATLAB implementation of turtlesim/SpawnRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'turtlesim/SpawnRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '57f001c49ab7b11d699f8606c1f4f7ff' % The MD5 Checksum of the message definition
        PropertyList = { 'X' 'Y' 'Theta' 'Name' } % List of non-constant message properties
        ROSPropertyList = { 'x' 'y' 'theta' 'name' } % List of non-constant ROS message properties
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
        Theta
        Name
    end
    methods
        function set.X(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'SpawnRequest', 'X');
            obj.X = single(val);
        end
        function set.Y(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'SpawnRequest', 'Y');
            obj.Y = single(val);
        end
        function set.Theta(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'SpawnRequest', 'Theta');
            obj.Theta = single(val);
        end
        function set.Name(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'SpawnRequest', 'Name');
            obj.Name = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.turtlesim.SpawnRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.turtlesim.SpawnRequest(strObj);
        end
    end
end
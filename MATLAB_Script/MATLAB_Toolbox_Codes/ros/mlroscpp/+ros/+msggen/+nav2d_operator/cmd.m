
classdef cmd < ros.Message
    %cmd MATLAB implementation of nav2d_operator/cmd
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'nav2d_operator/cmd' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '90c9a043660646e2102f124332ecb8b7' % The MD5 Checksum of the message definition
        PropertyList = { 'Velocity' 'Turn' 'Mode' } % List of non-constant message properties
        ROSPropertyList = { 'Velocity' 'Turn' 'Mode' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Velocity
        Turn
        Mode
    end
    methods
        function set.Velocity(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'cmd', 'Velocity');
            obj.Velocity = double(val);
        end
        function set.Turn(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'cmd', 'Turn');
            obj.Turn = double(val);
        end
        function set.Mode(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'cmd', 'Mode');
            obj.Mode = int8(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.nav2d_operator.cmd.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.nav2d_operator.cmd(strObj);
        end
    end
end
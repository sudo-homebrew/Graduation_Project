
classdef ThrustCommand < ros.Message
    %ThrustCommand MATLAB implementation of hector_uav_msgs/ThrustCommand
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'hector_uav_msgs/ThrustCommand' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'c61da3a8868a8b502eaf0b0abd839f57' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Thrust' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'thrust' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Thrust
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'ThrustCommand', 'Header')
            obj.Header = val;
        end
        function set.Thrust(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ThrustCommand', 'Thrust');
            obj.Thrust = single(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.hector_uav_msgs.ThrustCommand.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.hector_uav_msgs.ThrustCommand(strObj);
        end
    end
end
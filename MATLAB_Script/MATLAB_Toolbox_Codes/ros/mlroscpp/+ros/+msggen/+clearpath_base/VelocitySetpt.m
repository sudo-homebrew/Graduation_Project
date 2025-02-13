
classdef VelocitySetpt < ros.Message
    %VelocitySetpt MATLAB implementation of clearpath_base/VelocitySetpt
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'clearpath_base/VelocitySetpt' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '7484db97a3f5854502dee1b95a48014c' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Trans' 'Rot' 'Accel' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'trans' 'rot' 'accel' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Trans
        Rot
        Accel
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'VelocitySetpt', 'Header')
            obj.Header = val;
        end
        function set.Trans(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'VelocitySetpt', 'Trans');
            obj.Trans = double(val);
        end
        function set.Rot(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'VelocitySetpt', 'Rot');
            obj.Rot = double(val);
        end
        function set.Accel(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'VelocitySetpt', 'Accel');
            obj.Accel = double(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.clearpath_base.VelocitySetpt.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.clearpath_base.VelocitySetpt(strObj);
        end
    end
end

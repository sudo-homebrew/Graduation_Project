
classdef JointControllerState < ros.Message
    %JointControllerState MATLAB implementation of pr2_controllers_msgs/JointControllerState
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'pr2_controllers_msgs/JointControllerState' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'c0d034a7bf20aeb1c37f3eccb7992b69' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'SetPoint' 'ProcessValue' 'ProcessValueDot' 'Error' 'TimeStep' 'Command' 'P' 'I' 'D' 'IClamp' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'set_point' 'process_value' 'process_value_dot' 'error' 'time_step' 'command' 'p' 'i' 'd' 'i_clamp' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            '' ...
            '' ...
            '' ...
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
        Header
        SetPoint
        ProcessValue
        ProcessValueDot
        Error
        TimeStep
        Command
        P
        I
        D
        IClamp
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'JointControllerState', 'Header')
            obj.Header = val;
        end
        function set.SetPoint(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'JointControllerState', 'SetPoint');
            obj.SetPoint = double(val);
        end
        function set.ProcessValue(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'JointControllerState', 'ProcessValue');
            obj.ProcessValue = double(val);
        end
        function set.ProcessValueDot(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'JointControllerState', 'ProcessValueDot');
            obj.ProcessValueDot = double(val);
        end
        function set.Error(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'JointControllerState', 'Error');
            obj.Error = double(val);
        end
        function set.TimeStep(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'JointControllerState', 'TimeStep');
            obj.TimeStep = double(val);
        end
        function set.Command(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'JointControllerState', 'Command');
            obj.Command = double(val);
        end
        function set.P(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'JointControllerState', 'P');
            obj.P = double(val);
        end
        function set.I(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'JointControllerState', 'I');
            obj.I = double(val);
        end
        function set.D(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'JointControllerState', 'D');
            obj.D = double(val);
        end
        function set.IClamp(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'JointControllerState', 'IClamp');
            obj.IClamp = double(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.pr2_controllers_msgs.JointControllerState.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.pr2_controllers_msgs.JointControllerState(strObj);
        end
    end
end
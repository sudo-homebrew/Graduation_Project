
classdef PtuGotoGoal < ros.Message
    %PtuGotoGoal MATLAB implementation of ptu_control/PtuGotoGoal
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'ptu_control/PtuGotoGoal' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '3081c5893ca658a36b6caa436091a00d' % The MD5 Checksum of the message definition
        PropertyList = { 'Pan' 'Tilt' 'PanVel' 'TiltVel' } % List of non-constant message properties
        ROSPropertyList = { 'pan' 'tilt' 'pan_vel' 'tilt_vel' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
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
        PanVel
        TiltVel
    end
    methods
        function set.Pan(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'PtuGotoGoal', 'Pan');
            obj.Pan = single(val);
        end
        function set.Tilt(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'PtuGotoGoal', 'Tilt');
            obj.Tilt = single(val);
        end
        function set.PanVel(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'PtuGotoGoal', 'PanVel');
            obj.PanVel = single(val);
        end
        function set.TiltVel(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'PtuGotoGoal', 'TiltVel');
            obj.TiltVel = single(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.ptu_control.PtuGotoGoal.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.ptu_control.PtuGotoGoal(strObj);
        end
    end
end
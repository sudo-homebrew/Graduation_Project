
classdef EPOSState < ros.Message
    %EPOSState MATLAB implementation of epos_driver/EPOSState
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'epos_driver/EPOSState' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'bd7e8496790385978d37612e81cd065e' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'RawPosition' 'Position' 'RawSpeed' 'Speed' 'Acceleration' 'Current' 'Sync' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'raw_position' 'position' 'raw_speed' 'speed' 'acceleration' 'current' 'sync' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
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
        RawPosition
        Position
        RawSpeed
        Speed
        Acceleration
        Current
        Sync
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'EPOSState', 'Header')
            obj.Header = val;
        end
        function set.RawPosition(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'EPOSState', 'RawPosition');
            obj.RawPosition = int64(val);
        end
        function set.Position(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'EPOSState', 'Position');
            obj.Position = double(val);
        end
        function set.RawSpeed(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'EPOSState', 'RawSpeed');
            obj.RawSpeed = int64(val);
        end
        function set.Speed(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'EPOSState', 'Speed');
            obj.Speed = double(val);
        end
        function set.Acceleration(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'EPOSState', 'Acceleration');
            obj.Acceleration = double(val);
        end
        function set.Current(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'EPOSState', 'Current');
            obj.Current = int16(val);
        end
        function set.Sync(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'EPOSState', 'Sync');
            obj.Sync = logical(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.epos_driver.EPOSState.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.epos_driver.EPOSState(strObj);
        end
    end
end

classdef SEAJointState < ros.Message
    %SEAJointState MATLAB implementation of baxter_core_msgs/SEAJointState
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'baxter_core_msgs/SEAJointState' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'd36406dcbb6d860b1b39c4e28f81352b' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Name' 'CommandedPosition' 'CommandedVelocity' 'CommandedAcceleration' 'CommandedEffort' 'ActualPosition' 'ActualVelocity' 'ActualEffort' 'GravityModelEffort' 'GravityOnly' 'HysteresisModelEffort' 'CrosstalkModelEffort' 'HystState' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'name' 'commanded_position' 'commanded_velocity' 'commanded_acceleration' 'commanded_effort' 'actual_position' 'actual_velocity' 'actual_effort' 'gravity_model_effort' 'gravity_only' 'hysteresis_model_effort' 'crosstalk_model_effort' 'hystState' } % List of non-constant ROS message properties
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
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Name
        CommandedPosition
        CommandedVelocity
        CommandedAcceleration
        CommandedEffort
        ActualPosition
        ActualVelocity
        ActualEffort
        GravityModelEffort
        GravityOnly
        HysteresisModelEffort
        CrosstalkModelEffort
        HystState
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'SEAJointState', 'Header')
            obj.Header = val;
        end
        function set.Name(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'cell', 'string'};
            if isempty(val)
                % Allow empty [] input
                val = cell.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'SEAJointState', 'Name');
            obj.Name = cell(val);
        end
        function set.CommandedPosition(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'SEAJointState', 'CommandedPosition');
            obj.CommandedPosition = double(val);
        end
        function set.CommandedVelocity(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'SEAJointState', 'CommandedVelocity');
            obj.CommandedVelocity = double(val);
        end
        function set.CommandedAcceleration(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'SEAJointState', 'CommandedAcceleration');
            obj.CommandedAcceleration = double(val);
        end
        function set.CommandedEffort(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'SEAJointState', 'CommandedEffort');
            obj.CommandedEffort = double(val);
        end
        function set.ActualPosition(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'SEAJointState', 'ActualPosition');
            obj.ActualPosition = double(val);
        end
        function set.ActualVelocity(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'SEAJointState', 'ActualVelocity');
            obj.ActualVelocity = double(val);
        end
        function set.ActualEffort(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'SEAJointState', 'ActualEffort');
            obj.ActualEffort = double(val);
        end
        function set.GravityModelEffort(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'SEAJointState', 'GravityModelEffort');
            obj.GravityModelEffort = double(val);
        end
        function set.GravityOnly(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'SEAJointState', 'GravityOnly');
            obj.GravityOnly = double(val);
        end
        function set.HysteresisModelEffort(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'SEAJointState', 'HysteresisModelEffort');
            obj.HysteresisModelEffort = double(val);
        end
        function set.CrosstalkModelEffort(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'SEAJointState', 'CrosstalkModelEffort');
            obj.CrosstalkModelEffort = double(val);
        end
        function set.HystState(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'SEAJointState', 'HystState');
            obj.HystState = double(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.baxter_core_msgs.SEAJointState.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.baxter_core_msgs.SEAJointState(strObj);
        end
    end
end

classdef PlugCommand < ros.Message
    %PlugCommand MATLAB implementation of pr2_gazebo_plugins/PlugCommand
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'pr2_gazebo_plugins/PlugCommand' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '852b7035ee3e7fa6390824cf7b7e6dd1' % The MD5 Checksum of the message definition
        PropertyList = { 'AcPresent' 'ChargeRate' 'DischargeRate' 'Charge' } % List of non-constant message properties
        ROSPropertyList = { 'ac_present' 'charge_rate' 'discharge_rate' 'charge' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        AcPresent
        ChargeRate
        DischargeRate
        Charge
    end
    methods
        function set.AcPresent(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'PlugCommand', 'AcPresent');
            obj.AcPresent = logical(val);
        end
        function set.ChargeRate(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'PlugCommand', 'ChargeRate');
            obj.ChargeRate = double(val);
        end
        function set.DischargeRate(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'PlugCommand', 'DischargeRate');
            obj.DischargeRate = double(val);
        end
        function set.Charge(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'PlugCommand', 'Charge');
            obj.Charge = double(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.pr2_gazebo_plugins.PlugCommand.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.pr2_gazebo_plugins.PlugCommand(strObj);
        end
    end
end
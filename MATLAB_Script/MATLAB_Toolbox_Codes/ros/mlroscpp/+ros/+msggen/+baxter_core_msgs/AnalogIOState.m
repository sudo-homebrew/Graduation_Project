
classdef AnalogIOState < ros.Message
    %AnalogIOState MATLAB implementation of baxter_core_msgs/AnalogIOState
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'baxter_core_msgs/AnalogIOState' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '39af371963dc9e4447e91f430c720b33' % The MD5 Checksum of the message definition
        PropertyList = { 'Timestamp' 'Value' 'IsInputOnly' } % List of non-constant message properties
        ROSPropertyList = { 'timestamp' 'value' 'isInputOnly' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msg.Time' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Timestamp
        Value
        IsInputOnly
    end
    methods
        function set.Timestamp(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.Time'};
            validateattributes(val, validClasses, validAttributes, 'AnalogIOState', 'Timestamp')
            obj.Timestamp = val;
        end
        function set.Value(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'AnalogIOState', 'Value');
            obj.Value = double(val);
        end
        function set.IsInputOnly(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'AnalogIOState', 'IsInputOnly');
            obj.IsInputOnly = logical(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.baxter_core_msgs.AnalogIOState.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.baxter_core_msgs.AnalogIOState(strObj);
        end
    end
end

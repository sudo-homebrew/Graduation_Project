
classdef TemperatureArray < ros.Message
    %TemperatureArray MATLAB implementation of schunk_sdh/TemperatureArray
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'schunk_sdh/TemperatureArray' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '0aa09ef71eada777ee697d205df8b8f6' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Name' 'Temperature' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'name' 'temperature' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Name
        Temperature
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'TemperatureArray', 'Header')
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
            validateattributes(val, validClasses, validAttributes, 'TemperatureArray', 'Name');
            obj.Name = cell(val);
        end
        function set.Temperature(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'TemperatureArray', 'Temperature');
            obj.Temperature = double(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.schunk_sdh.TemperatureArray.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.schunk_sdh.TemperatureArray(strObj);
        end
    end
end
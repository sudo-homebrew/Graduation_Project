
classdef PressureArray < ros.Message
    %PressureArray MATLAB implementation of schunk_sdh/PressureArray
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'schunk_sdh/PressureArray' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '552b4f7037a43d9de82fe16651e48e29' % The MD5 Checksum of the message definition
        PropertyList = { 'SensorName' 'CellsX' 'CellsY' 'Pressure' } % List of non-constant message properties
        ROSPropertyList = { 'sensor_name' 'cells_x' 'cells_y' 'pressure' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        SensorName
        CellsX
        CellsY
        Pressure
    end
    methods
        function set.SensorName(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'PressureArray', 'SensorName');
            obj.SensorName = char(val);
        end
        function set.CellsX(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'PressureArray', 'CellsX');
            obj.CellsX = uint16(val);
        end
        function set.CellsY(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'PressureArray', 'CellsY');
            obj.CellsY = uint16(val);
        end
        function set.Pressure(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'PressureArray', 'Pressure');
            obj.Pressure = double(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.schunk_sdh.PressureArray.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.schunk_sdh.PressureArray(strObj);
        end
    end
end

classdef PowerSource < ros.Message
    %PowerSource MATLAB implementation of clearpath_base/PowerSource
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'clearpath_base/PowerSource' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'adbe384d7d69a337a7f2b6bf1d0139cb' % The MD5 Checksum of the message definition
        PropertyList = { 'Charge' 'Capacity' 'Present' 'InUse' 'Description' } % List of non-constant message properties
        ROSPropertyList = { 'charge' 'capacity' 'present' 'in_use' 'description' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Charge
        Capacity
        Present
        InUse
        Description
    end
    methods
        function set.Charge(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'PowerSource', 'Charge');
            obj.Charge = single(val);
        end
        function set.Capacity(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'PowerSource', 'Capacity');
            obj.Capacity = int16(val);
        end
        function set.Present(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'PowerSource', 'Present');
            obj.Present = logical(val);
        end
        function set.InUse(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'PowerSource', 'InUse');
            obj.InUse = logical(val);
        end
        function set.Description(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'PowerSource', 'Description');
            obj.Description = uint8(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.clearpath_base.PowerSource.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.clearpath_base.PowerSource(strObj);
        end
    end
end
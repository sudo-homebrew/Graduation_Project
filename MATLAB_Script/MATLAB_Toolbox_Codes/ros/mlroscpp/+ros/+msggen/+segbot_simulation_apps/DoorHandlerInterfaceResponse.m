
classdef DoorHandlerInterfaceResponse < ros.Message
    %DoorHandlerInterfaceResponse MATLAB implementation of segbot_simulation_apps/DoorHandlerInterfaceResponse
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'segbot_simulation_apps/DoorHandlerInterfaceResponse' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '38b8954d32a849f31d78416b12bff5d1' % The MD5 Checksum of the message definition
        PropertyList = { 'Success' 'Status' } % List of non-constant message properties
        ROSPropertyList = { 'success' 'status' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Success
        Status
    end
    methods
        function set.Success(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'DoorHandlerInterfaceResponse', 'Success');
            obj.Success = logical(val);
        end
        function set.Status(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'DoorHandlerInterfaceResponse', 'Status');
            obj.Status = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.segbot_simulation_apps.DoorHandlerInterfaceResponse.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.segbot_simulation_apps.DoorHandlerInterfaceResponse(strObj);
        end
    end
end
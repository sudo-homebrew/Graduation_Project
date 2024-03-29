
classdef StartControllerResponse < ros.Message
    %StartControllerResponse MATLAB implementation of dynamixel_controllers/StartControllerResponse
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'dynamixel_controllers/StartControllerResponse' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'a4d50a34d34f18de48e2436ff1472594' % The MD5 Checksum of the message definition
        PropertyList = { 'Success' 'Reason' } % List of non-constant message properties
        ROSPropertyList = { 'success' 'reason' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Success
        Reason
    end
    methods
        function set.Success(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'StartControllerResponse', 'Success');
            obj.Success = logical(val);
        end
        function set.Reason(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'StartControllerResponse', 'Reason');
            obj.Reason = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.dynamixel_controllers.StartControllerResponse.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.dynamixel_controllers.StartControllerResponse(strObj);
        end
    end
end

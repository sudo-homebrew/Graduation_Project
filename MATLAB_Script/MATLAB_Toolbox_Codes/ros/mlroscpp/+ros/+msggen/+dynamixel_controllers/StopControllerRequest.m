
classdef StopControllerRequest < ros.Message
    %StopControllerRequest MATLAB implementation of dynamixel_controllers/StopControllerRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'dynamixel_controllers/StopControllerRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'df2b10f2f876d82269ae3fc1e0538e11' % The MD5 Checksum of the message definition
        PropertyList = { 'ControllerName' } % List of non-constant message properties
        ROSPropertyList = { 'controller_name' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        ControllerName
    end
    methods
        function set.ControllerName(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'StopControllerRequest', 'ControllerName');
            obj.ControllerName = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.dynamixel_controllers.StopControllerRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.dynamixel_controllers.StopControllerRequest(strObj);
        end
    end
end
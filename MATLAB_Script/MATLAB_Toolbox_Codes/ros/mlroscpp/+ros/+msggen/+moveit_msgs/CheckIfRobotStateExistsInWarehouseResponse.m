
classdef CheckIfRobotStateExistsInWarehouseResponse < ros.Message
    %CheckIfRobotStateExistsInWarehouseResponse MATLAB implementation of moveit_msgs/CheckIfRobotStateExistsInWarehouseResponse
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'moveit_msgs/CheckIfRobotStateExistsInWarehouseResponse' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'e8c90de4adc1219c86af9c2874c0c1b5' % The MD5 Checksum of the message definition
        PropertyList = { 'Exists' } % List of non-constant message properties
        ROSPropertyList = { 'exists' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Exists
    end
    methods
        function set.Exists(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'CheckIfRobotStateExistsInWarehouseResponse', 'Exists');
            obj.Exists = logical(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.moveit_msgs.CheckIfRobotStateExistsInWarehouseResponse.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.moveit_msgs.CheckIfRobotStateExistsInWarehouseResponse(strObj);
        end
    end
end

classdef VehicleInfoGetResponse < ros.Message
    %VehicleInfoGetResponse MATLAB implementation of mavros_msgs/VehicleInfoGetResponse
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'mavros_msgs/VehicleInfoGetResponse' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '7b33b68f66a6b66456d3bed6fe1dfda0' % The MD5 Checksum of the message definition
        PropertyList = { 'Vehicles' 'Success' } % List of non-constant message properties
        ROSPropertyList = { 'vehicles' 'success' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.mavros_msgs.VehicleInfo' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Vehicles
        Success
    end
    methods
        function set.Vehicles(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.mavros_msgs.VehicleInfo.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.mavros_msgs.VehicleInfo'};
            validateattributes(val, validClasses, validAttributes, 'VehicleInfoGetResponse', 'Vehicles')
            obj.Vehicles = val;
        end
        function set.Success(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'VehicleInfoGetResponse', 'Success');
            obj.Success = logical(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.mavros_msgs.VehicleInfoGetResponse.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.mavros_msgs.VehicleInfoGetResponse(strObj);
        end
    end
end
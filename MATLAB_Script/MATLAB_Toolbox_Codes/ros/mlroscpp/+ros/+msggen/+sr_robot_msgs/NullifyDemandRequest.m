
classdef NullifyDemandRequest < ros.Message
    %NullifyDemandRequest MATLAB implementation of sr_robot_msgs/NullifyDemandRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'sr_robot_msgs/NullifyDemandRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'e776288d81b4da212263a7a8b3f035b3' % The MD5 Checksum of the message definition
        PropertyList = { 'NullifyDemand' } % List of non-constant message properties
        ROSPropertyList = { 'nullify_demand' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        NullifyDemand
    end
    methods
        function set.NullifyDemand(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'NullifyDemandRequest', 'NullifyDemand');
            obj.NullifyDemand = logical(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.sr_robot_msgs.NullifyDemandRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.sr_robot_msgs.NullifyDemandRequest(strObj);
        end
    end
end
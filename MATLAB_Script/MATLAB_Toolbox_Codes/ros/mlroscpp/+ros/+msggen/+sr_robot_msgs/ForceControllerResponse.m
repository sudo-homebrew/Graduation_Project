
classdef ForceControllerResponse < ros.Message
    %ForceControllerResponse MATLAB implementation of sr_robot_msgs/ForceControllerResponse
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'sr_robot_msgs/ForceControllerResponse' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'd3b1a74026985b10664594fe60a8c3eb' % The MD5 Checksum of the message definition
        PropertyList = { 'Configured' } % List of non-constant message properties
        ROSPropertyList = { 'configured' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Configured
    end
    methods
        function set.Configured(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ForceControllerResponse', 'Configured');
            obj.Configured = logical(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.sr_robot_msgs.ForceControllerResponse.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.sr_robot_msgs.ForceControllerResponse(strObj);
        end
    end
end
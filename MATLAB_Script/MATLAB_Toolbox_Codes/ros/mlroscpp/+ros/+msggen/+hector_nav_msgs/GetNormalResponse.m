
classdef GetNormalResponse < ros.Message
    %GetNormalResponse MATLAB implementation of hector_nav_msgs/GetNormalResponse
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'hector_nav_msgs/GetNormalResponse' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '9a5880458dbcd28bf7ed1889c8ac7f8e' % The MD5 Checksum of the message definition
        PropertyList = { 'Normal' } % List of non-constant message properties
        ROSPropertyList = { 'normal' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.geometry_msgs.Vector3' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Normal
    end
    methods
        function set.Normal(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Vector3'};
            validateattributes(val, validClasses, validAttributes, 'GetNormalResponse', 'Normal')
            obj.Normal = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.hector_nav_msgs.GetNormalResponse.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.hector_nav_msgs.GetNormalResponse(strObj);
        end
    end
end

classdef GetDistanceToObstacleRequest < ros.Message
    %GetDistanceToObstacleRequest MATLAB implementation of hector_nav_msgs/GetDistanceToObstacleRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'hector_nav_msgs/GetDistanceToObstacleRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '47dfdbd810b48d0a47b7ad67e4191bcc' % The MD5 Checksum of the message definition
        PropertyList = { 'Point' } % List of non-constant message properties
        ROSPropertyList = { 'point' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.geometry_msgs.PointStamped' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Point
    end
    methods
        function set.Point(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.PointStamped'};
            validateattributes(val, validClasses, validAttributes, 'GetDistanceToObstacleRequest', 'Point')
            obj.Point = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.hector_nav_msgs.GetDistanceToObstacleRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.hector_nav_msgs.GetDistanceToObstacleRequest(strObj);
        end
    end
end
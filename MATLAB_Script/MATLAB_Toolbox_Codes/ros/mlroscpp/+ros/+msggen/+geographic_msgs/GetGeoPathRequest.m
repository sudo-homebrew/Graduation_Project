
classdef GetGeoPathRequest < ros.Message
    %GetGeoPathRequest MATLAB implementation of geographic_msgs/GetGeoPathRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'geographic_msgs/GetGeoPathRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'cad6de11e4ae4ca568785186e1f99f89' % The MD5 Checksum of the message definition
        PropertyList = { 'Start' 'Goal' } % List of non-constant message properties
        ROSPropertyList = { 'start' 'goal' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.geographic_msgs.GeoPoint' ...
            'ros.msggen.geographic_msgs.GeoPoint' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Start
        Goal
    end
    methods
        function set.Start(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geographic_msgs.GeoPoint'};
            validateattributes(val, validClasses, validAttributes, 'GetGeoPathRequest', 'Start')
            obj.Start = val;
        end
        function set.Goal(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geographic_msgs.GeoPoint'};
            validateattributes(val, validClasses, validAttributes, 'GetGeoPathRequest', 'Goal')
            obj.Goal = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.geographic_msgs.GetGeoPathRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.geographic_msgs.GetGeoPathRequest(strObj);
        end
    end
end

classdef PolygonOnEnvironmentResponse < ros.Message
    %PolygonOnEnvironmentResponse MATLAB implementation of jsk_pcl_ros/PolygonOnEnvironmentResponse
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'jsk_pcl_ros/PolygonOnEnvironmentResponse' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '5d3fee08bf23ddff8ab543476a855d3f' % The MD5 Checksum of the message definition
        PropertyList = { 'Result' 'Reason' } % List of non-constant message properties
        ROSPropertyList = { 'result' 'reason' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Result
        Reason
    end
    methods
        function set.Result(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'PolygonOnEnvironmentResponse', 'Result');
            obj.Result = logical(val);
        end
        function set.Reason(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'PolygonOnEnvironmentResponse', 'Reason');
            obj.Reason = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.jsk_pcl_ros.PolygonOnEnvironmentResponse.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.jsk_pcl_ros.PolygonOnEnvironmentResponse(strObj);
        end
    end
end

classdef SetLinkStateResponse < ros.Message
    %SetLinkStateResponse MATLAB implementation of gazebo_msgs/SetLinkStateResponse
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'gazebo_msgs/SetLinkStateResponse' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '2ec6f3eff0161f4257b808b12bc830c2' % The MD5 Checksum of the message definition
        PropertyList = { 'Success' 'StatusMessage' } % List of non-constant message properties
        ROSPropertyList = { 'success' 'status_message' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Success
        StatusMessage
    end
    methods
        function set.Success(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'SetLinkStateResponse', 'Success');
            obj.Success = logical(val);
        end
        function set.StatusMessage(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'SetLinkStateResponse', 'StatusMessage');
            obj.StatusMessage = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.gazebo_msgs.SetLinkStateResponse.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.gazebo_msgs.SetLinkStateResponse(strObj);
        end
    end
end
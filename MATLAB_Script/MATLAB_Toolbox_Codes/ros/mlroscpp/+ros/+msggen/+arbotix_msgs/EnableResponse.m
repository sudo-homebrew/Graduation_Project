
classdef EnableResponse < ros.Message
    %EnableResponse MATLAB implementation of arbotix_msgs/EnableResponse
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'arbotix_msgs/EnableResponse' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '001fde3cab9e313a150416ff09c08ee4' % The MD5 Checksum of the message definition
        PropertyList = { 'State' } % List of non-constant message properties
        ROSPropertyList = { 'state' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        State
    end
    methods
        function set.State(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'EnableResponse', 'State');
            obj.State = logical(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.arbotix_msgs.EnableResponse.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.arbotix_msgs.EnableResponse(strObj);
        end
    end
end

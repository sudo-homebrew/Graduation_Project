
classdef LookAtFeedback < ros.Message
    %LookAtFeedback MATLAB implementation of cob_lookat_action/LookAtFeedback
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'cob_lookat_action/LookAtFeedback' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'cc9c64df4628f5bb500ddeb635768626' % The MD5 Checksum of the message definition
        PropertyList = { 'Status' 'Message' } % List of non-constant message properties
        ROSPropertyList = { 'status' 'message' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Status
        Message
    end
    methods
        function set.Status(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'LookAtFeedback', 'Status');
            obj.Status = logical(val);
        end
        function set.Message(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'LookAtFeedback', 'Message');
            obj.Message = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.cob_lookat_action.LookAtFeedback.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.cob_lookat_action.LookAtFeedback(strObj);
        end
    end
end
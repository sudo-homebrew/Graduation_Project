
classdef Groups < ros.Message
    %Groups MATLAB implementation of applanix_msgs/Groups
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'applanix_msgs/Groups' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '7541f8db49ba60489be5e8f369be32b5' % The MD5 Checksum of the message definition
        PropertyList = { 'Groups_' } % List of non-constant message properties
        ROSPropertyList = { 'groups' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Groups_
    end
    methods
        function set.Groups_(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = uint16.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'Groups', 'Groups_');
            obj.Groups_ = uint16(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.applanix_msgs.Groups.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.applanix_msgs.Groups(strObj);
        end
    end
end

classdef CommonFooter < ros.Message
    %CommonFooter MATLAB implementation of applanix_msgs/CommonFooter
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'applanix_msgs/CommonFooter' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '603956a77085dd6eb1088e0cade415b7' % The MD5 Checksum of the message definition
        PropertyList = { 'Checksum' 'End' } % List of non-constant message properties
        ROSPropertyList = { 'checksum' 'end' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
        END = '$#';
    end
    properties
        Checksum
        End
    end
    methods
        function set.Checksum(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'CommonFooter', 'Checksum');
            obj.Checksum = uint16(val);
        end
        function set.End(obj, val)
            validClasses = {'numeric'};
            val = val(:);
            validAttributes = {'vector', 'numel', 2};
            validateattributes(val, validClasses, validAttributes, 'CommonFooter', 'End');
            obj.End = uint8(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.applanix_msgs.CommonFooter.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.applanix_msgs.CommonFooter(strObj);
        end
    end
end

classdef StringArray < ros.Message
    %StringArray MATLAB implementation of rocon_std_msgs/StringArray
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'rocon_std_msgs/StringArray' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '51789d20146e565223d0963361aecda1' % The MD5 Checksum of the message definition
        PropertyList = { 'Strings' } % List of non-constant message properties
        ROSPropertyList = { 'strings' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Strings
    end
    methods
        function set.Strings(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'cell', 'string'};
            if isempty(val)
                % Allow empty [] input
                val = cell.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'StringArray', 'Strings');
            obj.Strings = cell(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.rocon_std_msgs.StringArray.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.rocon_std_msgs.StringArray(strObj);
        end
    end
end

classdef Num < ros.Message
    %Num MATLAB implementation of rosbridge_library/Num
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'rosbridge_library/Num' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '57d3c40ec3ac3754af76a83e6e73127a' % The MD5 Checksum of the message definition
        PropertyList = { 'Num_' } % List of non-constant message properties
        ROSPropertyList = { 'num' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Num_
    end
    methods
        function set.Num_(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Num', 'Num_');
            obj.Num_ = int64(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.rosbridge_library.Num.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.rosbridge_library.Num(strObj);
        end
    end
end
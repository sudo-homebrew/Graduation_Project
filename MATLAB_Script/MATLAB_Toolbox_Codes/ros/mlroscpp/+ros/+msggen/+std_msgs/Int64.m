
classdef Int64 < ros.Message
    %Int64 MATLAB implementation of std_msgs/Int64
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'std_msgs/Int64' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '34add168574510e6e17f5d23ecc077ef' % The MD5 Checksum of the message definition
        PropertyList = { 'Data' } % List of non-constant message properties
        ROSPropertyList = { 'data' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Data
    end
    methods
        function set.Data(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Int64', 'Data');
            obj.Data = int64(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.std_msgs.Int64.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.std_msgs.Int64(strObj);
        end
    end
end
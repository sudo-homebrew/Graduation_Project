
classdef Float32MultiArray < ros.Message
    %Float32MultiArray MATLAB implementation of std_msgs/Float32MultiArray
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'std_msgs/Float32MultiArray' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '6a40e0ffa6a17a503ac3f8616991b1f6' % The MD5 Checksum of the message definition
        PropertyList = { 'Layout' 'Data' } % List of non-constant message properties
        ROSPropertyList = { 'layout' 'data' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.MultiArrayLayout' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Layout
        Data
    end
    methods
        function set.Layout(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.MultiArrayLayout'};
            validateattributes(val, validClasses, validAttributes, 'Float32MultiArray', 'Layout')
            obj.Layout = val;
        end
        function set.Data(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = single.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'Float32MultiArray', 'Data');
            obj.Data = single(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.std_msgs.Float32MultiArray.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.std_msgs.Float32MultiArray(strObj);
        end
    end
end
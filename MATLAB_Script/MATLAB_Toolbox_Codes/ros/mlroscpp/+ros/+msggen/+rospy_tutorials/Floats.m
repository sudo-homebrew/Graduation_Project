
classdef Floats < ros.Message
    %Floats MATLAB implementation of rospy_tutorials/Floats
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'rospy_tutorials/Floats' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '420cd38b6b071cd49f2970c3e2cee511' % The MD5 Checksum of the message definition
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
            if isempty(val)
                % Allow empty [] input
                val = single.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'Floats', 'Data');
            obj.Data = single(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.rospy_tutorials.Floats.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.rospy_tutorials.Floats(strObj);
        end
    end
end
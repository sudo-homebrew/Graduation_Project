
classdef AssembleScans2Response < ros.Message
    %AssembleScans2Response MATLAB implementation of laser_assembler/AssembleScans2Response
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'laser_assembler/AssembleScans2Response' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '96cec5374164b3b3d1d7ef5d7628a7ed' % The MD5 Checksum of the message definition
        PropertyList = { 'Cloud' } % List of non-constant message properties
        ROSPropertyList = { 'cloud' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msg.sensor_msgs.PointCloud2' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Cloud
    end
    methods
        function set.Cloud(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.sensor_msgs.PointCloud2'};
            validateattributes(val, validClasses, validAttributes, 'AssembleScans2Response', 'Cloud')
            obj.Cloud = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.laser_assembler.AssembleScans2Response.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.laser_assembler.AssembleScans2Response(strObj);
        end
    end
end

classdef DiscoverMastersResponse < ros.Message
    %DiscoverMastersResponse MATLAB implementation of multimaster_msgs_fkie/DiscoverMastersResponse
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'multimaster_msgs_fkie/DiscoverMastersResponse' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'bc7525cc24dd3b880f044a2cdfb95aad' % The MD5 Checksum of the message definition
        PropertyList = { 'Masters' } % List of non-constant message properties
        ROSPropertyList = { 'masters' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.multimaster_msgs_fkie.ROSMaster' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Masters
    end
    methods
        function set.Masters(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.multimaster_msgs_fkie.ROSMaster.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.multimaster_msgs_fkie.ROSMaster'};
            validateattributes(val, validClasses, validAttributes, 'DiscoverMastersResponse', 'Masters')
            obj.Masters = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.multimaster_msgs_fkie.DiscoverMastersResponse.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.multimaster_msgs_fkie.DiscoverMastersResponse(strObj);
        end
    end
end
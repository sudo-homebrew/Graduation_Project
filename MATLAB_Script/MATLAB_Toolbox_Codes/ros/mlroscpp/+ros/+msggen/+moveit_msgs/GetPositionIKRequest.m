
classdef GetPositionIKRequest < ros.Message
    %GetPositionIKRequest MATLAB implementation of moveit_msgs/GetPositionIKRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'moveit_msgs/GetPositionIKRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '8388b54598336654bca82763f918a740' % The MD5 Checksum of the message definition
        PropertyList = { 'IkRequest' } % List of non-constant message properties
        ROSPropertyList = { 'ik_request' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.moveit_msgs.PositionIKRequest' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        IkRequest
    end
    methods
        function set.IkRequest(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.moveit_msgs.PositionIKRequest'};
            validateattributes(val, validClasses, validAttributes, 'GetPositionIKRequest', 'IkRequest')
            obj.IkRequest = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.moveit_msgs.GetPositionIKRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.moveit_msgs.GetPositionIKRequest(strObj);
        end
    end
end
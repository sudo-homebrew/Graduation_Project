
classdef SetComplianceMarginRequest < ros.Message
    %SetComplianceMarginRequest MATLAB implementation of dynamixel_controllers/SetComplianceMarginRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'dynamixel_controllers/SetComplianceMarginRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'daacbf1c0642fe923f2dfb9217a97b81' % The MD5 Checksum of the message definition
        PropertyList = { 'Margin' } % List of non-constant message properties
        ROSPropertyList = { 'margin' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Margin
    end
    methods
        function set.Margin(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'SetComplianceMarginRequest', 'Margin');
            obj.Margin = uint8(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.dynamixel_controllers.SetComplianceMarginRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.dynamixel_controllers.SetComplianceMarginRequest(strObj);
        end
    end
end
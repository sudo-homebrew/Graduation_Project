
classdef DesignatorResponse < ros.Message
    %DesignatorResponse MATLAB implementation of designator_integration_msgs/DesignatorResponse
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'designator_integration_msgs/DesignatorResponse' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'ebb08694894d8b049985faec7ee53bd1' % The MD5 Checksum of the message definition
        PropertyList = { 'Designators' } % List of non-constant message properties
        ROSPropertyList = { 'designators' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.designator_integration_msgs.Designator' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Designators
    end
    methods
        function set.Designators(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.designator_integration_msgs.Designator.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.designator_integration_msgs.Designator'};
            validateattributes(val, validClasses, validAttributes, 'DesignatorResponse', 'Designators')
            obj.Designators = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.designator_integration_msgs.DesignatorResponse.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.designator_integration_msgs.DesignatorResponse(strObj);
        end
    end
end
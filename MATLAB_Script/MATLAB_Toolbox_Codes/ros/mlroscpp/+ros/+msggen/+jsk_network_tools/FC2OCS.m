
classdef FC2OCS < ros.Message
    %FC2OCS MATLAB implementation of jsk_network_tools/FC2OCS
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'jsk_network_tools/FC2OCS' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '7a556e2b1084dcaa36eeac7b2f905853' % The MD5 Checksum of the message definition
        PropertyList = { 'JointAngles' 'LhandForce' 'RhandForce' 'LfootForce' 'RfootForce' 'ServoState' } % List of non-constant message properties
        ROSPropertyList = { 'joint_angles' 'lhand_force' 'rhand_force' 'lfoot_force' 'rfoot_force' 'servo_state' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        JointAngles
        LhandForce
        RhandForce
        LfootForce
        RfootForce
        ServoState
    end
    methods
        function set.JointAngles(obj, val)
            validClasses = {'numeric'};
            val = val(:);
            validAttributes = {'vector', 'numel', 32};
            validateattributes(val, validClasses, validAttributes, 'FC2OCS', 'JointAngles');
            obj.JointAngles = uint8(val);
        end
        function set.LhandForce(obj, val)
            validClasses = {'numeric'};
            val = val(:);
            validAttributes = {'vector', 'numel', 6};
            validateattributes(val, validClasses, validAttributes, 'FC2OCS', 'LhandForce');
            obj.LhandForce = uint8(val);
        end
        function set.RhandForce(obj, val)
            validClasses = {'numeric'};
            val = val(:);
            validAttributes = {'vector', 'numel', 6};
            validateattributes(val, validClasses, validAttributes, 'FC2OCS', 'RhandForce');
            obj.RhandForce = uint8(val);
        end
        function set.LfootForce(obj, val)
            validClasses = {'numeric'};
            val = val(:);
            validAttributes = {'vector', 'numel', 6};
            validateattributes(val, validClasses, validAttributes, 'FC2OCS', 'LfootForce');
            obj.LfootForce = uint8(val);
        end
        function set.RfootForce(obj, val)
            validClasses = {'numeric'};
            val = val(:);
            validAttributes = {'vector', 'numel', 6};
            validateattributes(val, validClasses, validAttributes, 'FC2OCS', 'RfootForce');
            obj.RfootForce = uint8(val);
        end
        function set.ServoState(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'FC2OCS', 'ServoState');
            obj.ServoState = logical(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.jsk_network_tools.FC2OCS.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.jsk_network_tools.FC2OCS(strObj);
        end
    end
end
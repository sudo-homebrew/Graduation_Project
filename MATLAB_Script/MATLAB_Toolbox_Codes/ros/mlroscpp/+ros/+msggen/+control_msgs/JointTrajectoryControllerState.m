
classdef JointTrajectoryControllerState < ros.Message
    %JointTrajectoryControllerState MATLAB implementation of control_msgs/JointTrajectoryControllerState
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'control_msgs/JointTrajectoryControllerState' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '10817c60c2486ef6b33e97dcd87f4474' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Desired' 'Actual' 'Error' 'JointNames' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'desired' 'actual' 'error' 'joint_names' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.trajectory_msgs.JointTrajectoryPoint' ...
            'ros.msggen.trajectory_msgs.JointTrajectoryPoint' ...
            'ros.msggen.trajectory_msgs.JointTrajectoryPoint' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Desired
        Actual
        Error
        JointNames
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'JointTrajectoryControllerState', 'Header')
            obj.Header = val;
        end
        function set.Desired(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.trajectory_msgs.JointTrajectoryPoint'};
            validateattributes(val, validClasses, validAttributes, 'JointTrajectoryControllerState', 'Desired')
            obj.Desired = val;
        end
        function set.Actual(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.trajectory_msgs.JointTrajectoryPoint'};
            validateattributes(val, validClasses, validAttributes, 'JointTrajectoryControllerState', 'Actual')
            obj.Actual = val;
        end
        function set.Error(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.trajectory_msgs.JointTrajectoryPoint'};
            validateattributes(val, validClasses, validAttributes, 'JointTrajectoryControllerState', 'Error')
            obj.Error = val;
        end
        function set.JointNames(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'cell', 'string'};
            if isempty(val)
                % Allow empty [] input
                val = cell.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'JointTrajectoryControllerState', 'JointNames');
            obj.JointNames = cell(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.control_msgs.JointTrajectoryControllerState.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.control_msgs.JointTrajectoryControllerState(strObj);
        end
    end
end
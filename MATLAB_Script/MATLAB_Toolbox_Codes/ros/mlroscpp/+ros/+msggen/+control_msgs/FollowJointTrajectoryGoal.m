
classdef FollowJointTrajectoryGoal < ros.Message
    %FollowJointTrajectoryGoal MATLAB implementation of control_msgs/FollowJointTrajectoryGoal
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'control_msgs/FollowJointTrajectoryGoal' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '69636787b6ecbde4d61d711979bc7ecb' % The MD5 Checksum of the message definition
        PropertyList = { 'Trajectory' 'PathTolerance' 'GoalTolerance' 'GoalTimeTolerance' } % List of non-constant message properties
        ROSPropertyList = { 'trajectory' 'path_tolerance' 'goal_tolerance' 'goal_time_tolerance' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.trajectory_msgs.JointTrajectory' ...
            'ros.msggen.control_msgs.JointTolerance' ...
            'ros.msggen.control_msgs.JointTolerance' ...
            'ros.msg.Duration' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Trajectory
        PathTolerance
        GoalTolerance
        GoalTimeTolerance
    end
    methods
        function set.Trajectory(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.trajectory_msgs.JointTrajectory'};
            validateattributes(val, validClasses, validAttributes, 'FollowJointTrajectoryGoal', 'Trajectory')
            obj.Trajectory = val;
        end
        function set.PathTolerance(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.control_msgs.JointTolerance.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.control_msgs.JointTolerance'};
            validateattributes(val, validClasses, validAttributes, 'FollowJointTrajectoryGoal', 'PathTolerance')
            obj.PathTolerance = val;
        end
        function set.GoalTolerance(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.control_msgs.JointTolerance.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.control_msgs.JointTolerance'};
            validateattributes(val, validClasses, validAttributes, 'FollowJointTrajectoryGoal', 'GoalTolerance')
            obj.GoalTolerance = val;
        end
        function set.GoalTimeTolerance(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.Duration'};
            validateattributes(val, validClasses, validAttributes, 'FollowJointTrajectoryGoal', 'GoalTimeTolerance')
            obj.GoalTimeTolerance = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.control_msgs.FollowJointTrajectoryGoal.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.control_msgs.FollowJointTrajectoryGoal(strObj);
        end
    end
end
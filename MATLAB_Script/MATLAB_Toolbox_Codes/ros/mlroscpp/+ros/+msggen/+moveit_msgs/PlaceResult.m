
classdef PlaceResult < ros.Message
    %PlaceResult MATLAB implementation of moveit_msgs/PlaceResult
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'moveit_msgs/PlaceResult' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '978b357573c8ba8c31923f06ac62d8de' % The MD5 Checksum of the message definition
        PropertyList = { 'ErrorCode' 'TrajectoryStart' 'TrajectoryStages' 'PlaceLocation' 'TrajectoryDescriptions' 'PlanningTime' } % List of non-constant message properties
        ROSPropertyList = { 'error_code' 'trajectory_start' 'trajectory_stages' 'place_location' 'trajectory_descriptions' 'planning_time' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.moveit_msgs.MoveItErrorCodes' ...
            'ros.msggen.moveit_msgs.RobotState' ...
            'ros.msggen.moveit_msgs.RobotTrajectory' ...
            'ros.msggen.moveit_msgs.PlaceLocation' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        ErrorCode
        TrajectoryStart
        TrajectoryStages
        PlaceLocation
        TrajectoryDescriptions
        PlanningTime
    end
    methods
        function set.ErrorCode(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.moveit_msgs.MoveItErrorCodes'};
            validateattributes(val, validClasses, validAttributes, 'PlaceResult', 'ErrorCode')
            obj.ErrorCode = val;
        end
        function set.TrajectoryStart(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.moveit_msgs.RobotState'};
            validateattributes(val, validClasses, validAttributes, 'PlaceResult', 'TrajectoryStart')
            obj.TrajectoryStart = val;
        end
        function set.TrajectoryStages(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.moveit_msgs.RobotTrajectory.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.moveit_msgs.RobotTrajectory'};
            validateattributes(val, validClasses, validAttributes, 'PlaceResult', 'TrajectoryStages')
            obj.TrajectoryStages = val;
        end
        function set.PlaceLocation(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.moveit_msgs.PlaceLocation'};
            validateattributes(val, validClasses, validAttributes, 'PlaceResult', 'PlaceLocation')
            obj.PlaceLocation = val;
        end
        function set.TrajectoryDescriptions(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'cell', 'string'};
            if isempty(val)
                % Allow empty [] input
                val = cell.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'PlaceResult', 'TrajectoryDescriptions');
            obj.TrajectoryDescriptions = cell(val);
        end
        function set.PlanningTime(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'PlaceResult', 'PlanningTime');
            obj.PlanningTime = double(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.moveit_msgs.PlaceResult.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.moveit_msgs.PlaceResult(strObj);
        end
    end
end
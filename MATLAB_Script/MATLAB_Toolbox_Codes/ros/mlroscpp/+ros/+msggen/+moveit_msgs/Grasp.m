
classdef Grasp < ros.Message
    %Grasp MATLAB implementation of moveit_msgs/Grasp
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'moveit_msgs/Grasp' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'e26c8fb64f589c33c5d5e54bd7b5e4cb' % The MD5 Checksum of the message definition
        PropertyList = { 'PreGraspPosture' 'GraspPosture' 'GraspPose' 'PreGraspApproach' 'PostGraspRetreat' 'PostPlaceRetreat' 'Id' 'GraspQuality' 'MaxContactForce' 'AllowedTouchObjects' } % List of non-constant message properties
        ROSPropertyList = { 'pre_grasp_posture' 'grasp_posture' 'grasp_pose' 'pre_grasp_approach' 'post_grasp_retreat' 'post_place_retreat' 'id' 'grasp_quality' 'max_contact_force' 'allowed_touch_objects' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.trajectory_msgs.JointTrajectory' ...
            'ros.msggen.trajectory_msgs.JointTrajectory' ...
            'ros.msggen.geometry_msgs.PoseStamped' ...
            'ros.msggen.moveit_msgs.GripperTranslation' ...
            'ros.msggen.moveit_msgs.GripperTranslation' ...
            'ros.msggen.moveit_msgs.GripperTranslation' ...
            '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        PreGraspPosture
        GraspPosture
        GraspPose
        PreGraspApproach
        PostGraspRetreat
        PostPlaceRetreat
        Id
        GraspQuality
        MaxContactForce
        AllowedTouchObjects
    end
    methods
        function set.PreGraspPosture(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.trajectory_msgs.JointTrajectory'};
            validateattributes(val, validClasses, validAttributes, 'Grasp', 'PreGraspPosture')
            obj.PreGraspPosture = val;
        end
        function set.GraspPosture(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.trajectory_msgs.JointTrajectory'};
            validateattributes(val, validClasses, validAttributes, 'Grasp', 'GraspPosture')
            obj.GraspPosture = val;
        end
        function set.GraspPose(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.PoseStamped'};
            validateattributes(val, validClasses, validAttributes, 'Grasp', 'GraspPose')
            obj.GraspPose = val;
        end
        function set.PreGraspApproach(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.moveit_msgs.GripperTranslation'};
            validateattributes(val, validClasses, validAttributes, 'Grasp', 'PreGraspApproach')
            obj.PreGraspApproach = val;
        end
        function set.PostGraspRetreat(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.moveit_msgs.GripperTranslation'};
            validateattributes(val, validClasses, validAttributes, 'Grasp', 'PostGraspRetreat')
            obj.PostGraspRetreat = val;
        end
        function set.PostPlaceRetreat(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.moveit_msgs.GripperTranslation'};
            validateattributes(val, validClasses, validAttributes, 'Grasp', 'PostPlaceRetreat')
            obj.PostPlaceRetreat = val;
        end
        function set.Id(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'Grasp', 'Id');
            obj.Id = char(val);
        end
        function set.GraspQuality(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Grasp', 'GraspQuality');
            obj.GraspQuality = double(val);
        end
        function set.MaxContactForce(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Grasp', 'MaxContactForce');
            obj.MaxContactForce = single(val);
        end
        function set.AllowedTouchObjects(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'cell', 'string'};
            if isempty(val)
                % Allow empty [] input
                val = cell.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'Grasp', 'AllowedTouchObjects');
            obj.AllowedTouchObjects = cell(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.moveit_msgs.Grasp.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.moveit_msgs.Grasp(strObj);
        end
    end
end
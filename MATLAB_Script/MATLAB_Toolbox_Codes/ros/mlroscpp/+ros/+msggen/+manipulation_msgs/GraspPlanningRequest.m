
classdef GraspPlanningRequest < ros.Message
    %GraspPlanningRequest MATLAB implementation of manipulation_msgs/GraspPlanningRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'manipulation_msgs/GraspPlanningRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '077dca08a07415d82d6ab047890161f4' % The MD5 Checksum of the message definition
        PropertyList = { 'Target' 'GraspsToEvaluate' 'MovableObstacles' 'ArmName' 'CollisionObjectName' 'CollisionSupportSurfaceName' } % List of non-constant message properties
        ROSPropertyList = { 'target' 'grasps_to_evaluate' 'movable_obstacles' 'arm_name' 'collision_object_name' 'collision_support_surface_name' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.manipulation_msgs.GraspableObject' ...
            'ros.msggen.manipulation_msgs.Grasp' ...
            'ros.msggen.manipulation_msgs.GraspableObject' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Target
        GraspsToEvaluate
        MovableObstacles
        ArmName
        CollisionObjectName
        CollisionSupportSurfaceName
    end
    methods
        function set.Target(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.manipulation_msgs.GraspableObject'};
            validateattributes(val, validClasses, validAttributes, 'GraspPlanningRequest', 'Target')
            obj.Target = val;
        end
        function set.GraspsToEvaluate(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.manipulation_msgs.Grasp.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.manipulation_msgs.Grasp'};
            validateattributes(val, validClasses, validAttributes, 'GraspPlanningRequest', 'GraspsToEvaluate')
            obj.GraspsToEvaluate = val;
        end
        function set.MovableObstacles(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.manipulation_msgs.GraspableObject.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.manipulation_msgs.GraspableObject'};
            validateattributes(val, validClasses, validAttributes, 'GraspPlanningRequest', 'MovableObstacles')
            obj.MovableObstacles = val;
        end
        function set.ArmName(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'GraspPlanningRequest', 'ArmName');
            obj.ArmName = char(val);
        end
        function set.CollisionObjectName(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'GraspPlanningRequest', 'CollisionObjectName');
            obj.CollisionObjectName = char(val);
        end
        function set.CollisionSupportSurfaceName(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'GraspPlanningRequest', 'CollisionSupportSurfaceName');
            obj.CollisionSupportSurfaceName = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.manipulation_msgs.GraspPlanningRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.manipulation_msgs.GraspPlanningRequest(strObj);
        end
    end
end
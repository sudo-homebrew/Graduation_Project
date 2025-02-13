
classdef QueryGraspsResult < ros.Message
    %QueryGraspsResult MATLAB implementation of cob_grasp_generation/QueryGraspsResult
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'cob_grasp_generation/QueryGraspsResult' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'fda0067ecf92d75eb0f0b5f6c25d3d37' % The MD5 Checksum of the message definition
        PropertyList = { 'GraspList' 'Success' } % List of non-constant message properties
        ROSPropertyList = { 'grasp_list' 'success' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.moveit_msgs.Grasp' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        GraspList
        Success
    end
    methods
        function set.GraspList(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.moveit_msgs.Grasp.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.moveit_msgs.Grasp'};
            validateattributes(val, validClasses, validAttributes, 'QueryGraspsResult', 'GraspList')
            obj.GraspList = val;
        end
        function set.Success(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'QueryGraspsResult', 'Success');
            obj.Success = logical(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.cob_grasp_generation.QueryGraspsResult.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.cob_grasp_generation.QueryGraspsResult(strObj);
        end
    end
end

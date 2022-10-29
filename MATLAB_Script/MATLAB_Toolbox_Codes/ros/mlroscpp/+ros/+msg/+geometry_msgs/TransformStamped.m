classdef TransformStamped < ros.msggen.geometry_msgs.TransformStamped
%TransformStamped Custom MATLAB implementation of geometry_msgs/TransformStamped type
%   This class adds functionality for applying the transformation that
%   is represented by the TransformStamped message.
%
%   TransformStamped methods:
%      apply    - Transform message entities

%   Copyright 2014-2020 The MathWorks, Inc.

    methods
        function obj = TransformStamped(varargin)
        %TransformStamped Constructor
        %   The arguments feed straight into the generated TransformStamped class.

            obj@ros.msggen.geometry_msgs.TransformStamped(varargin{:});
        end

        function tfEntity = apply(obj, entity)
        %APPLY Apply transform to message entities
        %   TFENTITY = APPLY(OBJ, ENTITY) applies the
        %   transformation represented by the TransformStamped object
        %   OBJ to the input ENTITY. ENTITY is a ROS
        %   message of a specific type and the transformed message will
        %   be returned in TFENTITY.
        %
        %   This function will determine the type of the input message
        %   ENTITY and apply the appropriate transformation method. If a
        %   particular message type cannot be handled by this object,
        %   an error will be displayed.
        %
        %   Supported message types include:
        %    - geometry_msgs/QuaternionStamped
        %    - geometry_msgs/Vector3Stamped
        %    - geometry_msgs/PointStamped
        %    - geometry_msgs/PoseStamped
        %    - sensor_msgs/PointCloud2

        % The TransformHelper class will validate the input
            th = ros.internal.TransformHelper(obj);
            tfEntity = th.transform(entity);
        end
    end

    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file

        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msg.geometry_msgs.TransformStamped.empty(0,1);
                return
            end

            % Create an empty object
            obj = ros.msg.geometry_msgs.TransformStamped(strObj);
        end
    end
end

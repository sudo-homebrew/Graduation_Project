function tfEntity = rosApplyTransform(tfmsg, entity)
%rosApplyTransform Apply transform to message entities
%   rosApplyTransform(TFMSG, ENTITY) applies the transformation
%   represented by the TransformStamped message struct TFMSG to the input
%   ENTITY. ENTITY is a ROS message struct of a specific type and the
%   transformed message will be returned in TFENTITY.
%
%   This function will determine the type of the input message ENTITY and
%   apply the appropriate transformation method. If a particular message
%   type cannot be handled by this function, an error will be displayed.
%
%   Supported message types include:
%    - geometry_msgs/QuaternionStamped
%    - geometry_msgs/Vector3Stamped
%    - geometry_msgs/PointStamped
%    - geometry_msgs/PoseStamped
%    - sensor_msgs/PointCloud2
%
%   Example:
%      % Create a TransformStamped message
%      % tformmsg is assumed to be a ROS message struct with type geometry_msgs/TransformStamped
%      tformmsg = rosmessage("geometry_msgs/TransformStamped","DataFormat","struct");
%      tformmsg.Transform.Rotation.W = 1.0;
%
%      % Create an input point message
%      pt = rosmessage("geometry_msgs/PointStamped","DataFormat","struct");
%      pt.Point.X = 5;
%      pt.Point.Z = 2.4;
%
%      % Transform the point
%      trpt = rosApplyTransform(tformmsg, pt);

%#codegen

%   Copyright 2020 The MathWorks, Inc.

% Ensure the generated code is not inlined
    coder.inline('never');

    validateattributes(tfmsg, {'struct'},{'scalar'},'rosApplyTransform','tfmsg');
    validateattributes(entity,{'struct'},{'scalar'},'rosApplyTransform','entity');

    % Create transform utility object based on the type of transform message
    tfUtil = ros.internal.TransformUtil.getInstance(tfmsg);
    % Set the frame id from transform msg, tfmsg
    entity = setHeaderFrameId(tfUtil, entity);

    % Based on the message type, call the appropriate transformation
    % function
    switch entity.MessageType
      case {'geometry_msgs/QuaternionStamped'}
        entity = transformQuaternion(tfUtil, entity);
      case {'geometry_msgs/Vector3Stamped'}
        entity = transformVector3(tfUtil, entity);
      case {'geometry_msgs/PointStamped'}
        entity = transformPoint(tfUtil, entity);
      case {'geometry_msgs/PoseStamped'}
        entity = transformPose(tfUtil, entity);
      case {'sensor_msgs/PointCloud2'}
        entity = transformPointCloud2(tfUtil, entity);
      otherwise
        coder.internal.assert(false,'ros:mlros:tf:MessageTypeNotSupported', ...
                              entity.MessageType, ['QuaternionStamped, Vector3Stamped, PointStamped, ', ...
                            'PoseStamped, PointCloud2']);
    end
    tfEntity = entity;
end

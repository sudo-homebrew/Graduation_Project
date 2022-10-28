function q = rosReadQuaternion(msg)
%rosReadQuaternion Returns the quaternion from a ROS/ROS 2 message struct
%   Q = rosReadQuaternion(MSG) returns a quaternion, Q,
%   represented by a quaternion object, from the quaternion
%   message struct, MSG.
%
%   Example:
%       % Create a geometry_msgs/Quaternion message struct
%       msg = rosmessage("geometry_msgs/Quaternion","DataFormat","struct");
%       msg.X = 1;
%       msg.Y = 3;
%       msg.Z = 6;
%       msg.W = 9;
%
%       % Read quaternion from the message
%       q = rosReadQuaternion(msg);
%
%   See also QUATERNION.

%   Copyright 2020 The MathWorks, Inc.
%#codegen

% Ensure the generated code is not inlined
    coder.inline('never');

    % Validate Input argument
    validateattributes(msg, {'struct'},{'scalar'},'rosReadQuaternion');

    % Code generation only supports geometry_msgs/Quaternion as input message
    if ~isempty(coder.target)
        coder.internal.assert(strcmp(msg.MessageType,'geometry_msgs/Quaternion'),...
                              'ros:mlroscpp:codegen:InvalidMsgForSpMsgFun',...
                              'rosReadQuaternion','geometry_msgs/Quaternion');
    end

    % Get binary map from input message
    if isfield(msg, 'X')
        % ROS message struct
        specialMsgUtil = ros.internal.SpecialMsgUtil;
    else
        % ROS 2 message struct
        specialMsgUtil = ros.internal.ros2.SpecialMsgUtil;
    end
    q = specialMsgUtil.readQuaternion(msg);
end

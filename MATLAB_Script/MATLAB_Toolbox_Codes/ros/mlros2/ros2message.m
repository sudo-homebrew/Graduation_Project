function msg = ros2message(messageType)
%ROS2MESSAGE Create ROS 2 message structures
%   MSG = ROS2MESSAGE("MESSAGETYPE") creates a structure compatible with
%   ROS 2 messages of type MESSAGETYPE.
%
%   MSG = ROS2MESSAGE(OBJ) creates and returns an empty message
%   structure MSG compatible with OBJ, which can be a ros2publisher,
%   ros2subscriber, ros2svcserver, or ros2svcclient.
%
%   MSG = ROS2MESSAGE(SUB) creates and returns an empty message
%   structure MSG compatible with ros2subscriber SUB.
%
%   Example:
%      % Create a pose message
%      poseMsg = ROS2MESSAGE("geometry_msgs/Pose2D")
%
%      % Create a ROS 2 node
%      node = ros2node("/node_1");
%
%      % Create publisher and compatible message
%      chatPub = ros2publisher(node,"/chatter","std_msgs/String");
%      chatMsg = ros2message(chatPub);
%
%      % Create subscriber and compatible message
%      laserSub = ros2subscriber(node,"/scan","sensor_msgs/LaserScan");
%      laserMsg = ros2message(laserSub);
%
%   See also ROS2.

%   Copyright 2019-2021 The MathWorks, Inc.
%#codegen
    coder.extrinsic(...
        'ros.codertarget.internal.getEmptyCodegenMsg',...
        'rostype.getServiceList');
    coder.internal.narginchk(1, 1, nargin) % narginchk(1, 1)
    messageType = convertStringsToChars(messageType);
    validateattributes(messageType, {'char', 'string'}, {'nonempty'}, ...
                       'ros2message', 'messageType');

    if isempty(coder.target)
        % Interpreted execution in MATLAB
        try
            % First make assumption message is requested
            msg = ros.internal.getEmptyMessage(messageType,'ros2');
        catch ex
            if strcmp(ex.identifier, 'ros:utilities:message:MessageNotFoundError')
                % If message not found, see if it may be a service
                try
                    msg = ros.internal.getEmptyMessage(strcat(messageType,'Request'),'ros2');
                catch
                    % If it is not a service either, rethrow original error
                    rethrow(ex)
                end
            else
                rethrow(ex)
            end
        end
    else
        % Codegen
        msgStructGenFcnName = coder.const(@ros.codertarget.internal.getEmptyCodegenMsg,messageType,'ros2');
        msgStructGenFcn = str2func(msgStructGenFcnName);
        msg = msgStructGenFcn();
    end
end

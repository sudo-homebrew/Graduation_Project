function map = rosReadOccupancyMap3D(msg)
%rosReadOccupancyMap3D Return an occupancyMap3D object from ROS/ROS 2 message struct
%   MAP = rosReadOccupancyMap3D(MSG) returns an occupancyMap3D object
%   MAP, reading the data from the MSG. Any binary or full octomap
%   message struct can be converted into a MAP containing probabilities as
%   per binary/full data on the MSG.
%   NOTE: ColorOcTree messages are not supported.

%   Example:
%       % Create a octomap_msgs/Octomap message struct
%       msg = rosmessage("octomap_msgs/Octomap","DataFormat","struct");
%
%       % Populate the ROS octomap message
%       msg.Id = 'OcTree';
%       msg.Resolution = 0.1;
%       msg.Data = int8(100*rand(100,1));
%
%       % Read the msg data and convert to occupancyMap3D
%       map = rosReadOccupancyMap3D(msg);

%   Copyright 2020 The MathWorks, Inc.


% Validate that input message is a single message struct
    validateattributes(msg, {'struct'},{'scalar'},'rosReadOccupancyMap3D');

    % Get map from input message
    if isfield(msg, 'Data')
        % ROS message struct
        specialMsgUtil = ros.internal.SpecialMsgUtil;
    else
        % ROS 2 message struct
        specialMsgUtil = ros.internal.ros2.SpecialMsgUtil;
    end
    map = specialMsgUtil.readOccupancyMap3D(msg);
end

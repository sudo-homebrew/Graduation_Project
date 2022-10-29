function msg = rosWriteOccupancyGrid(msg, map)
%rosWriteOccupancyGrid Write occupancyMap to a ROS/ROS 2 message struct
%   MSGOUT = rosWriteOccupancyGrid(MSG,MAP) will write occupancy
%   values and other meta information to the MSG from the
%   occupancyMap object MAP. The meta information
%   includes resolution, origin, width and height.
%
%   Example:
%       % Create an occupancy grid
%       map = occupancyMap(rand(10));
%
%       % Create a nav_msgs/OccupancyGrid message
%       msg = rosmessage("nav_msgs/OccupancyGrid","DataFormat","struct");
%
%       % Write the map data to the msg
%       msg = rosWriteOccupancyGrid(msg,map);

%   Copyright 2020 The MathWorks, Inc.
%#codegen

% Ensure the generated code is not inlined
    coder.inline('never');

    % Validate input argument
    validateattributes(msg, {'struct'},{'scalar'},'rosWriteOccupancyGrid');
    % Code generation only supports nav_msgs/OccupancyGrid as input message
    if ~isempty(coder.target)
        coder.internal.assert(strcmp(msg.MessageType,'nav_msgs/OccupancyGrid'),...
                              'ros:mlroscpp:codegen:InvalidMsgForSpMsgFun',...
                              'rosWriteOccupancyGrid','nav_msgs/OccupancyGrid');

        % Code generation does not support empty message struct
        if isfield(msg,'Data')
            coder.internal.assert(~isempty(msg.Data),...
                                  'ros:mlroscpp:codegen:EmptyInputMsg','rosWriteOccupancyGrid');
        else
            coder.internal.assert(~isequal(msg.data,0),...
                                  'ros:mlroscpp:codegen:EmptyInputMsg','rosWriteOccupancyGrid');
        end
    end

    % Get binary map from input message
    if isfield(msg, 'Data')
        % ROS message struct
        specialMsgUtil = ros.internal.SpecialMsgUtil;
    else
        % ROS 2 message struct
        specialMsgUtil = ros.internal.ros2.SpecialMsgUtil;
    end
    msg = specialMsgUtil.writeOccupancyGrid(msg, map);
end

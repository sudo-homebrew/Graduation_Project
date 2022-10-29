function map = rosReadOccupancyGrid(msg)
%rosReadOccupancyGrid Returns an occupancyMap object from ROS/ROS 2 message struct
%   MAP = rosReadOccupancyGrid(MSG) returns a
%   occupancyMap object MAP reading the data from the MSG.
%   All values in the MSG are converted to probabilities
%   between 0 and 1. The unknown values (-1) in the MSG data
%   are set as 0.5 in the MAP.
%
%   Example:
%       % Create a nav_msgs/OccupancyGrid message struct
%       msg = rosmessage("nav_msgs/OccupancyGrid","DataFormat","struct");
%
%       % Populate the ROS occupancy grid message
%       msg.Info.Height = 10;
%       msg.Info.Width = 10;
%       msg.Info.Resolution = 0.1;
%       msg.Data = int8(100*rand(100,1));
%
%       % Read the msg data and convert to occupancyMap
%       map = rosReadOccupancyGrid(msg);

%   Copyright 2020 The MathWorks, Inc.
%#codegen

% Ensure the generated code is not inlined
    coder.inline('never');

    % Validate input argument
    validateattributes(msg, {'struct'},{'scalar'},'rosReadOccupancyGrid');

    % Code generation only supports nav_msgs/OccupancyGrid as input message
    if ~isempty(coder.target)
        coder.internal.assert(strcmp(msg.MessageType,'nav_msgs/OccupancyGrid'),...
                              'ros:mlroscpp:codegen:InvalidMsgForSpMsgFun',...
                              'rosReadOccupancyGrid','nav_msgs/OccupancyGrid');

        % Code generation does not support empty message struct
        if isfield(msg,'Data')
            coder.internal.assert(~isempty(msg.Data),...
                                  'ros:mlroscpp:codegen:EmptyInputMsg','rosReadOccupancyGrid');
        else
            coder.internal.assert(~isequal(msg.data,0),...
                                  'ros:mlroscpp:codegen:EmptyInputMsg','rosReadOccupancyGrid');
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
    map = specialMsgUtil.readOccupancyGrid(msg);
end

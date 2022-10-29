function map = rosReadBinaryOccupancyGrid(msg, varargin)
%rosReadBinaryOccupancyGrid Returns a BinaryOccupancyGrid from ROS/ROS 2
%message struct
%   MAP = rosReadBinaryOccupancyGrid(MSG) returns a
%   BinaryOccupancyGrid object MAP reading the data from the MSG.
%   The default occupancy threshold is 50. All values in the MSG
%   data equal or greater than occupancy threshold are set as
%   occupied (1) in the MAP and less than occupancy threshold are
%   set as unoccupied (0) in the MAP. By default, the unknown
%   values (-1) in the MSG data are set as unoccupied (0) in the MAP.
%
%   MAP = rosReadBinaryOccupancyGrid(MSG,OT) returns a
%   BinaryOccupancyGrid object MAP using the occupancy
%   threshold OT and using the default for unknown values.
%
%   MAP = rosReadBinaryOccupancyGrid(MSG,OT,VAL) returns a
%   BinaryOccupancyGrid object MAP using the occupancy
%   threshold OT and using VAL in place of the unknown
%   values (-1).
%
%   Example:
%       % Create a nav_msgs/OccupancyGrid message struct
%       msg = rosmessage("nav_msgs/OccupancyGrid","DataFormat","struct");
%
%       % Populate the ROS occupancy grid message
%       msg.Info.Height = 10;
%       msg.Info.Width = 10;
%       msg.Info.Resolution = 0.1;
%       msg.Data = 100*rand(100,1);
%
%       % Read the msg data and convert to BinaryOccupancyGrid
%       map = rosReadBinaryOccupancyGrid(msg);
%
%       % Read the msg data with threshold
%       map = rosReadBinaryOccupancyGrid(msg,65);
%
%       % Read the msg data with threshold and replacement for
%       % unknown value
%       map = rosReadBinaryOccupancyGrid(msg,65,1);

%   Copyright 2020 The MathWorks, Inc.
%#codegen

% Ensure the generated code is not inlined
    coder.inline('never');

    % Validate input argument
    validateattributes(msg, {'struct'},{'scalar'},'rosReadBinaryOccupancyGrid');

    % Code generation only supports nav_msgs/OccupancyGrid as input message
    if ~isempty(coder.target)
        coder.internal.assert(strcmp(msg.MessageType,'nav_msgs/OccupancyGrid'),...
                              'ros:mlroscpp:codegen:InvalidMsgForSpMsgFun',...
                              'rosReadBinaryOccupancyGrid','nav_msgs/OccupancyGrid');

        % Code generation does not support empty message struct
        if isfield(msg,'Data')
            coder.internal.assert(~isempty(msg.Data),...
                                  'ros:mlroscpp:codegen:EmptyInputMsg','rosReadBinaryOccupancyGrid');
        else
            coder.internal.assert(~isequal(msg.data,0),...
                                  'ros:mlroscpp:codegen:EmptyInputMsg','rosReadBinaryOccupancyGrid');
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
    map = specialMsgUtil.readBinaryOccupancyGrid(msg, varargin{:});
end

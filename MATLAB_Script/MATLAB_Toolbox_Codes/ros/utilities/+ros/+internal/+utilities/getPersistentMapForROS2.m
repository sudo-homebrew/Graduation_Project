function msgMap = getPersistentMapForROS2(command)
%This function is for internal use only. It may be removed in the future.

%GETPERSISTENTMAPFORROS creates a persistent map for ROS2 which will be used to
%   store the message types as keys and their corresponding data structs as
%   values.

%   Copyright 2021 The MathWorks, Inc.

if nargin < 1
    command = '';
end

persistent msgMapROS2;
if isempty(msgMapROS2) || isequal(command,'clear')
    msgMapROS2 = containers.Map;
end
msgMap = msgMapROS2;
end
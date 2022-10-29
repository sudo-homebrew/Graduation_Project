function msgMap = getPersistentMapForROS(command)
%This function is for internal use only. It may be removed in the future.

%GETPERSISTENTMAPFORROS creates a persistent map for ROS which will be used to
%   store the message types as keys and their corresponding data structs as
%   values.

%   Copyright 2021 The MathWorks, Inc.

if nargin < 1
    command = '';
end

persistent msgMapROS1;
if isempty(msgMapROS1) || isequal(command,'clear')
    msgMapROS1 = containers.Map;
end
msgMap = msgMapROS1;
end
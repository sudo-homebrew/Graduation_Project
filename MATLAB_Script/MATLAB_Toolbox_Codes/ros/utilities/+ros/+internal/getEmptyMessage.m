function [data, info, msgInfo] = getEmptyMessage(msg, rosver, registry, type)
%This function is for internal use only. It may be removed in the future.

%   Copyright 2019-2021 The MathWorks, Inc.

%getEmptyMessage is commonly used in codegeneration and other routines

if nargin < 3
    registry = ros.internal.CustomMessageRegistry.getInstance(rosver);
end
if nargin < 4
    type = 'msg';
end

msgInfo = ros.internal.(rosver).getMessageInfo(msg, registry, type);
clnup = []; %#ok<NASGU>

if isequal(rosver,'ros')

    %Get the persistent map for ROS1.
    msgMapROS1 = ros.internal.utilities.getPersistentMapForROS;
    [data,info] = getCachedMap(msgMapROS1,msg,msgInfo,rosver);
else
    
    %Get the persistent map for ROS2.
    msgMapROS2 = ros.internal.utilities.getPersistentMapForROS2;

    %For each custom message, we need to do addpath only for the first time and
    %store the value of the message struct in this persistent map.
    if msgInfo.custom
        curpath = path;
        clnup = onCleanup(@(x)path(curpath));
        addpath(fullfile(msgInfo.installDir,'m'));
    end
    [data,info] = getCachedMap(msgMapROS2,msg,msgInfo,rosver);
end
end

function [data,info] = getCachedMap(msgMap,msg,msgInfo,rosver)
%getCachedMap returns the data struct and info struct of a message type. If
%the message type is already present in the persistent map, it just returns
%the value from it. Else, they will be calculated from 
if isKey(msgMap, msg)
    structInfo = msgMap(msg);
else
    [structInfo.data,structInfo.info] = getMessageDataAndInfo(msg, msgInfo, rosver);
    msgMap(msg) = structInfo; %#ok<NASGU> 
end
data = structInfo.data;
info = structInfo.info;
end

function [data, info] = getMessageDataAndInfo(msg, msgInfo, rosver)
try
    [data, info] = eval(msgInfo.msgStructGen);
catch ex
    if strcmp(ex.identifier, 'MATLAB:undefinedVarOrClass') || ...
            isempty(msgInfo.pkgName) || isempty(msgInfo.msgName)
        if isequal(rosver,'ros2')
            error(message('ros:utilities:message:MessageNotFoundError', msg, 'ros2 msg list'))
        else
            error(message('ros:utilities:message:MessageNotFoundError', msg, 'rosmsg list'))
        end
    else
        newEx = MException(message('ros:utilities:message:CreateError', msg));
        throw(newEx.addCause(ex));
    end
end
end

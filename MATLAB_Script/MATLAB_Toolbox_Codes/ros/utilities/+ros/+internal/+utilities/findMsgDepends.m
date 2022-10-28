function msgDependsMap = findMsgDepends(msgName, msgDependsMap, rosver)
%This function is for internal use only. It may be removed in the future.

%findMsgDepends finds all message dependencies based on info

%   Copyright 2019-2020 The MathWorks, Inc.

    if nargin < 2 || isempty(msgDependsMap)
        msgDependsMap = containers.Map();
    end

    if nargin < 3
        rosver = 'ros2';
    end

    % Iterate through nested messages to determine dependencies
    [msgStruct,info] = ros.internal.getEmptyMessage(msgName,rosver);
    msgFields = fieldnames(msgStruct);
    msgFields(strcmp(msgFields,'MessageType')) = [];
    for k = 1:numel(msgFields)
        % Check info MessageType, not struct, to account for Time and Duration
        if isfield(info.(msgFields{k}),'MessageType')
            msgType = info.(msgFields{k}).MessageType;
            [pkgName, msgName] = fileparts(msgType);
            if ~isempty(msgDependsMap) && msgDependsMap.isKey(pkgName)
                curVal = msgDependsMap(pkgName);
                msgDependsMap(pkgName) = unique([curVal, msgName]);
            else
                msgDependsMap(pkgName) = {msgName};
            end
            % Recurse inside those nested messages for further dependencies
            msgDependsMap = ros.internal.utilities.findMsgDepends(msgType,msgDependsMap,rosver);
        end
    end

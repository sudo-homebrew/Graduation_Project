function ros2ModelInfo = augmentModelInfo(ros2ModelInfo)
%This function is for internal use only. It may be removed in the future.

%augmentModelInfo Adds extra information that eases template based
%codegeneration

%   Copyright 2019-2021 The MathWorks, Inc.

msgIncludes = {};
publishers = ros2ModelInfo.Publishers.values;
msgIncludes = [msgIncludes cellfun(@(x)x.msgInfo.includeHeader, publishers, 'UniformOutput',false)];
subscribers = ros2ModelInfo.Subscribers.values;
msgIncludes = [msgIncludes cellfun(@(x)x.msgInfo.includeHeader, subscribers, 'UniformOutput',false)];
servicecallers = ros2ModelInfo.ServiceCallers.values;
msgIncludes = [msgIncludes cellfun(@(x)x.msgInfo.includeHeader, servicecallers, 'UniformOutput',false)];
ros2ModelInfo.msgIncludes = unique(msgIncludes);


% if field.MaxLen is NaN, then read the varlen info for that field from
% ros2ModelInfo

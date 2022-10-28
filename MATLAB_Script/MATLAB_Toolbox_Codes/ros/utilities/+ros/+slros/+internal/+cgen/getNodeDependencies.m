function info = getNodeDependencies(msgTypes)
%This function is for internal use only. It may be removed in the future.

%   INFO = getNodeDependencies(MSGTYPES) takes a list of ROS message types,
%   and returns the set of ROS packages that ship those message types.
%   INFO is a struct with fields 'messageList' and 'nodeDependencies'
%   (cell arrays of strings).
%
%   For example, if a message type is 'geometry_msgs/Point', the
%   corresponding package is 'geometry_msgs'.

%   Copyright 2014-2020 The MathWorks, Inc.

msgTypes = unique(msgTypes);

tokens = regexp(msgTypes, '^(\w*)/', 'tokens');
dependencies = {};
for i=1:numel(tokens)
    if isempty(tokens{i})
        continue;
    end

    if strcmp(tokens{i}{1}{1}, ros.slros.internal.bus.Util.TimePackage) || strcmp(tokens{i}{1}{1}, 'ros')
        % Keep time in message types, but no node dependency required
        continue;
    end

    dependencies{end+1} = tokens{i}{1}{1}; %#ok<AGROW>
end

info.messageList = msgTypes;
info.nodeDependencies = dependencies;

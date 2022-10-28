function [status, mesg] = colconToolsValidator(tool)
% This class is for internal use only. It may be removed in the future.

%   Copyright 2019 The MathWorks, Inc.

status = true;
toolName = getName(tool);
command = getCommand(tool);
if isequal(command,'cmake')
    try
        mesg = ros.ros2.internal.ColconBuilder.getCMakePath;
    catch ex
        status = false;
        mesg = ex.message;
    end
else
    % Nothing to do for now
    mesg =  [toolName, ': ', command];
end

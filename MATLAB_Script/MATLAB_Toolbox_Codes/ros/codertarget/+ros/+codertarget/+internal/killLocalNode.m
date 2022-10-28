function [status, result] = killLocalNode(nodeName)
%This function is for internal use only. It may be removed in the future.
% Kill a ROS node running on local host.

%   Copyright 2020 The MathWorks, Inc.

% Check if the process is running and kill if it is
if ros.codertarget.internal.isLocalNodeRunning(nodeName)
    [status, result] = killProcess(nodeName);
else
    % Process was not running to begin with. Return status = 0 to indicate
    % success
    status = false;
    result = '';
end
end

function [status, result]= killProcess(procName)
% killproc Kill a process
cmdMap = containers.Map({'win64','maci64','glnxa64'}, ...
    {sprintf('taskkill /F /IM %s',[procName '.exe']), ... use tasklist
    sprintf('killall %s',procName), ...  use killall
    sprintf('killall %s',procName) ...  use killall
    });
[status, result] = system(cmdMap(computer('arch')));
end

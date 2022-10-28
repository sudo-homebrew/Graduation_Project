function [status, result] = killProcessByPID(pid)
%This function is for internal use only. It may be removed in the future.

%   Copyright 2020-2021 The MathWorks, Inc.

% kills the process using its PID

if ispc
    [status, result] = system(sprintf('taskkill /F /T /PID %d"',pid));
else
    [status, result] = system(sprintf('pkill -P %d',pid)); 
end

end
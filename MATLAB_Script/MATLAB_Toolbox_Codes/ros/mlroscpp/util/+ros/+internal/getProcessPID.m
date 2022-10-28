function pid = getProcessPID(processName)
%This function is for internal use only. It may be removed in the future.

%   Copyright 2020-2021 The MathWorks, Inc.

% Create a map of system commands that will query for running application

isAppRunningCmdMap = containers.Map({'win64','maci64','glnxa64'}, ...
    {sprintf('tasklist /FI "WINDOWTITLE eq %s" /FO TABLE /NH', processName), ... use tasklist
    sprintf('ps ax | grep "%s" | grep -v "grep"', processName), ...  use pidof
    sprintf('ps ax | grep "%s" | grep -v "grep"', processName) ...  use pidof
    });

% Get the correct command
cmd = isAppRunningCmdMap(computer('arch'));
[status, result] = system(cmd);
result_data = strsplit(strtrim(result));

if isequal(status, 0)
    if ispc
        pid  = str2double(result_data{2});
    else
        pid = str2double(result_data{1});
    end
end
end
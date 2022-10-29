function [killStatus, result] = killROSProcess(processName,cmdWindow)
%This function is for internal use only. It may be removed in the future.

%   Copyright 2020-2021 The MathWorks, Inc.

% check if the process is running or not

    isrunning = checkIfRunning(processName);
    if isrunning
        [killStatus, result] = killproc(cmdWindow);
    else
        killStatus = false;
        result = false;
        %fprintf('Nothing to close');
    end
end

function isRunning = checkIfRunning(appName)
% checkIfRunning check if the application is running.

% Create a map of name of applications per platform
    appNameMap = containers.Map({'win64','maci64','glnxa64'}, ...
                                {[appName, '.exe'], appName, appName});
    % Create a map of system commands that will query for running application
    isAppRunningCmdMap = containers.Map({'win64','maci64','glnxa64'}, ...
                                        {sprintf('tasklist /FI "IMAGENAME eq %s" /FO list', appNameMap('win64')), ... use tasklist
                        sprintf('ps ax | grep "%s" | grep -v "grep"', appNameMap('maci64')), ...  use pidof
                        sprintf('ps ax | grep "%s" | grep -v "grep"', appNameMap('glnxa64')) ...  use pidof
                   });
    % Get the correct command
    cmd = isAppRunningCmdMap(computer('arch'));
    [status, result] = system(cmd);
    isRunning = false;
    % if status is non-zero, assume application was not running
    if isequal(status, 0)
        isRunning = contains(strtrim(result), appName);
    end
end

function [killStatus, result]= killproc(cmdWindow)
% killproc kill the application
    killStatus = false;
    % Create a map of name of applications per platform
    appNameMap = containers.Map({'win64','maci64','glnxa64'}, ...
                                {cmdWindow, cmdWindow, cmdWindow});
    % Create a map of system commands that will query for running application
    isAppRunningCmdMap = containers.Map({'win64','maci64','glnxa64'}, ...
                                        {sprintf('taskkill /FI "WINDOWTITLE eq %s"', appNameMap('win64')), ... use tasklist
                        sprintf('ps ax | grep "%s" | grep -v "grep"', appNameMap('maci64')), ...  use pidof
                        sprintf('ps ax | grep "%s" | grep -v "grep"', appNameMap('glnxa64')) ...  use pidof
                   });
    % Get the correct command
    cmd = isAppRunningCmdMap(computer('arch'));
    [status, result] = system(cmd);

    if ~ispc
        % if status zero, assume application is running and kill the application
        if isequal(status, 0)
            res = strsplit(strtrim(result));
            pidofprocess = res{1};
            cmd = ['kill -9 ' num2str(pidofprocess)];
            [status, result] = system(cmd);
        end
    end
    if isequal(status, 0)
        killStatus = true;
    end
end

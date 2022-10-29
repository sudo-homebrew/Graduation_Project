function waitForProcessToStart(processName, timeout)
%This function is for internal use only. It may be removed in the future.

%   Copyright 2020-2021 The MathWorks, Inc.

% wait for a given process to start

    tic;
    %fprintf('Waiting for %s....',processName);
    isrunning = checkIfRunning(processName);
    startTime = clock;
    % Stop Waiting for process to launch when the maximum timeout is reached.
    while ~isrunning
        fprintf('.');
        isrunning = checkIfRunning(processName);
        if(etime(clock, startTime) > timeout)
            %fprintf('Process has not been started');
            break
        end
    end
    disp(getString(message('ros:mlros:util:ProcessStarted',num2str(toc))));
end


function isRunning = checkIfRunning(appName)
% checkIfRunning check if the application is running.

% Create a map of name of applications per platform
    appNameMap = containers.Map({'win64','maci64','glnxa64'}, ...
                                {[appName, '.exe'], appName, appName});
    % Create a map of system commands that will query for running application
    isAppRunningCmdMap = containers.Map({'win64','maci64','glnxa64'}, ...
                                        {sprintf('tasklist /FI "IMAGENAME eq %s"', appNameMap('win64')), ... use tasklist
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

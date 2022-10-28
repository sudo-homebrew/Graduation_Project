function waitForMasterToStart(masterURI, timeout)
%This function is for internal use only. It may be removed in the future.

%   Copyright 2020-2021 The MathWorks, Inc.

% wait for a ROS master process to start

    tic;
    %fprintf('Waiting for %s....',processName);
    isrunning = checkIfRunning(masterURI);
    startTime = clock;
    % Stop Waiting for process to launch when the maximum timeout is reached.
    while ~isrunning
        fprintf('.');
        isrunning = checkIfRunning(masterURI);
        if(etime(clock, startTime) > timeout)
            hostandPortWithOuthttp = strsplit(masterURI,'://');
            hostandPort = strsplit(hostandPortWithOuthttp{2},':');
            portWithslash = strsplit(hostandPort{2},'/');
            port = portWithslash{1};
            portNumber = str2double(port);
            if ismember(portNumber, ros.internal.getCurrentlyUsedPorts)
                error(message('ros:mlros:core:InvalidPort', portNumber));
            else
                error(message('ros:mlros:core:FailedToStart', portNumber));
            end
        end
    end
    disp(getString(message('ros:mlros:util:ProcessStarted',num2str(toc))));
end


function isRunning = checkIfRunning(masterURI)
    isRunning = ros.internal.NetworkIntrospection.isMasterReachable(masterURI, 1);
end

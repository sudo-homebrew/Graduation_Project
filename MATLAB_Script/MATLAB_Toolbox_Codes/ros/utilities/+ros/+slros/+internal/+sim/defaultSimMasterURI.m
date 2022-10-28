function masterURI = defaultSimMasterURI(targetAddress)
%This function is for internal use only. It may be removed in the future.

%defaultSimMasterUri Returns ROS master URI used for simulation
%   The returned Master URI can be used by the deployed Simulink ROS node.
%
%   This uses the following precedence order for retrieving the Master URI:
%   1. The Master settings that the user made in Simulink > Tools > ROS >
%      Configure Network Addresses. If the setting is "Default", precedence
%      items (2) - (4) are considered. If the setting is "Custom", it will
%      take precedence.
%   2. Value of environment variable ROS_MASTER_URI
%   3. The URI of the Master that the global node is connected to (if
%      global node is active.
%   4. Default hostname 'localhost' and default port 11311. If this occurs,
%      the function tries to determine the IP address of the host machine
%      that is visible from the deployed node.

%   Copyright 2016-2020 The MathWorks, Inc.

% Recover ROS master connection information from Simulink settings
    rosMaster = ros.slros.internal.sim.ROSMaster;
    masterURI = rosMaster.MasterURI;

    if contains('http://localhost:11311', masterURI)
        % This is a special case, since the ROS master is running on the host
        % machine. The master might be in MATLAB or external to it, but we want
        % the deployed node to contact the ROS master at a definite IP address.
        % For this to happen, we need to determine the IP address of the
        % network interface that the device with the deployed node can reach.

        try
            % Get host IP address of network interface connecting to ROS device
            hostIP = ros.internal.Net.getAssociatedInterfaceAddress(targetAddress);
        catch
            % If this fails, try to determine the IP address by pinging
            % the ROS device through each interface
            intfInfo = ros.internal.Net.getAllIPv4Addresses;
            hostIP = '';
            for k = 1:numel(intfInfo)
                if pingHost(intfInfo(k).ipAddress, targetAddress)
                    % Ping through this interface succeeded
                    hostIP = intfInfo(k).ipAddress;
                    break;
                end
            end
            if isempty(hostIP)
                % No ping succeeded. Warn the user that our automatic choice of
                % MasterURI might not be appropriate
                hostIP = intfInfo(1).ipAddress;
                warning(message('ros:slros:deploy:UnabletoDetermineROSMasterIP',...
                                targetAddress, hostIP));
            end
        end
        masterURI = ['http://' hostIP ':11311'];
    end
end

function ret = pingHost(localIfIp, remoteHostIp)
    if ispc
        % -w: timeout (milliseconds)
        % -n: count
        % -l: buffer size
        % -S: interface
        cmd = ['ping -w 1000 -n 1 -S ' localIfIp ' ' remoteHostIp];
    elseif isunix
        % -W: timeout (seconds)
        % -c: count
        % -S: interface
        cmd = ['ping -W 1 -c 1 -S ' localIfIp ' ' remoteHostIp];
    end
    [st,msg] = system(cmd);
    if (st == 0) && ~isempty(regexpi(msg,'\sTTL='))
        ret = true;
    else
        ret = false;
    end
end

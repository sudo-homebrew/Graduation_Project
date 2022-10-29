function port = findOpenCorePort
%This function is for internal use only. It may be removed in the future.

%findOpenCorePort Get an open network port that is verified to be usable by ROS Master
%   The qeGetOpenPort test tool sometimes returns an open port
%   that is not usable by the ROS Master (see g1393752).
%   findOpenCorePort is modeled after qeGetOpenPort and ensures that the port
%   is really open and usable by ROS before returning the port number.
%   It also ensures that any ports open during verification are properly closed.
%
%   The port number will be in the standard TCP/IPv4 ephemeral port range
%   for the current platform. We use this range to be safe on all OSes
%   [49152 61000]
%
%   See also qeGetOpenPort.

%   Copyright 2020-2021 The MathWorks, Inc.

    persistent previouslyGottenPorts;
    persistent randStream;
    mlock;

    if isempty(previouslyGottenPorts)
        previouslyGottenPorts = zeros(0);
    end

    if isempty(randStream)
        % Make sure that ports are always random with custom random stream.
        % Generate seed based on MATLAB process ID. This reduces chances of two
        % instances of MATLAB launched at the same instant to use the same port
        % for roscore
        seed = feature('getpid');
        randStream = RandStream('mt19937ar', 'Seed', seed);
    end

    % The default minimum and maximum port values
    minCorePortDefault = 49152;
    maxCorePortDefault = 61000;

    % The user can override these limits by setting environment variables
    % ROS_MINIMUM_VALID_CORE_PORT and ROS_MAXIMUM_VALID_CORE_PORT
    minPortStr = getenv('ROS_MINIMUM_VALID_CORE_PORT');
    minCorePort = str2double(minPortStr);
    if isnan(minCorePort)
        minCorePort = minCorePortDefault;
    end

    maxPortStr = getenv('ROS_MAXIMUM_VALID_CORE_PORT');
    maxCorePort = str2double(maxPortStr);
    if isnan(maxCorePort)
        maxCorePort = maxCorePortDefault;
    end

    % List all possible ports and select randomly from them
    portsInUse = vertcat(previouslyGottenPorts, ros.internal.getCurrentlyUsedPorts);
    possiblePorts = minCorePort:maxCorePort;
    [~, idxPossiblePortsUsed] = intersect(possiblePorts, portsInUse);
    possiblePorts(idxPossiblePortsUsed) = [];
    if isempty(possiblePorts)
        error('findOpenCorePort:CouldNotFindPort', ['Could not find '...
                                                    ' an open port to use.']);
    else
        idxPort = randi(randStream, numel(possiblePorts));
        port = possiblePorts(idxPort);
    end

    % Append the new port to the array of previous found ports
    previouslyGottenPorts(end+1, 1) = port;
end

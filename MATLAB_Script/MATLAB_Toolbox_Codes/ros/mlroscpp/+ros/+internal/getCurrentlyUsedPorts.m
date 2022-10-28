function portList = getCurrentlyUsedPorts
%This function is for internal use only. It may be removed in the future.

%getCurrentlyUsedPorts Get a list of ports currently in use
%   This will include some ports that are potentially usable with a ROS
%   core, due to using a different network interface, or being in a state
%   that makes the port available. However, when randomly selecting a port
%   for use with the master, it makes sense to be cautious and eliminate as
%   many ports with potential issues as possible.

%   Copyright 2021 The MathWorks, Inc.

% Default output
    portList = zeros(0);

    % Network connection information command based on architecture
    % Looking for ports in-use with TCP connections
    archKeys = {'win64', 'glnxa64', 'maci64'};
    arch = computer('arch');
    cmdMap = containers.Map(archKeys, ...
                            {'netstat -q -n -p TCP', ...   % win64
                             'netstat -a -n -t', ...       % glnxa64
                             'netstat -a -n -p TCP'});         % maci64
    [status, netText] = system(cmdMap(arch));

    % Parse the output for port values following IP addresses or host names
    portPatternMap = ...
        containers.Map(archKeys, ...
                       {lookBehindBoundary(":")+digitsPattern+whitespaceBoundary("start"), ...  % win64
                        lookBehindBoundary(":")+digitsPattern+whitespaceBoundary("start"), ...  % glnx64
                        lookBehindBoundary(".")+digitsPattern+whitespaceBoundary("start")});    % maci64
    portPattern = portPatternMap(arch);
    if ~status
        portText = extract(netText, portPattern);
        portList = unique(rmmissing(str2double(portText)));
        portList(portList == 0) = [];
    end

end

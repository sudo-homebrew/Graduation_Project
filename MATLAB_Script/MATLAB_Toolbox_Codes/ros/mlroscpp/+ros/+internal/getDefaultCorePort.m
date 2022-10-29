function defaultCorePort = getDefaultCorePort
%This function is for internal use only. It may be removed in the future.
 
%defaultCorePort Set value of default Core port
%   getDefaultCorePort reads the value of the environment variable
%   'ROS_DEFAULT_CORE_PORT' and sets it to the value for default port of
%   starting ROS core. If this variable does not exist, the default value is
%   set to 11311

%   Copyright 2020 The MathWorks, Inc.

port = getenv('ROS_DEFAULT_CORE_PORT');

% If environment variable has been set to numeric value, use it. 
% Otherwise, default to port 11311. 
if ~isempty(port)
    defaultCorePort = str2double(port);
    if isnan(defaultCorePort)
        defaultCorePort = 11311;
    end
else
    defaultCorePort = 11311;
end
% Check for valid port number
validateattributes(defaultCorePort, {'double'},{'nonnegative','<=',65535})
 
end

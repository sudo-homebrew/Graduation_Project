function nodePort = getDefaultNodePort
%This function is for internal use only. It may be removed in the future.
 
%getDefaultNodePort Get the value of the port set by automatic selection in
% the core
%   getDefaultNodePort reads the value of the environment variable
%   'ROS_DEFAULT_NODE_PORT' so that node object can connect to a core that
%   has been started at an automatically selected port. If this variable does
%   not exist, the default value is set to 11311

%   Copyright 2020 The MathWorks, Inc.

corePort = getenv('ROS_DEFAULT_CORE_PORT');

% If environment variable has been set to numeric value, use it. 
% Otherwise, default to port 11311. 
if ~isempty(corePort)
    if isequal(corePort,'0')  && ~isempty(getenv('ROS_DEFAULT_NODE_PORT'))
       nodePort = str2double(getenv('ROS_DEFAULT_NODE_PORT'));
       return
    end
    nodePort = str2double(corePort);
    if isnan(nodePort)
        nodePort = 11311;
    end
else
    nodePort = 11311;
end
% Check for valid port number
validateattributes(nodePort, {'double'},{'nonnegative','<=',65535})

end

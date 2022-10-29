function startup
%startup Set ROS core port for testing
%   This startup script ensures that the ROS core will be launched on an
%   open network port. The enables the parallel publishing of examples in
%   BaT (using multiple simultaneous MATLAB sessions).
%   The environment variable is valid for the entire MATLAB session and will 
%   reset on exit.  The variable will not persist for other
%   MATLAB sessions.

%   Copyright 2016-2017 The MathWorks, Inc.

disp('Using ROS automatic port selection to permit multiple ROS instances');
setenv('ROS_DEFAULT_CORE_PORT', '0')
setenv('ROS_DEFAULT_NODE_PORT', '')
end
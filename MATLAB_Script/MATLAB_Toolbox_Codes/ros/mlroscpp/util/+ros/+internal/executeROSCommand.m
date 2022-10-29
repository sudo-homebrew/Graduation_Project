function [status, result] = executeROSCommand(cmdline, logFilePath, bgCmd)
%This function is for internal use only. It may be removed in the future.

%EXECUTEROSCOMMAND Execute a ROS command on the system ROS terminal

%   Copyright 2021 The MathWorks, Inc.

if nargin < 3
    bgCmd = '';
end

if nargin < 2
    logFileCmd = '';
else
    % Redirect stdout and stderr to logfile
    logFileCmd = ['> "' logFilePath '" 2>&1'];
end

% Construct first two arguments to the shell script
catkinPrefixPath = ['"' ros.internal.getCatkinPrefixPath '"'];
[~,~,pyenvDir] = ros.internal.createOrGetLocalPython;
pyenvDir = ['"' pyenvDir '"'];

% Construct command line
if ispc    
    % This assignment cannot be removed due to ROS-1682
    % If to be removed then edit matlab\sys\ros1\win64\ros1\_setup_util.py 
    % to include 'ros_package_path.bat' as an environment hook
    cmd = ['"' fullfile(fileparts(mfilename('fullpath')),'executeroscommand.bat') '"'];
    rosPackagePath = strrep(fullfile(matlabroot,'sys','ros1',...
        computer('arch'),'ros1','share'),'\','/');
    cmdline = ['set ROS_PACKAGE_PATH=' rosPackagePath ' && ' cmdline];
    execCommand = [cmd ' ' catkinPrefixPath ' ' pyenvDir ' ' cmdline];
    if nargin > 2
        execCommand = ['(' execCommand ') ' logFileCmd ' ' bgCmd ' '];
    end
else
    cmd = ['"' fullfile(fileparts(mfilename('fullpath')),'executeroscommand.sh') '"'];
    execCommand = [cmd ' ' catkinPrefixPath ' ' pyenvDir ' ' cmdline];
    if nargin > 2
        % Note semicolon is mandatory. Command is executed in the same
        % shell when braces are used to group commands
        execCommand = ['bash -c ''{ ' execCommand '; } ' logFileCmd ' ' bgCmd ''''];
    end
end

% Execute command on system shell
[status, result] = system(execCommand);
end


function [status, result] = runroscmd(cmdline, extrasetup, logFilePath)
%This function is for internal use only. It may be removed in the future.

%runroscmd runs ros command

%   Copyright 2019-2020 The MathWorks, Inc.

% Users can set the prefix path as an environment variable. If set that is
% where ros2 is installed.
if nargin < 3
    logFileCmd = '';
else
    logFileCmd = ['> "', logFilePath, '" 2>&1'];
end

if nargin < 2
    extrasetup = '" "';
end
catkinPrefixPath = ['"' ros.internal.getCatkinPrefixPath '"'];

[~,~,pyenvDir] = ros.internal.createOrGetLocalPython;
pyenvDir = ['"' pyenvDir '"'];

cmdmap = containers.Map({'win64','maci64','glnxa64'}, ...
    {['"' fullfile(fileparts(mfilename('fullpath')),'runroscmd') '"'], ...use .bat file
    ['"' fullfile(fileparts(mfilename('fullpath')),'runroscmd.sh') '"'],... use .sh
    ['"' fullfile(fileparts(mfilename('fullpath')),'runroscmd.sh') '"']});
cmd = cmdmap(computer('arch'));

if ispc
    % Need to find the fix for setting the ROS_PACKAGE_PATH
    rosPackagePath = strrep(fullfile(matlabroot,'sys','ros1',computer('arch'),'ros1','share'),'\','/');
    
    % Incase of "rosmaster --core" command, use different .bat file
    if contains(cmdline, 'rosmaster') && contains(cmdline, '--core')
        cmd = ['"' fullfile(fileparts(mfilename('fullpath')),'runrosmastercmd') '"'];
    end
    
    % This assignment cannot be removed due to ROS-1682
    % If to be removed then edit matlab\sys\ros1\win64\ros1\_setup_util.py 
    % to include 'ros_package_path.bat' as an environment hook
    setPackagePath = ['set ROS_PACKAGE_PATH=' rosPackagePath];
    cmdlinewindows = [setPackagePath ' && ' cmdline];
    execCommand = [cmd, ' ', extrasetup, ' ', catkinPrefixPath, ' ', pyenvDir, ' ', cmdlinewindows];
else
    binpath = [' "', fullfile(matlabroot,'bin',computer('arch')), '" '];
	execCommand = [cmd, ' ', extrasetup, ' ', catkinPrefixPath, ' ', pyenvDir, binpath, cmdline];
end

if nargin > 2
    pathEnv = ['"' getenv('PATH') '"'];
    execCommandArgsMap = containers.Map();
    execCommandArgsMap('win64')   = ['(',execCommand, ')' logFileCmd];
    execCommandArgsMap('glnxa64') = ['bash -c ', '''{ PATH=', pathEnv '; ' execCommand , '; } ' logFileCmd, ''''];
    execCommandArgsMap('maci64')  = ['bash -c ', '''{ PATH=', pathEnv '; ' execCommand , '; } ' logFileCmd, ''''];
    execCommand = execCommandArgsMap(computer('arch'));
end

[status, result] = system(execCommand);
if isequal(status,0) %no error
    result = strtrim(result);
end


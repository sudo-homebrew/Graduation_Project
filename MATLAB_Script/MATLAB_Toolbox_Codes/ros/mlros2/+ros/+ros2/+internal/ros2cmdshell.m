function [status, result] = ros2cmdshell
%This function is for internal use only. It may be removed in the future.

%   Copyright 2019 The MathWorks, Inc.

%ros2cmdshell starts a shell with the environment all set for users to use
%colcon and other commands interactively


% Users can set the prefix path as an environment variable. If set that is
% where ros2 is installed.
amentPrefixPath = ['"' ros.ros2.internal.getAmentPrefixPath '"'];

[~,~,pyenvDir] = ros.ros2.internal.createOrGetLocalPython;
pyenvDir = ['"' pyenvDir '"'];

tempFile = tempname('/tmp');
cmdmap = containers.Map({'win64','maci64','glnxa64'}, ...
    {['cmd /k " "' fullfile(fileparts(mfilename('fullpath')),'ros2cmdshell') '"'], ...use .bat file
     ['open -a Terminal.app "' tempFile '"'],... use .sh
     ['xterm -e "' tempFile '"']});
cmd = cmdmap(computer('arch'));

if ispc
    %need vcvars as well
    mexInfo = mex.getCompilerConfigurations('C++');
    [status, result] = system([cmd, ' "', strtrim(mexInfo.Details.CommandLineShell), '" ', amentPrefixPath, ' ', pyenvDir, ' " &']);
else
    contents = fileread(fullfile(fileparts(mfilename('fullpath')),'ros2cmdshell.sh'));
    contents = strrep(contents,'REPLACE_AMENT_PREFIX_PATH',amentPrefixPath);
    contents = strrep(contents,'REPLACE_LOCAL_PYTHON_VENV_PATH', pyenvDir);
    [fid, msg] = fopen(tempFile,'w');
    if fid == -1
        error(message('MATLAB:fileread:cannotOpenFile', tempFile, msg));
    end
    fwrite(fid, contents);
    fclose(fid);
    result = ['In a bash shell ''source ' tempFile '''.'];
    status = 0;
    if nargout < 2
        disp(result);
    end
end
if isequal(status,0) %no error
    result = strtrim(result);
end

function [status, result] = roscmdshell
%This function is for internal use only. It may be removed in the future.

%   Copyright 2019 The MathWorks, Inc.

%roscmdshell starts a shell with the environment all set for users to use
%catkin and other commands interactively


% Users can set the prefix path as an environment variable. If set that is
% where ros1 is installed.
    catkinPrefixPath = ['"' ros.internal.getCatkinPrefixPath '"'];

    [~,~,pyenvDir] = ros.internal.createOrGetLocalPython;
    pyenvDir = ['"' pyenvDir '"'];

    tempFile = tempname('/tmp');
    cmdmap = containers.Map({'win64','maci64','glnxa64'}, ...
                            {['cmd /k " "' fullfile(fileparts(mfilename('fullpath')),'roscmdshell') '"'], ...use .bat file
                        ['open -a Terminal.app "' tempFile '"'],... use .sh
                        ['xterm -e "' tempFile '"']});
    cmd = cmdmap(computer('arch'));

    if ispc
        %need vcvars as well
        mexInfo = mex.getCompilerConfigurations('C++');
        [status, result] = system([cmd, ' "', strtrim(mexInfo.Details.CommandLineShell), '" ', catkinPrefixPath, ' ', pyenvDir, ' " &']);
    else
        contents = fileread(fullfile(fileparts(mfilename('fullpath')),'roscmdshell.sh'));
        contents = strrep(contents,'REPLACE_CATKIN_PREFIX_PATH',catkinPrefixPath);
        contents = strrep(contents,'REPLACE_LOCAL_PYTHON_VENV_PATH', pyenvDir);
        contents = strrep(contents,'REPLACE_ROSSETUP_PATH',fileparts(mfilename('fullpath')));
        contents = strrep(contents,'REPLACE_MATLABBIN_PATH',fullfile(matlabroot,'bin',computer('arch')));
        [fid, msg] = fopen(tempFile,'w');
        if fid == -1
            error(message('MATLAB:fileread:cannotOpenFile', tempFile, msg));
        end
        fwrite(fid, contents);
        fclose(fid);
        result = message('ros:utilities:util:SourceTempFile',tempFile).getString();
        status = 0;
        if nargout < 2
            disp(result);
        end
    end
    if isequal(status,0) %no error
        result = strtrim(result);
    end

function [aPath, version] = getCMakeBinaryPath(minVer)
%This function is for internal use only. It may be removed in the future.

%getCMakeBinaryPath Find and return full-path to "cmake" binary executable
%   Searches for a cmake installation on the system path. If an
%   installation of "cmake" is found, the function verifies it against
%   required minimum version specified by input, MINVER, and returns the
%   fullpath and version.

%   Copyright 2020-2021 The MathWorks, Inc.

    cmdmap = containers.Map({'win64','maci64','glnxa64'}, ...
                            {'where cmake', ... for windows
                             'which cmake', ... for mac
                             'which cmake'}); ... for linux
    cmd = cmdmap(computer('arch'));
    [status, result] = system(cmd);
    if status ~= 0
        error(message('ros:utilities:util:CMakeNotFound', minVer));
    end
    aPath = strtrim(result);

    cmdmap = containers.Map({'win64','maci64','glnxa64'}, ...
                            {'cmake --version', ... for windows
                             'bash -c ''LD_LIBRARY_PATH= cmake --version''', ... for mac
                             'bash -c ''LD_LIBRARY_PATH= cmake --version'''}); ... for linux
    cmd = cmdmap(computer('arch'));
    [status, result] = system(cmd);
    if status ~= 0
        ex = MException(message('ros:utilities:util:CMakeNotFound', minVer));
        throw(addCause(ex, MException('ros:utilities:util:GenericException', result)));
    end
    % search the starting index where ''cmake version'' text is
    % present, since succeeding text will be the version number of
    % cmake
    cmakeVerTextIndx = strfind(lower(result), 'cmake version');
    % trim any text before the ''cmake version'' text
    trimmedVersionText = result(cmakeVerTextIndx:end);
    resSplit = strsplit(trimmedVersionText);
    version = ros.internal.utilities.getVersionVal(resSplit{3}); %the third one is version number
    reqVersion = ros.internal.utilities.getVersionVal(minVer);
    if version < reqVersion
        error(message('ros:utilities:util:CMakeNotFound', minVer));
    end
end

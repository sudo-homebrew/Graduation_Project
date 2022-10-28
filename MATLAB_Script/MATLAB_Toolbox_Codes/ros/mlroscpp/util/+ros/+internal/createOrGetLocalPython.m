function [localPythonPath, activatePath, pyenvDir] = createOrGetLocalPython(forceRecreateVenv)
%This function is for internal use only. It may be removed in the future.

%createOrGetLocalPython Creates a Python virtual environment for ROS1. 
%
% [localPythonPath, activatePath, pyenvDir] =
% ros.internal.createOrGetLocalPython() returns the Python executable,
% Python activation path and Python virtual environment folder. The Python
% virtual environment is automatically created if it does not exist. You
% must set the Python version using pyenv function prior to executing this
% function. 
%
% ros.internal.createOrGetLocalPython(true) Forces re-creation of the
% Python virtual environment. Use this form when you change the minor
% version of Python or when your Python virtual environment is corrupted.
% You must set the Python version using pyenv function prior to executing
% this function.
%
% This function automatically assigns Python virtual environment folder. In
% order to manually assign a root folder for Python virtual environment,
% set MY_PYTHON_VENV environment variable to a folder. Ensure that this
% folder does not contain any spaces. 
%
% Note: The Python environment created by this function is by ROS Toolbox
% for launching ROS core, generating custom messages and deploying a
% Simulink model as a ROS node. The Python virtual environment persists
% between MATLAB sessions.
% 
% Examples:
% 
% % Get current Python virtual environment parameters
% pyenv('Version','<Path to Python executable>')
% [localPythonPath, activatePath, pyenvDir] = ros.internal.createOrGetLocalPython
%
% % Re-create Python virtual environment
% pyenv('Version','<Path to Python executable>')
% ros.internal.createOrGetLocalPython(true)
% 
% % Create Python virtual environment under a given root folder
% setenv('MY_PYTHON_VENV','<Path with no spaces>')
% [~, ~, pyenvDir] = ros.internal.createOrGetLocalPython(true)
%
% See also pyenv, setenv, rosinit.

%   Copyright 2019-2022 The MathWorks, Inc.

    if nargin < 1
        forceRecreateVenv = false;
    end
     
    % Get the root directory where Python venv can be created
    venvRoot = getenv('MY_PYTHON_VENV');
    if contains(venvRoot,' ')
        % Python venv does not work in a directory with spaces
        error(message('ros:utilities:util:NoSpaceInPythonVenvDir', venvRoot));
    end
    if isempty(venvRoot)
        if ismac
            venvRoot = fullfile(getenv('HOME'),'.matlab',['R' version('-release')]); %on Mac's using prefdir causes several issues due to space in folder names
        else
            venvRoot = prefdir;
        end
    end
    % Create Python venv folder name
    pyenvDir = fullfile(venvRoot,'ros1',computer('arch'),'venv');
    if ispc
        pyExec = fullfile(pyenvDir,'Scripts','python.exe');
    else
        pyExec = fullfile(pyenvDir,'bin','python3');
    end
    if isfile(pyExec)
        [stat,pyver] = system([pyExec ' --version']);
        if isequal(stat,0)
            pyver = strsplit(pyver);
            pyver = ros.internal.utilities.getVersionVal(pyver{2});
            forceRecreateVenv = forceRecreateVenv || (pyver < 30900);
        else
            % If the above system command fails it might be because python environemnt has corrupted.
            % So it's good to remove the existing environment and re-create
            % the python virtual environment.
            forceRecreateVenv = true;
        end
    else
        % If we dont find any executable named python3, then it means
        % something got corrupted and we need to remove the existing
        % environment and re-create it.
        forceRecreateVenv = true;
    end

    % If forceRecreateVENV is true, remove the venv to re-create it
    if isfolder(pyenvDir) && forceRecreateVenv
        dotprinter = ros.internal.DotPrinter('ros:utilities:util:RemovingPreviousEnv'); %#ok<NASGU>
        attempts = 0;
        while isfolder(pyenvDir) && attempts < 20
            status = rmdir(pyenvDir,'s');
            attempts = attempts + 1;
        end
        if isfolder(pyenvDir) && ~status %still exist
            ros.internal.utilities.rmdirusingsys(pyenvDir);
        end
        clear dotprinter;
    end

   
    % find which python we are using
    pe = pyenv;
    if ~isfile(pe.Executable)
        % If pyenv is not explicitly set, it returns a default location
        % which is not valid in some Linux distributions, notably
        % RedHat.
        error(message('ros:utilities:util:InvalidPythonExecutable',pe.Executable));
    end
    pyver = pe.Version;
    
    % Get version supported by MATLAB
    versionval = ros.internal.utilities.getVersionVal(pyver);
    
    % version distributed with MATLAB was tested with Python 3.9
    requiredMinVersion = '3.9';
    requiredMaxVersion = '3.9';
    requiredVersionVal = ros.internal.utilities.getVersionVal(requiredMinVersion);
    requiredMaxVersionVal = ros.internal.utilities.getVersionVal(requiredMaxVersion);
    if (versionval < requiredVersionVal) || (versionval > requiredMaxVersionVal)
        error(message('ros:utilities:util:RequirePython', requiredMinVersion));
    end
    
    % In mac and Linux, users might have symbolic links. We need to find the
    % real path
    % pyexec = char(py.os.path.realpath(pe.Executable));
    if ispc
        pyhome = fileparts(pe.Executable);
        pyexec = fullfile(char(pyhome),'python.exe');
        if ~isfile(pyexec)
            error(message('ros:utilities:util:InvalidPythonExecutable',pyexec));
        end
    else
        cmd = ['"' char(pe.Executable) '" -c "import os.path; print(os.path.realpath(''' char(pe.Executable) '''))"'];
        [stat, res] = system(cmd);
        if stat ~= 0
            error(message('ros:utilities:util:ErrorCreatingPythonVenv',res));
        end
        pyexec = strtrim(res);
    end
    
    if ~isfolder(pyenvDir)
        % Create Python virtual environment
        dotprinter = ros.internal.DotPrinter('ros:utilities:util:CreatingPythonVENV'); %#ok<NASGU>
        [stat, res] = system(['"' pyexec '" -m venv ' pyenvDir]); %do replace in place as exist accounts for space
        if stat ~= 0
            error(message('ros:utilities:util:ErrorCreatingPythonVenv', res));
        end
        % For MAC, we need configs also
        if ismac
            [stat, res] = system(['cp ',fullfile(fileparts(pyexec),'python*-config'),' ',fullfile(pyenvDir,'bin')]);
            if stat ~= 0
                error(message('ros:utilities:util:ErrorCreatingPythonVenv', res));
            end
        end
        clear dotprinter;
    end

    % activate local python
    if ispc
        localPythonPath = fullfile(pyenvDir,'Scripts','python.exe');
        activatePath = fullfile(pyenvDir,'Scripts','activate');
    else
        localPythonPath = fullfile(pyenvDir,'bin','python3');
        activatePath = fullfile(pyenvDir,'bin','activate');
    end
    if ~isfile(localPythonPath)
        error(message('ros:utilities:util:ErrorCreatingPython3VenvPyexecNotFound',...
            localPythonPath,'ros.internal.createOrGetLocalPython(true)'));
    end

    % find the last package that will be installed
    % if found, assuming all other packages installed properly
    if ispc
        catkindir = fullfile(pyenvDir,'Lib','site-packages','catkin_pkg');
        foundCatkin = isfolder(catkindir);
    else
        startSearch = fullfile(pyenvDir,'lib');
        startSearch = replace(startSearch, ' ', '\ ');
        [stat, res] = system(['find ' startSearch ' -type d -name catkin_pkg -print']);
        assert(stat == 0, res);
        foundCatkin = ~isempty(res);
    end

    %if not found install packages
    if ~foundCatkin
        dotprinter = ros.internal.DotPrinter('ros:utilities:util:AddingReqdPackages'); %#ok<NASGU>

        packagesToInstall = ' catkin_pkg empy docutils pyparsing python_dateutil pyyaml rosdep rosdistro rosinstall rosinstall_generator rospkg setuptools six vcstools wstool defusedxml';
        % key is to protect filepaths with spaces
        if ispc
            cmdToRun = ['"' activatePath '" && "' localPythonPath '" -m pip install --force --no-index --find-links="' fullfile(matlabroot,'sys','ros1','share','python') '"' packagesToInstall];
        else
            %for some reason activate command does not have exec bit set
            [stat, res] = system(['chmod +x ' replace(activatePath,' ','\ ')]);
            assert(stat == 0, res);
            cmdToRun1 = ['sh -c "PATH=' fileparts(pyexec) ' ; ' replace(activatePath,' ','\ ') ' ; ' replace(localPythonPath,' ','\ ') ' -m pip install --no-index --find-links=' fullfile(replace(matlabroot,' ','\ '),'sys','ros1','share','python') ' pip==21.2.2 setuptools==57.0.0' '"'];
            [stat, res] = system(cmdToRun1);
            if stat ~= 0
              error(message('ros:utilities:util:ErrorInstallingPackages', res));
            end 
            cmdToRun = ['sh -c "PATH=' fileparts(pyexec) ' ; ' replace(activatePath,' ','\ ') ' ; ' replace(localPythonPath,' ','\ ') ' -m pip install --no-index --find-links=' fullfile(replace(matlabroot,' ','\ '),'sys','ros1','share','python') packagesToInstall '"'];
        end
        [stat, res] = system(cmdToRun);
        if stat ~= 0
            error(message('ros:utilities:util:ErrorInstallingPackages', res));
        end
        clear dotprinter;
    end

        % Copy files in Python include folder if nexcessary
        pydestDirMap = containers.Map({'win64','maci64','glnxa64'}, ...
                                      { ...
                                          fullfile(pyenvDir,'Scripts'),...
                                          pyenvDir,...
                                          pyenvDir,...
                   });
        pydestDir = pydestDirMap(computer('arch'));
        filesInInclude = dir(fullfile(pyenvDir,'include'));
        if isempty(filesInInclude) || numel(filesInInclude) < 3
            pyIncludeDir = ros.internal.utilities.findPyIncludeDir(pyexec);
            if isempty(pyIncludeDir) || ~isfolder(pyIncludeDir)
                warning(message('ros:utilities:util:NoPythonIncludeDir',pyIncludeDir));
            else
                [status, msg] = copyfile(pyIncludeDir,fullfile(pydestDir,'include'),'f');
                if ~status
                    error(message('ros:utilities:util:ErrorCopyingFile',...
                        pyIncludeDir,fullfile(pydestDir,'include'),msg));
                end
            end
        end
    
        % Copy libraries if needed
        destLibMap = containers.Map({'win64','maci64','glnxa64'}, ...
            { ...
            'python*.dll' ...
            'libpython*.dylib',...
            'libpython*.so',...
            });
        destMap = containers.Map({'win64','maci64','glnxa64'}, ...
            { ...
            fullfile(pydestDir,'libs') ...
            fullfile(pydestDir,'lib'),...
            fullfile(pydestDir,'lib'),...
            });
        dest = destMap(computer('arch'));
        filesInLib = dir(fullfile(dest,destLibMap(computer('arch'))));
        if isempty(filesInLib)
            pyLibDir = ros.internal.utilities.findPyLibDir(pyexec);
            if isempty(pyLibDir) || ~isfolder(pyLibDir)
                warning(message('ros:utilities:util:NoPythonLibDir',...
                    destLibMap(computer('arch'))));
            else
                srcMap = containers.Map({'win64','maci64','glnxa64'}, ...
                    { ...
                    fullfile(pyLibDir,'*.*') ...
                    fullfile(pyLibDir,'libpython*.*'),...
                    fullfile(pyLibDir,'libpython*.*'),...
                    });
                src = srcMap(computer('arch'));
                if ~isempty(dir(src))
                    [status, msg] = copyfile(src,dest,'f');
                    if ~status
                        error(message('ros:utilities:util:ErrorCopyingFile',...
                            src,dest,msg));
                    end
                end
            end
        end
    end

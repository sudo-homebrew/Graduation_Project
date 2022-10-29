function pyLibDir = findPyLibDir(pyexec)
%This function is for internal use only. It may be removed in the future.

%findPyLibDir Return Python library directory

%   Copyright 2020 The MathWorks, Inc.

% There is no portable way to find where libpython2.7.SONAME is
pyLibDir = '';
if ispc
    % In Windows we get the value of BINDIR and append libs to it
    cmd = ['"' pyexec '" -c "from distutils.sysconfig import get_config_var; ',...
        'print(get_config_var(\"BINDIR\"))"'];
    [st,res] = system(cmd);
    if st == 0
        binDir = fullfile(strtrim(res),'libs');
    end
    if ~isempty(binDir) && ~isempty(dir(fullfile(binDir,'python*.*')))
        pyLibDir = binDir;
        return;
    end
    
    % Try finding lib folder in the vicinity of Python install folder
    pyRoot = fileparts(pyexec);
    if isfolder(fullfile(pyRoot,'libs')) && ~isempty(dir(fullfile(pyRoot,'libs','python*.*')))
        pyLibDir = fullfile(pyRoot,'libs');
        return;
    end
    pyRoot = fileparts(pyRoot);
    if isfolder(fullfile(pyRoot,'libs')) && ~isempty(dir(fullfile(pyRoot,'libs','python*.*')))
        pyLibDir = fullfile(pyRoot,'libs');
        return;
    end
else    
    % MAC OS & Linux. Check LIBDIR and LIBPL configuration variables to
    % determine location of libpython
    cmd = ['"' pyexec '" -c "from distutils.sysconfig import get_config_var; print get_config_var(''LIBDIR'')"'];
    [st,res] = system(cmd);
    if st == 0
        libDir = strtrim(res);
        if ~isempty(libDir) && ~isempty(dir(fullfile(libDir,'libpython*.*')))
            pyLibDir = libDir;
            return;
        end
    end
    
    % Try LIBPL if LIBDIR does not provide the location of libpython
    cmd = ['"' pyexec '" -c "from distutils.sysconfig import get_config_var; print get_config_var(''LIBPL'')"'];
    [st,res] = system(cmd);
    if st == 0
        libPl = strtrim(res);
        if ~isempty(libPl) && ~isempty(dir(fullfile(libPl,'libpython*.*')))
            pyLibDir = libPl;
            return;
        end
    end
    
    % Try finding lib folder in the vicinity of Python install folder
    pyRoot = fileparts(pyexec);
    if isfolder(fullfile(pyRoot,'lib')) && ~isempty(dir(fullfile(pyRoot,'lib','libpython*.*')))
        pyLibDir = fullfile(pyRoot,'lib');
        return;
    end
    pyRoot = fileparts(pyRoot);
    if isfolder(fullfile(pyRoot,'lib')) && ~isempty(dir(fullfile(pyRoot,'lib','libpython*.*')))
        pyLibDir = fullfile(pyRoot,'lib');
        return;
    end
end
end
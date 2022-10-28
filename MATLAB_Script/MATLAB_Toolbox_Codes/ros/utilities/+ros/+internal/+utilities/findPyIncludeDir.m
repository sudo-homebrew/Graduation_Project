function pyIncludeDir = findPyIncludeDir(pyexec)
%This function is for internal use only. It may be removed in the future.

%findPyIncludeDir Return Python include directory

%   Copyright 2020-2021 The MathWorks, Inc.

% The following command will run fine in all platforms. No need to specify
% bash as shell interpreter for *NIX as we are not using bash specific
% syntax
pyIncludeDir = '';
cmd = ['"' pyexec '" -c "from sysconfig import get_paths as gp; print(gp()[''include''])"'];
[st,res] = system(cmd);
if st == 0
    pyIncludeDir = strtrim(res);
end
end
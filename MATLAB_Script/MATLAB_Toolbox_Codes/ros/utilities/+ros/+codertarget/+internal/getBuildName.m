function buildName = getBuildName(buildInfo)
%This function is for internal use only. It may be removed in the future.
%
%GETBUILDNAME Return build name

% Copyright 2020 The MathWorks, Inc.

% BuildInfo from MATLAB Coder does not accurately reflect the entry point
% function name. It sometimes gives project name and sometimes function
% name. Here we derive buildName out of build folder.
buildName = getBuildName(buildInfo);
buildDir = ros.codertarget.internal.getBuildDir(buildInfo);
[~,buildNameFromDir] = fileparts(buildDir);
if ~isequal(buildName,buildNameFromDir)
    buildName = buildNameFromDir;
end

end


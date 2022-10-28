function buildDir = getBuildDir(buildInfo)
%This function is for internal use only. It may be removed in the future.
%
%GETBUILDNAME Return build name

% Copyright 2020 The MathWorks, Inc.

buildDir = getLocalBuildDir(buildInfo);
if isempty(buildDir) || ~isfolder(buildDir)
    buildDir = getSourcePaths(buildInfo,true,{'BuildDir'});
    if isempty(buildDir)
        buildDir = {pwd};
    end
    buildDir = buildDir{1};
end
end


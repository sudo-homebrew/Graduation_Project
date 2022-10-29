function [pkgDirs, whichPkgs] = getPkgDirs(folderPath)
%This function is for internal use only. It may be removed in the future.

%GETPKGDIRS returns the directories which has msg packages in it.

%   Copyright 2020-2021 The MathWorks, Inc.

% Copy only message package sources (contains "msg" subdirectory)
dirInfo = dir(folderPath);
whichPkgs = [dirInfo.isdir] & ~ismember({dirInfo.name}, {'.', '..'});
for iPkg = find(whichPkgs)
    whichPkgs(iPkg) = isfolder(fullfile(folderPath, dirInfo(iPkg).name, 'msg')) || isfolder(fullfile(folderPath, dirInfo(iPkg).name, 'srv')) || isfolder(fullfile(folderPath, dirInfo(iPkg).name, 'action'));
end
%these directories have msg files that need to be built
pkgDirs = {dirInfo(whichPkgs).name};

end


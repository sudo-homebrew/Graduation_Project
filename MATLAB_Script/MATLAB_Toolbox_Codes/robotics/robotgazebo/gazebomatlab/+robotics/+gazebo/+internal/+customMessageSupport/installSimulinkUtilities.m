function installSimulinkUtilities(folderPathList)
%This function is for internal use only. It may be removed in the future.
%
%This function copies simulink utilities files into install folder
%

%   Copyright 2019 The MathWorks, Inc.

% copy Simulink message files into install folder
    src = folderPathList.simulinkUtilitiesFolderPath;
    dst = folderPathList.installFolderPath;
    copyfile(src, dst);

end

function installSimulinkMessages(folderPath)
%This function is for internal use only. It may be removed in the future.
%
%This function creates install folder and copies Simulink message files
%

%   Copyright 2019 The MathWorks, Inc.
%

% create install folder and sub-folders
    installFolderPath = fullfile(folderPath,'install');
    roboticsFolderPath = fullfile(installFolderPath,'+robotics');
    slgazeboFolderPath = fullfile(roboticsFolderPath,'+slgazebo');
    internalFolderPath = fullfile(slgazeboFolderPath,'+internal');
    msgsFolderPath = fullfile(internalFolderPath,'+msgs');

    mkdirIfNeeded(msgsFolderPath);

    % copy Simulink message files into install folder
    src = fullfile( folderPath, 'SimulinkMessage');
    dst = msgsFolderPath;
    copyfile(src, dst);

    % copy Gazebo_Msgs class into install folder
    src = fullfile( matlabroot, 'toolbox','robotics','robotgazebo','gazebomatlab','+robotics','+gazebo','+internal','+customMessageSupport','Gazebo_Msgs.m');
    dst = msgsFolderPath;
    copyfile(src, dst);

end

function mkdirIfNeeded(p)
    if ~exist(p, 'dir')
        mkdir(p);
    end
end

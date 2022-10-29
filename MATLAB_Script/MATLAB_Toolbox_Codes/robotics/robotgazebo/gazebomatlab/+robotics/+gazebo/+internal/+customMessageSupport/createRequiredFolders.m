function folderPathList = createRequiredFolders(folderPath)
%This function is for internal use only. It may be removed in the future.
%
% This creates folders required to store generated custom message utilities

%   Copyright 2019-2020 The MathWorks, Inc.
    if ( exist(folderPath,'dir'))
        % store folder path in struct
        folderPathList.simulinkMsgFolderPath = fullfile(folderPath,'SimulinkMessage');
        folderPathList.installFolderPath = fullfile(folderPath,'install');
        folderPathList.simulinkToPacketFolderPath = fullfile(folderPath,'SimulinkToPacket');
        folderPathList.packetToSimulinkFolderPath = fullfile(folderPath,'PacketToSimulink');
        folderPathList.customMsgHandlerFolderPath = fullfile(folderPath,'customMsgHandlers');
        folderPathList.customMsgHandlerSrcFolderPath = fullfile(folderPath,'customMsgHandlers','src');
        folderPathList.customMsgHandlerIncludeFolderPath = fullfile(folderPath,'customMsgHandlers','include');
        folderPathList.pluginUtilitiesFolderPath = fullfile(folderPath,'pluginUtilities');
        folderPathList.pluginUtilitiesSrcFolderPath = fullfile(folderPath,'pluginUtilities','src');
        folderPathList.pluginUtilitiesWorldFolderPath = fullfile(folderPath,'pluginUtilities','world');
        folderPathList.simulinkUtilitiesFolderPath = fullfile(folderPath,'simulinkUtilities');
        folderPathList.protoFilesFolderPath = fullfile(folderPath,'ProtoFiles');

        % create folder and copy .proto file
        cleanMakeDir(folderPathList.simulinkMsgFolderPath);
        cleanMakeDir(folderPathList.installFolderPath);
        cleanMakeDir(folderPathList.simulinkToPacketFolderPath);
        cleanMakeDir(folderPathList.packetToSimulinkFolderPath);
        cleanMakeDir(folderPathList.customMsgHandlerFolderPath);
        cleanMakeDir(folderPathList.customMsgHandlerSrcFolderPath);
        cleanMakeDir(folderPathList.customMsgHandlerIncludeFolderPath);
        cleanMakeDir(folderPathList.pluginUtilitiesFolderPath);
        cleanMakeDir(folderPathList.pluginUtilitiesSrcFolderPath);
        cleanMakeDir(folderPathList.pluginUtilitiesWorldFolderPath);
        cleanMakeDir(folderPathList.simulinkUtilitiesFolderPath);
        cleanMakeDir(folderPathList.protoFilesFolderPath);

    else
        folderPathList.simulinkMsgFolderPath = '';
        folderPathList.installFolderPath = '';
        folderPathList.simulinkToPacketFolderPath = '';
        folderPathList.packetToSimulinkFolderPath = '';
        folderPathList.customMsgHandlerFolderPath = '';
        folderPathList.customMsgHandlerSrcFolderPath = '';
        folderPathList.customMsgHandlerIncludeFolderPath = '';
        folderPathList.pluginUtilitiesFolderPath = '';
        folderPathList.pluginUtilitiesSrcFolderPath = '';
        folderPathList.pluginUtilitiesWorldFolderPath = '';
        folderPathList.simulinkUtilitiesFolderPath = '';
        folderPathList.protoFilesFolderPath = '';

    end

end

function cleanMakeDir(p)
    if ~exist(p, 'dir')
        % make
        mkdir(p);
    else
        %clean and make
        rmdir(p,'s');
        mkdir(p);
    end
end

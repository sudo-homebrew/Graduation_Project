function createCppCustomHandlersSourceFiles(customMsgStruct, PluginHandlerFolderPath)
%This function is for internal use only. It may be removed in the future.
%
%This function generates custom message handlers (.cpp and .hpp files) which
% converts Co-Sim Packet to Custom Message and vice-versa on Plugin side
%

%   Copyright 2019-2020 The MathWorks, Inc.

%% retrieve proto message name with namespace in Simulink message format
    [messsageName, nameSpaceString] = robotics.gazebo.internal.customMessageSupport.getMsgNameWithNamespace...
        (customMsgStruct.messageNameSpaceList, customMsgStruct.messageBaseName,'Simulink');

    %% read template of source (.cpp file) of custom message handlers
    cppHandlerString = fileread(fullfile(matlabroot, 'toolbox', 'robotics', 'robotgazebo','gazebomatlab',...
                                         '+robotics','+gazebo','+internal','+customMessageSupport','+templates','customMessageHandler.cpp.tmpl'));
    % add parent message name in the cpp handler string
    cppHandlerString = strrep(cppHandlerString,'{% msgInfo.MessageName %}',messsageName);
    cppHandlerString = strrep(cppHandlerString,'{% msgInfo.MessageBaseName %}',char(customMsgStruct.messageBaseName));
    cppHandlerString = strrep(cppHandlerString,'{% msgInfo.MessageNameSpace %}',nameSpaceString);

    % create custom message handlers .cpp file
    cppHandlerName = [messsageName,'CustomMessageHandler.cpp'];
    robotics.gazebo.internal.customMessageSupport.createFile(cppHandlerString,fullfile(PluginHandlerFolderPath,'src',cppHandlerName));

    %% read template of header (.hpp file) of custom message handlers
    hppHandlerString = fileread(fullfile(matlabroot, 'toolbox', 'robotics', 'robotgazebo', 'gazebomatlab',...
                                         '+robotics','+gazebo','+internal','+customMessageSupport','+templates','customMessageHandler.hpp.tmpl'));

    % add parent message name in the hpp handler string
    hppHandlerString = strrep(hppHandlerString,'{% msgInfo.MessageName %}',messsageName);
    hppHandlerString = strrep(hppHandlerString,'{% msgInfo.MessageBaseName %}',char(customMsgStruct.messageBaseName));
    hppHandlerString = strrep(hppHandlerString,'{% msgInfo.MessageNameSpace %}',nameSpaceString);
    hppHandlerString = strrep(hppHandlerString,'{% msgInfo.MessageNameCapital %}',upper(messsageName));
    hppHandlerString = strrep(hppHandlerString,'{% msgInfo.MessageFileName %}',erase(customMsgStruct.protoFileName,'.proto'));


    % create custom message handlers .hpp file
    hppHandlerName = [messsageName,'CustomMessageHandler.hpp'];
    robotics.gazebo.internal.customMessageSupport.createFile(hppHandlerString,fullfile(PluginHandlerFolderPath,'include',hppHandlerName));


end

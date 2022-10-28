function createSimulinkUtilitiesFiles(customMsgStruct, installFolderPath, sharedLibraryName )
%This function is for internal use only. It may be removed in the future.
%
% This function generates Simulink utilities files like importPacketField,
% importCustomMsgField, importCustomMessageList, importCustomCommandList
% and importGazeboCustomMessageBlkTemplate. This files called out from
% corresponding MATLAB files while running Simulink.
%

%   Copyright 2019-2020 The MathWorks, Inc.

    %% reads templates of MATLAB utilities file
    getPacketLog = fileread(fullfile(matlabroot, 'toolbox', 'robotics', 'robotgazebo', 'gazebomatlab',...
                                     '+robotics','+gazebo','+internal','+customMessageSupport','+templates','importPacketField.m.tmpl'));
    getFieldLog = fileread(fullfile(matlabroot, 'toolbox', 'robotics', 'robotgazebo', 'gazebomatlab',...
                                    '+robotics','+gazebo','+internal','+customMessageSupport','+templates','importCustomMsgField.m.tmpl'));
    getMessageListLog = fileread(fullfile(matlabroot, 'toolbox', 'robotics', 'robotgazebo', 'gazebomatlab',...
                                          '+robotics','+gazebo','+internal','+customMessageSupport','+templates','importCustomMessageList.m.tmpl'));
    getMessageTypeListLog = fileread(fullfile(matlabroot, 'toolbox', 'robotics', 'robotgazebo', 'gazebomatlab',...
                                              '+robotics','+gazebo','+internal','+customMessageSupport','+templates','importCustomMessageTypeList.m.tmpl'));
    getCommandListLog = fileread(fullfile(matlabroot, 'toolbox', 'robotics', 'robotgazebo', 'gazebomatlab',...
                                          '+robotics','+gazebo','+internal','+customMessageSupport','+templates','importCustomCommandList.m.tmpl'));
    getTemplateLog = fileread(fullfile(matlabroot, 'toolbox', 'robotics', 'robotgazebo', 'gazebomatlab',...
                                       '+robotics','+gazebo','+internal','+customMessageSupport','+templates','importGazeboCustomMessageBlkTemplate.m.tmpl'));

    % gazebo message type prefix
    gazeboMsgNameExt = char(robotics.gazebo.internal.topic.Category.CustomMessageName);
    gazeboMsgTypeExt = char(robotics.gazebo.internal.topic.Category.CustomMessageType);

    %% importCustomMessageList generation
    % This includes all custom message list, which will called in the
    % getCustomMessageList function

    customMsgString = "";

    for idx0 = 1:length(customMsgStruct)

        % create simulink message name
        customMsgString = join([customMsgString,"'",gazeboMsgTypeExt,customMsgStruct{idx0}.simulinkMessageName,"'"],"");

        if ( idx0 < length(customMsgStruct))
            customMsgString = join([customMsgString,",...", newline],"");
        end

    end

    % add custom message list in the template string
    getMessageListLog =  strrep(getMessageListLog,'{% msgInfo.messageList %}',customMsgString);

    % create importCustomMessageList file
    robotics.gazebo.internal.customMessageSupport.createFile(getMessageListLog,fullfile(installFolderPath,'importCustomMessageList.m'));

    %% importCustomMessageTypeList generation
    % This includes all custom message type list, which will called in the
    % simulink topic read function

    %customMsgString = "";
    customMsgTypeString = "'TestPose',";
    customMsgNameString = "'gazebo_msgs/TestPose',";

    for idx0 = 1:length(customMsgStruct)

        % retrieve proto namespace and replace '::' with '.'
        messageTypeName0 = strrep(char(customMsgStruct{idx0}.messageNameSpaceList),'::','.');
        % retrieve proto message name and create message type seen gazebo
        messageTypeName = customMsgStruct{idx0}.messageBaseName;
        if(~isempty(messageTypeName0))
            messageTypeName = [messageTypeName0,'.',messageTypeName];
        end

        % create simulink message type name
        customMsgTypeString = join([customMsgTypeString,"'",messageTypeName,"'"],"");

        % create simulink message name
        customMsgNameString = join([customMsgNameString,"'",gazeboMsgTypeExt,customMsgStruct{idx0}.simulinkMessageName,"'"],"");

        if ( idx0 < length(customMsgStruct))
            customMsgTypeString = join([customMsgTypeString,",...", newline],"");
            customMsgNameString = join([customMsgNameString,",...", newline],"");
        end

    end

    % add custom message list in the template string
    getMessageTypeListLog =  strrep(getMessageTypeListLog,'{% msgInfo.messageTypeList %}',customMsgTypeString);
    getMessageTypeListLog =  strrep(getMessageTypeListLog,'{% msgInfo.messageNameList %}',customMsgNameString);

    % create importCustomMessageList file
    robotics.gazebo.internal.customMessageSupport.createFile(getMessageTypeListLog,fullfile(installFolderPath,'importCustomMessageTypeList.m'));

    %% importCustomCommandList generation
    % This includes all custom command list, which will called in the
    % getCustomCommandList function

    customCommandString = "'TestPose',";

    for idx1 = 1:length(customMsgStruct)

        % create command name for custom message
        customCommandString = join([customCommandString,"'",customMsgStruct{idx1}.simulinkMessageName,"'"],"");

        if ( idx1 < length(customMsgStruct))
            customCommandString = join([customCommandString,",...", newline],"");
        end

    end

    % add custom command list in the template string
    getCommandListLog =  strrep(getCommandListLog,'{% msgInfo.commandList %}',customCommandString);

    % create importCustomCommandList file
    robotics.gazebo.internal.customMessageSupport.createFile(getCommandListLog,fullfile(installFolderPath,'importCustomCommandList.m'));

    %% importPacketField generation

    getPacketFieldString = join(["case 'gazebo_msgs/TestPose'",newline,"      packetData = [];",newline,"     "],"");

    for idx2 = 1: length(customMsgStruct)
        % create switch case string for custom message
        if(~isempty(customMsgStruct{idx2}.messageNameSpaceList))
            messsageName = [char(strrep(customMsgStruct{idx2}.messageNameSpaceList{1},'::','_')),'_',char(customMsgStruct{idx2}.messageBaseName)];
        else
            messsageName = char(customMsgStruct{idx2}.messageBaseName);
        end
        getPacketFieldString = join([getPacketFieldString, "case ","'",gazeboMsgTypeExt,customMsgStruct{idx2}.simulinkMessageName,"'",newline,...
                            "       packetData = simulinkToPacket_",lower(string(messsageName)),"(cmd);",newline],"");

    end

    % add switch case string in the template string
    getPacketLog =  strrep(getPacketLog,'{% msgInfo.SimToPacketSwitchCase %}',getPacketFieldString);

    % create importPacketField file
    robotics.gazebo.internal.customMessageSupport.createFile(getPacketLog,fullfile(installFolderPath,'importPacketField.m'));

    %% importCustomMsgField generation

    getCustomMsgFieldString = join(["case 'gazebo_msgs/TestPose'",newline,"      result = [];",newline,"     "],"");

    for idx3 = 1: length(customMsgStruct)

        % create switch case string for custom message
        if(~isempty(customMsgStruct{idx3}.messageNameSpaceList))
            messsageName = [char(strrep(customMsgStruct{idx3}.messageNameSpaceList{1},'::','_')),'_',char(customMsgStruct{idx3}.messageBaseName)];
        else
            messsageName = char(customMsgStruct{idx3}.messageBaseName);
        end
        getCustomMsgFieldString = join([getCustomMsgFieldString, "case ","'",gazeboMsgTypeExt,customMsgStruct{idx3}.simulinkMessageName,...
                            "'",newline,"       result = packetToSimulink_",...
                            lower(string(messsageName)),"(customData.message.data');",newline],"");

    end

    % add switch case string in the template string
    getFieldLog =  strrep(getFieldLog,'{% msgInfo.PacketToSimSwitchCase %}',getCustomMsgFieldString);

    % create importCustomMsgField file
    robotics.gazebo.internal.customMessageSupport.createFile(getFieldLog,fullfile(installFolderPath,'importCustomMsgField.m'));

    %% importGazeboCustomMessageBlkTemplate generation

    getCustomTemplateString = "";

    for idx4 = 1: length(customMsgStruct)

        % create switch case string for custom message
        getCustomTemplateString = join([getCustomTemplateString, "case ",num2str(idx4-1),...
                            newline,"       template = robotics.slgazebo.internal.msgs.",gazeboMsgNameExt,customMsgStruct{idx4}.simulinkMessageFileName,";",newline],"");

    end

    % add switch case string in the template string
    getTemplateLog =  strrep(getTemplateLog,'{% msgInfo.TemplateSwitchCase %}',getCustomTemplateString);

    % create importGazeboCustomMessageBlkTemplate file
    robotics.gazebo.internal.customMessageSupport.createFile(getTemplateLog,fullfile(installFolderPath,'importGazeboCustomMessageBlkTemplate.m'));

    %% create loadlibrary function

    % create loadlibrary header
    loadLibString = join(["%Copyright 2019 The MathWorks, Inc.",newline,newline,...
                        "if ~libisloaded( 'lib",sharedLibraryName,"' )",newline,...
                        "    loadlibrary lib",sharedLibraryName," lib",sharedLibraryName,".h ",newline...
                        "end",newline,],"");

    newHeaderFile = fopen(fullfile(installFolderPath,'importGazeboCustomLoadLibrary.m'), 'wt');
    fprintf(newHeaderFile, '%s\n\n', loadLibString);
    fclose(newHeaderFile);


    %% create unloadlibrary function

    % create loadlibrary header
    unloadLibString = join(["%Copyright 2019 The MathWorks, Inc.",newline,newline,...
                        "if libisloaded( 'lib",sharedLibraryName,"' )",newline,...
                        "    unloadlibrary lib",sharedLibraryName,newline...
                        "end",newline,],"");

    newHeaderFile = fopen(fullfile(installFolderPath,'importGazeboCustomUnloadLibrary.m'), 'wt');
    fprintf(newHeaderFile, '%s\n\n', unloadLibString);
    fclose(newHeaderFile);

end

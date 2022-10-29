function cppFileName = createPacketToSimulinkMEXSourceFile(customMsgStruct, folderPath)
%This function is for internal use only. It may be removed in the future.
%
%This function generates cpp file which converts Packet message to Simulink message

%   Copyright 2019-2020 The MathWorks, Inc.

%% read cpp template of packetToSimulink conversion file
    messageDefinationString = "";
    createMessageString = "";

    packetSimulinkLog = fileread(fullfile(matlabroot, 'toolbox', 'robotics', 'robotgazebo', 'gazebomatlab',...
                                          '+robotics','+gazebo','+internal','+customMessageSupport','+templates','packetToSimulink.cpp.tmpl'));

    protoFileName = erase(customMsgStruct.protoFileName,'.proto');
    % add proto file header
    packetSimulinkLog = strrep(packetSimulinkLog,'{% msgInfo.MessageFileName %}',protoFileName);

    % add message definition string inside cpp template string
    packetSimulinkLog = strrep(packetSimulinkLog,'{% msgInfo.MessageDefinition %}',customMsgStruct.messageBaseName);

    variableName = join([lower(customMsgStruct.messageBaseName),"_msg"],"");

    % declaring variable : TestMessage* testmessage_msg;
    MsgDefString = join(["std::shared_ptr<",customMsgStruct.messageBaseName,"> ",variableName,";",newline],"");
    messageDefinationString = join([messageDefinationString,MsgDefString]);

    % allocating variable : testmessage_msg = new TestMessage();
    MessageString = join([variableName," = "," std::make_shared<",customMsgStruct.messageBaseName,">();",newline],"");
    createMessageString = join([createMessageString,MessageString]);

    % retrieve namespace of current .proto message file
    nameSpaceString = "";
    for nIdx = 1:length(customMsgStruct.messageNameSpaceList)
        nameSpaceString = join([nameSpaceString,"using namespace ",string(customMsgStruct.messageNameSpaceList{nIdx}),";",newline],"");
    end


    packetSimulinkLog = strrep(packetSimulinkLog,'{% msgInfo.MessageNameSpace %}',nameSpaceString);
    packetSimulinkLog = strrep(packetSimulinkLog,'{% msgInfo.MessageName %}',variableName);
    packetSimulinkLog = strrep(packetSimulinkLog,'{% msgInfo.MessageDefinitionPtr %}', messageDefinationString);
    packetSimulinkLog = strrep(packetSimulinkLog,'{% msgInfo.createMessageDefinition %}',createMessageString);

    %% retrieve proto message name with namespace in Simulink message format
    [messsageName, ~ ]  = robotics.gazebo.internal.customMessageSupport.getMsgNameWithNamespace...
        (customMsgStruct.messageNameSpaceList, customMsgStruct.messageBaseName,'Simulink');

    cppFileName = fullfile(folderPath,['packetToSimulink_',lower(messsageName),'.cpp']);
    robotics.gazebo.internal.customMessageSupport.createFile(packetSimulinkLog, cppFileName);

end

function cppFileName = createSimulinkToPacketMEXSourceFile(customMsgStruct, folderPath)
%This function is for internal use only. It may be removed in the future.
%
% This function creates SimulinkToPacket conversion cpp file
% Here, message field names and variable are provided as struct input.
% Further, this function generates field string which contains cpp variable
% definitions and initializations.
% In addition, based on cpp template, this function creates
% SimulinkToPacket cpp file for each proto file
%

%   Copyright 2019-2020 The MathWorks, Inc.
%
%% read cpp template of simulink to packet conversion file
    simulinkPacketLog = fileread(fullfile(matlabroot, 'toolbox', 'robotics', 'robotgazebo', 'gazebomatlab',...
                                          '+robotics','+gazebo','+internal','+customMessageSupport','+templates','simulinkToPacket.cpp.tmpl'));

    %%
    messageDefinationString = "";
    createMessageString = "";

    protoFileName = erase(customMsgStruct.protoFileName,'.proto');
    % add proto file header
    simulinkPacketLog = strrep(simulinkPacketLog,'{% msgInfo.MessageFileName %}',protoFileName);

    % create variable name; for message name = TestMessage, variable name = testmessage_msg
    variableName = join([lower(customMsgStruct.messageBaseName),"_msg"],"");

    % serializing string syntax : std::string msg = testmessage_msg->SerializeAsString();
    dataFieldString = join(['std::string msg = ',variableName,'->SerializeAsString();'],"");

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

    simulinkPacketLog = strrep(simulinkPacketLog,'{% msgInfo.MessageName %}',variableName);
    simulinkPacketLog = strrep(simulinkPacketLog,'{% msgInfo.MessageNameSpace %}',nameSpaceString);
    simulinkPacketLog = strrep(simulinkPacketLog,'{% msgInfo.CustomMsgSupportData %}',dataFieldString);
    simulinkPacketLog = strrep(simulinkPacketLog,'{% msgInfo.MessageDefinition %}', messageDefinationString);
    simulinkPacketLog = strrep(simulinkPacketLog,'{% msgInfo.createMessageDefinition %}',createMessageString);

    %% retrieve proto message name with namespace in Simulink message format
    [messsageName, ~ ]  = robotics.gazebo.internal.customMessageSupport.getMsgNameWithNamespace...
        (customMsgStruct.messageNameSpaceList, customMsgStruct.messageBaseName,'Simulink');

    cppFileName = fullfile(folderPath,['simulinkToPacket_',lower(messsageName),'.cpp']);
    robotics.gazebo.internal.customMessageSupport.createFile(simulinkPacketLog, cppFileName );

end

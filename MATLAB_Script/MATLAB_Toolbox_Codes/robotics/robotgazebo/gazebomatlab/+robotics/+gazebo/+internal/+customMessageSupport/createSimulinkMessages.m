function createSimulinkMessages(protoFileStruct, customDetails, simulinkMsgFolderPath)
%This function is for internal use only. It may be removed in the future.
%
% This function creates simulink messages based on input MATLAB struct.
% The input MATLAB struct contains proto message details like variable
% names, datatypes, field types, etc.
%

%   Copyright 2019-2020 The MathWorks, Inc.
%
% input :
%               @param protoFileStruct           MATLAB struct containing proto message details
%               @param simulinkMsgFolderPath     Folder path of simulink message folder
%

% full path of simulink message template
    simulinkMessageTempPath = fullfile(matlabroot, 'toolbox', 'robotics', 'robotgazebo', 'gazebomatlab',...
                                       '+robotics','+gazebo','+internal','+customMessageSupport','+templates','simulinkMessage.m.tmpl');

    localFieldStruct.messageNameList = strings(0);
    localFieldStruct.parentMessageNameList = strings(0);
    localFieldStruct.enumNameList = strings(0);
    localFieldStruct.enumFieldList = {};
    localFieldStruct.enumFieldValueList = {};
    localFieldStruct.nameSpaceList = strings(0);
    localFieldStruct.simulinkMessageName = strings(0);
    localFieldStruct.simulinkTypeName = strings(0);

    % extract all message and enum names present in the all proto struct
    entryId0 = 1;
    entryId1 = 1;
    for idx0 = 1: length(protoFileStruct)
        for idx1 = 1: length(protoFileStruct{idx0}.messageStruct)
            if(~isempty(protoFileStruct{idx0}.messageStruct{idx1}.messageName))
                localFieldStruct.messageNameList(entryId0) = string(protoFileStruct{idx0}.messageStruct{idx1}.messageName);
                localFieldStruct.simulinkMessageName(entryId0) = string(protoFileStruct{idx0}.simulinkMessageName{idx1});
                localFieldStruct.simulinkTypeName(entryId0) = string(protoFileStruct{idx0}.simulinkTypeName{idx1});

                if(isempty(protoFileStruct{idx0}.messageNameSpace))
                    localFieldStruct.nameSpaceList(entryId0) = '';
                else
                    localFieldStruct.nameSpaceList(entryId0) = string(protoFileStruct{idx0}.messageNameSpace);
                end

                if(isempty(protoFileStruct{idx0}.messageStruct{idx1}.parentMessageNames))
                    localFieldStruct.parentMessageNameList(entryId0) = '';
                else
                    % if want nested and more parent msg then remove
                    % parentMessageNames(end) => parentMessageNames
                    localFieldStruct.parentMessageNameList(entryId0) = string(protoFileStruct{idx0}.messageStruct{idx1}.parentMessageNames(end));
                end

                entryId0 = entryId0 + 1;
            end
            if(~isempty(protoFileStruct{idx0}.messageStruct{idx1}.enumName))
                for idx2 = 1:length(protoFileStruct{idx0}.messageStruct{idx1}.enumName)
                    localFieldStruct.enumNameList(entryId1) = string( protoFileStruct{idx0}.messageStruct{idx1}.enumName{idx2});
                    localFieldStruct.enumFieldList{end+1} = {string( protoFileStruct{idx0}.messageStruct{idx1}.enumFields{idx2})};
                    localFieldStruct.enumFieldValueList{end+1} = {string(protoFileStruct{idx0}.messageStruct{idx1}.enumFieldsValue{idx2})};
                    entryId1 = entryId1 + 1;
                end
            end
        end
    end

    for idx0 = 1: length(protoFileStruct)
        for idx1 = 1: length(protoFileStruct{idx0}.messageStruct)

            customMessageStruct = protoFileStruct{idx0};

            % retrieve Simulink message file name and message type
            simulinkMessageName = customMessageStruct.simulinkMessageName{idx1};
            simulinkMessageTypeName = customMessageStruct.simulinkTypeName{idx1};

            % create simulink message file
            simMsgFileName = join(['Gazebo_msgs_custom_',simulinkMessageName,'.m'],"");

            % generate simulink message string
            simMsgString  = robotics.gazebo.internal.customMessageSupport.generateSimulinkMsgString(...
                customMessageStruct.messageStruct{idx1}, localFieldStruct, customDetails, ...
                simulinkMessageName , simulinkMessageTypeName, simulinkMessageTempPath);

            robotics.gazebo.internal.customMessageSupport.createFile(simMsgString,fullfile(simulinkMsgFolderPath,simMsgFileName));

        end
    end

end

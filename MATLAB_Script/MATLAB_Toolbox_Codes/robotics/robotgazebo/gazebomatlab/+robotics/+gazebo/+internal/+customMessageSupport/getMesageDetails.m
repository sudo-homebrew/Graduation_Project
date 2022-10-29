function customDetails = getMesageDetails(customMessageStruct)
%This function is for internal use only. It may be removed in the future.
%
% This function retrieves message details from custom message struct
% These details are needed in creating C++ source files.
% These details are all message names, its parent names and
% all repeated message names

%   Copyright 2019-2020 The MathWorks, Inc.

% store all message names and its parent names
    customDetails.messageNameList = {};

    customDetails.parentMessageNameList  = {};

    % store namespace
    customDetails.nameSpace = {};

    % store Simulink Message name
    customDetails.simulinkMessageName = {};

    % store Simulink Message Type name
    customDetails.simulinkTypeName = {};

    % retrieve all repeated field names
    customDetails.repeatedMessageName = {};

    % store custom message '.proto' file names
    customDetails.protoFileName = {};

    % store enum fields per message
    customDetails.enumFieldName = {};

    % store enum names per message
    customDetails.enumFields = {};

    % store enum values per enum name
    customDetails.enumFieldsValue = {};

    % store details needed to generate utility files
    customDetails.utilities = {};

    for Idx0 = 1: length(customMessageStruct)
        for Idx1 = 1:length(customMessageStruct{Idx0})

            for Idx2 = 1: length(customMessageStruct{Idx0}{Idx1}.messageStruct)

                customDetails.messageNameList{end+1}  = customMessageStruct{Idx0}{Idx1}.messageStruct{Idx2}.messageName;
                customDetails.parentMessageNameList{end+1}   = customMessageStruct{Idx0}{Idx1}.messageStruct{Idx2}.parentMessageNames;
                customDetails.nameSpace{end+1} = customMessageStruct{Idx0}{Idx1}.messageNameSpace;
                customDetails.simulinkMessageName{end+1} = customMessageStruct{Idx0}{Idx1}.simulinkMessageName{Idx2};
                customDetails.simulinkTypeName{end+1} = customMessageStruct{Idx0}{Idx1}.simulinkTypeName{Idx2};
                customDetails.protoFileName{end+1} = customMessageStruct{Idx0}{Idx1}.protoFileName;
                customDetails.enumFieldName{end+1} = customMessageStruct{Idx0}{Idx1}.messageStruct{Idx2}.enumName;
                customDetails.enumFields{end+1} = customMessageStruct{Idx0}{Idx1}.messageStruct{Idx2}.enumFields;
                customDetails.enumFieldsValue{end+1} = customMessageStruct{Idx0}{Idx1}.messageStruct{Idx2}.enumFieldsValue;

                % find and store repeated message names all across proto
                % details
                for typeIdx = 1:length(customMessageStruct{Idx0}{Idx1}.messageStruct{Idx2}.fieldType)
                    if( ~isempty(customMessageStruct{Idx0}{Idx1}.messageStruct{Idx2}.fieldName{typeIdx}))
                        if(strcmp(customMessageStruct{Idx0}{Idx1}.messageStruct{Idx2}.fieldType{typeIdx} ,"repeated"))
                            customDetails.repeatedMessageName{end+1} = customMessageStruct{Idx0}{Idx1}.messageStruct{Idx2}.fieldName{typeIdx};
                        end
                    end
                end
            end

            % store parent message details
            parentMessage.messageNameSpaceList = customMessageStruct{Idx0}{Idx1}.messageNameSpace;
            parentMessage.simulinkMessageFileName = customMessageStruct{Idx0}{Idx1}.simulinkMessageName{Idx2};
            parentMessage.simulinkMessageName = customMessageStruct{Idx0}{Idx1}.simulinkTypeName{Idx2};
            parentMessage.importedNameSpaceList = customMessageStruct{Idx0}{Idx1}.importedNameSpace;
            parentMessage.messageBaseName = customMessageStruct{Idx0}{Idx1}.messageBaseName;
            parentMessage.protoFileName = customMessageStruct{Idx0}{Idx1}.protoFileName;
            parentMessage.importedProtoFileName = customMessageStruct{Idx0}{Idx1}.importedFileNames;

            customDetails.utilities{end+1} = parentMessage;
        end
    end
end

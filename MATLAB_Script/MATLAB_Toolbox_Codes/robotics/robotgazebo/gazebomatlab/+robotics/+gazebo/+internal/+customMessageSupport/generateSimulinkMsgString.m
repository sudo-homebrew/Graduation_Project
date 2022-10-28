function codeString = generateSimulinkMsgString(fieldDataStruct, localFieldStruct, customDetails,...
                                                simulinkMessageName,simulinkMessageTypeName, templateFilename)
    %This function is for internal use only. It may be removed in the future.
    %
    % This function generates simulink message string based on input MATLAB
    % structure of proto file fields. Here, simulink message string is created
    % based on standard simulink message template.

    %   Copyright 2019-2020 The MathWorks, Inc.

    % input :
    %       @param fieldDataStruct           MATLAB struct containing proto message details
    %       @param messageNameList           List of all message names defined in several proto files
    %       @param enumNameList              List of all enum names defined in several proto files
    %       @param templateFilename          Simulink Message template file name

    %%
    % this is the path where simulink messages are stored
    extPath = "robotics.slgazebo.internal.msgs.";

    gazeboFileExt = char(robotics.gazebo.internal.topic.Category.CustomMessageName);

    %%
    % msgDataTypeList stores mapping between protobuf (required, optional, repeated, oneof fields) and
    % simulink datatype. The structure look like below,
    % msgDataTypeList = [protobuf datatype ;
    %                     simulink datatype]

    msgDataTypeList = ["double","float","int32","int64","uint32","uint64",...
                       "sint32","sint64","fixed32","fixed64","sfixed32",...
                       "sfixed64","bool","string","bytes";...
                       "double","single","int32","double","uint32","double",...
                       "int32","double","uint32","double","int32",...
                       "double","logical","uint8","uint8"];

    %%
    % Reads template file of simulink message as a string
    messageLog = fileread(templateFilename);

    codeString = strrep(messageLog,'{% msgInfo.baseName %}',simulinkMessageName);
    codeString = strrep(codeString,'{% msgInfo.MessageType %}',simulinkMessageTypeName);

    % defining retrieved message field and definition variables
    messageFieldString = "";
    messageDefString = "";

    %%
    % retires field name, datatype and stores as simulink message string
    for i=1:size(fieldDataStruct.fieldName,2)
        % retrieves required field name and datatype
        field_name = fieldDataStruct.fieldName{i};
        field_type = fieldDataStruct.dataType{i};
        field_messgae_name = fieldDataStruct.fieldMessageName{i};

        % Simulink message goes into infinite loop if parent message
        % defined as a field in child message. Thus, avoiding such
        % condition by checking child message contains parent message as a
        % field
        inhertateMessageNames = strsplit(fieldDataStruct.importedMessageLink{i},".");
        searchId = strcmp(inhertateMessageNames,string(field_messgae_name));

        if(any(searchId))
            parentMessageName = join(inhertateMessageNames(searchId)," ");
            error(message('robotics:robotgazebo:gazebogenmsg:UnsupportedProtoStructure',...
                          field_messgae_name, parentMessageName, field_name));
        end

        if( ~isempty(field_name) && ~isempty(field_type))

            field_type = strsplit(field_type,'.');
            field_type = field_type{end};
            % find out the field datatype is standard or not
            % if field type is not standard then it is considered as message

            switch(fieldDataStruct.fieldType{i})

              case "required"

                tempCell = msgDataTypeList(1,:);
                idx = find(cellfun(@(x)strcmp(x,field_type),tempCell)==1);

                if(~isempty(idx))

                    % this considered as message field
                    mfield_type = msgDataTypeList(2,idx);

                    mfield_value = join(["(",fieldDataStruct.defaultValue{i},")"],"");

                    if( strcmp(field_type,"string") || strcmp(field_type,"bytes") )
                        tempString0 = [field_name,' ', mfield_type, newline];
                        if(strcmp( fieldDataStruct.defaultValue{i},"0" ))
                            tempString1 = "";
                        else
                            tempString1 = ['obj.',field_name,' ','=',' ',mfield_type,mfield_value,';',newline];
                        end
                    else
                        tempString0 = [field_name,newline];
                        tempString1 = ['obj.',field_name,' ','=',' ',mfield_type,mfield_value,';',newline];
                    end

                else

                    if(strcmp(field_type,"message"))
                        % find field type in the message name list
                        % and retrieve its namespace
                        simulinkMsgName = searchSimulinkMessageName(field_messgae_name,localFieldStruct,customDetails);

                        % this considered as datatype field
                        tempString0 = [field_name,' ',extPath,gazeboFileExt,simulinkMsgName,newline];
                        tempString1 = [field_name,'_',' = ',extPath,gazeboFileExt,simulinkMsgName,';',newline, ...
                                       'obj.',field_name,' ',' = ',' ',field_name,'_',';',newline ];
                    else
                        % for Enum field
                        mfield_type = "int32";
                        %default_field = fieldDataStruct.defaultValue{i};
                        mfield_value = join(["(",fieldDataStruct.defaultValue{i},")"],"");

                        tempString0 = [field_name,newline];
                        tempString1 = ['obj.',field_name,' ','=',' ',mfield_type,mfield_value,';',newline];

                    end
                end

              case "oneof"
                tempCell = msgDataTypeList(1,:);
                idx = find(cellfun(@(x)strcmp(x,field_type),tempCell)==1);

                if(~isempty(idx))
                    % this considered as message field
                    mfield_type = msgDataTypeList(2,idx);
                    mfield_value = join(["(",fieldDataStruct.defaultValue{i},")"],"");

                    if( strcmp(field_type,"string") || strcmp(field_type,"bytes") )
                        tempString0 = [field_name,' ', mfield_type, newline];
                        if(strcmp( fieldDataStruct.defaultValue{i},"0" ))
                            tempString1 = "";
                        else
                            tempString1 = ['obj.',field_name,' ','=',' ',mfield_type,mfield_value,';',newline];
                        end
                    else
                        tempString0 = [field_name,newline];
                        tempString1 = ['obj.',field_name,' ','=',' ',mfield_type,mfield_value,';',newline];
                    end
                    tempString0 = [tempString0,field_name,'_status',newline]; %#ok<AGROW>
                    tempString1 = [tempString1,'obj.',field_name,'_status ','=',' ','logical(false)',';',newline]; %#ok<AGROW>

                else

                    if(strcmp(field_type,"message"))
                        % find field type in the message name list
                        % and retrieve its namespace
                        simulinkMsgName = searchSimulinkMessageName(field_messgae_name,localFieldStruct,customDetails);

                        % this considered as datatype field
                        tempString0 = [field_name,' ',extPath,gazeboFileExt,simulinkMsgName,newline];
                        tempString1 = [field_name,'_',' = ',extPath,gazeboFileExt,simulinkMsgName,';',newline, ...
                                       'obj.',field_name,' ',' = ',' ',field_name,'_',';',newline ];
                    else
                        % for Enum field
                        mfield_type = "int32";
                        %default_field = fieldDataStruct.defaultValue{i};
                        mfield_value = join(["(",fieldDataStruct.defaultValue{i},")"],"");

                        tempString0 = [field_name,newline];
                        tempString1 = ['obj.',field_name,' ','=',' ',mfield_type,mfield_value,';',newline];

                    end

                    tempString0 = [tempString0,field_name,'_status',newline]; %#ok<AGROW>
                    tempString1 = [tempString1,'obj.',field_name,'_status ','=',' ','logical(false)',';',newline]; %#ok<AGROW>
                end

              case "repeated"
                tempCell = msgDataTypeList(1,:);
                idx = find(cellfun(@(x)strcmp(x,field_type),tempCell)==1);
                if(~isempty(idx))
                    % this considered as message field
                    mfield_type = msgDataTypeList(2,idx);

                    tempString0 = [field_name,' ', mfield_type, newline];
                    tempString1 = "";%['obj.',field_name,' ','=',' ',mfield_type,mfield_value,';',newline];

                    if( strcmp(field_type,"string") || strcmp(field_type,"bytes") )
                        tempString0 = [tempString0, field_name,'_size ', 'uint32', newline]; %#ok<AGROW>
                        tempString1 = [tempString1, "" ,newline]; %#ok<AGROW>
                    end

                else

                    if(strcmp(field_type,"message"))
                        % find field type in the message name list
                        % and retrieve its namespace
                        simulinkMsgName = searchSimulinkMessageName(field_messgae_name,localFieldStruct,customDetails);

                        % this considered as datatype field

                        tempString0 = [field_name,' ',extPath,gazeboFileExt,simulinkMsgName,newline];
                        tempString1 = [field_name,'_',' = ',extPath,gazeboFileExt,simulinkMsgName,';',newline, ...
                                       'obj.',field_name,' ',' = ',' ',field_name,'_',';',newline ];
                    else
                        % for Enum field
                        mfield_type = "int32";
                        tempString0 = [field_name,' ', mfield_type, newline];
                        tempString1 = "";%['obj.',field_name,' ','=',' ',mfield_type,mfield_value,';',newline];

                    end

                end

              case "optional"

                tempCell = msgDataTypeList(1,:);
                idx = find(cellfun(@(x)strcmp(x,field_type),tempCell)==1);
                if(~isempty(idx))
                    % this considered as message field
                    mfield_type = msgDataTypeList(2,idx);
                    mfield_value = join(["(",fieldDataStruct.defaultValue{i},")"],"");

                    if( strcmp(field_type,"string") || strcmp(field_type,"bytes") )
                        tempString0 = [field_name,' ', mfield_type, newline];
                        if(strcmp( fieldDataStruct.defaultValue{i},"0" ))
                            tempString1 = "";
                        else
                            tempString1 = ['obj.',field_name,' ','=',' ',mfield_type,mfield_value,';',newline];
                        end
                    else
                        tempString0 = [field_name,newline];
                        tempString1 = ['obj.',field_name,' ','=',' ',mfield_type,mfield_value,';',newline];
                    end

                else

                    if(strcmp(field_type,"message"))
                        % find field type in the message name list
                        % and retrieve its namespace
                        simulinkMsgName = searchSimulinkMessageName(field_messgae_name,localFieldStruct,customDetails);

                        % this considered as datatype field
                        tempString0 = [field_name,' ',extPath,gazeboFileExt,simulinkMsgName,newline];
                        tempString1 = [field_name,'_',' = ',extPath,gazeboFileExt,simulinkMsgName,';',newline, ...
                                       'obj.',field_name,' ',' = ',' ',field_name,'_',';',newline ];
                    else
                        % for Enum field
                        mfield_type = "int32";
                        %default_field = fieldDataStruct.defaultValue{i};
                        mfield_value = join(["(",fieldDataStruct.defaultValue{i},")"],"");

                        tempString0 = [field_name,newline];
                        tempString1 = ['obj.',field_name,' ','=',' ',mfield_type,mfield_value,';',newline];

                    end
                end

            end

            % concatenate string over each required field
            tempString0 = join(tempString0,'');
            messageFieldString = strcat(messageFieldString,tempString0);
            tempString1 = join(tempString1,'');
            messageDefString = strcat(messageDefString,tempString1);
        end
    end

    %%
    % replace message field string in the simulink template string
    codeString = replace(codeString,'{% msgInfo.field %}',messageFieldString);

    % replace message definition string in the simulink template string
    codeString = replace(codeString,'{% msgInfo.definition %}',messageDefString);

    % remove non-essential characters
    codeString = regexprep(codeString,'\r*\n[\s\r\n]*\n','\n');

end

% find out the field_type is from same message or imported from another
% file. First, search in local message name and then in global message
% names. Further, construct Simulink message name based on NameSpace,
% Basename and Parentname
function simulinkMsgName = searchSimulinkMessageName(field_type, localFieldStruct, globalFieldStruct)

% search in local message list
    index = find(cellfun(@(x)strcmp(x,field_type),localFieldStruct.messageNameList)==1);

    if(isempty(index))
        % search in global message list
        index = find(cellfun(@(x)strcmp(x,field_type),globalFieldStruct.messageNameList)==1);
        if( numel(index) > 1)
            index = index(end); %%% NEED To check. This case is hitting or not
        end
        % retrieve Simulink Message Name
        simulinkMsgName = globalFieldStruct.simulinkMessageName{index};
    else
        % retrieve Simulink Message Name
        simulinkMsgName = localFieldStruct.simulinkMessageName(index);
    end

end

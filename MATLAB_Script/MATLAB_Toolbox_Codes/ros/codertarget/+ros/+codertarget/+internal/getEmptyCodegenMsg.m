function [msgStructGen,msgStructGenFullPath,info,msgInfo] = getEmptyCodegenMsg(messageType,rosver,folder)
%This function is for internal use only. It may be removed in the future.

%   Copyright 2020-2021 The MathWorks, Inc.
    if nargin < 3
        folder = pwd;
    end

    % Get message struct definition and detailed info
    [msgStruct,info,msgInfo] = ros.internal.getEmptyMessage(messageType,rosver);
    msgName = [msgInfo.pkgName '_' msgInfo.msgName];
    hashString = ros.slros.internal.bus.Util.hashString(msgName);
    % MATLAB has 63 character limit on function names
    maxCharLimit = 63;
    chopLen = length('Struct') + length(['_', hashString]);
    endIdx = maxCharLimit - chopLen;
    msgStructGen = [msgName 'Struct'];
    if maxCharLimit < (length(msgName) + length('Struct'))
        msgStructGen = [msgName(1:endIdx),'_',hashString,'Struct'];
    end

    % Loop over fieldNames to determine referenced message types. Generate
    % codegen message definitions of all subtypes before generating the message
    % struct definition for this type
    % Create a MATLAB file that defines the message struct for codegen
    s = StringWriter;
    s.addcr('function msg = %s',msgStructGen);
    s.addcr('%% Message struct definition for %s',messageType);
    s.addcr('coder.inline("never")');
    s1 = StringWriter; % Used to hold buffer that will be appended to s
    msgFields = fields(msgStruct);

    nestedVarLenMsgs = msgFields(cellfun(@(x)isNestedMsgVarLen(info,x),msgFields));
    % Call the appropriate function to generate the empty message
    [s,s1] = getEmptyMsgDefinition(s,s1,rosver,msgStruct,msgFields,info,messageType,folder);
        
    % Declare all the nested var len messages here
    for fieldName_idx = nestedVarLenMsgs'
        s1.addcr('msg.%s = msg.%s([],1);',fieldName_idx{1},fieldName_idx{1});
    end

    % Define C++ struct name, create epilog
    s.addcr('coder.cstructname(msg,''%s'');',[msgStructGen '_T']);
    s.append(s1); % Add variable size information
    s.addcr('if ~isempty(coder.target)');
    s.addcr('coder.ceval(''//'',coder.rref(msg));'); % Force struct definition to be always generated
    s.addcr('end');
    s.addcr('end');
    s.indentCode;

    % Add messageType to the list of processed messages
    cgenInfo = ros.codertarget.internal.ROSMATLABCgenInfo.getInstance;
    addMessageType(cgenInfo,messageType);

    % Add serviceType to the list of processed services, this is only
    % required for response message since the same service type has already
    % been processed when calling request message.
    if strcmp(messageType(end-7:end),'Response')
        addServiceType(cgenInfo,messageType(1:end-8));
    end

    % Write out the MATLAB file to given folder
    clear(msgStructGen); % Clear in-memory function definition
    msgDefnFolder = fullfile(folder,'msgdef');
    if ~isfolder(msgDefnFolder)
        [success,errorMsg] = mkdir(folder,'msgdef');
        if ~success
            error(message('ros:mlroscpp:codegen:MkdirError','msgdef',folder,errorMsg));
        end
    end
    if ~isOnPath(msgDefnFolder)
        addpath(msgDefnFolder);
    end
    msgStructGenFullPath = s.write(fullfile(msgDefnFolder,[msgStructGen '.m']));
    % Update message definition function cache
    rehash;
end

function [s,s1] = getEmptyMsgDefinition(s,s1,rosver,msgStruct,msgFields,info,messageType,folder)
sizeUpperBound = 1e9; % This large number ensures arrays are handled using coder::array

s.addcr('msg = struct(...');
for k = 1:numel(msgFields)
    fieldName = msgFields{k};
    if isequal(fieldName,'MessageType')
        fieldValue = ['''' msgStruct.(fieldName) ''''];
    else
        if ~isfield(info.(fieldName),'MLdataType')
            mlDataType = 'emptystruct';
        else
            mlDataType = info.(fieldName).MLdataType;
        end

        fieldType = mlDataType;
        strlen   = info.(fieldName).maxstrlen;
        fieldSize = info.(fieldName).MaxLen;
        if strcmpi('ros',rosver)
            isBoundedArray        = false;
            getBuiltinFieldValue  = @getBuiltinFieldValue_ros;
            getCharFieldValue     = @getCharFieldValue_ros;
            getStrConstFieldValue = @getStringConstFieldValue_ros;
            getVarStrFieldValue   = @getVarStringFieldValue_ros;
        else
            isBoundedArray        = ros.internal.ros2.MessageUtil.isBoundedArray(info.(fieldName));
            getBuiltinFieldValue  = @getBuiltinFieldValue_ros2;
            getCharFieldValue     = @getCharFieldValue_ros2;
            getStrConstFieldValue = @getStringConstFieldValue_ros2;
            getVarStrFieldValue   = @getVarStringFieldValue_ros2;
        end
        switch mlDataType
            case 'emptystruct'
                fieldValue = sprintf('struct');
            case 'struct'
                % The type is another message.
                % Create a codegen message type definition for the message type
                % referenced in this field. This is required before we can define
                % this message composed of other message types.
                fieldValue = ros.codertarget.internal.getEmptyCodegenMsg(info.(fieldName).MessageType,rosver,folder);
                if isnan(fieldSize)
                    % Declare field to have variable size
                    s1.addcr('coder.varsize(''msg.%s'',[%d 1],[1 0]);',fieldName,sizeUpperBound);
                elseif isBoundedArray
                    % Declare field to have variable size with upper-bound
                    s1.addcr('coder.varsize(''msg.%s'',[%d 1],[1 0]);',fieldName,fieldSize);                    
                else
                    if ~isequal(fieldSize,1)
                        fieldValue = sprintf('repmat(%s,[%d,1])',fieldValue,fieldSize);
                    end
                end
            case 'string'
                % Special rules apply to 'char','string'
                if info.(fieldName).constant
                    % Constant string
                    fieldValue = getStrConstFieldValue(msgStruct,fieldType,fieldSize,fieldName,strlen);
                else
                    fieldSize = info.(fieldName).MaxLen;
                    if isnan(fieldSize) || iscellstr(msgStruct.(fieldName)) %#ok<ISCLSTR>
                        % Field type is a string[] which maps to a cell
                        % string type in MATLAB. This is not supported by
                        % MATLAB Coder hence we issue a warning here and
                        % skip this field.
                        warning(message('ros:mlroscpp:codegen:UnsupportedMessageField',...
                            fieldName,messageType))
                        fieldValue = '';
                    else
                        fieldValue = getVarStrFieldValue(fieldType,fieldSize,strlen,isBoundedArray);
                        % Declare field to have variable size
                        if isnan(strlen)
                            s1.addcr('coder.varsize(''msg.%s'',[1 %d],[0 1]);',fieldName,sizeUpperBound);
                        else
                            s1.addcr('coder.varsize(''msg.%s'',[1 %d],[0 1]);',fieldName,strlen);
                        end
                    end
                end
            case 'char'
                fieldValue = getCharFieldValue(fieldType,fieldSize,strlen,isBoundedArray);
                if isnan(fieldSize)
                    % Declare field to have variable size
                    s1.addcr('coder.varsize(''msg.%s'',[1 %d],[0 1]);',fieldName,sizeUpperBound);
                elseif isBoundedArray
                    s1.addcr('coder.varsize(''msg.%s'',[1 %d],[0 1]);',fieldName,fieldSize);
                end
            otherwise
                if contains(mlDataType,'int64','IgnoreCase',true)
                    cgenInfo = ros.codertarget.internal.ROSMATLABCgenInfo.getInstance;
                    addMessageTypeWithInt64(cgenInfo,messageType);
                end
                % Simple numeric data type
                fieldValue = getBuiltinFieldValue(msgStruct,fieldType,fieldSize,fieldName,info.(fieldName).constant,isnan(fieldSize) || isBoundedArray);
                if isnan(fieldSize)
                    % Declare field to have variable size
                    s1.addcr('coder.varsize(''msg.%s'',[%d 1],[1 0]);',fieldName,sizeUpperBound);
                elseif isBoundedArray
                    s1.addcr('coder.varsize(''msg.%s'',[%d 1],[1 0]);',fieldName,fieldSize);
                end
        end
    end

    if ~isempty(fieldValue)
        % An empty fieldValue indicates it is not supported for codegen
        if k ~= 1
            % Append separator from previous iteration
            s.addcr(',...');
        end
        s.add('''%s'',%s',fieldName,fieldValue);
    end
end
s.addcr(');');
end

%% ROS helper functions
function fieldValue = getBuiltinFieldValue_ros(msgStruct,fieldType,fieldSize,fieldName,isConstant, ~)
    if isConstant
        fieldValue = sprintf('ros.internal.ros.messages.ros.default_type(''%s'',%d,%s)',...
            fieldType,fieldSize,string(msgStruct.(fieldName)));
    else
        fieldValue = sprintf('ros.internal.ros.messages.ros.default_type(''%s'',%d)',...
            fieldType,fieldSize);
    end
end

function fieldValue = getCharFieldValue_ros(fieldType,~,~,~)
    fieldValue = sprintf('ros.internal.ros.messages.ros.char(''%s'',0)',...
        fieldType);
end

function fieldValue = getStringConstFieldValue_ros(msgStruct,fieldType,fieldSize,fieldName,~)
    fieldValue = sprintf('ros.internal.ros.messages.ros.char(''%s'',%d,''%s'')',...
        fieldType,fieldSize,string(msgStruct.(fieldName)));
end

function fieldValue = getVarStringFieldValue_ros(fieldType,~,~,~)
    fieldValue = sprintf('ros.internal.ros.messages.ros.char(''%s'',0)',...
        fieldType);
end


%% ROS 2 helper functions
function fieldValue = getBuiltinFieldValue_ros2(msgStruct,fieldType,fieldSize,fieldName,isConstant,isBounded)
% GETBUILTINFIELDVALUE_ROS2 Get field value for built-in types
% (logical/numeric) for ROS 2
    if isConstant
        fieldValue = sprintf('ros.internal.ros2.messages.ros2.default_type(''%s'',%d,0,%s)',...
            fieldType,fieldSize,string(msgStruct.(fieldName)));
    else
        fieldValue = sprintf('ros.internal.ros2.messages.ros2.default_type(''%s'',%d,%d)',...
            fieldType,fieldSize,isBounded);
    end
end

function fieldValue = getCharFieldValue_ros2(fieldType,fieldSize,strlen,isBounded)
% GETCHARFIELDVALUE_ROS2 Get field value char type
    fieldValue = sprintf('ros.internal.ros2.messages.ros2.char(''%s'',%d,%d,%d,0,32)',...
                        fieldType,fieldSize,strlen,isBounded);
end

function fieldValue = getStringConstFieldValue_ros2(msgStruct,fieldType,fieldSize,fieldName,strlen)
% GETCHARFIELDVALUE_ROS2 Get field value char type
    fieldValue =  sprintf('ros.internal.ros2.messages.ros2.char(''%s'',%d,%d,0,''%s'',0)',...
                            fieldType,fieldSize,strlen,string(msgStruct.(fieldName)));
end

function fieldValue = getVarStringFieldValue_ros2(fieldType,fieldSize,strlen,isBounded)
    fieldValue = sprintf('ros.internal.ros2.messages.ros2.char(''%s'',%d,%d,%d)',...
                                fieldType,fieldSize,strlen,isBounded || isnan(strlen));
end

function ret = isNestedMsgVarLen(info,fname)
    ret = ~isequal(fname,'MessageType') && isequal('struct',info.(fname).MLdataType) ...
          && isnan(info.(fname).MaxLen);
end

function onPath = isOnPath(folder)
    pathCell = [pwd regexp(path, pathsep, 'split')];
    if ispc  % Windows is not case-sensitive
        onPath = any(strcmpi(folder, pathCell));
    else
        onPath = any(strcmp(folder, pathCell));
    end
end

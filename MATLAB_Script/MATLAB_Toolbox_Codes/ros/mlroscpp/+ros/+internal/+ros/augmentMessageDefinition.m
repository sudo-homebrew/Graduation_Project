function msgDefn = augmentMessageDefinition(msgDefn, registry)
%This function is for internal use only. It may be removed in the future.

%   Copyright 2019-2021 The MathWorks, Inc.

%augmentMessageDefinition adds extra information to ease the emitter
%the fundamental idea is to have emitter template to be simple
%emitter can iterate over the message files as specified in the
%msgDefn.order and emit the corresponding code

%See end of file for a guided example

if isfield(msgDefn,'msgFields') %there are empty messages
    msgDefn.fieldNames = fields(msgDefn.msgFields);
else
    msgDefn.fieldNames = {};
end

stdMsgRoot = {fullfile(matlabroot,'sys','ros1',computer('arch'),'ros1','share'), ...
    fullfile(matlabroot,'toolbox','ros','mlroscpp','custom_messages',...
    'share')};
msgDefn = getSubConstructor(msgDefn, 0, '', '','', '', msgDefn.msgInfo);

if ~isempty(msgDefn.fieldNames)
    [msgDefn.msgFields, msgDefn.order, msgDefn.matpath, msgDefn.msgpath, Index] = ...
        updateFields(msgDefn.msgFields, 0, ...
        '', '', '', '', ...
        msgDefn.fieldNames, msgDefn.msgInfo,stdMsgRoot,[],{},0, registry, msgDefn.msgInfo.pkgName);
    msgDefn.Prefix = cell(1,numel(Index));
    msgDefn.RequiredFields = msgDefn.order(Index);
    itr = 1;
    for ii = Index
        msgDefn.Prefix{itr} = strcat('_',strrep(msgDefn.msgpath{ii},'.','_'));
        itr = itr + 1;
    end
    %msgDefn = setFieldAndItsChildrenToInCopy(msgDefn);
else
    msgDefn.msgFields = {};
    msgDefn.order = {};
    msgDefn.matpath = {};
    msgDefn.msgpath = {};
    msgDefn.RequiredFields = {};
end
end

%-------------------------------------------------------------------------
% updateFields
%-------------------------------------------------------------------------
function [msgFields, order, matpath, msgpath, idx, types, numOfElementsInOrder] = updateFields(msgFields, fldCount, ...
    prefixForVar, ordprefix, matprefix, msgprefix, ...
    fieldNames, msgInfo, stdMsgRoot, idxprefix, typeprefix, numOfElementsInOrder, registry, pkgName)
%updateFields updates the field information. Mostly adds publisher line and
%subscriber line for use in the templates
%this method can be called recursively
order = {};
matpath = {};
msgpath = {};
types = typeprefix;
idx = idxprefix;
for i = 1:numel(fieldNames)
    %for each field build the order, matpath and msgpath
    %this is needed as the recursion happens
    order = [order, [ordprefix,fieldNames{i}]]; %#ok<AGROW>
    numOfElementsInOrder = numOfElementsInOrder + 1;
    matpath = [matpath, [matprefix,fieldNames{i}]]; %#ok<AGROW>
    msgpath = [msgpath, [msgprefix, fieldNames{i}]]; %#ok<AGROW>
    field = msgFields.(fieldNames{i});
    %if the field has an MLdataType during the parse, we know we have all
    %the information needed and can build the pub and sub lines
    if isfield(field,'MLdataType')
        field.pubLine = {};
        field.subLine = {};
        %if this field is not part of a copy then can safely set that we
        %are not in copy
        if ~isfield(field, 'inCopy')
            field.inCopy = false;
        end
        field = getPubSubLine(field, fldCount, prefixForVar, matpath{end}, msgpath{end}, fieldNames{i}, msgInfo);
    else
        %It is a subtype and we need to handle the subtype
        assert(isfield(field,'MessageType'),'Expecting a new MessageType');
        fldType = split(field.MessageType, '/');
        if ~isequal(fldType{1},pkgName)
            field.msgInfo.isSamePackage = false;
        end

        if ~any(ismember(types,field.MessageType))
            types = [types, {field.MessageType}]; %#ok<AGROW>
            idx = [idx numOfElementsInOrder];%#ok<AGROW>
        end
        %we have hit a new type, we need to get the msgInfo for this type
        
        field.msgInfo = ros.internal.ros.getMessageInfo(field.MessageType, registry);
        fldType = split(field.MessageType, '/');
        if ~isequal(fldType{1},pkgName)
            field.msgInfo.isSamePackage = false;
            msgFields.(fieldNames{i}).msgInfo.isSamePackage = false;
        end
        location = '';
        try
            if ~field.msgInfo.custom
                location = ros.internal.utilities.locateMessage(field.msgInfo.pkgName, field.msgInfo.msgName, stdMsgRoot, 'msg');
            end
        catch
            location = '';
        end
        
        % do not check for ros/Time and ros.Duration as their libs are
        % not existing in 3p/ros1
        if isempty(location) && ~isequal(field.MessageType, 'ros/Time') && ~isequal(field.MessageType, 'ros/Duration')
            field.msgInfo.custom = true;
            field.msgInfo.msgStructGen = ['ros.internal.ros.custommessages.' field.msgInfo.pkgName '.' lower(field.msgInfo.msgName(1)) field.msgInfo.msgName(2:end)];
        end
        
        if isfield(field,'msgFields')
            field.fieldNames = fields(field.msgFields);
        else
            field.fieldNames = {};
        end
        %for subtypes we need to construct a struct for the field
        fldPrefix = [prefixForVar '_' fieldNames{i}];
        field = getSubConstructor(field, fldCount, fldPrefix, matpath{end}, msgpath{end}, fieldNames{i}, msgInfo);
        
        if ~isempty(field.fieldNames) || (isfield(field,'MessageType') && isempty(field.fieldNames))
            %while processing the subtype we need to handle arrays of
            %subtypes as special. So we need the copy-line
            %OR
            %we have a sub-field which is of a different type
            if field.count ~= 0 || ~isequal(field.msgInfo, msgInfo)
                fldType = split(field.MessageType, '/');
                field.copyData = [fldType{1} '_msg_' fldType{2} '_common'];
                field.fldType = [msgInfo.msgCppClassName,'::_',fieldNames{i},'_type'];
                field.pubLine = {};
                field.subLine = {};
                %if this field is not part of a copy then can safely set that we
                %are not in copy
                if ~isfield(field, 'inCopy')
                    field.inCopy = false;
                end
                field = getCopyLine(field, fldCount, prefixForVar, matpath{end}, msgpath{end}, fieldNames{i}, msgInfo);
                %if this field is inCopy, just make sure sub fields are also in
                %inCopy. This way we can filter out into separate routines
                for subIdx = 1:numel(field.fieldNames)
                    field.msgFields.(field.fieldNames{subIdx}).inCopy = true;
                end
            else
                %if it is single field then the assignment must happen at
                %the end, hence we put in a separate field endSubline and
                %emit at the end of the routine
                field.endSubLine{1} = sprintf('%soutArray[ctr]["%s"] = %soutArray;',...
                    prefixForVar, ros.internal.utilities.convertROSFieldsToClassFields(fieldNames{i}, msgInfo.msgName), fldPrefix);
            end
            if ~isempty(field.fieldNames)
                %recursion
                innerOrdPrefix = [ordprefix, fieldNames{i}, '.msgFields.'];
                innerMatPrefix = [matprefix, fieldNames{i}, ','];
                innerMsgPrefix = [msgprefix, fieldNames{i}, '.'];
                [   field.msgFields, innerOrd, innerMat, innerMsg, idx, types, numOfElementsInOrder] = ...
                    updateFields(field.msgFields, field.count, ...
                    fldPrefix, innerOrdPrefix, innerMatPrefix, innerMsgPrefix, ...
                    field.fieldNames, field.msgInfo, stdMsgRoot, idx, types, numOfElementsInOrder, registry, pkgName);
                order = [order, innerOrd{:}]; %#ok<AGROW>
                matpath = [matpath, innerMat{:}]; %#ok<AGROW>
                msgpath = [msgpath, innerMsg{:}]; %#ok<AGROW>
            end
        end
    end
    msgFields.(fieldNames{i}) = field;
end
end

%-------------------------------------------------------------------------
% getSubConstructor
%-------------------------------------------------------------------------
function field = getSubConstructor(field, fldCount, prefixForVar, matpath, msgpath, fieldName, msgInfo) %#ok<INUSL,INUSD>
%getSubConstructor gives the constructor line for the subscriber.

fldNamesForStruct = cellfun(@(c)ros.internal.utilities.convertROSFieldsToClassFields(c,field.MessageType),field.fieldNames,'UniformOutput',false);
%Time and Duration do not get a MessageType field, not being proper nested messages
if ~any(strcmp(field.MessageType, {'ros/Time', 'ros/Duration'}))
    fldNamesForStruct = ['MessageType'; fldNamesForStruct];
end
fldNamesForStruct = sprintf('"%s",',fldNamesForStruct{:});
fldNamesForStruct = ['{',fldNamesForStruct(1:end-1),'}'];

field.subConstructor = sprintf('auto %soutArray = factory.createStructArray({size,1},%s);', ...
    prefixForVar, fldNamesForStruct);
end

%-------------------------------------------------------------------------
% concatMatpathForPrefix
%-------------------------------------------------------------------------
function prefix = concatMatpathForPrefix(matpathSplit, startIdx, endIdx)
%concatMatpathForPrefix concatenates the matpathSplit with _ to create the
%prefix so we can match what getPubSubLine uses
prefix = matpathSplit{startIdx};
for i = startIdx+1:endIdx
    prefix = [prefix '_' matpathSplit{i}]; %#ok<AGROW>
end
end

%-------------------------------------------------------------------------
% getCopyLine
%-------------------------------------------------------------------------
function field = getCopyLine(field, fldCount, prefixForVar, matpath, msgpath, fieldName, msgInfo)
%getCopyLine gets the copy line for publisher and subscriber
%getCopyLine is called with two contexts...field is the field that needs to
%be copied. fldCount is the parent's fldCount. Both are needed
matpathSplit = split(matpath,',');
nelemMatPath = numel(matpathSplit);
valArr = 'arr';
%two cases need to be handled:
%if the parent is a single count
if ~isnan(fldCount) && fldCount < 1
    %elemIdx = '[0]';
    %if the types of the field are same
    if isequal(msgpath, fieldName)
        %then we are using msg-> hence we need msgPath from msg
        %and we need to start from beginning to get the field
        msgAccess = sprintf('msg->%s', msgpath);
        %msgAccessSub = sprintf('msg->%s', msgpath);
        %recursively get to the array element much like getPubSubLine
        for i = 1:nelemMatPath-1
            varPrefix = concatMatpathForPrefix(matpathSplit, 1, i); %char({[matpathSplit{1:i}]});
            field.pubLine{end+1} = sprintf('const matlab::data::StructArray _%s_arr = %s["%s"];', ...
                varPrefix, valArr, ros.internal.utilities.convertROSFieldsToClassFields(matpathSplit{i}, msgInfo.msgName));
            valArr = sprintf('_%s_arr',  varPrefix);
        end
    else
        %we assume we are in a separate routine that is passed by const ref
        %so the array is val and we need to start from the second item down
        msgAccess = sprintf('val.%s', fieldName);
        %msgAccessSub = sprintf('val.%s', fieldName);
    end
    %since fldCount is 0 then we need to assign to [0]
    field.pubLine{end+1} = sprintf('const matlab::data::StructArray %s%s_arr = %s["%s"];', ...
        prefixForVar, fieldName, valArr, ros.internal.utilities.convertROSFieldsToClassFields(fieldName, msgInfo.msgName));
else
    %very similar logic to getPubSubLine, we have a variable array
    %bounded or unbounded, we will be in a sub routine and we need to call
    %elemIdx = '';
    msgAccess = sprintf('val.%s', fieldName);
    if ~isfield(field,'inCopy') || (isfield(field,'inCopy') && ~field.inCopy)
        for i = 2:nelemMatPath-1
            varPrefix = concatMatpathForPrefix(matpathSplit, 1, i); %char({[matpathSplit{1:i}]});
            field.pubLine{end+1} = sprintf('const matlab::data::StructArray _%s_arr = %s["%s"];', ...
                varPrefix, valArr, ros.internal.utilities.convertROSFieldsToClassFields(matpathSplit{i}, msgInfo.msgName));
            valArr = sprintf('_%s_arr',  varPrefix);
        end
    end
    field.pubLine{end+1} = sprintf('const matlab::data::StructArray %s%s_arr = %s["%s"];', ...
        prefixForVar, fieldName, valArr, ros.internal.utilities.convertROSFieldsToClassFields(fieldName, msgInfo.msgName));
end
field.MLdataType = 'struct';
%if the field count is specified, then we check the size of the array
if ~isnan(field.count) && ~field.varsize && field.count ~= 0
    field.pubLine{end+1} = sprintf('if (%s%s_arr.getNumberOfElements() < %d)',...
        prefixForVar, fieldName, field.count);
    field.pubLine{end+1} = sprintf('\tthrow(std::invalid_argument("Field ''%s'' must have minimum of %d elements."));',...
        ros.internal.utilities.convertROSFieldsToClassFields(fieldName, msgInfo.msgName), field.count);
end
%if the field count is not specified and is a dynamic array
%we need to use idx to access the array. Therefore need the declaration
if ~isnan(field.count) && ~field.varsize && field.count ~= 0
    field.pubLine{end+1} = sprintf('size_t idx_%s = 0;',fieldName);
end
field.subLine{end+1} = sprintf('auto currentElement_%s = (msg + ctr)->%s;',fieldName, fieldName);
%if unspecified or specified size, we process one struct at a time
if isnan(field.count) || field.count ~= 0
    %iterate over the elements of the array and copy the elements to msg
    field.pubLine{end+1} = sprintf('for (auto _%sarr : %s_arr) {', ...
        fieldName, fieldName);
    field.pubLine{end+1} = sprintf('\t%s _val;',...
        field.msgInfo.msgCppClassName);
    
    field.pubLine{end+1} = sprintf('auto msgClassPtr_%s = getCommonObject<%s>("%s_msg_%s_common",loader);',...
        fieldName, field.msgInfo.msgCppClassName, field.msgInfo.pkgName,field.msgInfo.msgName);
    field.pubLine{end+1} = sprintf('msgClassPtr_%s->copy_from_struct(&_val,_%sarr,loader);',...
        fieldName, fieldName);
    field.subLine{end+1} = sprintf('auto msgClassPtr_%s = getCommonObject<%s>("%s_msg_%s_common",loader);',...
        fieldName, field.msgInfo.msgCppClassName, field.msgInfo.pkgName,field.msgInfo.msgName);
    field.subLine{end+1} = sprintf('%soutArray[ctr]["%s"] = msgClassPtr_%s->get_arr(factory,&currentElement_%s[0],loader,currentElement_%s.size());',...
        prefixForVar, ros.internal.utilities.convertROSFieldsToClassFields(fieldName, msgInfo.msgName), fieldName, fieldName, fieldName);
    
    
    if field.varsize || isnan(field.count)
        field.pubLine{end+1} = sprintf('\t%s.push_back(_val);',...
            msgAccess);
    else
        field.pubLine{end+1} = sprintf('\t%s[idx_%s++] = _val;',...
            msgAccess,fieldName);
    end
    
    field.pubLine{end+1} = '}';
    field.copyStructType = 'matlab::data::StructArray';
else
    %a different type, so just call copy to get the value
    %copy handles the sub fields
    %As an optimization we can directly copy into msg
    %field.pubLine{end+1} = sprintf('%s _val;',...
    %    field.msgInfo.msgCppClassName);
    %field.pubLine{end+1} = sprintf('copy_from_arr%s(_val,%s%s_arr);',...
    %    field.copyData, prefixForVar, fieldName);
    %field.pubLine{end+1} = sprintf('%s = _val;',...
    %   msgAccess);
    
    field.pubLine{end+1} = sprintf('auto msgClassPtr_%s = getCommonObject<%s>("%s_msg_%s_common",loader);',...
        fieldName, field.msgInfo.msgCppClassName,field.msgInfo.pkgName,field.msgInfo.msgName);
    field.pubLine{end+1} = sprintf('msgClassPtr_%s->copy_from_struct(&%s,%s%s_arr[0],loader);',...
        fieldName, msgAccess, prefixForVar, fieldName);
    field.subLine{end+1} = sprintf('auto msgClassPtr_%s = getCommonObject<%s>("%s_msg_%s_common",loader);',...
        fieldName, field.msgInfo.msgCppClassName,field.msgInfo.pkgName,field.msgInfo.msgName);
    field.subLine{end+1} = sprintf('%soutArray[ctr]["%s"] = msgClassPtr_%s->get_arr(factory, &currentElement_%s, loader);',...
        prefixForVar, ros.internal.utilities.convertROSFieldsToClassFields(fieldName, msgInfo.msgName), fieldName, fieldName);
    
    field.copyStructType = 'matlab::data::StructArray';
end
end

%-------------------------------------------------------------------------
% getPubSubLine
%-------------------------------------------------------------------------
function field = getPubSubLine(field, fldCount, prefixForVar, matpath, msgpath, fieldName, msgInfo)
%getPubSubLine updates the given field's pubLine and subLine
matpathSplit = split(matpath,',');
nelemMatPath = numel(matpathSplit);
valArr = 'arr';
if ~isnan(fldCount) && fldCount < 1
    %elemIdx = '[0]';
    %if the field is in copy then we just worry about the field in the
    %parent
    if field.inCopy
        msgAccess = sprintf('val.%s', fieldName);
    else
        msgAccess = sprintf('msg->%s', msgpath);
        %we are at the top level and we need to get to the element recursive and
        %there are no arrays or copies in our path
        for i = 1:nelemMatPath - 1
            varPrefix = char({[matpathSplit{1:i}]});
            field.pubLine{i} = sprintf('const matlab::data::StructArray _%s_arr = %s["%s"];', ...
                varPrefix, valArr, ros.internal.utilities.convertROSFieldsToClassFields(matpathSplit{i}, msgInfo.msgName));
            valArr = sprintf('_%s_arr',  varPrefix);
        end
    end
else
    %elemIdx = '';
    msgAccess = sprintf('val.%s', fieldName);
end

field.subLine{end+1} = sprintf('auto currentElement_%s = (msg + ctr)->%s;',fieldName, fieldName);

% if char
% elseif string
% else all other types
if isequal(field.MLdataType, 'char')
    field.pubLine{end+1} = sprintf('const matlab::data::CharArray %s%s_arr = %s["%s"];', ...
        prefixForVar, fieldName, valArr, ros.internal.utilities.convertROSFieldsToClassFields(fieldName, msgInfo.msgName));
    field.pubLine{end+1} = sprintf('const std::string %s%s_str = %s%s_arr.toAscii();',...
        prefixForVar, fieldName, prefixForVar, fieldName);
    if field.count ~= 0
        %array
        %find the number of elements nelem and copy over
        %char[] data or char[x] data or char[<=x] data
        if isnan(field.count)
            %char[] data
            field.pubLine{end+1} = sprintf('size_t nelem = %s%s_str.length();', ...
                prefixForVar, fieldName);
            if ~isnan(field.defaultValue)
                %do a reset now as we will do push back
                field.pubLine{end+1} = sprintf('%s.clear();',msgAccess);
            end
        else
            if field.varsize
                %byte[<=x]
                %even if we have more data, use only the count
                field.pubLine{end+1} = sprintf('size_t nelem = std::min<size_t>(%d,%s%s_str.length());',...
                    field.count, prefixForVar, fieldName);
            else
                field.pubLine{end+1} = sprintf('size_t nelem = %d;',...
                    field.count);
            end
            
        end
        
        %iterate over nelems and copy over
        if field.varsize || isnan(field.count)
            field.pubLine{end+1} = sprintf('\t%s.resize(nelem);', msgAccess);
        end
        field.pubLine{end+1} = sprintf('\tstd::copy(%s%s_arr.begin(), %s%s_arr.begin()+nelem, %s.begin());', ...
            prefixForVar, fieldName, ...
            prefixForVar, fieldName, ...
            msgAccess);
    else
        %single element
        %char data
        field.pubLine{end+1} = sprintf('%s = %s%s_str[0];', ...
            msgAccess, prefixForVar, fieldName);
    end
    %handle the sublines
    if fldCount ~= 0
        %parent is an array, therefore outArray[idx][fieldname] = ...
        %char[] data or char[x] data or char[<=x] data
        if ~isnan(field.count) && field.count < 1
            %char
            field.subLine{end+1} = sprintf('\t%soutArray[idx]["%s"] = factory.createCharArray(std::string(1, val[idx].%s));',...
                prefixForVar, ros.internal.utilities.convertROSFieldsToClassFields(fieldName, msgInfo.msgName), fieldName);
        else
            %char[], char[<=x], a bounded array so we have to use begin and end
            field.subLine{end+1} = sprintf('\t%soutArray[idx]["%s"] = factory.createCharArray(std::string(val[idx].%s.begin(), val[idx].%s.end()));',...
                prefixForVar, ros.internal.utilities.convertROSFieldsToClassFields(fieldName, msgInfo.msgName), fieldName, fieldName);
        end
    else
        %parent is a single element, therefore outArray[ctr][fieldname] = ...
        if ~isnan(field.count) && field.count < 1
            %char[] data or char[x] data
            field.subLine{end+1} = sprintf('%soutArray[ctr]["%s"] = factory.createCharArray(std::string(1,currentElement_%s));',...
                prefixForVar, ros.internal.utilities.convertROSFieldsToClassFields(fieldName, msgInfo.msgName), fieldName);
        else
            %char[<=x], a bounded array so we have to use begin and end
            field.subLine{end+1} = sprintf('%soutArray[ctr]["%s"] = factory.createCharArray(std::string(currentElement_%s.begin(),currentElement_%s.end()));',...
                prefixForVar, ros.internal.utilities.convertROSFieldsToClassFields(fieldName, msgInfo.msgName), fieldName, fieldName);
        end
    end
elseif isequal(field.MLdataType,'string')
    if field.count ~= 0
        field.pubLine{end+1} = sprintf('const matlab::data::CellArray %s%s_cellarr = %s["%s"];', ...
            prefixForVar, fieldName, valArr, ros.internal.utilities.convertROSFieldsToClassFields(fieldName, msgInfo.msgName));
        %array
        %find the number of elements nelem and copy over
        %string[] data or string[x] data or string[<=x] data
        if isnan(field.count)
            %string[] data
            field.pubLine{end+1} = sprintf('size_t nelem = %s%s_cellarr.getNumberOfElements();', ...
                prefixForVar, fieldName);
            if ~isnan(field.defaultValue)
                %do a reset now as we will do push back
                field.pubLine{end+1} = sprintf('%s.clear();',msgAccess);
            end
        else
            if field.varsize
                %byte[<=x]
                %even if we have more data, use only the count
                field.pubLine{end+1+1} = sprintf('size_t nelem = std::min<size_t>(%d,%s%s_cellarr.getNumberOfElements());',...
                    field.count, prefixForVar, fieldName);
            else
                field.pubLine{end+1+1} = sprintf('size_t nelem = %d;\n',...
                    field.count);
            end
        end
        field.pubLine{end+1} = sprintf('for (size_t idx=0; idx < nelem; ++idx){');
        field.pubLine{end+1} = sprintf('\tconst matlab::data::CharArray %s%s_arr = %s%s_cellarr[idx];', ...
            prefixForVar, fieldName, prefixForVar, fieldName);
        substrrestrict = '';
        if ~isnan(field.maxstrlen)
            field.pubLine{end+1} = sprintf('\tsize_t narray = std::min<size_t>(%d, %s%s_arr.getNumberOfElements());', ...
                field.maxstrlen, prefixForVar,fieldName);
            substrrestrict = '.substr(0, narray)';
        end
        if field.varsize || isnan(field.count)
            %if there is a value for the string length then consider
            %the characters up to given length only.
            field.pubLine{end+1} = sprintf('\t%s.push_back(%s%s_arr.toAscii()%s);', ...
                msgAccess, prefixForVar, fieldName, substrrestrict);
        else
            field.pubLine{end+1} = sprintf('\t%s[idx] = %s%s_arr.toAscii()%s;', ...
                msgAccess, prefixForVar, fieldName, substrrestrict);
        end
        field.pubLine{end+1} = sprintf('}');
    else
        %single element
        field.pubLine{end+1} = sprintf('const matlab::data::CharArray %s%s_arr = %s["%s"];', ...
            prefixForVar, fieldName, valArr, ros.internal.utilities.convertROSFieldsToClassFields(fieldName, msgInfo.msgName));
        substrrestrict = '';
        if ~isnan(field.maxstrlen)
            field.pubLine{end+1} = sprintf('size_t narray = std::min<size_t>(%d, %s%s_arr.getNumberOfElements());', ...
                field.maxstrlen, prefixForVar,fieldName);
            substrrestrict = '.substr(0, narray)';
        end
        %string data
        field.pubLine{end+1} = sprintf('%s = %s%s_arr.toAscii()%s;', ...
            msgAccess, prefixForVar, fieldName, substrrestrict);
    end
    if fldCount ~= 0
        %parent is an array, therefore outArray[idx][fieldname] = ...
        if ~isnan(field.count) && field.count < 1
            %string data
            field.subLine{end+1} = sprintf('\t%soutArray[idx]["%s"] = factory.createCharArray(val[idx].%s);',...
                prefixForVar, ros.internal.utilities.convertROSFieldsToClassFields(fieldName, msgInfo.msgName), fieldName);
        else
            %string[], string[<=], string[x]
            field.subLine{end+1} = sprintf('\tauto %s%soutCell = factory.createCellArray({val[idx].%s.size(),1});', ...
                prefixForVar, fieldName, fieldName);
            field.subLine{end+1} = sprintf('\tfor(size_t idxin = 0; idxin < val[idx].%s.size(); ++ idxin){',...
                fieldName);
            field.subLine{end+1} = sprintf('\t\t %s%soutCell[idxin] = factory.createCharArray(val[idx].%s[idxin]);',...
                prefixForVar, fieldName, fieldName);
            field.subLine{end+1} = sprintf('\t}');
            field.subLine{end+1} = sprintf('\t%soutArray[idx]["%s"] = %s%soutCell;',...
                prefixForVar, ros.internal.utilities.convertROSFieldsToClassFields(fieldName, msgInfo.msgName), prefixForVar, fieldName);
        end
    else
        %parent is a single element, therefore outArray[ctr][fieldname] = ...
        if ~isnan(field.count) && field.count < 1
            %string data
            field.subLine{end+1} = sprintf('%soutArray[ctr]["%s"] = factory.createCharArray(currentElement_%s);',...
                prefixForVar, ros.internal.utilities.convertROSFieldsToClassFields(fieldName, msgInfo.msgName), fieldName);
        else
            %string[], string[<=], string[x]
            field.subLine{end+1} = sprintf('auto %s%soutCell = factory.createCellArray({currentElement_%s.size(),1});', ...
                prefixForVar, fieldName, fieldName);
            field.subLine{end+1} = sprintf('for(size_t idxin = 0; idxin < currentElement_%s.size(); ++ idxin){', fieldName);
            field.subLine{end+1} = sprintf('\t%s%soutCell[idxin] = factory.createCharArray(currentElement_%s[idxin]);',...
                prefixForVar, fieldName, fieldName);
            field.subLine{end+1} = sprintf('}');
            field.subLine{end+1} = sprintf('%soutArray[ctr]["%s"] = %s%soutCell;',...
                prefixForVar, ros.internal.utilities.convertROSFieldsToClassFields(fieldName, msgInfo.msgName), prefixForVar, fieldName);
        end
    end
else
    field.pubLine{end+1} = sprintf('const matlab::data::TypedArray<%s> %s%s_arr = %s["%s"];', ...
        field.CPPdataType, prefixForVar, fieldName, valArr, ros.internal.utilities.convertROSFieldsToClassFields(fieldName, msgInfo.msgName));
    
    if field.count ~= 0
        %array
        %find the number of elements nelem and copy over
        %byte[] data or byte[x] data or byte[<=x] data
        if isnan(field.count)
            %byte[] data
            field.pubLine{end+1} = sprintf('size_t nelem = %s%s_arr.getNumberOfElements();', ...
                prefixForVar, fieldName);
            if ~isnan(field.defaultValue)
                %do a reset now as we will do push back
                field.pubLine{end+1} = sprintf('%s.clear();',msgAccess);
            end
        else
            if field.varsize
                %byte[<=x]
                %even if we have more data, use only the count
                field.pubLine{end+1} = sprintf('size_t nelem = std::min<size_t>(%d,%s%s_arr.getNumberOfElements());',...
                    field.count, prefixForVar, fieldName);
            else
                field.pubLine{end+1} = sprintf('size_t nelem = %d;\n',...
                    field.count);
            end
        end
        if field.varsize || isnan(field.count)
            field.pubLine{end+1} = sprintf('\t%s.resize(nelem);', msgAccess);
        end
        field.pubLine{end+1} = sprintf('\tstd::copy(%s%s_arr.begin(), %s%s_arr.begin()+nelem, %s.begin());', ...
            prefixForVar, fieldName, ...
            prefixForVar, fieldName, ...
            msgAccess);
    else
        %single element
        %byte data
        field.pubLine{end+1} = sprintf('%s = %s%s_arr[0];', ...
            msgAccess, prefixForVar, fieldName);
    end
    if fldCount ~= 0
        %parent is an array, therefore outArray[idx][fieldname] = ...
        if ~isnan(field.count) && field.count < 1
            if ~isnan(field.constantValue) || isequal(field.CPPdataType,'bool')
                
                %According to ROS wiki bool is aliased to uint8_t. But we
                %are handling bool as bool, so we are type casting from
                %uint8 to bool when the ROS datatype is bool.
                field.subLine{end+1} = sprintf('\t%soutArray[idx]["%s"] = factory.createScalar(static_cast<%s>(val[idx].%s));',...
                    prefixForVar, ros.internal.utilities.convertROSFieldsToClassFields(fieldName, msgInfo.msgName), field.CPPdataType, fieldName);
            else
                field.subLine{end+1} = sprintf('\t%soutArray[idx]["%s"] = factory.createScalar(val[idx].%s);',...
                    prefixForVar, ros.internal.utilities.convertROSFieldsToClassFields(fieldName, msgInfo.msgName), fieldName);
            end
        else
            if isequal(field.CPPdataType,'bool')
                %byte[] data or byte[x] data or byte[<=x] data
                
                %According to ROS wiki bool is aliased to uint8_t. But we
                %are handling bool as bool, so we are creating a
                %std::vector of bool from uint8.
                field.subLine{end+1} = sprintf('auto %s_%s = std::vector<%s>(std::begin(val[idx].%s),std::end(val[idx].%s));',...
                    fieldName, field.CPPdataType, field.CPPdataType, fieldName, fieldName);
                field.subLine{end+1} = sprintf('\t%soutArray[idx]["%s"] = factory.createArray<std::vector<%s>::const_iterator, %s>({%s_%s.size(),1}, %s_%s.begin(), %s_%s.end());',...
                    prefixForVar, ros.internal.utilities.convertROSFieldsToClassFields(fieldName, msgInfo.msgName), field.CPPdataType, field.CPPdataType, fieldName, field.CPPdataType, fieldName, field.CPPdataType, fieldName, field.CPPdataType);
            else
                field.subLine{end+1} = sprintf('\t%soutArray[idx]["%s"] = factory.createArray<%s::_%s_type::const_iterator, %s>({val[idx].%s.size(),1}, val[idx].%s.begin(), val[idx].%s.end());',...
                    prefixForVar, ros.internal.utilities.convertROSFieldsToClassFields(fieldName, msgInfo.msgName), msgInfo.msgCppClassName, fieldName, field.CPPdataType, fieldName, fieldName, fieldName);
            end
        end
    else
        %parent is a single element, therefore outArray[ctr][fieldname] = ...
        if ~isnan(field.count) && field.count < 1
            %byte data
            if ~isnan(field.constantValue) || isequal(field.CPPdataType,'bool')
                
                %According to ROS wiki bool is aliased to uint8_t. But we
                %are handling bool as bool, so we are type casting from
                %uint8 to bool when the ROS datatype is bool.
                field.subLine{end+1} = sprintf('%soutArray[ctr]["%s"] = factory.createScalar(static_cast<%s>(currentElement_%s));',...
                    prefixForVar, ros.internal.utilities.convertROSFieldsToClassFields(fieldName, msgInfo.msgName), field.CPPdataType, fieldName);
            else
                field.subLine{end+1} = sprintf('%soutArray[ctr]["%s"] = factory.createScalar(currentElement_%s);',...
                    prefixForVar, ros.internal.utilities.convertROSFieldsToClassFields(fieldName, msgInfo.msgName), fieldName);
            end
        else
            if isequal(field.CPPdataType,'bool')
                %byte[] data or byte[x] data or byte[<=x] data
                
                %According to ROS wiki bool is aliased to uint8_t. But we
                %are handling bool as bool, so we are creating a
                %std::vector of bool from uint8.
                field.subLine{end+1} = sprintf('auto %s_%s = std::vector<%s>(std::begin(currentElement_%s),std::end(currentElement_%s));',...
                    fieldName, field.CPPdataType, field.CPPdataType, fieldName, fieldName);
                field.subLine{end+1} = sprintf('%soutArray[ctr]["%s"] = factory.createArray<std::vector<%s>::const_iterator, %s>({%s_%s.size(),1}, %s_%s.begin(), %s_%s.end());',...
                    prefixForVar, ros.internal.utilities.convertROSFieldsToClassFields(fieldName, msgInfo.msgName), field.CPPdataType, field.CPPdataType, fieldName, field.CPPdataType, fieldName, field.CPPdataType, fieldName, field.CPPdataType);
            else
                field.subLine{end+1} = sprintf('%soutArray[ctr]["%s"] = factory.createArray<%s::_%s_type::const_iterator, %s>({currentElement_%s.size(),1}, currentElement_%s.begin(), currentElement_%s.end());',...
                    prefixForVar, ros.internal.utilities.convertROSFieldsToClassFields(fieldName, msgInfo.msgName), msgInfo.msgCppClassName, fieldName, field.CPPdataType, fieldName, fieldName, fieldName);
            end
        end
    end
end
end

%{
Starting example: diagnostic_msgs/DiagnosticArray
+---------------------+
| DiagnosticArray     |
+---------------------+
| header +------------------>+-------------------+
|                     | 1  1 | std_msgs/Header   |
| status +            |      +-------------------+
|        |            |      | stamp  +---------------->+-------------------------+
+---------------------+      |                   |1   1 | builtin_interfaces/Time |
         |1                  | frame_id (string) |      +-------------------------+
         |                   |                   |      |sec (int32)              |
         |                   +-------------------+      |                         |
         |                                              |nanosec (uint32)         |
         |  *                                           +-------------------------+
         +--->+----------------------+
              | DiagnosticStatus     |
              +----------------------+
              | OK (byte, 0)         |
              | WARN (byte, 1)       |
              | ERROR (byte, 2)      |
              | STALE (byte, 3)      |
              |                      |
              | level (byte)         |
              | name (string)        |
              | message (string)     |
              | hardware_id (string) |
              |                      |            +--------------+
              | values   +----------------------->+ KeyValue     |
              +----------------------+            +--------------+
                                                  |key (string)  |
                                                  |              |
                                                  |value (string)|
                                                  +--------------+

The msg file is parsed and input to this function looks like:
msgDefn =
    MessageType: 'diagnostic_msgs/DiagnosticArray'
       filePath: 'B:\matlab\sys\ros1\win64\ros1\share\diagnostic_msgs\msg\DiagnosticArray.msg'
      msgFields: [1x1 struct]

msgDefn.msgFields.header
      MessageType: 'std_msgs/Header'
         filePath: 'B:\matlab\sys\ros1\win64\ros1\share\std_msgs\msg\Header.msg'
        msgFields: [1x1 struct]
            count: 0
    constantValue: NaN
     defaultValue: NaN
          varsize: 0

msgDefn.msgFields.status
      MessageType: 'diagnostic_msgs/DiagnosticStatus'
         filePath: 'B:\matlab\sys\ros1\win64\ros1\share\diagnostic_msgs\msg\DiagnosticStatus.msg'
        msgFields: [1x1 struct]
            count: NaN
    constantValue: NaN
     defaultValue: NaN
          varsize: 0

msgDefn.msgFields.status.msgFields.level
       MLdataType: 'int8'
      CPPdataType: 'int8_t'
      ROSdataType: 'byte'
            count: 0
    constantValue: NaN
     defaultValue: NaN
          varsize: 0
        maxstrlen: NaN

This input is augmented with information such that we can generate using
templates:
To publish we copy from arr[0] to msg
Following is the sequence of the calls and highlighted decision points that
are in updateFields, getCopyLine and getPubSubLine

copy_from_struct
    -> copy copy_from_struct of header (Since Header is a message of
    different package, we need to create a static class_loader instance of
    std_msgs/Header and then call the copy_from_struct of header using this
    Instance).
        -> copy copy_from_struct of stamp
           seq
           frame_id
        -> foreach value, copy_from_struct of status (Here Status is a
        message of the same package, so we just need to create a static
        instance of the diagnostic_msgs/DiagnosticStatus and call its copy_from_struct).
           -> copy val and all subfields
           
To subscribe we copy from msg to arr
get_arr
    -> get get_arr of header (Since Header is a message of
    different package, we need to create a static class_loader instance of
    std_msgs/Header and then call the get_arr of header using this
    Instance).
        -> get get_arr of stamp
           seq
           frame_id
        -> foreach value, get_arr of status (Here Status is a
        message of the same package, so we just need to create a static
        instance of the diagnostic_msgs/DiagnosticStatus and call its get_arr).

The contents of copy_from are in pubLine
and contents of get_arr are in subLine
In addition we need constructors and last line copy for getting arr


The output is as follows:
msgDefn

       MessageType: 'diagnostic_msgs/DiagnosticArray'
          filePath: 'B:\matlab\sys\ros1\win64\ros1\share\diagnostic_msgs\msg\DiagnosticArray.msg'
         msgFields: [1x1 struct]
        fieldNames: {2x1 cell}
           msgInfo: [1x1 struct]
    subConstructor: 'auto outArray = factory.createStructArray({size,1},{"MessageType","Header","Status"});'
             order: {1x18 cell}
           matpath: {1x18 cell}
           msgpath: {1x18 cell}
            Prefix: {'_header'  '_header_stamp'  '_status'  '_status_values'}
    RequiredFields: {'header'  'header.msgFields.stamp'  'status'  'status.msgFields.values'}

msgDefn.msgFields.header
       MessageType: 'std_msgs/Header'
          filePath: 'B:\matlab\sys\ros1\win64\ros1\share\std_msgs\msg\Header.msg'
         msgFields: [1x1 struct]
             count: 0
     constantValue: NaN
      defaultValue: NaN
           varsize: 0
           msgInfo: [1x1 struct]
        fieldNames: {3x1 cell}
    subConstructor: 'auto _headeroutArray = factory.createStructArray({1,size},{"MessageType","Seq","Stamp","FrameId"});'
          copyData: 'std_msgs_msg_Header_common'
           fldType: 'diagnostic_msgs::DiagnosticArray::_header_type'
           pubLine: {1x3 cell}
           subLine: {1x3 cell}
            inCopy: 0
        MLdataType: 'struct'
    copyStructType: 'matlab::data::StructArray'

msgDefn.msgFields.status
       MessageType: 'diagnostic_msgs/DiagnosticStatus'
          filePath: 'B:\matlab\sys\ros1\win64\ros1\share\diagnostic_msgs\msg\DiagnosticStatus.msg'
         msgFields: [1x1 struct]
             count: NaN
     constantValue: NaN
      defaultValue: NaN
           varsize: 0
           msgInfo: [1x1 struct]
        fieldNames: {9x1 cell}
    subConstructor: 'auto _statusoutArray = factory.createStructArray({1,size},{"MessageType","OK","WARN","ERROR","STALE","Level","Name","Message","HardwareId","Values"});'
          copyData: 'diagnostic_msgs_msg_DiagnosticStatus_common'
           fldType: 'diagnostic_msgs::DiagnosticArray::_status_type'
           pubLine: {1×7 cell}
           subLine: {1×3 cell}
            inCopy: 0
        MLdataType: 'struct'
    copyStructType: 'matlab::data::Struct'

msgDefn.msgFields.status.msgFields.level
        MLdataType: 'int8'
       CPPdataType: 'int8_t'
      ROSdataType: 'byte'
            count: 0
    constantValue: NaN
      defaultValue: NaN
          varsize: 0
           inCopy: 1
           pubLine: {'const matlab::data::TypedArray<int8_t> _statuslevel_arr = arr["Level"];'  'val.level = _statuslevel_arr[0];'}
           subLine: {1x2 cell}

Following is explanation of fields
       MessageType: [package/message name] as provided by user
          filePath: [Path from where the message was taken]
         msgFields: [Recursive structure of message fields]
        fieldNames: [cell array of fields, so it is easy to iterate in template]
           msgInfo: [structure with class names and c++ types]
    subConstructor: [constructor line to create an array for MATLAB]
             order: [Order of iteration so it is easy to iterate in template]
           matpath: [matrix path, i.e. field.msgFields.field.... in the same order as above]
           msgpath: [msg->field->field... in the same order as above]
            Prefix: [prefix for the variables used in the get_arr_ functions... used to return the appropriate variable]
    RequiredFields: [list of fields in list order that need a copy_arr_ and get_arr functions]

       MessageType: [typically for a sub field which is a different type]
          filePath: [path where it was found]
         msgFields: [fields of this sub type]
             count: 0, # or NaN - number of items, 0 when it is only one, #
                    when it is a bounded array, NaN when it is an unbounded
                    array
     constantValue: value or NaN - when it is a constant a non NaN value
           varsize: 0 or 1 - default is 0. To be used in conjunction with count.
		            if this is a bounded array, varsize = 1.
		            e.g. <= x, count will be 4 and varsize = 1.
           msgInfo: Same as above but for this subtype
        fieldNames: Same as above but for this subtype
    subConstructor: Same as above but for this subtype
          copyData: postfix used to make copy_from_struct%s and get_arr%s.
                    This also signifies that this is a copy_from_struct or
                    get_arr of the dependent message.
           fldType: c++ type used as parameter type for msg
           pubLine: contents of pub line (built in getCopyLine and/or
                    getPubSubLine )
           subLine: contents of sub line (built in getCopyLine and/or
                    getPubSubLine )
            inCopy: 0 or 1 - signifies if this field is already in a sub
                    copy method
        MLdataType: 'struct' for sub fields
    copyStructType: 'matlab::data::StructArray' or 'matlab::data::Struct'

For the leafiest field
       MLdataType: 'uint8', 'logical', ...
      CPPdataType: 'uint8_t', 'bool', ...
      ROSdataType: 'byte', 'bool', ...
            count: same as above
    constantValue: same as above
          varsize: same as above
           inCopy: same as above
          pubLine: {'const matlab::data::TypedArray<int8_t> _statuslevel_arr = arr["level"];'  'val.level = _statuslevel_arr[0];'}
          subLine: {'_statusoutArray[idx]["level"] = factory.createScalar(val[idx].level);'}

##################
Type mapping table:
##################
+-------------------+------------+------------+-------------------------+--------------------------------------+-------------+----------------+
| Data-type         |    MaxLen  |    MinLen  |      example            |              CPP                     | MATLAB      |   Simulink     |
+-------------------+------------+------------+-------------------------+--------------------------------------+-------------+----------------+
|  char             |     1      |     1      |     char data           |  uint8 data;                         |    ''       |  uint8         |
+-------------------+------------+------------+-------------------------+--------------------------------------+-------------+----------------+
|  char             |     NaN    |     0      |     char [] data        |  std::vector<uint8> data;            |    ''       |  uint8 [Nx1]   |
+-------------------+------------+------------+-------------------------+--------------------------------------+-------------+----------------+
|  char             |     4      |     0      |     char [<=4] data     |  BoundedVector<uint8,4> data;        |    ''       |  uint8         |
+-------------------+------------+------------+-------------------------+--------------------------------------+-------------+----------------+
|  string           |     1      |     1      |     string data;        |  std::string data;                   |   {''}      |  stringtype    |
+-------------------+------------+------------+-------------------------+--------------------------------------+-------------+----------------+
|  string           |     NaN    |     0      |     string [] data      |  std::vector<std::string> data;      |   {''}      |  stringtype[]  |
+-------------------+------------+------------+-------------------------+--------------------------------------+-------------+----------------+
|  string           |     4      |     0      |     string [<=4] data     |  BoundedVector<std::string> data;  | {'',...}    |  stringtype[4] |
+-------------------+------------+------------+-------------------------+--------------------------------------+-------------+----------------+
|  double           |     4      |     0      |     double data;        |  BoundedVector<float64> data;        |  double     |  double[4]     |
+-------------------+------------+------------+-------------------------+--------------------------------------+-------------+----------------+
| (u)int(8/16/32/64)|     4      |     0      |     double data;        |  BoundedVector<type> data;           |  type[]     |  type[4]       |
+-------------------+------------+------------+-------------------------+--------------------------------------+-------------+----------------+
|  time             |            |            |     time data;          |  ros::Time data;                     | ros/Time    |  ros/Time      |
+-------------------+------------+------------+-------------------------+--------------------------------------+-------------+----------------+
|  duration         |            |            |     duration data;      |  ros::Duration data;                 | ros/Duration|  ros/Duration  |
+-------------------+------------+------------+-------------------------+--------------------------------------+-------------+----------------+
%}

% LocalWords:  msgDefn matpath msgpath MLdataType custommessages endSubline soutArray fldCount sarr
% LocalWords:  nelem sublines fieldname cellarr narray substr sout idxin nanosec varsize CPPdataType
% LocalWords:  ROSdataType maxstrlen subfields headerout fld statusout statuslevel stringtype

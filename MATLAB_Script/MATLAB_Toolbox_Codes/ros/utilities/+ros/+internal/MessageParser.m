classdef MessageParser < handle
    %This class is for internal use only. It may be removed in the future.
    
    %MessageParser Get information from message files
    %   Provides utilities for extracting information on the contents of
    %   message definition files. The extracted information includes message
    %   field names, data types, and default values if applicable.
    %
    %
    %   MessageParser methods:
    %
    %      getMessageDefinition - Returns complete information of message.
    %
    %   Usage in ROS2's Emitter:
    %      parser = ros.internal.MessageParser('geometry_msgs/Pose2D',{fullfile(matlabroot, 'sys', 'ros2', computer('arch'), 'ros2', 'share')});
    %
    %   Usage in ROS1's Emitter:
    %      map = ros.internal.utilities.getSpecialMapForROS;
    %      parser = ros.internal.MessageParser('geometry_msgs/Pose2D',{fullfile(matlabroot, 'sys', 'ros2', computer('arch'), 'ros2', 'share')},map);
    
    %   Copyright 2019-2021 The MathWorks, Inc.
    
    
    properties (Constant, Access = private)
        %ROSdataTypes - List of available ROS datatypes
        ROSDataTypes = {'bool', 'byte', 'char', ...
            'int8', 'uint8', ...
            'int16', 'uint16', ...
            'int32', 'uint32', ...
            'float32', 'float64',...
            'int64', 'uint64', ...
            'string', 'wstring'}
        
        %MLdataTypes  - List of available MATLAB datatypes
        MLDataTypes = {'logical', 'uint8', 'char', ...
            'int8', 'uint8', ...
            'int16', 'uint16', ...
            'int32', 'uint32', ...
            'single', 'double', ...
            'int64', 'uint64', ...
            'string', 'string'}
        
        CPPDataTypes = { 'bool', 'uint8_t', 'char', ...
            'int8_t', 'uint8_t', ...
            'int16_t', 'uint16_t', ...
            'int32_t', 'uint32_t', ...
            'float', 'double',...
            'int64_t', 'uint64_t', ...
            'std::string', 'std::u16string'}
    end
    
    properties (Access = private)
        %dirsToLook - Cell array of directories to search for the given msg
        DirsToLook
        
        %messageType - Represents the message package name
        MessagePackageName
        
        %message - Holds the message file name.
        MessageFileName
        
        %SpecialMap - Represents the special map which contains
        %data structure for special datatypes.
        SpecialMap
    end
    
    properties (SetAccess = public, GetAccess = public)
        %MLTypeConverter - Returns the MATLAB datatype for ROS datatype.
        MLTypeConverter
        
        %CPPTypeConverter - Returns the CPP datatype for ROS datatype.
        CPPTypeConverter
        
        %Input message given
        Msg
    end
    
    properties (Constant, Hidden)
        %InputPattern
        InputPattern = "\\'|'";
        
        %OutputPattern
        OutputPattern = "''";
    end
    
    methods
        
        function obj = MessageParser(msg, dirsToLook, specialMap)
            %MessageParser constructor constructs the message parser object.
            
            if (nargin < 3)
                specialMap = containers.Map;
            end
            obj.SpecialMap = specialMap;
            obj.Msg = convertStringsToChars(msg);
            [obj.MessagePackageName,obj.MessageFileName,obj.DirsToLook] = ...
                ros.internal.utilities.messageConstructor(obj.Msg,dirsToLook);
            [obj.MLTypeConverter, obj.CPPTypeConverter] = ...
                                     createROSToMlAndCppDataTypesMap(obj);
            if specialMap.isKey('special_config_handler')
                hSpecialConfig = specialMap('special_config_handler');
                hSpecialConfig(obj);
            end
        end
        
        function messageDefinition = getMessageDefinition(obj)
            %getMessageDefinition returns a structure which contains
            %   information about the message such as its attributes
            %   messageTypes and its location.
            %
            %   messageDefinition = getMessageDefinition(parser) returns the
            %   data structure which contains FilePath, MessageType and msgFields
            %   of message. msgFields is a struct, whos field names match
            %   those of the parsed message. Each of those is, in turn,
            %   a structure with msgFields ...
            %
            %   Example:
            %      % MessagePackage - 'geometry_msgs'
            %      % MessageFileName - 'Pose2D'
            %      msgType = 'geometry_msgs/Pose2D';
            %
            %      % msgPath should be the location where MessagePackage
            %      % exists.
            %      msgPath = {fullfile(matlabroot, 'sys', 'ros2', computer('arch'), 'ros2', 'share')};
            %      parser = ros.internal.MessageParser(msgType, msgPath);
            %      msgInfo = getMessageDefinition(parser);
            %
            %      Here msgPath should be the location, where
            %      MessagePackage exists
            
            % Creating an empty list for Detecting Circular Dependency
            CirDependList={};  
            
            % Using helper function to run main tasks
            messageDefinition = getMessageDefinitionHelper(obj,CirDependList);
        end
        
        function messageDefinition = getMessageDefinitionHelper(obj,CirDependList)
            %getMessageDefinitionHelper returns a structure which contains
            %   information about the message such as its attributes
            %   messageTypes and its location.

            filePath = ros.internal.utilities.locateMessage(...
                obj.MessagePackageName,obj.MessageFileName,obj.DirsToLook,'msg');
            contentsOfFile = getMessageFileContentsWithoutComments(obj,filePath);
            
            % Get the contents of file which has no comments or white spaces
            % in it and get constant values if any.
            [contentsOfFile,constantValue,defaultValue,varsize,strlen] = ...
                getUpdatedMessageFileContents(obj,contentsOfFile); 
            
            messageType = strcat(obj.MessagePackageName,'/',obj.MessageFileName);
            if any((strcmp(CirDependList,messageType)))
                error(message(...
                    'ros:utilities:messageparser:RecursiveCall'));
            else
               CirDependList{end+1}=messageType;
            end

            messageDefinition = getDataStructure(obj,filePath,contentsOfFile,...
                constantValue,defaultValue,varsize,strlen,CirDependList);
        end
        
        function messageDefinition = getDataStructure(obj,filePath,contentsOfFile,...
                constantValue,defaultValue,varsize,strlen,CirDependList)
            %getDataStructure returns the complete data structure for the message.
            
            constantValues = cellstr(constantValue);
            defaultValues = cellstr(defaultValue);
            constantValues = regexprep(constantValues,'\s+#.*',''); %remove comments
            defaultValues = regexprep(defaultValues,'\s+#.*',''); %remove comments
            
            % Return the messageType.
            messageDefinition.MessageType = ...
                strcat(obj.MessagePackageName,'/',obj.MessageFileName);
                        
            % Return the file path.
            messageDefinition.filePath = filePath;
            dataStructure.value = string.empty;
                
            % For each line in the file, return its data types and values.
            for i = 1:numel(contentsOfFile)
                % Split the current line into two, and store them in
                % dataType and value
                
                currentLine = strsplit(contentsOfFile(i));
                [dataType, dataStructure.value(i)] = currentLine{:};
                
                % If the datatype is an array, then get the count of datatypes
                % with the data structure.
                if numel(strsplit(dataType,'['))>1
                    
                    % split the datatype using '[' and get the datatype.
                    dataType = strsplit(dataType,'[');
                    [dataType, numberOfDataTypes] = dataType{:};
                    comparator = strcmp(dataType,obj.ROSDataTypes);
                    if ~any(comparator)
                        
                        % Check whether the datatype is special datatype
                        % or not, by checking in the map.
                        if isKey(obj.SpecialMap,dataType)
                            
                            % If the datatype is special datatype then 
                            % return its data structure from the map. 
                            dataStructure.dataType{i} = obj.SpecialMap(dataType);
                            dataStructure.dataType{i}.count = ...
                                str2double(getCountOfDataTypes(obj,...
                                numberOfDataTypes));
                        else
                            
                            % If it doesn't match with any existing datatypes 
                            % and special datatypes, it means that, it is a
                            % nested message, so get the datatypes and 
                            % values of the nested message.
                            dataStructure.dataType{i} = ...
                                nestedMessageParser(obj,dataType,CirDependList);
                            dataStructure.dataType{i}.count = ...
                                str2double(getCountOfDataTypes(obj,...
                                numberOfDataTypes));
                            
                            % For the cases like geometry_msgs/Pose position,
                            % constantValues and defaultValues are not
                            % applicable, so we can define them as NaN.
                            dataStructure.dataType{i}.constantValue = NaN;
                            dataStructure.dataType{i}.defaultValue = NaN;
                            dataStructure.dataType{i}.varsize = ...
                                varsize{i};
                        end
                    else
                        dataStructure.dataType{i}.MLdataType = ...
                            obj.MLTypeConverter(dataType);
                        dataStructure.dataType{i}.CPPdataType = ...
                            obj.CPPTypeConverter(dataType);
                        dataStructure.dataType{i}.ROSdataType = dataType;
                        dataStructure.dataType{i}.count = ...
                            str2double(getCountOfDataTypes(obj,...
                            numberOfDataTypes));
                        
                        if ~isequal(constantValues{i}, ' ')
                            if isstring(constantValues{i})||~isempty(strfind(constantValues{i},'"'))
                                
                                % for the case string STRINGCONST = "test",
                                % it should return constantValue as "test"
                                % Here the value of constantValues{i} will
                                % be like ''%dtest'', but it should be only
                                % '%dtest', hence we are taking the characters
                                % from 2 to end-1, i.e. we are removing the 
                                % quotes from start and end of the string.
                                stringConstantValue = constantValues{i}(2:end-1);
                                
                                % If the expression is '%d\'test', then the  
                                % output should be '%d''test'.
                                updatedStringConstantValue = getUpdatedString(obj,stringConstantValue);
                                dataStructure.dataType{i}.constantValue = ...
                                    string(updatedStringConstantValue);
                            else
                                
                                % if the datatype is of any other cases
                                % like uint8, float32, bool etc just return
                                % the value as datatype(value).
                                % for example uint32 uint32_val = 25,
                                % constantValue = uint32(25)
                                dataStructure.dataType{i}.constantValue = ...
                                    feval(obj.MLTypeConverter(dataType),str2double(constantValues{i}));
                            end
                        else
                            
                            % if there doesn't exits any constantValue then
                            % constantValue will be NaN                            
                            dataStructure.dataType{i}.constantValue = NaN;
                        end
                        
                        if ~isequal(defaultValues{i}, ' ')
                            if (isstring(defaultValues{i})||~isempty(strfind(defaultValues{i},'"'))) && (isequal(numel(defaultValues{i}),1))
                                
                                % for the case string strdefault "test",
                                % it should return defaultValue as "test"
                                % Here the value of defaultValues{i} will
                                % be like ''%dtest'', but it should be only
                                % '%dtest', hence we are taking the characters
                                % from 2 to end-1.
                                stringDefaultValue = defaultValues{i}(2:end-1);
                                
                                % If the expression is '%d\'test', then the  
                                % output should be '%d''test'.
                                updatedStringDefaultValue = getUpdatedString(obj,stringDefaultValue);
                                dataStructure.dataType{i}.defaultValue = ...
                                    string(updatedStringDefaultValue);
                                
                            elseif numel(defaultValues{i})>1
                                
                                if (isequal(dataType,'string') || isequal(dataType,'wstring'))
                                    
                                    % If the array of defaultValues is of
                                    % type string, they need to be handled
                                    % carefully.

                                    % For example consider a case: string
                                    % string[2] str_samples ["str1","str2"]
                                    pattern = '"((?:\\"|[^"])*)"|''((?:\\''|[^''])*)''';
                                    stringDefaultValue = regexp(defaultValues{i}, pattern, 'tokens');
                                    for k = 1:numel(stringDefaultValue)
                                        updatedStringArrayDefaultValue{k} = getUpdatedString(obj,stringDefaultValue{k}); %#ok<AGROW>
                                    end
                                    dataStructure.dataType{i}.defaultValue = ...
                                         string(updatedStringArrayDefaultValue);
                                else
                                    
                                    % for the case uint32 samples [11 33 21 42]
                                    % defaultValue = uint32([11 33 21 42])
                                    dataStructure.dataType{i}.defaultValue = ...
                                        feval(obj.MLTypeConverter(dataType),...
                                        (str2num(defaultValues{i}))'); %#ok<ST2NM>
                                end
                            else
                                
                                % if the datatype is of any other cases
                                % like uint8, float32, bool etc just return
                                % the value as datatype(value).
                                % for example uint32 uint32_val 25,
                                % defaultValue = uint32(25)
                                dataStructure.dataType{i}.defaultValue = ...
                                    feval(obj.MLTypeConverter(dataType),str2double(defaultValues{i}));
                            end
                        else
                            
                            % if there doesn't exits any defaultValue then
                            % defaultValue will be NaN
                            dataStructure.dataType{i}.defaultValue = NaN;
                        end
                        
                        dataStructure.dataType{i}.varsize = ...
                            varsize{i};
                        dataStructure.dataType{i}.maxstrlen = ...
                            str2double(strlen(i));
                    end
                else
                    
                    % Check whether the datatype matches with any existing ROS
                    % datatypes.
                    comparator = strcmp(dataType,obj.ROSDataTypes);
                    if ~any(comparator)
                        
                        if isKey(obj.SpecialMap,dataType)
                            
                            % If the datatype is special datatype then
                            % return its data structure from the map.
                            dataStructure.dataType{i} = obj.SpecialMap(dataType);
                        else
                            
                            % If it doesn't match with any existing datatypes 
                            % and special datatypes, it means that, it is a
                            % nested message, so get the datatypes and 
                            % values of the nested message.
                            dataStructure.dataType{i} = ...
                                nestedMessageParser(obj,dataType,CirDependList);
                            dataStructure.dataType{i}.count = 0;
                            
                            % If the datatype is not standard datatype like
                            % int, float etc, then constantValues and
                            % defaultValues are not applicable, so values of
                            % defaultValue and constantValue will be NaN.
                            dataStructure.dataType{i}.constantValue = NaN;
                            dataStructure.dataType{i}.defaultValue = NaN;
                            dataStructure.dataType{i}.varsize = ...
                                varsize{i};
                        end
                    else
                        
                        % If it matches with any existing data types, then store
                        % that datatype and value into final data structure.
                        dataStructure.dataType{i}.MLdataType = ...
                            obj.MLTypeConverter(dataType);
                        dataStructure.dataType{i}.CPPdataType = ...
                            obj.CPPTypeConverter(dataType);
                        dataStructure.dataType{i}.ROSdataType = dataType;
                        dataStructure.dataType{i}.count = 0;

                        if ~isequal(constantValues{i}, ' ')
                            dataStructure.dataType{i}.rawConstantValue = constantValues{i};
                            if (isequal(obj.MLTypeConverter(dataType),'string') ...
                                || isequal(obj.MLTypeConverter(dataType),'char')) && isnan(str2double(constantValues{i}))   
                                % Get rid of enclosing single | double quotes
                                stringConstantValue = regexprep(constantValues{i},'^(''|")(.*)\1$','$2');
                               
                                % If the expression is '%d\'test', then the  
                                % output should be '%d''test'.
                                updatedStringConstantValue = getUpdatedString(obj,stringConstantValue);
                                dataStructure.dataType{i}.constantValue = ...
                                    feval(obj.MLTypeConverter(dataType),updatedStringConstantValue);                   
                            else
                                
                                % if the datatype is of any other cases
                                % like uint8, float32, bool etc just return
                                % the value as datatype(value).
                                % for example uint32 uint32_val = 25,
                                % constantValue = uint32(25)
                                dataStructure.dataType{i}.constantValue = ...
                                    feval(obj.MLTypeConverter(dataType),str2double(constantValues{i}));
                            end
                        else
                            
                            % if there doesn't exits any constantValue then
                            % constantValue will be NaN                            
                            dataStructure.dataType{i}.constantValue = NaN;
                        end                        

                        if ~isequal(defaultValues{i}, ' ')
                            if (isequal(obj.MLTypeConverter(dataType),'string')||isequal(obj.MLTypeConverter(dataType),'char'))&&isnan(str2double(defaultValues{i}))           
                                % for the case string strdefault "test",
                                % it should return defaultValue as "test"
                                % Here the value of defaultValues{i} will
                                % be like ''%dtest'', but it should be only
                                % '%dtest', hence we are taking the characters
                                % from 2 to end-1.
                                stringDefaultValue = defaultValues{i}(2:end-1);
                                
                                % If the expression is '%d\'test', then the  
                                % output should be '%d''test'.
                                updatedStringDefaultValue = getUpdatedString(obj,stringDefaultValue);
                                dataStructure.dataType{i}.defaultValue = ...
                                    feval(obj.MLTypeConverter(dataType),updatedStringDefaultValue);
                            else
                                
                                % if the datatype is of any other cases
                                % like uint8, float32, bool etc, just return
                                % the value as datatype(value).
                                % for example uint32 uint32_val 25,
                                % defaultValue = uint32(25)                                
                                dataStructure.dataType{i}.defaultValue = ...
                                    feval(obj.MLTypeConverter(dataType),str2double(defaultValues{i}));
                            end
                        else
                            
                            % if there doesn't exits any defaultValue then
                            % defaultValue will be NaN                            
                            dataStructure.dataType{i}.defaultValue = NaN;
                        end        
                        
                        dataStructure.dataType{i}.varsize = ...
                            varsize{i};
                        dataStructure.dataType{i}.maxstrlen = ...
                            str2double(strlen(i));                        
                    end
                end
            end
            for i = 1:numel(dataStructure.value)
                
                % Get the final data structure of message data
                messageDefinition.msgFields.(dataStructure.value(i)) = ...
                    dataStructure.dataType{i};
            end    
            [obj.MessagePackageName, obj.MessageFileName] = fileparts(obj.Msg);
        end  
        
        function contentsOfFile = ...
                getMessageFileContentsWithoutComments(~,location)
            %getMessageFileContentsWithoutComments returns the contents of
            %   message file with no comments in it.
            
            %Read the file and get contents of it.
            k = 1;
            contentsOfFile = string.empty;
            file = fileread(location);
            fileData = splitlines(strtrim(file));
            
            % For each line, check whether it starts with comment or
            % whether it is an empty line. If any of these conditions
            % satisfies the ignore that line and return the remaining lines.
            for i = 1:numel(fileData)
                fileData{i} = strtrim(fileData{i});
                if ~startsWith(fileData{i}, "#") && ~isempty(fileData{i})
                    contentsOfFile(k) = fileData{i};
                    
                    % Remove comments in the line if any.
                    contentsOfFile(k) = regexprep(contentsOfFile(k),'\s+#.*','');
                    contentsOfFile(k) = strtrim(contentsOfFile(k));
                    k = k+1;
                end
            end
        end
        
        function [contentsOfFile,constantValue,defaultValue,varsize,strlen] = ...
                getUpdatedMessageFileContents(obj, contentsOfFile)
            %getUpdatedMessageFileContents returns constant values in the message
            %   and checks if the nested messages are defined with its
            %   package. If not defined with its package, then returns
            %   that nested message including its package.
            
            constantValue = string.empty;
            defaultValue = string.empty;
            strlen = string.empty;
            varsize = cell.empty;
            for i = 1:numel(contentsOfFile)
                
                varsize{i} = false;
                % If there exists any constant values in a message file,
                % return them.
                if numel(strsplit(contentsOfFile(i),'=')) > 1
                    
                    % Matches the size limiter characteristic to bounded strings
                    % e.g. string<=5 my_string
                    matchBoundStr = regexp(contentsOfFile(i),'[<=][>=]','match');
                    
                    % Matches the size limiter characteristic to bounded arrays
                    % e.g. uint8[<=5] my_int
                    matchBoundArr = regexp(contentsOfFile(i),'[<=','match');
                    if isempty(matchBoundStr)
                        
                        % For the cases like uint8 UINT8CONST = 250
                        contents = strsplit(contentsOfFile(i),'=');
                        [contentsOfFile(i),constantValue(i)] = contents{:};
                        contentsOfFile(i) = strtrim(contentsOfFile(i));
                        constantValue(i) = strtrim(constantValue(i));
                        if isequal(constantValue(i),"true")
                            
                            % For a case like bool BOOLCONST = true
                            constantValue(i) = "1";
                        elseif isequal(constantValue(i),"false")
                            
                            % For a case like bool BOOLCONST = false
                            constantValue(i) = "0";
                        end
                    elseif ~isempty(matchBoundArr)
                        
                        % Found a line like float[<=3] dim
                        constantValue(i) = ' ';
                        varsize{i} = true;
                        contentsOfFile(i) = replace(contentsOfFile(i),matchBoundStr,'');
                    else 
                        
                        % For the cases like string<=2[5] five_integers_array_of_string_up_to_ten_characters_each
                        constantValue(i) = ' ';
                        contentsOfFile(i) = replace(contentsOfFile(i),matchBoundStr,'');                        
                    end
                else
                    
                    % If the field doesn't have any '=' in it, then
                    % constantValue is not applicable to it.
                    constantValue(i) = ' ';
                end
                currentLine = strsplit(contentsOfFile(i));
                if isequal(numel(currentLine),2)
                    
                    % If size of currentLine is 2, it means it has only
                    % datatype and varName, so just return them.
                    [dataType, value] = currentLine{:};
                    defaultValue(i) = ' ';
                else
                    
                    % If size of currentLine is greater than 2, then it
                    % means that it has a default value defined, so first
                    % element will be datatype and second element
                    % will be variable name and the rest of the part will be
                    % defaultValue.
                    dataType = currentLine{1};
                    value = currentLine{2};
                    
                    % For a case like string STRCONST "Hello world"
                    defaultValue(i) = strjoin(currentLine(3:end));
                    
                    % Remove the empty spaces if any
                    defaultValue(i) = strtrim(defaultValue(i));
                    dataType = strtrim(dataType);
                end
                
                if isequal(defaultValue(i),"true")
                    
                    % For a case like bool BOOLCONST true
                    defaultValue(i) = "1";
                elseif isequal(defaultValue(i),"false")
                    
                    % For a case like bool BOOLCONST false
                    defaultValue(i) = "0";
                end

                % Check if the datatype is an array or not.
                if numel(strsplit(dataType,'['))>1
                    dataType = strsplit(dataType,'[');
                    [dataType, remainingpart] = dataType{:};
                    [dataType, strlen(i)] = getStringLength(obj,dataType);
                    comparator = strcmp(dataType,obj.ROSDataTypes);
                    
                    % Check if the datatype matches with any of standard
                    % datatypes.
                    if ~any(comparator)

                        % If it doesn't match, then it's a nested message or
                        % a special datatype. So check whether it is a
                        % special datatype
                        if (numel(strsplit(dataType,'/')) == 1) && ~isKey(obj.SpecialMap,dataType)
                            
                            % If message is nested message, but of same package
                            % then return the message by including the current
                            % package.
                            dataType = [obj.MessagePackageName,'/',dataType]; %#ok<AGROW>
                        end
                        dataTypeWithPackage = [dataType, '[', remainingpart];
                        contentsOfFile(i) = ...
                            [dataTypeWithPackage, ' ', value];
                    else
                        contentsOfFile(i) = [dataType, '[' remainingpart ' ', value];
                    end
                else
                    
                    % get the string length if applicable, i.e. for the
                    % cases like string<=5[] string_test, maxstrlen
                    % should be returned as 5.
                    [dataType,strlen(i)] = getStringLength(obj,dataType);
                    comparator = strcmp(dataType,obj.ROSDataTypes);
                    
                    % Check if the datatype matches with any of standard
                    % datatypes.
                    if ~any(comparator)
                        
                        % If it doesn't match, then it's a nested message or
                        % a special datatype. So check whether it is a
                        % special datatype
                        if (numel(strsplit(dataType,'/')) == 1) && ~isKey(obj.SpecialMap,dataType)
                            
                            % If message is nested message, but of same package
                            % then return the message by including the current
                            % package.
                            dataTypeWithPackage = [obj.MessagePackageName,...
                                '/',dataType];
                            contentsOfFile(i) = ...
                                [dataTypeWithPackage, ' ', value];
                        end
                    else
                        contentsOfFile(i) = [dataType, ' ', value];
                    end
                end
            end
        end
                
    end
    
    methods (Access = private)

        function [MLDataType, CPPDataType] = createROSToMlAndCppDataTypesMap(obj)
            %createROSToMlAndCppDataTypesMap creates a map of ROS datatypes
            %   corresponding to its MATLAB and CPP datatypes.
            
            MLDataType = containers.Map(obj.ROSDataTypes, obj.MLDataTypes);
            CPPDataType = containers.Map(obj.ROSDataTypes, obj.CPPDataTypes);
        end

        function [dataStructure] = nestedMessageParser(obj, dataType, CirDependList)
            %nestedMessageParser returns a structure which contains all
            %   information about the message. This is invoked, if there
            %   exists a nested message in a message.
            
            dataType = strsplit(dataType,'/');
            
            % If the dataType contains a '/', it means the nested message
            % is depending on another message package, and if the dataType
            % doesn't contain '/', it means the nested message belongs to
            % the current message package but different message file.
            if isequal(numel(dataType),2)
                [obj.MessagePackageName,obj.MessageFileName] = dataType{:};
            end
            dataStructure = getMessageDefinitionHelper(obj, CirDependList);
        end
        
        function count = getCountOfDataTypes(~, numberOfDataTypes)
            %getCountOfDataTypes returns number of elements in array.
            
            count = strsplit(numberOfDataTypes, ']');
            count = count{1};
        end
        
        function [dataType,strlen] = getStringLength(~, dataType)
            %getStringLength returns the length of the string specified.
            
            if contains(dataType,'string')
                strlen = dataType(isstrprop(dataType,'digit'));
                dataType = dataType(isstrprop(dataType,'alpha'));
            else
                strlen = ' ';
            end
        end
        
        function updatedStringValue = getUpdatedString(obj, stringValue)
            %getUpdatedString checks the given string and replaces 
            %   "\'" with "''" if any.
            
            updatedStringValue = regexprep(stringValue,obj.InputPattern,obj.OutputPattern);
            updatedStringValue = regexprep(updatedStringValue,'\\"','"');
        end

    end
    
end

% LocalWords:  ROSdataTypes MLdataTypes wstring dirs messageparser STRINGCONST dtest strdefault BOOLCONST STRCONST maxstrlen

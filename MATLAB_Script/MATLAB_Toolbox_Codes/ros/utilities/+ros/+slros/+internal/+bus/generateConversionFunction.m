function conversionFcn = generateConversionFunction(emptyMsg, info, type, pkgName, msgName, cachedMap, refCachedMap, direction, genFolder)
% generateConversionFunction Generate conversion functions for Simulink bus
% to ROS Message and ROS Message to Simulink bus

%   Copyright 2021 The MathWorks, Inc.

rosType = validatestring(type, {'ros','ros2'});
simDirection = validatestring(direction, {'msgToBus','busToMsg'});
conversionFcnStr = ['bus_conv_fcns.' [rosType '.'] [simDirection '.'] [pkgName '.'] msgName];

% generate top-level conversion function
generateFunctionFromTemplates(emptyMsg,rosType,pkgName,msgName,info,genFolder,simDirection);
if isequal(type,'ros')
    matPath = l_convertMATPath(info.MatPath, info);
else
    matPath = info.MatPath;
end
for k = matPath
    % generate nested conversion functions
    thisInfo = eval(['info.',(k{1})]);
    if isfield(thisInfo, 'MessageType')
        [pkgName,msgName] = fileparts(thisInfo.MessageType);
        [emptyMsg, msgInfo]= ros.internal.getEmptyMessage(thisInfo.MessageType,type);
        cachedMap(thisInfo.MessageType) = emptyMsg;
        if ~isKey(refCachedMap,thisInfo.MessageType)
            generateFunctionFromTemplates(emptyMsg, rosType, pkgName, msgName, msgInfo,genFolder, simDirection)
            refCachedMap(msgInfo.MessageType) = emptyMsg;
        elseif ~isequal(refCachedMap(thisInfo.MessageType),emptyMsg)
            generateFunctionFromTemplates(emptyMsg, rosType, pkgName, msgName, msgInfo,genFolder, simDirection)
            refCachedMap(msgInfo.MessageType) = emptyMsg;
        end
    end
    if isequal(thisInfo.MLdataType,'string')
        [emptyMsg, msgInfo]= ros.internal.getEmptyMessage('std_msgs/String',type);
        cachedMap('std_msgs/String') = emptyMsg;
        if ~isKey(refCachedMap,'std_msgs/String')
            generateFunctionFromTemplates(emptyMsg, rosType, 'std_msgs', 'String', msgInfo,genFolder, simDirection)
            refCachedMap(msgInfo.MessageType) = emptyMsg;
        elseif ~isequal(refCachedMap('std_msgs/String'),emptyMsg)
            generateFunctionFromTemplates(emptyMsg, rosType, 'std_msgs', 'String', msgInfo,genFolder, simDirection)
            refCachedMap(msgInfo.MessageType) = emptyMsg;
        end
    end
end
conversionFcn = str2func(conversionFcnStr);
end

function generateFunctionFromTemplates(emptyMsg, rosType, pkgName, msgName, msgInfo, genFolder, simDirection)
fcnFolderName = fullfile(genFolder,['+' rosType],['+' simDirection],['+' pkgName]);
if ~exist(fcnFolderName,'dir')
    mkdir(fcnFolderName);
end
fcnFileName = fullfile(fcnFolderName,[msgName '.m']);
tmplInfo.MsgInfo = msgInfo;
tmplInfo.MsgName = msgName;
tmplInfo.EmptyMsg = emptyMsg;
tmplInfo.RosVersion = rosType;
tmplInfo.MessageType = [pkgName,'/',msgName];
tmpl = ros.internal.emitter.MLTemplate;
tmpl.loadFile(fullfile(toolboxdir('ros'),'utilities','resources',[simDirection,'.m.tmpl']));
tmpl.outFile = fcnFileName;
tmpl.render(tmplInfo,2);
end

function ret = l_convertMATPath(matPath, info)
ret = cell(1,numel(matPath));
i = 1;
for k = matPath
    j = 1;
    allFields = strsplit(k{1},'.');
    classFields = cell(1,numel(allFields));
    thisInfo = info;
    for m = allFields
        classFields{j} = ros.internal.utilities.convertROSFieldsToClassFields(m{1}, thisInfo.MessageType);
        thisInfo = thisInfo.(classFields{j});
        j = j+1;
    end
    ret{i} = strjoin(classFields,'.');
    i=i+1;
end
end

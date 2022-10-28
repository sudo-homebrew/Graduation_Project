function [files, msgDependsForRequest, msgDependsForResponse] = clientserverEmitter(srv,dirsToLook,rosver,refMsgTypeCheckSumMap,registry,checkSumMap,outDir,outDirStruct,outDirExtra)
%This function is for internal use only. It may be removed in the future.

%CLIENTSERVEREMITTER generates service-related files for given ROS version
% clientserverEmitter(SRV,DIRSTOLOOK,ROSVER,OUTDIR,OUTDIRSTRUCT,OUTDIREXTRA)
% generates files related to the service type SRV for ROS version ROSVER.
% The service definition is expected to be found in the package
% subdirectory of one of DIRSTOLOOK. The service client and server C++
% files will be generated in OUTDIR. The ROS message struct creation MATLAB
% files will be generated in OUTDIRSTRUCT. If ROS 1, the message class
% MATLAB files will be generated in OUTDIREXTRA.
%
% Example:
%    files = ros.internal.clientserverEmitter('diagnostic_msgs/AddDiagnostics',...
%        {'B:\matlab\sys\ros1\win64\ros1\share'},...
%        'ros',...
%        'B:\matlab\toolbox\ros\mlroscpp\messages\src\std_msgs',...
%        'B:\matlab\toolbox\ros\mlroscpp\+ros\+internal\+ros\+messages',...
%        'B:\matlab\toolbox\ros\mlroscpp\+ros\+msggen\+diagnostic_msgs')

%   Copyright 2019-2021 The MathWorks, Inc.

% Parse the message into MessageParser and get its data structure.
    if nargin < 7
        outDir = pwd;
    end
    if nargin < 8
        outDirStruct = outDir;
    end
    if nargin < 9
        outDirExtra = outDir;
    end

    if isequal(rosver,'ros2')
        rospath = 'mlros2';
        parser = ros.internal.ServiceParser(srv,dirsToLook);
    else
        rospath = 'mlroscpp';
        map = ros.internal.utilities.getSpecialMapForROS;
        parser = ros.internal.ServiceParser(srv,dirsToLook,map);
    end

    [msgDefnForRequest, msgDefnForResponse] = getServiceDefinition(parser);
    msgDefnForRequestActual = msgDefnForRequest;
    msgDefnForResponseActual = msgDefnForResponse;
    msgDefnForRequestActual.msgInfo = ros.internal.(rosver).getMessageInfo(msgDefnForRequestActual.MessageType, registry);
    msgDefnForResponseActual.msgInfo = ros.internal.(rosver).getMessageInfo(msgDefnForResponseActual.MessageType, registry);
    msgDefnForRequestActual = ros.internal.(rosver).augmentMessageDefinition(msgDefnForRequestActual, registry);
    msgDefnForResponseActual = ros.internal.(rosver).augmentMessageDefinition(msgDefnForResponseActual, registry);
    msgDefnForRequest = ros.internal.(rosver).augmentServiceDefinition(msgDefnForRequest,parser.Srv,'Request', registry);
    msgDefnForResponse = ros.internal.(rosver).augmentServiceDefinition(msgDefnForResponse,parser.Srv,'Response', registry);

    if isequal(rosver,'ros')
        msgSpecRequest = ros.internal.(rosver).createMsgSpec(msgDefnForRequest);
        msgDefnForRequest.MD5Checksum = ros.internal.(rosver).computeMsgMd5(msgSpecRequest, refMsgTypeCheckSumMap, msgDefnForRequest);

        msgSpecResponse = ros.internal.(rosver).createMsgSpec(msgDefnForResponse);
        msgDefnForResponse.MD5Checksum = ros.internal.(rosver).computeMsgMd5(msgSpecResponse, refMsgTypeCheckSumMap, msgDefnForResponse);
    else
        msgDefnForRequest.MD5Checksum = '';
        msgDefnForResponse.MD5Checksum = '';
    end

    %store the Md5 checksum of base type in the refMsgTypeCheckSumMap
    refMsgTypeCheckSumMap(msgDefnForRequest.MessageType) = msgDefnForRequest.MD5Checksum;
    refMsgTypeCheckSumMap(msgDefnForResponse.MessageType) = msgDefnForResponse.MD5Checksum;

    msgFields = [msgDefnForRequest.fieldNames ; msgDefnForResponse.fieldNames];
    [isValid, errMsgFieldNames] = ros.internal.isValidFieldName(msgFields);

    if~isValid
       error(message(...
            'ros:utilities:custommsg:SrvFieldNameError',[msgDefnForRequest.msgInfo.msgName '.srv'],errMsgFieldNames));
    end

    %we go only one level. By depends on this level, CMake/Colcon should
    %find other dependencies
    msgDependsForRequest = {};
    msgDependsForResponse = {};
    for i = 1:numel(msgDefnForRequest.fieldNames)
        if isfield(msgDefnForRequest.msgFields.(msgDefnForRequest.fieldNames{i}),'MessageType')
            msgDependsForRequest = [msgDependsForRequest msgDefnForRequest.msgFields.(msgDefnForRequest.fieldNames{i}).MessageType]; %#ok<AGROW>
        end
    end
    msgDependsForRequest = unique(msgDependsForRequest);
    msgDependsForRequest = setdiff(msgDependsForRequest,{'ros/Time','ros/Duration'});

    for i = 1:numel(msgDefnForResponse.fieldNames)
        if isfield(msgDefnForResponse.msgFields.(msgDefnForResponse.fieldNames{i}),'MessageType')
            msgDependsForResponse = [msgDependsForResponse msgDefnForResponse.msgFields.(msgDefnForResponse.fieldNames{i}).MessageType]; %#ok<AGROW>
        end
    end
    msgDependsForResponse = unique(msgDependsForResponse);
    msgDependsForResponse = setdiff(msgDependsForResponse,{'ros/Time','ros/Duration'});
	
    requestCommonTemplate = fullfile(matlabroot,'toolbox','ros',rospath,'+ros','+internal',['+',rosver],'request_common.cpp.tmpl');
    responseCommonTemplate = fullfile(matlabroot,'toolbox','ros',rospath,'+ros','+internal',['+',rosver],'response_common.cpp.tmpl');
    structGenTemplate = fullfile(matlabroot,'toolbox','ros',rospath,'+ros','+internal',['+',rosver],'struct.m.tmpl');
    
    svcCppFileName = [msgDefnForRequest.msgInfo.pkgName,'_',msgDefnForRequest.msgInfo.msgName,'_service.cpp'];
    structGenFileNameForRequest = fullfile(outDirStruct,[lower(msgDefnForRequest.msgInfo.srvName(1)) msgDefnForRequest.msgInfo.srvName(2:end) '.m']);
    structGenFileNameForResponse = fullfile(outDirStruct,[lower(msgDefnForResponse.msgInfo.srvName(1)) msgDefnForResponse.msgInfo.srvName(2:end) '.m']);

    requestMsgUpToDate = false;
    responseMsgUpToDate = false;
    
    %support for rebuild: Based on the differences in the values of cached checksum Map (checkSumMap) of previous build and
    %calculated checksum Map of current build (refMsgTypeCheckSumMap), the generation is done, for both request and response
    if isKey(refMsgTypeCheckSumMap, msgDefnForRequest.MessageType) && isKey(checkSumMap, msgDefnForRequest.MessageType)
        if strcmp(refMsgTypeCheckSumMap(msgDefnForRequest.MessageType), checkSumMap(msgDefnForRequest.MessageType))
            requestMsgUpToDate = true; %No change in msg Definition for request.
        end
    end

    if ~requestMsgUpToDate
        % Generate publisher.cpp and subscriber.cpp files for the message.
        createOutput(msgDefnForRequest,structGenTemplate,structGenFileNameForRequest);
    end

    if isKey(refMsgTypeCheckSumMap, msgDefnForResponse.MessageType) && isKey(checkSumMap, msgDefnForResponse.MessageType)
        if strcmp(refMsgTypeCheckSumMap(msgDefnForResponse.MessageType), checkSumMap(msgDefnForResponse.MessageType))
            responseMsgUpToDate = true; %No change in msg Definition for response
        end
    end

    if ~responseMsgUpToDate
        % Generate publisher.cpp and subscriber.cpp files for the message.
        createOutput(msgDefnForResponse,structGenTemplate,structGenFileNameForResponse);
    end
	
	if ~requestMsgUpToDate || ~responseMsgUpToDate
	    createOutput(msgDefnForRequest,requestCommonTemplate,fullfile(outDir,svcCppFileName));%create
	    createOutput(msgDefnForResponse,responseCommonTemplate,fullfile(outDir,svcCppFileName),'at');%append
	end
	
    files = {svcCppFileName, structGenFileNameForRequest, structGenFileNameForResponse};

    if isequal(rosver,'ros')
        classGenTemplate = fullfile(matlabroot,'toolbox','ros',rospath,'+ros','+internal',['+' rosver],'msg_class.m.tmpl');

        classGenFileNameForRequest = fullfile(outDirExtra,[msgDefnForRequest.msgInfo.srvName '.m']);
        classGenFileNameForResponse = fullfile(outDirExtra,[msgDefnForResponse.msgInfo.srvName '.m']);

        if ~requestMsgUpToDate
            createOutput(msgDefnForRequest, classGenTemplate, classGenFileNameForRequest)
        end
        if ~responseMsgUpToDate
            createOutput(msgDefnForResponse, classGenTemplate, classGenFileNameForResponse)
        end

        files = [files classGenFileNameForRequest classGenFileNameForResponse];
    end

end

function createOutput(data,tmplFile,outFile, outFileOperation)
%Load the given template and render the data in it.
    tmpl = ros.internal.emitter.MLTemplate;
    tmpl.loadFile(tmplFile);
    tmpl.outFile = outFile;
    if (nargin == 4 && isequal(outFileOperation,'at'))
        tmpl.outFileOperation = outFileOperation;
    end   
    tmpl.render(data, 2);
end

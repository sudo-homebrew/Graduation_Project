function [files, msgDepends] = pubsubEmitter(msg,dirsToLook,rosver,refMsgTypeCheckSumMap,registry,checkSumMap,outDir,outDirStruct,outDirExtra)
%This function is for internal use only. It may be removed in the future.

%PUBSUBEMITTER generates message-related files for specified ROS version
% pubsubEmitter(MSG,DIRSTOLOOK,ROSVER,OUTDIR,OUTDIRSTRUCT,OUTDIREXTRA)
% generates files related to message type MSG for ROS version ROSVER. The
% message definition is expected to be found in the package subdirectory of
% one of DIRSTOLOOK. The publisher and subscriber C++ files will be
% generated in OUTDIR. The ROS message struct creation MATLAB file will be
% generated in OUTDIRSTRUCT. If ROS 1, the message class definition MATLAB
% file will be generated in OUTDIREXTRA; if ROS 2, the message struct
% converter MATLAB files will be generated in OUTDIREXTRA.
%
%  cppKeyWords is a Map that contains the cpp keywords as they cannot be
%  field names in msgs/srvs
%
% Example:
%    files = ros.internal.pubsubEmitter('std_msgs/Char',...
%              {'B:\matlab\sys\ros2\win64\ros2\share'},...
%              'ros2',...
%              'B:\matlab\toolbox\ros\mlros2\messages\src\std_msgs',...
%              'B:\matlab\toolbox\ros\mlros2\+ros\+internal\+ros2\+messages',...
%              'B:\matlab\toolbox\ros\mlros2\+ros\+internal\+ros2\+converters')

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
        parser = ros.internal.MessageParser(msg,dirsToLook);
    else
        rospath = 'mlroscpp';
        map = ros.internal.utilities.getSpecialMapForROS;
        parser = ros.internal.MessageParser(msg,dirsToLook,map);
    end

    msgDefn = getMessageDefinition(parser);
    msgDefn.msgInfo = ros.internal.(rosver).getMessageInfo(msgDefn.MessageType, registry);
    msgDefn = ros.internal.(rosver).augmentMessageDefinition(msgDefn, registry);
    msgSpec = ros.internal.ros.createMsgSpec(msgDefn);
    msgDefn.MD5Checksum = ros.internal.ros.computeMsgMd5(msgSpec, refMsgTypeCheckSumMap, msgDefn);

    %store MD5 checksum of base type as well in the refMsgTypeCheckSumMap
    refMsgTypeCheckSumMap(msgDefn.MessageType) = msgDefn.MD5Checksum;
    [isValid, errMsgFieldNames] = ros.internal.isValidFieldName(msgDefn.fieldNames);

    if ~isValid
       error(message(...
            'ros:utilities:custommsg:MsgFieldNameError',[msgDefn.msgInfo.msgName '.msg'],errMsgFieldNames));
    end

    %we go only one level. By depends on this level, CMake/Colcon should
    %find other dependencies
    msgDepends = {};
    for i = 1:numel(msgDefn.fieldNames)
        if isfield(msgDefn.msgFields.(msgDefn.fieldNames{i}),'MessageType')
            msgDepends = [msgDepends msgDefn.msgFields.(msgDefn.fieldNames{i}).MessageType]; %#ok<AGROW>
        end
    end
    msgDepends = unique(msgDepends);
    msgDepends = setdiff(msgDepends,{'ros/Time','ros/Duration'});

    commonTemplate = fullfile(matlabroot,'toolbox','ros',rospath,'+ros','+internal',['+',rosver],'common.cpp.tmpl');
    commonFileName = [msgDefn.msgInfo.pkgName,'_',msgDefn.msgInfo.msgName,'_message.cpp'];

    %support for rebuild: Based on the differences in the values of cached MD5 Map (checkSumMap) of previous build and
    %calculated checksum Map of current build (refMsgTypeCheckSumMap), the generation is done, for both request and response
    msgUpToDate = false;
    if isKey(refMsgTypeCheckSumMap, msgDefn.MessageType) && isKey(checkSumMap, msgDefn.MessageType)
       if strcmp(refMsgTypeCheckSumMap(msgDefn.MessageType), checkSumMap(msgDefn.MessageType))
           msgUpToDate = true; %No change in Msg Definition
       end
    end

    if ~msgUpToDate
        createOutput(msgDefn,commonTemplate,fullfile(outDir,commonFileName));
%         createOutput(msgDefn,publisherTemplate,fullfile(outDir,pubFileName));
%         createOutput(msgDefn,subscriberTemplate,fullfile(outDir,subFileName));
    end

    if isequal(rosver,'ros2')
        structGenTemplate = fullfile(matlabroot,'toolbox','ros',rospath,'+ros','+internal',['+',rosver],'struct.m.tmpl');
        converter1To2Template = fullfile(matlabroot,'toolbox','ros','mlros2','+ros','+internal','+ros2','converterGeneration1To2.m.tmpl');
        converter2To1Template = fullfile(matlabroot,'toolbox','ros','mlros2','+ros','+internal','+ros2','converterGeneration2To1.m.tmpl');

        structGenFileName = fullfile(outDirStruct,[lower(msgDefn.msgInfo.msgName(1)) msgDefn.msgInfo.msgName(2:end) '.m']);
        converter1To2GenFileName = fullfile(outDirExtra,[msgDefn.msgInfo.baseName '_1To2_Converter.m']);
        converter2To1GenFileName = fullfile(outDirExtra,[msgDefn.msgInfo.baseName '_2To1_Converter.m']);
        files = {commonFileName, structGenFileName,...
                 converter1To2GenFileName, converter2To1GenFileName};

        if ~msgUpToDate
            createOutput(msgDefn,structGenTemplate,structGenFileName);
            createOutput(msgDefn,converter1To2Template,converter1To2GenFileName);
            createOutput(msgDefn,converter2To1Template,converter2To1GenFileName);
        end
    else
        structGenTemplate = fullfile(matlabroot,'toolbox','ros',rospath,'+ros','+internal',['+',rosver],'struct.m.tmpl');

        structGenFileName = fullfile(outDirStruct,[lower(msgDefn.msgInfo.msgName(1)) msgDefn.msgInfo.msgName(2:end) '.m']);

        classGenTemplate = fullfile(matlabroot,'toolbox','ros',rospath,'+ros','+internal',['+' rosver],'msg_class.m.tmpl');

        classGenFileName = fullfile(outDirExtra,[msgDefn.msgInfo.msgName '.m']);

        files = {commonFileName, structGenFileName,classGenFileName};

        if ~msgUpToDate
            createOutput(msgDefn,structGenTemplate,structGenFileName);
            createOutput(msgDefn, classGenTemplate, classGenFileName);
        end
    end

end

function createOutput(data,tmplFile,outFile)
%Load the given template and render the data in it.

    tmpl = ros.internal.emitter.MLTemplate;
    tmpl.loadFile(tmplFile);
    tmpl.outFile = outFile;
    tmpl.render(data, 2);
end

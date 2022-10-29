function [files, msgDependsForGoal, msgDependsForResult, msgDependsForFeedback] = actionEmitter(action,dirsToLook,rosver, refMsgTypeCheckSumMap,registry,checkSumMap,outDir,outDirStruct,outDirExtra)
%This function is for internal use only. It may be removed in the future.

%ACTIONEMITTER generates action-related files for specified ROS version
% actionEmitter(ACTION,DIRSTOLOOK,ROSVER,OUTDIR,OUTDIRSTRUCT,OUTDIREXTRA)
% generates files related to action type ACTION for ROS version ROSVER. THE
% action definition is expected to be found in the package subdirectory of
% one of DIRSTOLOOK. The action client C++ files will be generated in
% OUTDIR. The ROS message struct creation MATLAB files will be generated in
% OUTDIRSTRUCT. If ROS 1, the message class definition MATLAB files will be
% generated in OUTDIREXTRA.
%
% Example:
%  files = ros.internal.actionEmitter('actionlib/Test',...
%      {'B:\matlab\sys\ros1\win64\ros1\share'},...
%      'ros',...
%      'B:\matlab\toolbox\ros\mlroscpp\messages\src\actionlib',...
%      'B:\matlab\toolbox\ros\mlroscpp\+ros\+internal\+ros\+messages', ...
%      'B:\matlab\toolbox\ros\mlroscpp\+ros\+msggen\+actionlib')

%   Copyright 2020-2021 The MathWorks, Inc.

% Parse the message into MessageParser and get its data structure.
    if nargin < 4
        outDir = pwd;
    end
    if nargin < 5
        outDirStruct = outDir;
    end
    if nargin < 6
        outDirExtra = outDir;
    end

    if isequal(rosver,'ros2')
        rospath = 'mlros2';
        parser = ros.internal.ActionParser(action,dirsToLook);
    else
        rospath = 'mlroscpp';
        map = ros.internal.utilities.getSpecialMapForROS;
        parser = ros.internal.ActionParser(action,dirsToLook,map);
    end

    [msgDefnForGoal, msgDefnForResult, msgDefnForFeedback] = getActionDefinition(parser);
    
    %Any action messages will have common dependencies but will vary only
    %   fieldnames related to goal, result or feedback. Any ActionGoal.msg
    %   will have first two fieldnames as "std_msgs/Header Header" and
    %   "actionlib_msgs/GoalID GoalId" and varies with the third field.
    %   Similarly ActionFeedback.msg and ActionResult.msg for actions will
    %   have its first two fields as common.
    
    %Get the message definition for std_msgs/Header.
    parserForHeader = ros.internal.MessageParser('std_msgs/Header',dirsToLook,map);
    msgDefnForHeader = getMessageDefinition(parserForHeader);
    msgDefnForHeader.count = 0;
    msgDefnForHeader.constantValue = NaN;
    msgDefnForHeader.defaultValue = NaN;
    msgDefnForHeader.varsize = 0;
    
    %Get the message definition for actionlib_msgs/GoalID
    parserForGoalID = ros.internal.MessageParser('actionlib_msgs/GoalID',dirsToLook,map);
    msgDefnForGoalID = getMessageDefinition(parserForGoalID);
    msgDefnForGoalID.count = 0;
    msgDefnForGoalID.constantValue = NaN;
    msgDefnForGoalID.defaultValue = NaN;
    msgDefnForGoalID.varsize = 0;
    
    %Get the message definition for actionlib_msgs/GoalStatus
    parserForGoalStatus = ros.internal.MessageParser('actionlib_msgs/GoalStatus',dirsToLook,map);
    msgDefnForGoalStatus = getMessageDefinition(parserForGoalStatus);
    msgDefnForGoalStatus.count = 0;
    msgDefnForGoalStatus.constantValue = NaN;
    msgDefnForGoalStatus.defaultValue = NaN;
    msgDefnForGoalStatus.varsize = 0;
    
    %Get the message definition for the action goal, action result and
    %   action feedback.
    msgDefnForActionGoal = getMessageDefinitionForActionGoal(action, msgDefnForGoal, msgDefnForHeader, msgDefnForGoalID);
    msgDefnForActionResult = getMessageDefinitionForActionResult(action, msgDefnForResult, msgDefnForHeader, msgDefnForGoalStatus);
    msgDefnForActionFeedback = getMessageDefinitionForActionFeedback(action, msgDefnForFeedback, msgDefnForHeader, msgDefnForGoalStatus);
    msgDefnForAction = getMessageDefinitionForAction(action, msgDefnForActionGoal, msgDefnForActionResult, msgDefnForActionFeedback);
    
    msgDefnForGoal.msgInfo = ros.internal.(rosver).getActionInfo(msgDefnForGoal.MessageType,parser.Action,'Goal');
    msgDefnForGoal = ros.internal.(rosver).augmentMessageDefinition(msgDefnForGoal, registry);
    msgSpecForGoal = ros.internal.ros.createMsgSpec(msgDefnForGoal);
    msgDefnForGoal.MD5Checksum = ros.internal.ros.computeMsgMd5(msgSpecForGoal, refMsgTypeCheckSumMap, msgDefnForGoal);

    %store MD5 checksum of base type as well in the refMsgTypeCheckSumMap
    refMsgTypeCheckSumMap(msgDefnForGoal.MessageType) = msgDefnForGoal.MD5Checksum;
    
    msgDefnForResult.msgInfo = ros.internal.(rosver).getActionInfo(msgDefnForResult.MessageType,parser.Action,'Result');
    msgDefnForResult = ros.internal.(rosver).augmentMessageDefinition(msgDefnForResult, registry);
    msgSpecForResult = ros.internal.ros.createMsgSpec(msgDefnForResult);
    msgDefnForResult.MD5Checksum = ros.internal.ros.computeMsgMd5(msgSpecForResult, refMsgTypeCheckSumMap, msgDefnForResult);

    %store MD5 checksum of base type as well in the refMsgTypeCheckSumMap
    refMsgTypeCheckSumMap(msgDefnForResult.MessageType) = msgDefnForResult.MD5Checksum;
    
    msgDefnForFeedback.msgInfo = ros.internal.(rosver).getActionInfo(msgDefnForFeedback.MessageType,parser.Action,'Feedback');
    msgDefnForFeedback = ros.internal.(rosver).augmentMessageDefinition(msgDefnForFeedback, registry);
    msgSpecForFeedback = ros.internal.ros.createMsgSpec(msgDefnForFeedback);
    msgDefnForFeedback.MD5Checksum = ros.internal.ros.computeMsgMd5(msgSpecForFeedback, refMsgTypeCheckSumMap, msgDefnForFeedback);

    %store MD5 checksum of base type as well in the refMsgTypeCheckSumMap
    refMsgTypeCheckSumMap(msgDefnForFeedback.MessageType) = msgDefnForFeedback.MD5Checksum;
    
    %Get the message definitions from augmentMessageDefinition for action
    %   goal, action result and action feedback and the store their
    %   md5CheckSums.
    [msgDefnForActionGoal,refMsgTypeCheckSumMap] = getAugmentedMessageDefinitionAndMd5ChecksumMap(msgDefnForActionGoal,rosver,registry,refMsgTypeCheckSumMap);
    [msgDefnForActionResult,refMsgTypeCheckSumMap] = getAugmentedMessageDefinitionAndMd5ChecksumMap(msgDefnForActionResult,rosver,registry,refMsgTypeCheckSumMap);
    [msgDefnForActionFeedback,refMsgTypeCheckSumMap] = getAugmentedMessageDefinitionAndMd5ChecksumMap(msgDefnForActionFeedback,rosver,registry,refMsgTypeCheckSumMap);
    [msgDefnForAction,refMsgTypeCheckSumMap] = getAugmentedMessageDefinitionAndMd5ChecksumMap(msgDefnForAction,rosver,registry,refMsgTypeCheckSumMap);
    
    msgFields = [msgDefnForGoal.fieldNames ; msgDefnForResult.fieldNames ; msgDefnForFeedback.fieldNames];
    [isValid, errMsgFieldNames] = ros.internal.isValidFieldName(msgFields);

    if~isValid
       error(message(...
            'ros:utilities:custommsg:ActionFieldNameError',[msgDefnForGoal.msgInfo.msgName '.action'],errMsgFieldNames));
    end
    
    %we go only one level. By depends on this level, CMake/Colcon should
    %find other dependencies
    msgDependsForGoal = {};
    msgDependsForResult = {};
    msgDependsForFeedback = {};
    
    for i = 1:numel(msgDefnForGoal.fieldNames)
        if isfield(msgDefnForGoal.msgFields.(msgDefnForGoal.fieldNames{i}),'MessageType')
            msgDependsForGoal = [msgDependsForGoal msgDefnForGoal.msgFields.(msgDefnForGoal.fieldNames{i}).MessageType]; %#ok<AGROW>
        end
    end
    msgDependsForGoal = unique(msgDependsForGoal);
    msgDependsForGoal = setdiff(msgDependsForGoal,{'ros/Time','ros/Duration'});

    for i = 1:numel(msgDefnForResult.fieldNames)
        if isfield(msgDefnForResult.msgFields.(msgDefnForResult.fieldNames{i}),'MessageType')
            msgDependsForResult = [msgDependsForResult msgDefnForResult.msgFields.(msgDefnForResult.fieldNames{i}).MessageType]; %#ok<AGROW>
        end
    end
    msgDependsForResult = unique(msgDependsForResult);
    msgDependsForResult = setdiff(msgDependsForResult,{'ros/Time','ros/Duration'});
    
    for i = 1:numel(msgDefnForFeedback.fieldNames)
        if isfield(msgDefnForFeedback.msgFields.(msgDefnForFeedback.fieldNames{i}),'MessageType')
            msgDependsForFeedback = [msgDependsForFeedback msgDefnForFeedback.msgFields.(msgDefnForFeedback.fieldNames{i}).MessageType]; %#ok<AGROW>
        end
    end
    msgDependsForFeedback = unique(msgDependsForFeedback);

    msgDependsForFeedback = setdiff(msgDependsForFeedback,{'ros/Time','ros/Duration'});

    structGenTemplate = fullfile(matlabroot,'toolbox','ros',rospath,'+ros','+internal',['+',rosver],'struct.m.tmpl');
    actCppHeadersTemplate = fullfile(matlabroot,'toolbox','ros',rospath,'+ros','+internal',['+',rosver],'action_header.tmpl');
    actCppBodyCommonTemplate = fullfile(matlabroot,'toolbox','ros',rospath,'+ros','+internal',['+',rosver],'action_common.tmpl');
    actCppFooterTemplate = fullfile(matlabroot,'toolbox','ros',rospath,'+ros','+internal',['+',rosver],'action_footer.tmpl');
    commonTemplate = fullfile(matlabroot,'toolbox','ros',rospath,'+ros','+internal',['+',rosver],'common.cpp.tmpl');
    
    structGenFileNameForGoal = fullfile(outDirStruct,[lower(msgDefnForGoal.msgInfo.msgName(1)) msgDefnForGoal.msgInfo.msgName(2:end) '.m']);
    structGenFileNameForResult = fullfile(outDirStruct,[lower(msgDefnForResult.msgInfo.msgName(1)) msgDefnForResult.msgInfo.msgName(2:end) '.m']);
    structGenFileNameForFeedback = fullfile(outDirStruct,[lower(msgDefnForFeedback.msgInfo.msgName(1)) msgDefnForFeedback.msgInfo.msgName(2:end) '.m']);
    actCppOutFile = [msgDefnForGoal.msgInfo.pkgName,'_',msgDefnForGoal.msgInfo.actionName,'_action.cpp'];
    
    structGenFileNameForActionGoal = fullfile(outDirStruct,[lower(msgDefnForActionGoal.msgInfo.msgName(1)) msgDefnForActionGoal.msgInfo.msgName(2:end) '.m']);
    structGenFileNameForActionResult = fullfile(outDirStruct,[lower(msgDefnForActionResult.msgInfo.msgName(1)) msgDefnForActionResult.msgInfo.msgName(2:end) '.m']);
    structGenFileNameForActionFeedback = fullfile(outDirStruct,[lower(msgDefnForActionFeedback.msgInfo.msgName(1)) msgDefnForActionFeedback.msgInfo.msgName(2:end) '.m']);
    structGenFileNameForAction = fullfile(outDirStruct,[lower(msgDefnForAction.msgInfo.msgName(1)) msgDefnForAction.msgInfo.msgName(2:end) '.m']);    
    
    actionGoalCommonFile = [msgDefnForActionGoal.msgInfo.pkgName,'_',msgDefnForActionGoal.msgInfo.msgName,'_message.cpp'];
    actionResultCommonFile = [msgDefnForActionResult.msgInfo.pkgName,'_',msgDefnForActionResult.msgInfo.msgName,'_message.cpp'];
    actionFeedbackCommonFile = [msgDefnForActionFeedback.msgInfo.pkgName,'_',msgDefnForActionFeedback.msgInfo.msgName,'_message.cpp'];
    actionCommonFile = [msgDefnForAction.msgInfo.pkgName,'_',msgDefnForAction.msgInfo.msgName,'_message.cpp'];

    
	goalMsgUpToDate = false;
    resultMsgUpToDate = false;
    feedbackMsgUpToDate = false;
    %support for rebuild: Based on the differences in the values of cached checksum Map (checkSumMap) of previous build and
    %calculated checksum Map of current build (refMsgTypeCheckSumMap), the
    %generation is done, for goal, result and feedback.
    if isKey(refMsgTypeCheckSumMap, msgDefnForGoal.MessageType) && isKey(checkSumMap, msgDefnForGoal.MessageType)
        if strcmp(refMsgTypeCheckSumMap(msgDefnForGoal.MessageType), checkSumMap(msgDefnForGoal.MessageType))
            goalMsgUpToDate = true;
        end
    end
    
    if isKey(refMsgTypeCheckSumMap, msgDefnForResult.MessageType) && isKey(checkSumMap, msgDefnForResult.MessageType)
        if strcmp(refMsgTypeCheckSumMap(msgDefnForResult.MessageType), checkSumMap(msgDefnForResult.MessageType))
            resultMsgUpToDate = true;
        end
    end
    
    if isKey(refMsgTypeCheckSumMap, msgDefnForFeedback.MessageType) && isKey(checkSumMap, msgDefnForFeedback.MessageType)
        if strcmp(refMsgTypeCheckSumMap(msgDefnForFeedback.MessageType), checkSumMap(msgDefnForFeedback.MessageType))
            feedbackMsgUpToDate = true;
        end
    end
    
	% Generate struct files for the message.
    if ~goalMsgUpToDate
        createOutput(msgDefnForGoal,structGenTemplate,structGenFileNameForGoal);
        createOutput(msgDefnForActionGoal,structGenTemplate,structGenFileNameForActionGoal)
        createOutput(msgDefnForActionGoal,commonTemplate,fullfile(outDir,actionGoalCommonFile));
    end
    if ~resultMsgUpToDate
        createOutput(msgDefnForResult,structGenTemplate,structGenFileNameForResult);
        createOutput(msgDefnForActionResult,structGenTemplate,structGenFileNameForActionResult);
        createOutput(msgDefnForActionResult,commonTemplate,fullfile(outDir,actionResultCommonFile));
    end
    if ~feedbackMsgUpToDate
        createOutput(msgDefnForFeedback,structGenTemplate,structGenFileNameForFeedback);
        createOutput(msgDefnForActionFeedback,structGenTemplate,structGenFileNameForActionFeedback);
        createOutput(msgDefnForActionFeedback,commonTemplate,fullfile(outDir,actionFeedbackCommonFile));
    end

    if ~goalMsgUpToDate || ~resultMsgUpToDate || ~feedbackMsgUpToDate
        %Generate action CPP file
        createOutput(msgDefnForGoal,actCppHeadersTemplate,fullfile(outDir,actCppOutFile)); %create
        createOutput(msgDefnForGoal,actCppBodyCommonTemplate,fullfile(outDir,actCppOutFile),'at'); %append
        createOutput(msgDefnForResult,actCppBodyCommonTemplate,fullfile(outDir,actCppOutFile),'at'); %append
        createOutput(msgDefnForFeedback,actCppBodyCommonTemplate,fullfile(outDir,actCppOutFile),'at'); %append
        createOutput(msgDefnForGoal,actCppFooterTemplate,fullfile(outDir,actCppOutFile),'at'); %append
        
        createOutput(msgDefnForAction,commonTemplate,fullfile(outDir,actionCommonFile));
        createOutput(msgDefnForAction,structGenTemplate,structGenFileNameForAction);
    end
	
    files = { actCppOutFile, structGenFileNameForGoal, structGenFileNameForResult, ...
              structGenFileNameForFeedback, structGenFileNameForActionGoal, structGenFileNameForActionResult, ...
              structGenFileNameForActionFeedback, structGenFileNameForAction, actionCommonFile, ...
              actionGoalCommonFile, actionResultCommonFile, actionFeedbackCommonFile};

    if isequal(rosver,'ros')
        classGenTemplate = fullfile(matlabroot,'toolbox','ros',rospath,'+ros','+internal',['+',rosver],'msg_class.m.tmpl');

        classGenFileNameForGoal = fullfile(outDirExtra,[msgDefnForGoal.msgInfo.msgName '.m']);
        classGenFileNameForResult = fullfile(outDirExtra,[msgDefnForResult.msgInfo.msgName '.m']);
        classGenFileNameForFeedback = fullfile(outDirExtra,[msgDefnForFeedback.msgInfo.msgName '.m']);
        
        classGenFileNameForActionGoal = fullfile(outDirExtra,[msgDefnForActionGoal.msgInfo.msgName '.m']);
        classGenFileNameForActionResult = fullfile(outDirExtra,[msgDefnForActionResult.msgInfo.msgName '.m']);
        classGenFileNameForActionFeedback = fullfile(outDirExtra,[msgDefnForActionFeedback.msgInfo.msgName '.m']);
        
        classGenFileNameForAction = fullfile(outDirExtra,[msgDefnForAction.msgInfo.msgName '.m']);

        if ~goalMsgUpToDate
            createOutput(msgDefnForGoal, classGenTemplate, classGenFileNameForGoal)
            createOutput(msgDefnForActionGoal, classGenTemplate, classGenFileNameForActionGoal)
        end
        
        if ~resultMsgUpToDate
            createOutput(msgDefnForResult, classGenTemplate, classGenFileNameForResult)
            createOutput(msgDefnForActionResult, classGenTemplate, classGenFileNameForActionResult)
        end
        
        if ~feedbackMsgUpToDate
            createOutput(msgDefnForFeedback, classGenTemplate, classGenFileNameForFeedback)
            createOutput(msgDefnForActionFeedback, classGenTemplate, classGenFileNameForActionFeedback)
        end
        
        if ~goalMsgUpToDate || ~resultMsgUpToDate || ~feedbackMsgUpToDate
            createOutput(msgDefnForAction, classGenTemplate, classGenFileNameForAction)
        end

        files = [files classGenFileNameForGoal classGenFileNameForResult classGenFileNameForFeedback ...
            classGenFileNameForActionGoal classGenFileNameForActionResult classGenFileNameForActionFeedback ...
            classGenFileNameForAction];
    end

end

function createOutput(data,tmplFile,outFile,outFileOperation)
%Load the given template and render the data in it.

    tmpl = ros.internal.emitter.MLTemplate;
    tmpl.loadFile(tmplFile);
    tmpl.outFile = outFile;
    if (nargin == 4 && isequal(outFileOperation,'at'))
        tmpl.outFileOperation = outFileOperation;
    end
    tmpl.render(data, 2);
end

function msgDefnForActionGoal = getMessageDefinitionForActionGoal(action,msgDefnForGoal,msgDefnForHeader,msgDefnForGoalID)
%Get the message definition for the action goal.

msgDefnForActionGoal.MessageType = [action 'ActionGoal'];
msgDefnForActionGoal.msgFields.header = msgDefnForHeader;
msgDefnForActionGoal.msgFields.goal_id = msgDefnForGoalID;
msgDefnForActionGoal.msgFields.goal = msgDefnForGoal;
msgDefnForActionGoal.msgFields.goal.count = 0;
msgDefnForActionGoal.msgFields.goal.constantValue = NaN;
msgDefnForActionGoal.msgFields.goal.defaultValue = NaN;
msgDefnForActionGoal.msgFields.goal.varsize = 0;
end

function msgDefnForActionResult = getMessageDefinitionForActionResult(action,msgDefnForResult,msgDefnForHeader,msgDefnForGoalStatus)
%Get the message definition for the action result.

msgDefnForActionResult.MessageType = [action 'ActionResult'];
msgDefnForActionResult.msgFields.header = msgDefnForHeader;
msgDefnForActionResult.msgFields.status = msgDefnForGoalStatus;
msgDefnForActionResult.msgFields.result = msgDefnForResult;
msgDefnForActionResult.msgFields.result.count = 0;
msgDefnForActionResult.msgFields.result.constantValue = NaN;
msgDefnForActionResult.msgFields.result.defaultValue = NaN;
msgDefnForActionResult.msgFields.result.varsize = 0;
end

function msgDefnForActionFeedback = getMessageDefinitionForActionFeedback(action,msgDefnForFeedback,msgDefnForHeader,msgDefnForGoalStatus)
%Get the message definition for the action feedback.

msgDefnForActionFeedback.MessageType = [action 'ActionFeedback'];
msgDefnForActionFeedback.msgFields.header = msgDefnForHeader;
msgDefnForActionFeedback.msgFields.status = msgDefnForGoalStatus;
msgDefnForActionFeedback.msgFields.feedback = msgDefnForFeedback;
msgDefnForActionFeedback.msgFields.feedback.count = 0;
msgDefnForActionFeedback.msgFields.feedback.constantValue = NaN;
msgDefnForActionFeedback.msgFields.feedback.defaultValue = NaN;
msgDefnForActionFeedback.msgFields.feedback.varsize = 0;
end

function msgDefnForAction = getMessageDefinitionForAction(action,msgDefnForActionGoal,msgDefnForActionResult,msgDefnForActionFeedback)
%Get the message definition for the action.

msgDefnForAction.MessageType = [action 'Action'];
msgDefnForAction.msgFields.action_goal = msgDefnForActionGoal;

msgDefnForAction.msgFields.action_goal.count = 0;
msgDefnForAction.msgFields.action_goal.constantValue = NaN;
msgDefnForAction.msgFields.action_goal.defaultValue = NaN;
msgDefnForAction.msgFields.action_goal.varsize = 0;

msgDefnForAction.msgFields.action_result = msgDefnForActionResult;

msgDefnForAction.msgFields.action_result.count = 0;
msgDefnForAction.msgFields.action_result.constantValue = NaN;
msgDefnForAction.msgFields.action_result.defaultValue = NaN;
msgDefnForAction.msgFields.action_result.varsize = 0;

msgDefnForAction.msgFields.action_feedback = msgDefnForActionFeedback;

msgDefnForAction.msgFields.action_feedback.count = 0;
msgDefnForAction.msgFields.action_feedback.constantValue = NaN;
msgDefnForAction.msgFields.action_feedback.defaultValue = NaN;
msgDefnForAction.msgFields.action_feedback.varsize = 0;
end

function [msgDefn,refMsgTypeCheckSumMap] = getAugmentedMessageDefinitionAndMd5ChecksumMap(msgDefn,rosver,registry,refMsgTypeCheckSumMap)
%get all the fields of message definition required for the generation of cpp files.

msgDefn.msgInfo = ros.internal.(rosver).getMessageInfo(msgDefn.MessageType,registry);
msgDefn = ros.internal.(rosver).augmentMessageDefinition(msgDefn,registry);
msgSpec = ros.internal.ros.createMsgSpec(msgDefn);
msgDefn.MD5Checksum = ros.internal.ros.computeMsgMd5(msgSpec,refMsgTypeCheckSumMap,msgDefn);

%store MD5 checksum of base type as well in the refMsgTypeCheckSumMap
refMsgTypeCheckSumMap(msgDefn.MessageType) = msgDefn.MD5Checksum;
end

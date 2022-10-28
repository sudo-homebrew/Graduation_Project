function rosProjectInfo = getProjectInfo(configData, mdlName, isRefModel, refLibrary, modelParseFcn, buildDir)
%This function is for internal use only. It may be removed in the future.
%
% GETPROJECTINFO Generate a ROS project structure that contains the
% information about the model for use with the ROS project generation.
%
%  import ros.slros.internal.cgen.*
%  data = codertarget.data.getData(getActiveConfigSet(mdlName));
%  refLibrary = 'ros2lib';
%  modelParseFcn = @ros.slros2.internal.bus.Util.getROS2BlocksInModel;
%  getProjectInfo(data,mdlName,false,refLibrary, modelParseFcn);
%
%  ans =
%
%   struct with fields:
%
%     PackageInformation: [1x1 struct]
%                 Folder: 'L:\R2019b\ros'
%              Workspace: 'L:\R2019b\ros'
%             Publishers: [1x1 containers.Map]
%            Subscribers: [1x1 containers.Map]
%            ExtmodeInfo: [1x1 struct]

%   Copyright 2020-2022 The MathWorks, Inc.

%% Coder target and Code-generation info

% arguments
%     configData (1,1) struct
%     mdlName (1,1) char {mustBeNonempty, mustBeLoaded(mdlName)}
%     isRefModel (1,1) logical
%     refLibrary (1,1) char {mustBeNonempty}
%     modelParseFcn (1,1) function_handle {mustBeNonempty}
%     buildDir {mustBeAFolder}
% end

% Store Package information

    import ros.codertarget.internal.Util

    rosProjectInfo.ModelName = mdlName;
    rosProjectInfo.PackageInformation = configData.Packaging;

    % Store ROS2 Install folder and workspace
    rosProjectInfo.Folder = configData.ROS2Install.Folder;
    rosProjectInfo.Workspace = configData.ROS2Install.Workspace;
    rosProjectInfo.needNodeInterface = true;
    rosProjectInfo.hasModelRefs = false;
    rosProjectInfo.IncludeMdlTermFcn = get_param(mdlName,'IncludeMdlTerminateFcn');

    % Get C++ class interface Settings
    if strcmp(get_param(mdlName, 'CodeInterfacePackaging'), 'C++ class')
        rtwCPPClassObj = get_param(mdlName,'RTWCPPFcnClass');
        rosProjectInfo.StepMethodName = rtwCPPClassObj.getStepMethodName;
        rosProjectInfo.ModelClassName = rtwCPPClassObj.ModelClassName;
    else
        rosProjectInfo.StepMethodName = [];
        rosProjectInfo.ModelClassName = [];
    end
    rosProjectInfo.ModelRTMVarName = [mdlName, '_M'];
    [isSingleTasking, sampleTimes, hasExplicitPartitions] = Util.getSchedulerData(mdlName, buildDir);
    rosProjectInfo.isSingleTasking = isSingleTasking;
    rosProjectInfo.HasExplicitPartitions = hasExplicitPartitions;
    rosProjectInfo.SampleTimes = sampleTimes;
    nanoSecs = rosProjectInfo.SampleTimes(1).Value(1)*1e9;
    % Convert SampleTime to nanoseconds
    rosProjectInfo.SampleTimeNsecs = sprintf('%d',int64(round(nanoSecs)));
    % will be added later
    rosProjectInfo.ExtmodeInfo = [];

    %% Pub/Sub/Svc info
    % get all the ROS2 blocks
    if isRefModel
        [~,~, pubSubMsgBlks,~,~,~,svcCallBlks] = feval(modelParseFcn,mdlName);
    else
        % @todo update the usage of edit-time filter filterOutCodeInactiveVariantSubsystemChoices()
        % instead use the post-compile filter codeCompileVariants() - g2604300
        [refMdls, ~] = find_mdlrefs(mdlName ,'MatchFilter',@Simulink.match.internal.filterOutCodeInactiveVariantSubsystemChoices); % look only inside code active choice of VSS
        pubSubMsgBlks = {};
        svcCallBlks = {};
        for mdlIter = 1:numel(refMdls)
            thisModel = refMdls{mdlIter};
            load_system(thisModel);
            [~, ~, allPubSubBlks,~,~,~,allSvcCallBlks] = feval(modelParseFcn,thisModel);
            pubSubMsgBlks = [pubSubMsgBlks;allPubSubBlks]; %#ok<AGROW> % cannot estimate the size until we visit all the model-blocks
            svcCallBlks = [svcCallBlks;allSvcCallBlks]; %#ok<AGROW> % cannot estimate the size until we visit all the model-blocks
        end
    end
    % -------------------------------------------------------------------------
    % Create publisher/subscriber MAP with following format
    % --------------+----------------------------------------------------------
    %   Key (char)  |                          Value (structure)
    % --------------+----------------------------------------------------------
    %               | .BlockID = '<BlkName_id>', .BusName = '<SL_Bus_busname>',
    %  BlkName      | .msgInfo.msgCPPClassName = 'msg::type'
    %               |
    % --------------+----------------------------------------------------------

    Publishers = containers.Map();
    Subscribers = containers.Map();
    MessageTypes = cell(numel(pubSubMsgBlks)+2*numel(svcCallBlks),1);
    registry = ros.internal.CustomMessageRegistry.getInstance('ros2');
    % loop over all the blocks and populate the map
    for ii=1:numel(pubSubMsgBlks)
        thisBlk = pubSubMsgBlks{ii};
        % replace all newline characters to <space>
        thisKey = strrep(thisBlk, newline, ' ');
        % get the message type of the block
        msgType = get_param(thisBlk, 'messageType');
        MessageTypes{ii} = msgType;
        % convert message type to bus name
        busName = ros.slros2.internal.bus.Util.rosMsgTypeToBusName(msgType);
        % get message info to derive C++ class name
        msgInfo = ros.internal.ros2.getMessageInfo(msgType, registry);
        switch(get_param(thisBlk,'ReferenceBlock'))
          case [refLibrary '/Publish']
            Publishers(thisKey) = struct('BlockID', get_param([thisBlk,'/SinkBlock'], 'BlockId'), ...
                                         'BusName', busName, ...
                                         'msgInfo', msgInfo);
          case [refLibrary '/Subscribe']
            Subscribers(thisKey) = struct('BlockID', get_param([thisBlk,'/SourceBlock'], 'BlockId'), ...
                                          'BusName', busName, ...
                                          'msgInfo', msgInfo);
        end
    end

    % -------------------------------------------------------------------------
    % Create ServiceCaller MAP with following format
    % --------------+----------------------------------------------------------
    %   Key (char)  |                          Value (structure)
    % --------------+----------------------------------------------------------
    %               | .BlockID = '<BlkName_id>', .InputBusName = '<SL_Bus_busname>',
    %  BlkName      | .OutputBusName = '<SL_Bus_busname>',
    %               | .msgInfo.msgBaseCppClassName = 'pkg::srv::type'
    % --------------+----------------------------------------------------------
    ServiceCallers = containers.Map();
    ServiceTypes = cell(numel(svcCallBlks),1);
    % loop over all the blocks and populate the map
    for ii=1:numel(svcCallBlks)
        thisBlk = svcCallBlks{ii};
        % replace all newline characters to <space>
        thisKey = strrep(thisBlk, newline, ' ');
        % get the service type of the block
        svcType = get_param(thisBlk, 'serviceType');
        % add service into service type list
        ServiceTypes{ii} = svcType;
        % add service request and response into message type list
        MessageTypes{numel(pubSubMsgBlks)+2*ii-1} = [svcType 'Request'];
        MessageTypes{numel(pubSubMsgBlks)+2*ii} = [svcType 'Response'];
        % convert input and output service type to bus name
        inputBusName = ros.slros2.internal.bus.Util.rosMsgTypeToBusName([svcType 'Request']);
        outputBusName = ros.slros2.internal.bus.Util.rosMsgTypeToBusName([svcType 'Response']);
        % get service info to derive C++ class name
        msgInfo = ros.internal.ros2.getServiceInfo([svcType 'Request'], svcType, 'Request');
        % this is required for ProjectTool information
        reqMsgInfo = ros.internal.ros2.getMessageInfo([svcType 'Request'], registry);
        reqMsgStruct = struct('msgInfo',reqMsgInfo);
        ServiceCallers(thisKey) = struct('BlockID', get_param([thisBlk,'/ServiceCaller'],'BlockId'), ...
                                         'InputBusName', inputBusName, 'OutputBusName', outputBusName, 'msgInfo', msgInfo, ...
                                         'Request',reqMsgStruct);
    end

    rosProjectInfo.Publishers = Publishers;
    rosProjectInfo.Subscribers = Subscribers;
    rosProjectInfo.ServiceCallers = ServiceCallers;
    rosProjectInfo.MessageTypes = MessageTypes;
    rosProjectInfo.ServiceTypes = ServiceTypes;

end

function ret = mustBeLoaded(mdlName)
    ret = bdIsLoaded(mdlName);
end

% LocalWords:  Extmode RTWCPP chrono

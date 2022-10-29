function nodeinfo = postCodeGenHook(hCS, buildInfo)
%This function is for internal use only. It may be removed in the future.

%   INFO = postCodeGenHook(hCS, BUILDINFO) takes a Simulink.ConfigSet
%   object, hCS, and a RTW.BuildInfo object, BUILDINFO, and generates all
%   the ROS initialization and ROS<->Bus conversion code for the model in
%   the build area. NODEINFO is a struct with fields 'messageList' and
%   'nodeDependencies' (cell arrays of strings). INFO.nodeDependencies is
%   the list of required ROS packages, based upon the set of messages types
%   used in the model.
%
%   This function is called from ros.codertarget.internal.onAfterCodeGen
%
%   For testing purposes, this function can be invoked as follows:
%     hCS = getActiveConfigSet(gcs);
%     buildinfo = RTW.BuildInfo;
%     buildinfo.ModelName = bdroot(gcs);
%     info = ros.slros.internal.cgen.postCodeGenHook(hCS, buildinfo)

%   Copyright 2014-2022 The MathWorks, Inc.

    import ros.codertarget.internal.Util
    validateattributes(hCS, {'Simulink.ConfigSet','coder.EmbeddedCodeConfig',...
                             'coder.CodeConfig'}, {'scalar'});
    validateattributes(buildInfo, {'RTW.BuildInfo'}, {'scalar'});

    if ros.codertarget.internal.isMATLABConfig(hCS)
        model = ros.codertarget.internal.getBuildName(buildInfo);
    else
        model = buildInfo.ModelName;
    end

    % Find a header file that looks like <modelname>_types.h. This contains
    % definitions of the bus structs, and needs to be included by the bus
    % conversion header
    existingIncludeFiles = buildInfo.getIncludeFiles(true, true);
    modelTypesHdrIdx = find(contains(existingIncludeFiles, [model '_types.']));
    if ~isempty(modelTypesHdrIdx)
        [~, fname, ext] = fileparts( existingIncludeFiles{modelTypesHdrIdx} );
        modelTypesHdr = [fname ext];
    else
        modelTypesHdr = '';
    end

    % Get the build directory (that's where we will put the generated files)
    buildDir = ros.codertarget.internal.getBuildDir(buildInfo);
    if ros.codertarget.internal.isMATLABConfig(hCS)
        rosProjectInfo = loc_mlGetProjectInfo(hCS, model);

        % Generate main.cpp for MATLAB Coder
        % Generate message conversion functions
        cgenInfo = ros.codertarget.internal.ROSMATLABCgenInfo.getInstance;
        if ~isempty(getMessageTypesWithInt64(cgenInfo)) && ~hCS.HardwareImplementation.ProdLongLongMode
            warning(message('ros:mlroscpp:codegen:SetProdLongLongMode',model));
        end

        % Set node name an retrieve node information
        setNodeName(cgenInfo,model);
        nodeinfo = getNodeDependencies(cgenInfo);
        uniqueMsgList = setdiff(unique(nodeinfo.messageList),{'ros/Time','ros/Duration'});
        % Write message information to the project info for use with
        % ProjectTool and local deployment
        rosProjectInfo.MessageInfoArray = cell(1,numel(uniqueMsgList));
        for k=1:numel(uniqueMsgList)
            [~,~,msgInfo] = ros.internal.getEmptyMessage(uniqueMsgList{k},'ros');
            rosProjectInfo.MessageInfoArray{k} = msgInfo;
        end
        % Set image block dependency field to empty here to avoid conflict
        rosProjectInfo.ImageDepends = {''};
        rosProjectInfo.HasCodeChanged = true;
        % Save project info used by ProjectTool
        save(fullfile(buildDir, 'rosModelInfo.mat'), 'rosProjectInfo');

        % Generate main.cpp
        mainFile = ros.codertarget.internal.generateMain(buildDir,model);
        addSourceFiles(buildInfo,mainFile);

        % Generate message conversion functions, struct type definitions, etc.
        serviceTypes = getServiceTypes(cgenInfo);
        if ~isempty(cgenInfo.getMessageTypes)
            typesHeader = [model,'_types.h'];
            conversionFiles = ros.slros.internal.cgen.generateAllConversionFcns(...
                cgenInfo.getMessageTypes, serviceTypes, hCS, typesHeader, buildDir, ...
                'BusUtilityObject',ros.slroscpp.internal.bus.Util, ...
                'CodeGenUtilityObject',ros.slroscpp.internal.cgen.Util);
            msgConvertData.HasCoderArray = contains(fileread(fullfile(buildDir,typesHeader)),'coder_array.h');
            msgConvertData.NodeName = model;
            msgConvertData.ROSVer = 'ros';
            tmpl = ros.internal.emitter.MLTemplate;
            tmpl.loadFile(fullfile(toolboxdir('ros'),'codertarget','templates','mlroscpp_msgconvert_utils.h.tmpl'));
            tmpl.outFile = fullfile(buildDir, 'mlroscpp_msgconvert_utils.h');
            tmpl.render(msgConvertData,2);
            addSourceFiles(buildInfo,  fullfile(buildDir,conversionFiles.SourceFiles));
            addIncludeFiles(buildInfo, fullfile(buildDir,conversionFiles.HeaderFiles));
            addIncludeFiles(buildInfo, fullfile(buildDir,'mlroscpp_msgconvert_utils.h'));
        end
    else
        isTopLevelModel = Util.isTopLevelModel(buildInfo);
        rosProjectInfo = loc_slGetProjectInfo(hCS, buildInfo, isTopLevelModel, model, buildDir);

        rosProjectInfo.HasCodeChanged =  ros.codertarget.internal.hasModelChanged;        
        % Get C++ class interface Settings
        rosProjectInfo = ros.codertarget.internal.getCppClassDefinition(...
            hCS,buildDir,rosProjectInfo);
        modelinfo = ros.slros.internal.cgen.ROSModelInfo(model);
        msgTypes = modelinfo.messageTypesInModel();
        svcTypes = modelinfo.serviceTypesInModel();

        % Get the dependencies for this node (packages that it requires)
        nodeinfo = modelinfo.getNodeDependencies();
        rosProjectInfo.RefModelNodeInfo = nodeinfo;
        % Set image block dependency field to empty here to avoid conflict
        rosProjectInfo.ImageDepends = {''};
        % Add meta data for all the blocks in the model for use with Project tool
        rosProjectInfo = augmentBlockInfo(modelinfo, rosProjectInfo);
        if isTopLevelModel
            rosProjectInfo = loc_augmentModelInfo(rosProjectInfo);
            % generate platform agnostic interface/scheduler using c++11 standard
            % APIs for top-model
            if (Util.isROSControlEnabled(model))
                Util.generateROSControlFiles(rosProjectInfo, buildDir, buildInfo);
            else
                loc_generateNodeInterface(rosProjectInfo, buildInfo, buildDir);
            end
        end

        %Update the Project info with model specific GPU data
        ros.internal.gpucoder.updateBuildInfo(hCS, buildInfo, rosProjectInfo.RemoteBuild);
        rosProjectInfo.GPUFlags = ros.internal.gpucoder.getFlags(hCS);

        % project info used by ProjectTool
        save(fullfile(buildDir, 'rosModelInfo.mat'), 'rosProjectInfo');

        % Generate all the conversion functions
        conversionFiles = ros.slros.internal.cgen.generateAllConversionFcns(...
            msgTypes, svcTypes, model, modelTypesHdr, buildDir, ...
            'BusUtilityObject', ros.slroscpp.internal.bus.Util, ...
            'CodeGenUtilityObject', ros.slroscpp.internal.cgen.Util);
        % Generate the initialization functions
        initFiles = ros.slros.internal.cgen.generateInitializationFcns(...
            modelinfo, buildDir, isTopLevelModel);
        % Update buildInfo
        for k=1:numel(conversionFiles.HeaderFiles)
            buildInfo.addIncludeFiles(fullfile(buildDir,conversionFiles.HeaderFiles{k}));
        end
        for k=1:numel(initFiles.HeaderFiles)
            buildInfo.addIncludeFiles(fullfile(initFiles.HeaderFiles{k}));
        end
        if (codertarget.data.getParameterValue(hCS, 'DetectTaskOverruns'))
            overrunDetectionSrcFile = {fullfile(toolboxdir('ros'),'codertarget','src','slros_detect_overrun.cpp')};
        else
            overrunDetectionSrcFile = {};
        end

        sources = [conversionFiles.SourceFiles initFiles.SourceFiles, overrunDetectionSrcFile];
        for i=1:numel(sources)
            buildInfo.addSourceFiles(sources{i});
        end
    end

end

function rosProjectInfo = loc_mlGetProjectInfo(hCS, modelName)
% Store Package information
    rosProjectInfo = struct;
    rosProjectInfo.ModelName = modelName;
    rosProjectInfo.NodeName = modelName;
    % Generate a structure that ROS project builder expects to see. The
    % structure contains package information
    rosProjectInfo.PackageInformation = struct(...
        'MaintainerName',hCS.Hardware.PackageMaintainerName,...
        'MaintainerEmail',hCS.Hardware.PackageMaintainerEmail,...
        'License',hCS.Hardware.PackageLicense,...
        'Version',hCS.Hardware.PackageVersion);
    rosProjectInfo.ROS = struct(...
        'Install',hCS.Hardware.ROSFolder,...
        'CatkinWS',hCS.Hardware.CatkinWorkspace,...
        'RemoteBuild',~isequal(hCS.Hardware.DeployTo,'Localhost'));
    % There is no time stepping in MATLAB
    rosProjectInfo.ROS.ROSTimeStepping = false;
    rosProjectInfo.ROS.ROSTimeNotification = false;

    % Set build arguments needed for remote build
    rosProjectInfo.RemoteBuild = rosProjectInfo.ROS.RemoteBuild;
    rosProjectInfo.BuildArguments = loc_getCMakeBuildArgs();
    rosProjectInfo.StepMethodName = [];
    rosProjectInfo.ModelClassName = [];
end

function rosProjectInfo = loc_slGetProjectInfo(hCS, buildInfo, isTopLevelModel, modelName, buildDir)
% Store Package information
    import ros.codertarget.internal.Util

    rosProjectInfo = struct;
    configData = codertarget.data.getData(hCS);
    rosProjectInfo.ModelName = modelName;
    % For a future use: node name can be different
    rosProjectInfo.NodeName = modelName;
    rosProjectInfo.PackageInformation = configData.Packaging;
    rosProjectInfo.ROS = configData.ROS;
    % For models before R2020a, ROSTimeStepping and ROSTimeNotification
    % properties were saved as characters instead of logical. Convert the
    % characters to get the logical value.
    if isfield(configData.ROS, 'ROSTimeStepping')
        rosProjectInfo.ROS.ROSTimeStepping = loc_convertParameterToLogical(configData.ROS.ROSTimeStepping);
        rosProjectInfo.ROS.ROSTimeNotification = loc_convertParameterToLogical(configData.ROS.ROSTimeNotification);
    else
        rosProjectInfo.ROS.ROSTimeStepping = false;
        rosProjectInfo.ROS.ROSTimeNotification = false;
    end
    rosProjectInfo.needNodeInterface = true;
    rosProjectInfo.hasModelRefs = false;
    % RemoteBuild is true for ROS Melodic by default
    rosProjectInfo.RemoteBuild = true;
    rosProjectInfo.BuildArguments = [];
    if isfield(configData.ROS, 'RemoteBuild') && islogical(configData.ROS.RemoteBuild)
        rosProjectInfo.BuildArguments = loc_getCMakeBuildArgs();
        rosProjectInfo.RemoteBuild = configData.ROS.RemoteBuild;
    end
    rosProjectInfo.IncludeMdlTermFcn = get_param(modelName,'IncludeMdlTerminateFcn');
    rosProjectInfo.ModelRTMVarName = [modelName, '_M'];
    [isSingleTasking, sampleTimes, hasExplicitPartitions] = Util.getSchedulerData(modelName, buildDir);
    rosProjectInfo.isSingleTasking = isSingleTasking;
    rosProjectInfo.HasExplicitPartitions = hasExplicitPartitions;
    rosProjectInfo.SampleTimes = sampleTimes;
    nanoSecs = rosProjectInfo.SampleTimes(1).Value(1)*1e9;
    % Convert SampleTime to nanoseconds
    rosProjectInfo.SampleTimeNsecs = sprintf('%d',int64(round(nanoSecs)));
    rosProjectInfo.ExtmodeInfo = [];
    if isTopLevelModel
        % Add External mode information
        extModeInfo.ExtmodeSim = Util.isExternalModeBuild(buildInfo);
        if extModeInfo.ExtmodeSim
            % get values from the ExtMode configset
            extModeInfo.Port = codertarget.attributes.getExtModeData('Port', hCS);
            extModeInfo.RunInBackground = codertarget.attributes.getExtModeData('RunInBackground', hCS);
            extModeInfo.Verbose = codertarget.attributes.getExtModeData('Verbose', hCS);
        else
            % set default values
            extModeInfo.Port = '17725';
            extModeInfo.RunInBackground = true;
            extModeInfo.Verbose = '0';
        end
        rosProjectInfo.ExtmodeInfo = extModeInfo;
    end
end


function outVal= loc_convertParameterToLogical(val)
    if ischar(val)
        outVal = logical(str2double(val));
    else
        outVal = logical(val);
    end
end




function loc_generateNodeInterface(modelInfo, buildInfo, bDir)
    rosNodeConsts = ros.slros.internal.cgen.Constants.getNodeInterface;
    if isequal(get_param(modelInfo.ModelName,'CodeInterfacePackaging'),'C++ class')
        % when C++ class is selected for packaging - template is different
        headerTempl = strrep(rosNodeConsts.HeaderTemplate,'nonreusablefcn','modelclass');
        sourceTempl = strrep(rosNodeConsts.SourceFileTemplate,'nonreusablefcn','modelclass');
    else
        headerTempl = rosNodeConsts.HeaderTemplate;
        sourceTempl = rosNodeConsts.SourceFileTemplate;
    end
    % generate node interface header
    loc_createOutput(modelInfo, headerTempl, ...
                     fullfile(bDir, rosNodeConsts.HeaderFile));
    % generate node interface CPP
    loc_createOutput(modelInfo, sourceTempl, ...
                     fullfile(bDir, rosNodeConsts.SourceFile));
    % generate ROS Main CPP
    loc_createOutput(modelInfo, rosNodeConsts.MainTemplate, ...
                     fullfile(bDir, rosNodeConsts.MainFile));
    % Add node related build artifacts to buildInfo. Add only the
    % names of the header and CPP file as the buildInfo already has the
    % SourcePaths/IncludePaths for code-generation folder and will look for
    % those headers and includes
    buildInfo.addIncludeFiles(rosNodeConsts.HeaderFile);
    buildInfo.addSourceFiles(rosNodeConsts.SourceFile);
    buildInfo.addSourceFiles(rosNodeConsts.MainFile);
end

%--------------------------------------------------------------------------
function loc_createOutput(data,tmplFile,outFile)
%Load the given template and render the data in it.

    tmpl = ros.internal.emitter.MLTemplate;
    tmpl.loadFile(tmplFile);
    tmpl.outFile = outFile;
    tmpl.render(data, 2);
    % smart-indent and beautify the generated header and C++ files
    isaMexFileOnPath = 3;
    if (isaMexFileOnPath == exist('c_beautifier','file'))
        c_beautifier(outFile);
    end
end

function rosProjectInfo = loc_augmentModelInfo(rosProjectInfo)
%augmentModelInfo Adds extra information that supplements template based
%codegeneration for CMakeFileLists.txt and package.xml with ROS project
%generation for local build

    if rosProjectInfo.RemoteBuild
        % ModelInfo is used by ProjectTool only for local build
        return;
    end
    msgIncludes = {};
    publishers = rosProjectInfo.Publishers.values;
    msgIncludes = [msgIncludes cellfun(@(x)x.msgInfo.includeHeader, ...
                                       publishers, 'UniformOutput',false)];

    subscribers = rosProjectInfo.Subscribers.values;
    msgIncludes = [msgIncludes cellfun(@(x)x.msgInfo.includeHeader, ...
                                       subscribers, 'UniformOutput',false)];

    srvCallers = rosProjectInfo.ServiceCallers.values;

    msgIncludes = [msgIncludes cellfun(@(x)x.Request.msgInfo.includeHeader, ...
                                       srvCallers, 'UniformOutput',false)];
    msgIncludes = [msgIncludes cellfun(@(x)x.Response.msgInfo.includeHeader, ...
                                       srvCallers, 'UniformOutput',false)];

    rosProjectInfo.msgIncludes = unique(msgIncludes);

end

function cmakeBuildArgs  = loc_getCMakeBuildArgs()
% LOC_GETCMAKEBUILDARGS Returns cmake arguments build the ROS node using MATLAB
% ROS installation in MATLAB_ROOT/sys/ros1

    extraROSMessageCmakePath = fullfile(toolboxdir('ros'),'mlroscpp','custom_messages');
    cmakePrefixPath = strrep(['"' ros.internal.getCatkinPrefixPath ';' extraROSMessageCmakePath '"'],'\','/');
    cmakeBuildArgs = sprintf([' -DALIAS_ROS1_CLASS_LOADER=1 ',...
                              ' -DBoost_NAMESPACE="mwboost" ',...
                              ' -DCMAKE_PREFIX_PATH=%s'],cmakePrefixPath);

end
